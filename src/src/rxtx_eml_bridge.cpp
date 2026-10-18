#include "targets.h"

#if defined(EML_BRIDGE)

#include "eml_bridge.h"
#include "common.h"
#include "FHSS.h"
#include "hwTimer.h"
#include "logging.h"
#if defined(PLATFORM_ESP32)
#include <esp_system.h>
#endif

#if !defined(USE_AIRPORT_AT_BAUD) || !defined(RADIO_SX127X)
#error "EML_BRIDGE requires the AirPort UART bridge (USE_AIRPORT_AT_BAUD) on an SX127x target"
#endif

#if defined(TARGET_TX)
static constexpr EmlRole_e kRole = EML_COORDINATOR;
#elif defined(TARGET_RX)
static constexpr EmlRole_e kRole = EML_RESPONDER;
#else
#error "EML_BRIDGE needs TARGET_TX or TARGET_RX"
#endif

// Both modules of a pair must be built with the same frame size (part of configuration_id)
#if !defined(EML_FRAME_SIZE) || EML_FRAME_SIZE == 64
static constexpr EmlMacProfile_s kProfile = EML_PROFILE_LORA64;
#elif EML_FRAME_SIZE == 32
static constexpr EmlMacProfile_s kProfile = EML_PROFILE_LORA32;
#else
#error "EML_FRAME_SIZE must be 32 or 64"
#endif
static_assert(kProfile.frame_size <= RXBuffSize && kProfile.frame_size <= OTA_MAX_PACKET_SIZE, "radio buffers must hold an EML frame");
static constexpr uint32_t kSlotUs = kProfile.slot_us;

extern bool webserverPreventAutoStart;

#if defined(PLATFORM_ESP32)
static portMUX_TYPE emlMux = portMUX_INITIALIZER_UNLOCKED;
#define EML_CRITICAL_ENTER() portENTER_CRITICAL(&emlMux)
#define EML_CRITICAL_EXIT()  portEXIT_CRITICAL(&emlMux)
#else
#define EML_CRITICAL_ENTER() noInterrupts()
#define EML_CRITICAL_EXIT()  interrupts()
#endif

// ---- State ------------------------------------------------------------------------
// The transport rings replace the legacy AirPort FIFOs (P3.5/P4.6): 4096 B per
// direction on the ESP32 TX, 1024 B on the ESP8285 RX.
static EmlTransport<AP_MAX_BUF_LEN> transport;
static EmlTransportUart<AP_MAX_BUF_LEN> uartAdapter(transport);
static EmlTransportConfig_s transportConfig;
static EmlUartStats_s uartStats;
static EmlMacStats_s macStats;

static EmlMailbox rxMailbox;          // radio ISR -> foreground
static EmlMailbox txMailbox;          // foreground -> timer ISR
static EmlTicketRing<4> sentRing;     // timer ISR -> foreground, before any RX of the same pair

enum : uint8_t { MAC_OFF = 0, MAC_LOCKED, MAC_SEARCH };
static volatile uint8_t macMode = MAC_OFF;
static volatile bool macSuspended = false;
// transport.generation() as seen by the ISRs; RX descriptors are stamped with it when armed
static volatile uint32_t macGeneration = 0;

// Slot clock: written by the timer ISR, by the foreground only with the timer stopped
static EmlSlotClock_s slotClock;
// Published boundary snapshot, seqlock-protected (odd = being written)
static volatile uint32_t boundarySeq = 0;
static volatile uint32_t boundarySlot = 0;
static volatile uint16_t boundaryHop = 0;
static volatile uint32_t boundaryUs = 0;
static volatile bool txActive = false;

struct EmlArmedRx_s
{
    EmlRxContext_s context;
    uint32_t start_us;
    uint8_t flags;
    bool valid;
    bool taken;
};
static EmlArmedRx_s armed;                 // ISR owned (timer + DIO ISRs do not nest)
static EmlRadioDescriptor_s isrTx[2];      // timer ISR scratch for TX descriptors

// FHSS copies in DRAM: the ISRs never touch the const domain table
static uint32_t freqStart = 0;
static uint32_t freqSpread = 0;
static uint32_t syncFreq = 0;
static uint16_t hopCount = 1;

// Foreground only
static uint32_t prepSlot = 0xFFFFFFFFUL;
static bool prepLate = false;
static uint32_t peerProcessedSlot = 0xFFFFFFFFUL;
static uint32_t lastHeardSlot = 0;
static uint16_t researchSlots = 0;

// ---- ISR side ---------------------------------------------------------------------
static uint32_t ICACHE_RAM_ATTR hopFreq(uint16_t hop)
{
    return freqStart + (freqSpread * FHSSsequence[hop] / FREQ_SPREAD_SCALE) - FreqCorrection;
}

static void ICACHE_RAM_ATTR armRx(uint32_t slot, uint16_t hop, uint32_t startUs, uint8_t flags)
{
    armed.context.generation = macGeneration;
    armed.context.slot = slot;
    armed.context.hop = hop;
    armed.start_us = startUs;
    armed.flags = flags;
    armed.taken = false;
    armed.valid = true;
}

static void ICACHE_RAM_ATTR publishBoundary(uint32_t slot, uint16_t hop, uint32_t us)
{
    boundarySeq = boundarySeq + 1U;
    boundarySlot = slot;
    boundaryHop = hop;
    boundaryUs = us;
    boundarySeq = boundarySeq + 1U;
}

// Own slot: transmit the newest descriptor prepared for exactly this slot and
// generation, after re-checking them here (transport canSend() is foreground only).
static void ICACHE_RAM_ATTR ownBoundary(uint32_t slot, uint16_t hop, uint32_t nowUs)
{
    int8_t found = -1;
    uint8_t next = 0;
    while (txMailbox.pop(&isrTx[next]))
    {
        const EmlRadioDescriptor_s &d = isrTx[next];
        if (d.ticket.slot == slot && d.ticket.generation == macGeneration && d.length == kProfile.frame_size)
        {
            found = (int8_t)next;
            next ^= 1U;
        }
    }

    Radio.SetFrequencyReg(hopFreq(hop), SX12XX_Radio_All, false);
    if (found >= 0)
    {
        txActive = true;
        Radio.TXnb(isrTx[found].bytes, false, nullptr, SX12XX_Radio_1);
        // Reported before the peer slot starts, i.e. before any RX that can ACK it
        sentRing.push(isrTx[found].ticket);
        ++macStats.tx_frames;
    }
    else
    {
        ++macStats.tx_no_frame;
        if (kRole == EML_COORDINATOR)
        {
            Radio.RXnb();
            armRx(slot + 1U, hop, nowUs + kSlotUs, 0);
        }
    }
}

static void ICACHE_RAM_ATTR peerBoundary(uint32_t slot, uint16_t hop, uint32_t nowUs)
{
    if (kRole == EML_RESPONDER)
    {
        // Even slot: hop (the pair's new channel) and listen for the coordinator
        Radio.SetFrequencyReg(hopFreq(hop), SX12XX_Radio_All, false);
        Radio.RXnb();
        armRx(slot, hop, nowUs, 0);
    }
    else if (armed.valid && armed.context.slot == slot)
    {
        armed.start_us = nowUs; // RX was armed at TX done; use the real boundary
    }
    else
    {
        Radio.RXnb();
        armRx(slot, hop, nowUs, 0);
    }
}

static void ICACHE_RAM_ATTR emlTimerTock()
{
    uint32_t nowUs = micros();
    bool onTime = true;
    if (kRole == EML_COORDINATOR)
    {
        // The ESP32 timer interrupt is not IRAM-flagged, so it is masked while the flash
        // cache is off (config commit). The alarm keeps its grid: count the boundaries that
        // were masked so the slot/hop clock stays with the responder, and skip the radio
        // on a boundary that is serviced late.
        uint32_t elapsed = nowUs - boundaryUs;
        while (elapsed >= kSlotUs + kSlotUs / 2U)
        {
            emlClockAdvance(slotClock);
            elapsed -= kSlotUs;
            ++macStats.missed_boundaries;
        }
        if (elapsed > kSlotUs + EML_GUARD_US)
        {
            onTime = false;
            nowUs -= elapsed - kSlotUs;
        }
    }
    emlClockAdvance(slotClock);
    const uint32_t slot = slotClock.slot;
    const uint16_t hop = slotClock.hop;

    if (onTime && !macSuspended && macMode == MAC_LOCKED)
    {
        if (txActive)
        {
            // TX done never arrived inside the slot budget; abandon that TX
            txActive = false;
            ++macStats.tx_overrun;
        }
        if (emlOwnSlot(slot, kRole))
            ownBoundary(slot, hop, nowUs);
        else
            peerBoundary(slot, hop, nowUs);
    }
    // Published last so the foreground never sees this slot before its TX report
    publishBoundary(slot, hop, nowUs);
}

static void ICACHE_RAM_ATTR emlTimerTick() {}

static void ICACHE_RAM_ATTR emlTxDone()
{
    if (!txActive)
        return;
    txActive = false;
    if (macSuspended || macMode != MAC_LOCKED)
        return;
    if (kRole == EML_COORDINATOR)
    {
        // The responder answers on the same hop in the next slot
        Radio.RXnb();
        armRx(slotClock.slot + 1U, slotClock.hop, boundaryUs + kSlotUs, 0);
    }
}

static bool ICACHE_RAM_ATTR emlRxDone(SX12xxDriverCommon::rx_status const status)
{
    const uint32_t rxUs = micros();
    if (status != SX12xxDriverCommon::SX12XX_RX_OK || macSuspended || !armed.valid || armed.taken || txActive)
        return false;
    // One frame per armed slot; the rendezvous search accepts every candidate
    if (!(armed.flags & EML_DESC_SEARCH))
        armed.taken = true;
    if (rxMailbox.push(Radio.RXdataBuffer, kProfile.frame_size, armed.context, nullptr, rxUs, armed.start_us, armed.flags))
        ++macStats.rx_frames;
    else
        ++macStats.rx_dropped;
    return true;
}

// ---- Foreground -------------------------------------------------------------------
struct EmlBoundary_s
{
    uint32_t slot;
    uint16_t hop;
    uint32_t us;
};

static EmlBoundary_s readBoundary()
{
    EmlBoundary_s b;
    uint32_t seq;
    do
    {
        seq = boundarySeq;
        b.slot = boundarySlot;
        b.hop = boundaryHop;
        b.us = boundaryUs;
    } while ((seq & 1U) || seq != boundarySeq);
    return b;
}

static void publishGeneration()
{
    macGeneration = transport.generation();
}

static uint32_t freshEpochSeed()
{
#if defined(PLATFORM_ESP32)
    uint32_t seed = esp_random();
#else
    uint32_t seed = ESP.random();
#endif
    seed ^= micros() * 2654435761UL;
    return seed ? seed : 1U;
}

// Hop of a slot at most two slots after the boundary snapshot
static uint16_t hopOf(const EmlBoundary_s &b, uint32_t slot)
{
    EmlSlotClock_s c = {b.slot, b.hop, hopCount};
    while (c.slot != slot)
        emlClockAdvance(c);
    return c.hop;
}

static void prepareFor(uint32_t slot, uint16_t hop)
{
    alignas(4) uint8_t frame[EML_FRAME_SIZE_64];
    EmlTxTicket_s ticket;
    const bool ready = transport.prepare(slot, hop, frame, sizeof(frame), &ticket);
    publishGeneration();
    if (!ready)
        return;
    const EmlRxContext_s context = {ticket.generation, slot, hop};
    txMailbox.push(frame, kProfile.frame_size, context, &ticket);
}

static void enterSearch(uint32_t now, bool countLoss)
{
    hwTimer::stop();
    transport.reset(now);
    publishGeneration();
    EML_CRITICAL_ENTER();
    macMode = MAC_SEARCH;
    txActive = false;
    Radio.SetFrequencyReg(syncFreq, SX12XX_Radio_All, false);
    Radio.RXnb();
    armRx(slotClock.slot, 0, 0, EML_DESC_SEARCH);
    EML_CRITICAL_EXIT();
    prepSlot = peerProcessedSlot = 0xFFFFFFFFUL;
    if (countLoss)
    {
        ++macStats.lock_losses;
        DBGLN("EML: lost coordinator, searching");
    }
}

// Responder: a HELLO heard on the rendezvous channel defines slot, hop and phase
static void acquire(const EmlRadioDescriptor_s &d)
{
    EmlFrameView_s view;
    EmlSync_s sync;
    if (emlFrameDecode(d.bytes, d.length, kProfile.frame_size, &view) != EML_FRAME_OK ||
        view.header.pkt_type != EML_PKT_TYPE_SYNC ||
        emlSyncDecode(view.payload, view.header.payload_len, &sync) != EML_SYNC_OK ||
        sync.phase != EML_SYNC_HELLO || sync.sender_id != transportConfig.peer_id ||
        sync.receiver_id != transportConfig.local_id || sync.configuration_id != transportConfig.configuration_id ||
        sync.hop_index >= hopCount || FHSSsequence[sync.hop_index] != sync_channel)
        return;

    EmlAcquire_s a;
    if (!emlAcquire(slotClock.slot, sync.slot_counter, sync.hop_index, hopCount, d.rx_us, micros(), kProfile, &a))
        return;

    // Load the clock so the first timer boundary starts a.first_slot
    slotClock.slot = a.clock_slot;
    slotClock.hop = a.clock_hop;
    publishBoundary(a.clock_slot, a.clock_hop, a.first_boundary_us - kSlotUs);

    const EmlRxContext_s context = {d.context.generation, a.hello_slot, sync.hop_index};
    const EmlReceiveResult_e result = transport.receive(d.bytes, d.length, context, emlNextOwnSlot(a.clock_slot, kRole));
    publishGeneration();
    if (result != EML_RX_CONTROL)
        return;

    EML_CRITICAL_ENTER();
    armed.valid = false;
    macMode = MAC_LOCKED;
    EML_CRITICAL_EXIT();
    lastHeardSlot = a.hello_slot;
    prepSlot = peerProcessedSlot = 0xFFFFFFFFUL;

    if (emlOwnSlot(a.first_slot, kRole))
    {
        const EmlBoundary_s b = {a.clock_slot, a.clock_hop, 0};
        prepareFor(a.first_slot, hopOf(b, a.first_slot));
        prepSlot = a.first_slot;
    }

    // Start the timer on the boundary; its first tock begins a.first_slot
#if defined(PLATFORM_ESP8266)
    const uint32_t resumeLeadUs = 20; // ESP8266 hwTimer fires 20 us after resume()
#else
    const uint32_t resumeLeadUs = 0;
#endif
    if ((int32_t)(a.first_boundary_us - resumeLeadUs - micros()) < 0)
    {
        enterSearch(emlNextOwnSlot(a.clock_slot, kRole), false); // processed too late, wait for the next HELLO
        return;
    }
    while ((int32_t)(a.first_boundary_us - resumeLeadUs - micros()) > 0) {}
    hwTimer::resume();
    ++macStats.acquisitions;
    DBGLN("EML: locked slot=%u hop=%u", a.first_slot, a.clock_hop);
}

static void trackPhase(const EmlRadioDescriptor_s &d)
{
    int32_t correction = emlPhaseCorrection(d.rx_us, d.slot_start_us, kProfile);
    const int32_t limit = (int32_t)(kSlotUs / 4U);
    if (correction > limit) correction = limit;
    if (correction < -limit) correction = -limit;
    hwTimer::phaseShift(correction / 2);
}

static void handleRx(const EmlRadioDescriptor_s &d, uint32_t now)
{
    if (d.flags & EML_DESC_SEARCH)
    {
        if (macMode == MAC_SEARCH)
            acquire(d);
        return;
    }
    if (macMode != MAC_LOCKED)
        return;

    if (kRole == EML_RESPONDER)
    {
        EmlFrameView_s view;
        if (emlFrameDecode(d.bytes, d.length, kProfile.frame_size, &view) == EML_FRAME_OK)
        {
            lastHeardSlot = d.context.slot;
            trackPhase(d);
        }
    }
    transport.receive(d.bytes, d.length, d.context, now);
    publishGeneration();
    peerProcessedSlot = d.context.slot;
}

static void updateConnectionState()
{
    const bool active = transport.active();
    if (active && connectionState != connected)
    {
        webserverPreventAutoStart = true; // a working link never auto-starts Wi-Fi
        setConnectionState(connected);
    }
    else if (!active && connectionState == connected)
    {
        setConnectionState(disconnected);
    }
}

void emlBridgeBegin()
{
    hopCount = FHSSgetSequenceCount();
    freqStart = FHSSconfig->freq_start;
    freqSpread = freq_spread;
    syncFreq = FHSSgetInitialFreq();

    const EmlFhssRecord_s fhss = {FHSSconfig->freq_start, FHSSconfig->freq_stop, FHSSconfig->freq_count, OtaGetUidSeed(), hopCount};
    transportConfig = emlMacTransportConfig(kRole, emlDeriveLinkIds(UID, kProfile, fhss), kProfile, hopCount);
    researchSlots = (uint16_t)(transportConfig.link_timeout_slots + 4U);

    Radio.RXdoneCallback = &emlRxDone;
    Radio.TXdoneCallback = &emlTxDone;
    // P4.2: BW500 / SF6 (implicit header, fixed N) / CR4/5; errata 2.1 is applied by SetBandwidthCodingRate()
    Radio.Config(SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_5, syncFreq, kProfile.preamble, (UID[5] & 0x01), kProfile.frame_size);

    emlClockInit(slotClock, hopCount);
    publishBoundary(slotClock.slot, slotClock.hop, micros());
    if (Radio.PayloadLength != kProfile.frame_size ||
        !transport.begin(transportConfig, freshEpochSeed(), emlNextOwnSlot(slotClock.slot, kRole)))
    {
        ERRLN("EML: bridge configuration rejected");
        setConnectionState(radioFailed);
        return;
    }
    publishGeneration();
    DBGLN("EML: N=%u slot=%uus toa=%uus hops=%u cfg=%x", kProfile.frame_size, kSlotUs, emlProfileToaUs(kProfile),
          hopCount, transportConfig.configuration_id);

    hwTimer::init(kRole == EML_RESPONDER ? emlTimerTick : nullptr, emlTimerTock);
    hwTimer::updateInterval(kSlotUs);
    setConnectionState(disconnected);

    if (kRole == EML_COORDINATOR)
    {
        macMode = MAC_LOCKED;
        hwTimer::resume();
    }
    else
    {
        enterSearch(emlNextOwnSlot(slotClock.slot, kRole), false);
    }
}

void emlBridgeLoop()
{
    if (macMode == MAC_OFF || connectionState > MODE_STATES)
        return;

    EmlBoundary_s b = readBoundary();
    uint32_t now = emlNextOwnSlot(b.slot, kRole);

    // 1. TX reports first: an ACK in the following RX must find the DATA marked sent
    EmlTxTicket_s ticket;
    while (sentRing.pop(&ticket))
    {
        if (!transport.sent(ticket, ticket.slot))
            ++macStats.sent_unmatched;
    }

    // 2. Received frames (acquisition can reload the clock)
    EmlRadioDescriptor_s d;
    while (rxMailbox.pop(&d))
    {
        handleRx(d, now);
        b = readBoundary();
        now = emlNextOwnSlot(b.slot, kRole);
    }

    // 3. Timeouts, lock supervision
    transport.tick(now);
    publishGeneration();
    if (kRole == EML_RESPONDER && macMode == MAC_LOCKED && b.slot - lastHeardSlot > researchSlots)
    {
        enterSearch(now, true);
    }

    // 4. Prepare the next own slot: early (stale ACK, covers a lost peer frame) and
    //    once more after the peer frame of the preceding slot was processed
    if (macMode == MAC_LOCKED)
    {
        const int32_t remaining = (int32_t)(b.us + (now - b.slot) * kSlotUs - micros());
        if (remaining > (int32_t)EML_PREP_MARGIN_US)
        {
            if (prepSlot != now)
            {
                prepSlot = now;
                prepLate = false;
                prepareFor(now, hopOf(b, now));
            }
            else if (!prepLate && peerProcessedSlot == now - 1U)
            {
                prepLate = true;
                ++macStats.late_prepares;
                prepareFor(now, hopOf(b, now));
            }
        }
    }

    updateConnectionState();
}

void emlBridgeSuspend(bool suspend)
{
    macSuspended = suspend;
}

void emlBridgeReadSerial(Stream &port)
{
    emlReadSerial(port, uartAdapter, transport.active(), uartStats);
}

void emlBridgeDrainSerial(Stream &port, uint32_t maxBytes)
{
    emlDrainSerial(port, uartAdapter, uartStats, maxBytes);
}

bool emlBridgeActive() { return transport.active(); }
const EmlTransportStats_s &emlBridgeTransportStats() { return transport.stats(); }
const EmlUartStats_s &emlBridgeUartStats() { return uartStats; }
const EmlMacStats_s &emlBridgeMacStats() { return macStats; }

#endif // EML_BRIDGE
