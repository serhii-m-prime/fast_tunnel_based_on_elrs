#include "targets.h"

#if defined(EML_BRIDGE)

#include "eml_bridge.h"
#include "common.h"
#include "FHSS.h"
#include "hwTimer.h"
#include "logging.h"
#include "eml_band.h"
#include "eml_tdd.h"
#include "eml_power.h"
#include "eml_qos.h"
#include "eml_role.h"
#include "POWERMGNT.h"
#include "options.h"
#if defined(PLATFORM_ESP32)
#include <esp_system.h>
#else
#include <user_interface.h>
#endif

#if !defined(USE_AIRPORT_AT_BAUD) || !defined(RADIO_SX127X)
#error "EML_BRIDGE requires the AirPort UART bridge (USE_AIRPORT_AT_BAUD) on an SX127x target"
#endif

#if defined(TARGET_TX)
static constexpr EmlRole_e kDefaultRole = EML_COORDINATOR;
#elif defined(TARGET_RX)
static constexpr EmlRole_e kDefaultRole = EML_RESPONDER;
#else
#error "EML_BRIDGE needs TARGET_TX or TARGET_RX"
#endif

// Both modules of a pair must be built with the same frame size and PHY mode; both are
// part of configuration_id, so a mismatched pair never completes the handshake.
#if !defined(EML_FRAME_SIZE) || EML_FRAME_SIZE == 64
static constexpr uint8_t kFrameSize = EML_FRAME_SIZE_64;
static constexpr EmlMacProfile_s kLoraProfile = EML_PROFILE_LORA64;
#if defined(EML_FSK_EXPERIMENTAL_300K)
static constexpr EmlMacProfile_s kFskProfile = EML_PROFILE_FSK64_300K; // P5.7 experiment, unverified
#else
static constexpr EmlMacProfile_s kFskProfile = EML_PROFILE_FSK64;
#endif
#elif EML_FRAME_SIZE == 32
static constexpr uint8_t kFrameSize = EML_FRAME_SIZE_32;
static constexpr EmlMacProfile_s kLoraProfile = EML_PROFILE_LORA32;
static constexpr EmlMacProfile_s kFskProfile = EML_PROFILE_FSK32;
#if defined(EML_FSK_EXPERIMENTAL_300K)
#error "EML_FSK_EXPERIMENTAL_300K is defined for 64-byte frames only"
#endif
#else
#error "EML_FRAME_SIZE must be 32 or 64"
#endif
static_assert(kFrameSize <= RXBuffSize && kFrameSize <= OTA_MAX_PACKET_SIZE, "radio buffers must hold an EML frame");
static_assert(kLoraProfile.frame_size == kFrameSize && kFskProfile.frame_size == kFrameSize, "one N for all PHYs");

// P5.5 PHY order, index 0 preferred:
//   EML_PHY_MODE 0 = LoRa only, 1 = FSK only, 2 (default) = FSK with LoRa fallback
#if !defined(EML_PHY_MODE)
#define EML_PHY_MODE 2
#endif
#if EML_PHY_MODE == 0
static const EmlMacProfile_s kPhyOrder[] = {kLoraProfile};
#elif EML_PHY_MODE == 1 || EML_PHY_MODE == 2
#if !defined(SX127X_FSK_SUPPORT)
#error "FSK PHY modes need SX127X_FSK_SUPPORT"
#endif
#if EML_PHY_MODE == 1
static const EmlMacProfile_s kPhyOrder[] = {kFskProfile};
#else
static const EmlMacProfile_s kPhyOrder[] = {kFskProfile, kLoraProfile};
#endif
#else
#error "EML_PHY_MODE must be 0, 1 or 2"
#endif

extern bool webserverPreventAutoStart;

#if defined(PLATFORM_ESP32)
static portMUX_TYPE emlMux = portMUX_INITIALIZER_UNLOCKED;
#define EML_CRITICAL_ENTER() portENTER_CRITICAL(&emlMux)
#define EML_CRITICAL_EXIT()  portEXIT_CRITICAL(&emlMux)
#else
#define EML_CRITICAL_ENTER() noInterrupts()
#define EML_CRITICAL_EXIT()  interrupts()
#endif

#ifndef EML_ARQ_WINDOW
#define EML_ARQ_WINDOW 4
#endif
#ifndef EML_ADAPTIVE_TDD
#define EML_ADAPTIVE_TDD 1
#endif
#ifndef EML_SHORT_ACK
#define EML_SHORT_ACK 1
#endif
static_assert(EML_ARQ_WINDOW >= 1 && EML_ARQ_WINDOW <= 4, "EML_ARQ_WINDOW must be 1..4");

// ---- State ------------------------------------------------------------------------
// The transport rings replace the legacy AirPort FIFOs (P3.5/P4.6): 4096 B per
// direction on the ESP32 TX, 1024 B on the ESP8285 RX.
static EmlTransport<AP_MAX_BUF_LEN, EML_ARQ_WINDOW> transport;
static EmlTransportUart<AP_MAX_BUF_LEN, EML_ARQ_WINDOW> outputUartAdapter(transport);
static EmlQosIngress<512> qosIngress;
static EmlQosUart<512> inputUartAdapter(qosIngress);
static EmlTransportConfig_s transportConfig;
static EmlUartStats_s uartStats;
static EmlMacStats_s macStats;

static EmlPhyPlan_s phyPlan;
static EmlBandSearch bandSearch;
static EmlBandController bandController;
static EmlTddController tddController;
static EmlDynamicPower dynamicPower;
static EmlTddPlan_s macTddPlan = {};
static bool adaptiveTdd = EML_ADAPTIVE_TDD && EML_ARQ_WINDOW > 1;
static volatile bool useShortAck = false;
static uint32_t tddGeneration = 0, acknowledgedBytes = 0;
static uint8_t pllMasks[EML_PHY_MAX] = {};
static const EmlMacProfile_s *profile = &kPhyOrder[0];
static EmlFhssRecord_s fhssRecord;
static EmlLinkQuality linkQuality;
static int16_t rssiAvgX16 = 0;
static int16_t snrAvgX16 = 0;
static bool rssiAvailable = false, snrAvailable = false;
static EmlLinkReport_s peerLink = {};
static bool peerLinkValid = false;
static uint32_t lastValidMs = 0;
static uint8_t missedAckCount = 0, watchedSequence = 0;
static bool wasActive = false;
static EmlRole_e runtimeRole = kDefaultRole;
static EmlRoleElection roleElection;
static uint32_t lastHelloConfiguration = 0; // diagnostics: configuration_id of the last HELLO heard
static EmlBridgeDiag_s diag = {};            // bench diagnostics, foreground only
static uint32_t qosGeneration = 0;
static uint32_t qosPumpedSlot = 0xFFFFFFFFUL;

static EmlMailbox rxMailbox;          // radio ISR -> foreground
static EmlMailbox txMailbox;          // foreground -> timer ISR
static EmlTicketRing<4> sentRing;     // timer ISR -> foreground, before any RX of the same pair

enum : uint8_t { MAC_OFF = 0, MAC_LOCKED, MAC_SEARCH };
static volatile uint8_t macMode = MAC_OFF;
static volatile bool macSuspended = false;
// transport.generation() as seen by the ISRs; RX descriptors are stamped with it when armed
static volatile uint32_t macGeneration = 0;
// Slot length of the active PHY (ISR and foreground)
static volatile uint32_t slotUs = kPhyOrder[0].slot_us;

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
    uint8_t length;
    bool valid;
    bool taken;
};
static EmlArmedRx_s armed;                 // ISR owned (timer + DIO ISRs do not nest)
static EmlRadioDescriptor_s isrTx[2];      // timer ISR scratch for TX descriptors

// FHSS copies in DRAM: the ISRs never touch the const domain table
static volatile uint32_t freqStart = 0;
static uint32_t freqSpread = 0;
static volatile uint32_t syncFreq = 0;
static uint16_t hopCount = 1;
static volatile uint8_t macBand = 0;
static volatile bool bandSwitchArmed = false;
static volatile uint32_t bandSwitchSlot = 0, bandSwitchStart = 0, bandSwitchSync = 0;
static volatile uint8_t bandSwitchTarget = 0;
static uint8_t foregroundBand = 0;

// Foreground only
static uint32_t prepSlot = 0xFFFFFFFFUL;
static bool prepLate = false;
static uint32_t peerProcessedSlot = 0xFFFFFFFFUL;
static uint32_t lastHeardSlot = 0;
static uint16_t researchSlots = 0;

// ---- ISR side ---------------------------------------------------------------------
static uint8_t ICACHE_RAM_ATTR physicalLength(uint32_t slot)
{
    return useShortAck && emlShortAckSlot(slot) ? EML_ACK_FRAME_SIZE : kFrameSize;
}

static uint32_t nextOwn(uint32_t slot) { return emlTddNextOwn(slot, runtimeRole, macTddPlan); }

// Adaptive TDD must see the whole backlog: the pump hands the transport one frame
// at a time, so most queued UART bytes wait in the QoS ingress.
static size_t txBacklog() { return transport.queuedTx() + qosIngress.queued(); }

// P9.1 fast turnaround (default): Standby -> FRF -> FSTX/FSRX, TX/RX entered from the
// locked synthesizer. 0: conventional Standby -> FRF, the TX/RX request runs the
// chip's own FS + PLL-lock sequence (bench comparison for FSK).
#ifndef EML_FAST_TURNAROUND
#define EML_FAST_TURNAROUND 1
#endif
static void ICACHE_RAM_ATTR tuneFor(uint32_t freq, bool tx)
{
#if EML_FAST_TURNAROUND
    Radio.PrepareFrequencyFast(freq, tx);
#else
    (void)tx;
    Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false);
#endif
}

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
    armed.length = flags & EML_DESC_SEARCH ? kFrameSize : physicalLength(slot);
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
        if (d.ticket.slot == slot && d.ticket.generation == macGeneration && d.length == physicalLength(slot))
        {
            found = (int8_t)next;
            next ^= 1U;
        }
    }

    armed.valid = false;
    Radio.SetPacketLength(physicalLength(slot));
    if (found >= 0)
    {
        tuneFor(hopFreq(hop), true);
        txActive = true;
        Radio.TXnb(isrTx[found].bytes, false, nullptr, SX12XX_Radio_1);
        if (FHSSsequence[hop] == sync_channel)
            ++macStats.tx_sync_channel;
        // Reported before the peer slot starts, i.e. before any RX that can ACK it
        sentRing.push(isrTx[found].ticket);
        ++macStats.tx_frames;
    }
    else
    {
        ++macStats.tx_no_frame;
        tuneFor(hopFreq(hop), false);
        Radio.RXnb();
        armRx(slot, hop, nowUs, 0);
    }
}

static void ICACHE_RAM_ATTR peerBoundary(uint32_t slot, uint16_t hop, uint32_t nowUs)
{
    if (armed.valid && armed.context.slot == slot && armed.context.hop == hop)
    {
        armed.start_us = nowUs;
    }
    else
    {
        Radio.SetPacketLength(physicalLength(slot));
        tuneFor(hopFreq(hop), false);
        Radio.RXnb();
        armRx(slot, hop, nowUs, 0);
    }
}

static void ICACHE_RAM_ATTR emlTimerTock()
{
    uint32_t nowUs = micros();
    bool onTime = true;
    if (runtimeRole == EML_COORDINATOR)
    {
        // The ESP32 timer interrupt is not IRAM-flagged, so it is masked while the flash
        // cache is off (config commit). The alarm keeps its grid: count the boundaries that
        // were masked so the slot/hop clock stays with the responder, and skip the radio
        // on a boundary that is serviced late.
        const uint32_t slotLen = slotUs;
        uint32_t elapsed = nowUs - boundaryUs;
        while (elapsed >= slotLen + slotLen / 2U)
        {
            emlClockAdvance(slotClock);
            elapsed -= slotLen;
            ++macStats.missed_boundaries;
        }
        if (elapsed > slotLen + EML_GUARD_US)
        {
            onTime = false;
            nowUs -= elapsed - slotLen;
        }
    }
    emlClockAdvance(slotClock);
    const uint32_t slot = slotClock.slot;
    const uint16_t hop = slotClock.hop;

    if (bandSwitchArmed && !(slot & 1U) && (int32_t)(slot - bandSwitchSlot) >= 0)
    {
        // DRAM-only O(1) transition; the same channel permutation/hop continues.
        freqStart = bandSwitchStart;
        syncFreq = bandSwitchSync;
        macBand = bandSwitchTarget;
        bandSwitchArmed = false;
        armed.valid = false;
    }

    if (onTime && !macSuspended && macMode == MAC_LOCKED)
    {
        if (txActive)
        {
            // TX done never arrived inside the slot budget; abandon that TX
            txActive = false;
            ++macStats.tx_overrun;
        }
        if (emlTddOwner(slot, macTddPlan) == runtimeRole)
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
    EmlSlotClock_s next = slotClock;
    emlClockAdvance(next);
    if (emlTddOwner(next.slot, macTddPlan) != runtimeRole)
    {
        Radio.SetPacketLength(physicalLength(next.slot));
        tuneFor(hopFreq(next.hop), false);
        Radio.RXnb();
        armRx(next.slot, next.hop, boundaryUs + slotUs, 0);
    }
}

static bool ICACHE_RAM_ATTR emlRxDone(SX12xxDriverCommon::rx_status const status)
{
    const uint32_t rxUs = micros();
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
    {
        ++macStats.rx_irq_errors;
        return false;
    }
    if (macSuspended || !armed.valid || armed.taken || txActive)
        return false;
    // One frame per armed slot; the rendezvous search accepts every candidate
    if (!(armed.flags & EML_DESC_SEARCH))
        armed.taken = true;
    // LoRa: packet RSSI register; FSK: RSSI sampled by the driver at PayloadReady
    const int8_t rssi = Radio.GetLastPacketRSSI(SX12XX_Radio_1);
    uint8_t flags = armed.flags;
    int8_t snr = 0;
    if (!Radio.IsFsk())
    {
        snr = Radio.GetLastPacketSNRRaw(SX12XX_Radio_1) / RADIO_SNR_SCALE;
        flags |= EML_DESC_HAS_SNR;
    }
    if (rxMailbox.push(Radio.RXdataBuffer, armed.length, armed.context, nullptr, rxUs, armed.start_us, flags, rssi, snr))
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
    if (qosGeneration != macGeneration)
    {
        qosGeneration = macGeneration;
        qosIngress.clear();
        qosPumpedSlot = 0xFFFFFFFFUL;
    }
    if (tddGeneration != macGeneration)
    {
        tddGeneration = macGeneration;
        tddController.begin(runtimeRole, adaptiveTdd, slotClock.slot);
        EML_CRITICAL_ENTER();
        macTddPlan = {};
        EML_CRITICAL_EXIT();
    }
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

// Hop of a slot at most four slots after the boundary snapshot
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
    EmlBandMessage_s command;
    const uint8_t missedAck = transport.pending() && transport.peerFlowControl() != EML_FLOW_CTRL_PAUSE ? missedAckCount : 0;
    const bool shortFrame = physicalLength(slot) == EML_ACK_FRAME_SIZE;
    bool control = false;
    if (transport.active() && !shortFrame && emlTddAnchor(slot, runtimeRole) && (slot & 7U) == (runtimeRole == EML_COORDINATOR ? 0U : 3U))
    {
        const uint8_t lq = linkQuality.samples() >= 10 ? linkQuality.lq() : 100;
        const EmlLinkReport_s local = {profile->phy_profile, lq, (uint8_t)(100U - lq),
            rssiAvailable ? (int8_t)(rssiAvgX16 / 16) : (int8_t)-127,
            (int8_t)(snrAvgX16 / 16), snrAvailable};
        const bool metricsSlot = (slot & 63U) == (runtimeRole == EML_COORDINATOR ? 16U : 51U);
        if (metricsSlot && !bandController.busy() && !tddController.negotiating())
        {
            command = emlLinkMessage(runtimeRole, local);
            control = true;
        }
        else
        {
            control = bandController.command(slot, linkQuality.lq(), linkQuality.samples(), missedAck, &command);
            if (!control && !bandController.busy())
                control = tddController.command(slot, txBacklog(), transport.flowControl(), transport.pending(), &command);
        }
    }
    qosIngress.flushIdle(millis());
    // Bench diagnostics: why a new DATA could not take UART bytes in this own slot
    if (transport.active() && !shortFrame && qosPumpedSlot != slot)
    {
        if (control) ++macStats.pump_control;
        else if (transport.unissuedTx() != 0) ++macStats.pump_unissued;
        else if (transport.inFlight() >= EML_ARQ_WINDOW) ++macStats.pump_window;
        else if (transport.peerFlowControl() == EML_FLOW_CTRL_PAUSE) ++macStats.pump_flow;
        else if (!qosIngress.frontSize()) ++macStats.pump_empty;
    }
    if (transport.active() && !shortFrame && !control && transport.unissuedTx() == 0 &&
        transport.inFlight() < EML_ARQ_WINDOW && transport.peerFlowControl() != EML_FLOW_CTRL_PAUSE &&
        qosPumpedSlot != slot)
    {
        uint8_t payload[EML_MAX_PAYLOAD_LEN];
        bool high = false;
        const size_t packetLength = qosIngress.frontSize(&high);
        const size_t queued = qosIngress.peek(payload, emlMaxPayloadForFrameSize(kFrameSize), &high);
        if (queued && transport.uartSpace() >= queued && transport.acceptUart(payload, queued) == queued)
        {
            qosIngress.noteFragment(packetLength, queued, high);
            qosIngress.consume(queued, high);
            qosPumpedSlot = slot;
            ++macStats.pump_ok;
        }
    }
    const bool ready = transport.prepare(slot, hop, frame, sizeof(frame), &ticket, control ? &command : nullptr, shortFrame);
    publishGeneration();
    if (!ready)
        return;
    const EmlRxContext_s context = {ticket.generation, slot, hop};
    txMailbox.push(frame, shortFrame ? EML_ACK_FRAME_SIZE : kFrameSize, context, &ticket);
    if (FHSSsequence[hop] == sync_channel)
    {
        memcpy(diag.last_tx_sync_frame, frame, kFrameSize);
        diag.last_tx_sync_length = shortFrame ? EML_ACK_FRAME_SIZE : kFrameSize;
    }
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
    Radio.SetPacketLength(kFrameSize);
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
    ++macStats.search_frames;
    if (diag.search_frames_seen == 0 || d.rssi < diag.search_rssi_min) diag.search_rssi_min = d.rssi;
    if (diag.search_frames_seen == 0 || d.rssi > diag.search_rssi_max) diag.search_rssi_max = d.rssi;
    ++diag.search_frames_seen;
    if (emlFrameDecode(d.bytes, d.length, kFrameSize, &view) != EML_FRAME_OK)
    {
        ++macStats.search_bad_frame; // CRC/header: noise, foreign link or other PHY/frame size
        memcpy(diag.last_bad_frame, d.bytes, kFrameSize);
        diag.last_bad_length = d.length;
        diag.last_bad_rssi = d.rssi;
        return;
    }
    if (view.header.pkt_type != EML_PKT_TYPE_SYNC ||
        emlSyncDecode(view.payload, view.header.payload_len, &sync) != EML_SYNC_OK ||
        sync.phase != EML_SYNC_HELLO)
    {
        ++macStats.search_not_hello;
        return;
    }
    lastHelloConfiguration = sync.configuration_id;
    if (sync.sender_id != transportConfig.peer_id || sync.receiver_id != transportConfig.local_id)
    {
        ++macStats.search_id_mismatch; // different UID or both nodes in the same role
        return;
    }
    if (sync.configuration_id != transportConfig.configuration_id)
    {
        ++macStats.search_cfg_mismatch; // PHY/TDD/band policy/FHSS settings differ
        return;
    }
    if (sync.hop_index >= hopCount || FHSSsequence[sync.hop_index] != sync_channel)
    {
        ++macStats.search_off_hop; // e.g. a strong adjacent-channel HELLO leaking into the sync channel
        diag.last_off_hop = sync.hop_index;
        diag.last_off_hop_channel = sync.hop_index < hopCount ? FHSSsequence[sync.hop_index] : 0xFF;
        diag.last_off_hop_rssi = d.rssi;
        return;
    }

    EmlAcquire_s a;
    if (!emlAcquire(slotClock.slot, sync.slot_counter, sync.hop_index, hopCount, d.rx_us, micros(), *profile, &a))
        return;

    // Load the clock so the first timer boundary starts a.first_slot
    slotClock.slot = a.clock_slot;
    slotClock.hop = a.clock_hop;
    publishBoundary(a.clock_slot, a.clock_hop, a.first_boundary_us - slotUs);

    const EmlRxContext_s context = {d.context.generation, a.hello_slot, sync.hop_index};
    const EmlReceiveResult_e result = transport.receive(d.bytes, d.length, context, nextOwn(a.clock_slot));
    publishGeneration();
    if (result != EML_RX_CONTROL)
        return;

    EML_CRITICAL_ENTER();
    armed.valid = false;
    macMode = MAC_LOCKED;
    EML_CRITICAL_EXIT();
    lastHeardSlot = a.hello_slot;
    prepSlot = peerProcessedSlot = 0xFFFFFFFFUL;
    linkQuality.reset(a.hello_slot);
    linkQuality.received(a.hello_slot);

    if (emlTddOwner(a.first_slot, macTddPlan) == runtimeRole)
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
        ++macStats.search_late;
        enterSearch(nextOwn(a.clock_slot), false); // processed too late, wait for the next HELLO
        return;
    }
    while ((int32_t)(a.first_boundary_us - resumeLeadUs - micros()) > 0) {}
    hwTimer::resume();
    ++macStats.acquisitions;
    lastValidMs = millis();
    DBGLN("EML: locked phy=%u slot=%u hop=%u", profile->phy_profile, a.first_slot, a.clock_hop);
}

static void trackPhase(const EmlRadioDescriptor_s &d)
{
    EmlMacProfile_s receivedProfile = *profile;
    receivedProfile.frame_size = d.length; // compact FSK ACK has its own exact ToA
    int32_t correction = emlPhaseCorrection(d.rx_us, d.slot_start_us, receivedProfile);
    const int32_t limit = (int32_t)(slotUs / 4U);
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

    const EmlReceiveResult_e result = transport.receive(d.bytes, d.length, d.context, now);
    if (result == EML_RX_REJECTED)
    {
        memcpy(diag.last_invalid_frame, d.bytes, d.length <= sizeof(diag.last_invalid_frame) ? d.length : sizeof(diag.last_invalid_frame));
        diag.last_invalid_length = d.length;
        diag.last_invalid_rssi = d.rssi;
        diag.last_invalid_offset_us = (int32_t)(d.rx_us - d.slot_start_us);
    }
    if (result != EML_RX_REJECTED && result != EML_RX_STALE)
    {
        diag.last_valid_offset_us = (int32_t)(d.rx_us - d.slot_start_us);
        // Only session/configuration/freshness-validated frames supervise the link.
        lastValidMs = millis();
        linkQuality.received(d.context.slot);
        if (!rssiAvailable)
        {
            rssiAvgX16 = (int16_t)(d.rssi * 16);
            rssiAvailable = true;
        }
        else
            rssiAvgX16 = (int16_t)(rssiAvgX16 + ((int16_t)(d.rssi * 16) - rssiAvgX16) / 8);
        if (d.flags & EML_DESC_HAS_SNR)
        {
            if (!snrAvailable) snrAvgX16 = (int16_t)(d.snr * 16);
            else snrAvgX16 = (int16_t)(snrAvgX16 + ((int16_t)(d.snr * 16) - snrAvgX16) / 8);
            snrAvailable = true;
        }
        if (runtimeRole == EML_RESPONDER)
        {
            lastHeardSlot = d.context.slot;
            trackPhase(d);
        }
        if (result == EML_RX_CONTROL && transport.active())
        {
            EmlFrameView_s view;
            EmlBandMessage_s command;
            if (emlFrameDecode(d.bytes, d.length, kFrameSize, &view) == EML_FRAME_OK &&
                view.header.pkt_type == EML_PKT_TYPE_SYNC &&
                emlBandMessageRead(view.payload, view.header.payload_len, &command))
            {
                if (command.op <= EML_BAND_STATUS)
                    bandController.receive(command, d.context.slot, lastValidMs);
                else if (command.op <= EML_TDD_STATUS)
                    tddController.receive(command, d.context.slot);
                else
                {
                    EmlLinkReport_s report;
                    if (emlLinkReport(command, &report))
                    {
                        peerLink = report;
                        peerLinkValid = true;
                        if (dynamicPower.receive(report, lastValidMs))
                            POWERMGNT::setPower((PowerLevels_e)dynamicPower.power());
                    }
                }
            }
        }
    }
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

// Configure the radio, transport and timer for the search tuple. The slot clock and
// generation counting continue across PHY changes; the transport restarts its search.
static bool applyPhy()
{
    profile = &phyPlan.profiles[bandSearch.phy()];
    EML_CRITICAL_ENTER();
    macMode = MAC_OFF;
    armed.valid = false;
    txActive = false;
    EML_CRITICAL_EXIT();
    hwTimer::stop();
    Radio.SetTxIdleMode();
    foregroundBand = macBand = bandSearch.band();
    freqStart = emlBandStartReg(macBand);
    freqSpread = EML_CHANNEL_STEP_REG * FREQ_SPREAD_SCALE;
    syncFreq = emlBandChannelReg(macBand, EML_SYNC_CHANNEL);
    bandSwitchArmed = false;

    bool ok;
#if defined(SX127X_FSK_SUPPORT)
    if (emlIsFsk(*profile))
    {
        // P5.1: GFSK packet mode, fixed N, sync word from the binding UID
        SX127xFskParams_s fsk = {profile->bitrate, profile->fdev_hz, profile->rxbw_hz,
                                 (uint8_t)(profile->shaping << 5), profile->preamble, EML_FSK_SYNC_BYTES, {}};
        emlFskSyncWord(UID, fsk.sync);
        ok = Radio.ConfigFsk(fsk, syncFreq, kFrameSize);
    }
    else
#endif
    {
        // P4.2: BW500 / SF6 (implicit header, fixed N) / CR4/5; errata 2.1 is applied by SetBandwidthCodingRate()
        Radio.Config(SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_5, syncFreq, profile->preamble, (UID[5] & 0x01), kFrameSize);
        ok = true;
    }
    ok = ok && Radio.PayloadLength == kFrameSize;

    slotUs = profile->slot_us;
    useShortAck = EML_SHORT_ACK && emlIsFsk(*profile);
    EmlLinkIds_s ids = emlDeriveLinkIds(UID, *profile, fhssRecord, true, EML_ARQ_WINDOW, adaptiveTdd, useShortAck);
    const uint8_t runtimePolicy[] = {'P', '8', 1, firmwareOptions.bridge_default_band,
        (uint8_t)firmwareOptions.bridge_adaptive_fallback, firmwareOptions.bridge_lq_threshold};
    ids.configuration = emlFnv1a(runtimePolicy, sizeof(runtimePolicy), ids.configuration);
    if (!ids.configuration) ids.configuration = 1;
    transportConfig = emlMacTransportConfig(runtimeRole, ids, *profile, hopCount,
                                             EML_ARQ_WINDOW, adaptiveTdd, useShortAck);
    researchSlots = (uint16_t)(transportConfig.link_timeout_slots + 4U);
    const uint32_t now = nextOwn(slotClock.slot);
    ok = ok && transport.begin(transportConfig, freshEpochSeed(), now);
    publishGeneration();
    if (!ok)
    {
        ERRLN("EML: bridge configuration rejected");
        setConnectionState(radioFailed);
        return false;
    }
    DBGLN("EML: phy=%u N=%u slot=%uus toa=%uus hops=%u cfg=%x", profile->phy_profile, kFrameSize, profile->slot_us,
          emlProfileToaUs(*profile), hopCount, transportConfig.configuration_id);

    hwTimer::updateInterval(profile->slot_us);
    prepSlot = peerProcessedSlot = 0xFFFFFFFFUL;
    linkQuality.reset(slotClock.slot + 1U);
    bandController.begin(runtimeRole, macBand, bandSearch.mask(), millis(), firmwareOptions.bridge_lq_threshold);
    rssiAvgX16 = snrAvgX16 = 0;
    rssiAvailable = snrAvailable = false;
    peerLinkValid = false;
    wasActive = false;
    lastValidMs = millis();
    missedAckCount = watchedSequence = 0;
    if (runtimeRole == EML_COORDINATOR)
    {
        publishBoundary(slotClock.slot, slotClock.hop, micros());
        macMode = MAC_LOCKED;
        hwTimer::resume();
    }
    else
    {
        enterSearch(now, false);
    }
    return true;
}

// P9.4: a role change restarts the search on a stopped timer, like a reboot into
// the other role. It happens only while no session exists.
static void applyRole(EmlRole_e role)
{
    hwTimer::stop();
    EML_CRITICAL_ENTER();
    macMode = MAC_OFF;
    armed.valid = false;
    txActive = false;
    runtimeRole = role;
    macTddPlan = {};
    EML_CRITICAL_EXIT();
    // The coordinator is a tock-only slot master, the responder follows with phase shifts
    hwTimer::init(role == EML_RESPONDER ? emlTimerTick : nullptr, emlTimerTock);
    tddController.begin(role, adaptiveTdd, slotClock.slot);
    bandSearch.begin(phyPlan, role, pllMasks, millis(), firmwareOptions.bridge_default_band);
    ++macStats.role_changes;
    DBGLN("EML: auto role -> %s", role == EML_COORDINATOR ? "master" : "slave");
    applyPhy();
}

static void beginRoleElection()
{
    // R = coordinator band cycle + longest band dwell (see eml_role.h)
    uint32_t cycleMs = 0, dwellMs = 0;
    for (uint8_t p = 0; p < phyPlan.count; ++p)
    {
        cycleMs += EML_BAND_COUNT * phyPlan.coordinator_dwell_ms[p];
        if (phyPlan.coordinator_dwell_ms[p] > dwellMs) dwellMs = phyPlan.coordinator_dwell_ms[p];
    }
    const uint32_t rendezvousMs = cycleMs + dwellMs;
    // Both nodes of a pair share UID; the chip MAC separates their draws.
    uint8_t mac[6];
#if defined(PLATFORM_ESP32)
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#else
    wifi_get_macaddr(STATION_IF, mac);
#endif
    roleElection.begin(firmwareOptions.bridge_role == EML_OPTION_ROLE_AUTO,
                       firmwareOptions.bridge_role == EML_OPTION_ROLE_SLAVE ? EML_RESPONDER : EML_COORDINATOR,
                       rendezvousMs + dwellMs, 2U * rendezvousMs + dwellMs, dwellMs,
                       emlFnv1a(mac, sizeof(mac), freshEpochSeed()), millis());
    runtimeRole = roleElection.role();
}

void emlBridgeBegin()
{
    qosIngress.begin((EmlTrafficMode_e)firmwareOptions.bridge_traffic_mode);
    hopCount = FHSSgetSequenceCount();
    freqStart = FHSSconfig->freq_start;
    freqSpread = freq_spread;
    syncFreq = FHSSgetInitialFreq();
    fhssRecord = {FHSSconfig->freq_start, FHSSconfig->freq_stop, FHSSconfig->freq_count, OtaGetUidSeed(), hopCount};

    const EmlMacProfile_s selected = firmwareOptions.bridge_phy == EML_OPTION_PHY_LORA_SF6 ?
        kLoraProfile : kFskProfile;
    phyPlan = emlPhyPlan(&selected, 1, FHSSconfig->freq_count);
    beginRoleElection();
    adaptiveTdd = EML_ADAPTIVE_TDD && EML_ARQ_WINDOW > 1 &&
        firmwareOptions.bridge_tdd_mode == EML_OPTION_TDD_ADAPTIVE;
    // Positive PLL test on the FSK page at cell edges and the two extreme
    // channel centres, repeated to reject intermittent lock. No RF TX occurs.
    uint8_t positiveMask = 0;
    for (uint8_t band = 0; band < EML_BAND_COUNT; ++band)
    {
        const uint32_t probes[] = {emlSx1276Frf(emlBandEdgeHz(band)), emlBandStartReg(band),
            emlBandChannelReg(band, EML_CHANNEL_COUNT - 1), emlSx1276Frf(emlBandEndHz(band))};
        bool ok = true;
        for (uint8_t repeat = 0; repeat < 3 && ok; ++repeat)
            for (uint32_t frequency : probes) if (!Radio.ProbePll(frequency)) { ok = false; break; }
        if (ok) positiveMask |= (uint8_t)(1U << band);
    }
    for (uint8_t p = 0; p < phyPlan.count; ++p)
    {
        pllMasks[p] = positiveMask;
        if (!emlIsFsk(phyPlan.profiles[p]))
        {
            for (uint8_t band = 0; band < EML_BAND_COUNT; ++band)
            {
                if (!(positiveMask & (1U << band))) continue;
                Radio.Config(SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_5, emlBandStartReg(band),
                             kLoraProfile.preamble, (UID[5] & 1U), kFrameSize);
                if (!Radio.ProbeLoraPll(emlSx1276Frf(emlBandEdgeHz(band))) ||
                    !Radio.ProbeLoraPll(emlSx1276Frf(emlBandEndHz(band)))) pllMasks[p] &= (uint8_t)~(1U << band);
            }
        }
        if (!firmwareOptions.bridge_adaptive_fallback)
            pllMasks[p] &= (uint8_t)(1U << firmwareOptions.bridge_default_band);
    }
    if (!bandSearch.begin(phyPlan, runtimeRole, pllMasks, millis(), firmwareOptions.bridge_default_band))
    {
        macMode = MAC_OFF;
        setConnectionState(radioFailed); // normal Wi-Fi recovery/bootloader remains available
        return;
    }

    Radio.RXdoneCallback = &emlRxDone;
    Radio.TXdoneCallback = &emlTxDone;
    emlClockInit(slotClock, hopCount);
    publishBoundary(slotClock.slot, slotClock.hop, micros());
    hwTimer::init(runtimeRole == EML_RESPONDER ? emlTimerTick : nullptr, emlTimerTock);
    setConnectionState(disconnected);
    const uint8_t maximum = firmwareOptions.bridge_power < POWERMGNT::getMaxPower() ?
        firmwareOptions.bridge_power : POWERMGNT::getMaxPower();
    dynamicPower.begin(firmwareOptions.bridge_dynamic_power, POWERMGNT::getMinPower(), maximum,
                       POWERMGNT::currPower(), firmwareOptions.bridge_lq_threshold, millis());
    applyPhy();
}

// P6: coordinated band transitions preserve transport state. Emergency discovery
// is a new session, with the same explicit RAM discard contract as a reconnect.
static void updatePhy()
{
    const uint32_t nowMs = millis();
    if (foregroundBand != macBand)
    {
        foregroundBand = macBand;
        bandController.completed(foregroundBand, nowMs);
        linkQuality.reset(boundarySlot + 1U);
        lastValidMs = nowMs;
    }
    const bool active = transport.active();
    const EmlBoundary_s b = readBoundary();
    if (active)
        roleElection.update(nowMs, true);
    if (active && !wasActive)
    {
        lastValidMs = nowMs;
        missedAckCount = 0;
        watchedSequence = transport.nextTxSequence();
    }
    // A stalled foreground may miss the last opportunity to publish the switch
    // to the ISR. Do not leave an armed negotiation stuck forever on the old band.
    const bool missedHandover = bandController.armed() && bandController.target() != macBand &&
        !bandSwitchArmed && (int32_t)(b.slot - bandController.switchSlot()) >= 0;
    const bool sessionLost = wasActive && !active;
    const bool emergency = missedHandover || (active && nowMs - lastValidMs >= emlTddEmergencyMs(*profile, adaptiveTdd)) || sessionLost;
    wasActive = active;
    if (emergency)
    {
        ++macStats.emergency_searches;
        if (missedHandover) ++macStats.emergency_handover;
        else if (sessionLost) ++macStats.emergency_session_lost;
        else ++macStats.emergency_timeout;
        bandSearch.begin(phyPlan, runtimeRole, pllMasks, nowMs, firmwareOptions.bridge_default_band);
        applyPhy();
        return;
    }
    if (!active)
    {
        const bool held = runtimeRole == EML_COORDINATOR ? transport.state() != EML_HELLO : macMode == MAC_LOCKED;
        if (roleElection.update(nowMs, held))
        {
            applyRole(roleElection.role());
            return;
        }
        if (bandSearch.update(nowMs, held || roleElection.listening())) applyPhy();
        return;
    }
    // A receiver intentionally withholding ACK under UART backpressure is not
    // evidence of RF degradation. Restart the retry watch once PAUSE is lifted.
    if (!transport.pending() || transport.peerFlowControl() == EML_FLOW_CTRL_PAUSE ||
        acknowledgedBytes != transport.stats().acknowledged_bytes)
    { missedAckCount = 0; acknowledgedBytes = transport.stats().acknowledged_bytes; }
    if (!tddController.negotiating())
        bandController.consider(b.slot, nowMs, linkQuality.lq(), linkQuality.samples(), transport.pending() ? missedAckCount : 0);
    if (bandController.armed() && !bandSwitchArmed && bandController.target() != macBand)
    {
        const uint8_t target = bandController.target();
        const uint32_t start = emlBandStartReg(target), sync = emlBandChannelReg(target, EML_SYNC_CHANNEL);
        EML_CRITICAL_ENTER();
        if ((int32_t)(bandController.switchSlot() - boundarySlot) > 0)
        {
            bandSwitchSlot = bandController.switchSlot();
            bandSwitchTarget = target;
            bandSwitchStart = start;
            bandSwitchSync = sync;
            bandSwitchArmed = true;
            ++macStats.band_handovers;
        }
        EML_CRITICAL_EXIT();
    }
}

// TX reports must reach the transport before anything that can replace its
// ticket (frame preparation) or consume an ACK (RX). The ISR pushes a report
// before it publishes that slot's boundary, so draining after reading a
// boundary covers every TX up to that boundary.
static void drainTxReports()
{
    EmlTxTicket_s ticket;
    while (sentRing.pop(&ticket))
    {
        const uint32_t retries = transport.stats().retransmitted;
        if (!transport.sent(ticket, ticket.slot)) ++macStats.sent_unmatched;
        else if (ticket.type == EML_PKT_TYPE_SYNC && ticket.phase >= EML_TDD_PROPOSE && ticket.phase <= EML_TDD_STATUS)
            tddController.sent(ticket.phase, ticket.slot);
        else if (ticket.type == EML_PKT_TYPE_SYNC && ticket.phase >= EML_BAND_PROPOSE && ticket.phase <= EML_BAND_STATUS)
            bandController.sent(ticket.phase);
        else if (transport.stats().retransmitted != retries && missedAckCount < 255) ++missedAckCount;
    }
}

void emlBridgeLoop()
{
    if (macMode == MAC_OFF || connectionState > MODE_STATES)
        return;

    EmlBoundary_s b = readBoundary();
    uint32_t now = nextOwn(b.slot);

    // 1. TX reports first: an ACK in the following RX must find the DATA marked sent
    drainTxReports();

    // 2. Received frames (acquisition can reload the clock)
    EmlRadioDescriptor_s d;
    while (rxMailbox.pop(&d))
    {
        drainTxReports();
        handleRx(d, now);
        b = readBoundary();
        now = nextOwn(b.slot);
    }

    // 3. Timeouts, lock supervision
    transport.tick(now);
    publishGeneration();
    if (runtimeRole == EML_RESPONDER && macMode == MAC_LOCKED && b.slot - lastHeardSlot > researchSlots)
    {
        enterSearch(now, true);
    }
    if (macMode == MAC_LOCKED)
    {
        // one pair of grace so a frame still in flight is not counted as lost
        linkQuality.finalizeExpectedBefore(b.slot - 1U, [](uint32_t slot) { return emlTddOwner(slot, macTddPlan) != runtimeRole; });
    }
    tddController.tick(b.slot, txBacklog(), transport.flowControl(), transport.peerFlowControl(),
                       emlMaxPayloadForFrameSize(kFrameSize), bandController.busy());
    if (tddController.takeDirty())
    {
        EML_CRITICAL_ENTER();
        macTddPlan = tddController.plan();
        EML_CRITICAL_EXIT();
    }
    updatePhy();
    if (dynamicPower.tick(millis()))
        POWERMGNT::setPower((PowerLevels_e)dynamicPower.power());
    if (macMode == MAC_OFF)
        return;
    b = readBoundary();
    now = nextOwn(b.slot);
    // A boundary that passed since step 1 has its TX report queued already
    drainTxReports();

    // 4. Prepare the next own slot: early (stale ACK, covers a lost peer frame) and
    //    once more after the peer frame of the preceding slot was processed
    if (macMode == MAC_LOCKED)
    {
        const int32_t remaining = (int32_t)(b.us + (now - b.slot) * slotUs - micros());
        if (remaining > (int32_t)EML_PREP_MARGIN_US)
        {
            if (prepSlot != now)
            {
                // The previous own slot went out without the post-peer refresh (fresh ACK)
                if (prepSlot != 0xFFFFFFFFUL && !prepLate) ++macStats.late_missed;
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
    inputUartAdapter.setNow(millis());
    emlReadSerial(port, inputUartAdapter, transport.active(), uartStats);
}

void emlBridgeDrainSerial(Stream &port, uint32_t maxBytes)
{
    emlDrainSerial(port, outputUartAdapter, uartStats, maxBytes);
}

bool emlBridgeActive() { return transport.active(); }
EmlRole_e emlBridgeRole() { return runtimeRole; }
size_t emlBridgeQosBuffered() { return 512U - qosIngress.free(); }
uint32_t emlBridgeTddPeerQueued() { return tddController.peerQueued(); }

EmlBridgeDiag_s emlBridgeDiag()
{
    EmlBridgeDiag_s d = diag;
    d.role = runtimeRole;
    d.mac_mode = macMode;
    d.session_state = transport.state();
    d.configuration_id = transportConfig.configuration_id;
    d.local_id = transportConfig.local_id;
    d.peer_id = transportConfig.peer_id;
    d.last_hello_configuration_id = lastHelloConfiguration;
    d.slot_us = slotUs;
    d.hops = hopCount;
    d.pll_mask = pllMasks[0];
    d.search_band = bandSearch.band();
    d.election_phase_ms = roleElection.phaseMs();
    d.sync_channel = sync_channel;
    d.fast_turnaround = EML_FAST_TURNAROUND;
    return d;
}
const EmlTransportStats_s &emlBridgeTransportStats() { return transport.stats(); }
const EmlUartStats_s &emlBridgeUartStats() { return uartStats; }
const EmlMacStats_s &emlBridgeMacStats() { return macStats; }
const EmlTddStats_s &emlBridgeTddStats() { return tddController.stats(); }
const EmlQosStats_s &emlBridgeQosStats() { return qosIngress.stats(); }

EmlLinkStats_s emlBridgeLinkStats()
{
    EmlLinkStats_s stats;
    stats.phy_profile = profile->phy_profile;
    stats.band = macBand;
    stats.tdd_mode = emlTddMode(boundarySlot, macTddPlan);
    stats.arq_window = EML_ARQ_WINDOW;
    stats.in_flight = transport.inFlight();
    stats.pll_band_mask = bandSearch.mask();
    stats.lq = linkQuality.lq();
    stats.per = (uint8_t)(linkQuality.samples() ? 100U - stats.lq : 0U);
    stats.rssi = (int8_t)(rssiAvgX16 / 16);
    stats.snr = (int8_t)(snrAvgX16 / 16);
    stats.snr_valid = snrAvailable;
    stats.peer_lq = peerLinkValid ? peerLink.lq : 0;
    stats.peer_per = peerLinkValid ? peerLink.per : 0;
    stats.peer_rssi = peerLinkValid ? peerLink.rssi : 0;
    stats.peer_snr = peerLinkValid ? peerLink.snr : 0;
    stats.peer_snr_valid = peerLinkValid && peerLink.snr_valid;
#if defined(SX127X_FSK_SUPPORT)
    stats.fsk_fifo_overruns = Radio.FskFifoOverruns;
    stats.fsk_pll_waits = Radio.FskPllWaits;
    stats.fsk_pll_timeouts = Radio.FskPllTimeouts;
#else
    stats.fsk_fifo_overruns = stats.fsk_pll_waits = stats.fsk_pll_timeouts = 0;
#endif
    return stats;
}

#endif // EML_BRIDGE
