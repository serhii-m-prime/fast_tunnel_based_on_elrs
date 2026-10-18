#ifndef H_EML_MAC
#define H_EML_MAC

#include "eml_transport.h"

/**
 * EML bridge MAC: PHY profiles, LoRa time-on-air, slot budget, the shared
 * slot/hop clock and responder acquisition (P4.2-P4.5). Pure logic without
 * Arduino, radio or timer calls so it is native-testable; the firmware glue
 * lives in src/src/rxtx_eml_bridge.cpp. Functions used from ISRs are forced
 * inline so they land in the IRAM caller.
 *
 * Symmetric TDD (P4.4): slots alternate coordinator (even) -> responder (odd).
 * This is NOT the stock ELRS telemetry ratio: there is no RC/telemetry packet
 * semantic, both directions get exactly one slot per pair. A pair shares one
 * FHSS hop; the hop index advances at every even slot whether or not anything
 * was received (EML_PROTOCOL.md §6.2).
 */

// ---- Timing budget (microseconds) -------------------------------------------------
// Slot boundary -> first preamble symbol on air (LoRa): timer ISR, TX descriptor pop,
// frequency write, 64-byte FIFO burst (~53 us at 10 MHz SPI, P4.1) and SX1276
// FSTX/PLL + PA ramp. FSK profiles carry their own value.
#define EML_TX_START_US        250U
// Last payload symbol -> RX callback timestamp (LoRa): demodulator latency (~1 symbol),
// DIO0 ISR, IRQ flag read and the FIFO burst read.
#define EML_RX_DONE_US         200U
// FSK: no LoRa symbol latency; PayloadReady follows the last bit, then ISR + FIFO read
#define EML_FSK_TX_START_US    200U
#define EML_FSK_RX_DONE_US     150U
// Foreground receive() + prepare() + mailbox push before the next own boundary.
#define EML_PROC_US            600U
// Residual responder alignment error and timer/ISR jitter.
#define EML_GUARD_US           300U
// The responder places its boundaries this much ahead of the coordinator so it is
// already receiving when the coordinator preamble starts.
#define EML_RESPONDER_BIAS_US  100U
// Foreground prepares a frame only while at least this much time remains before
// the target boundary; prepare+push must finish inside it (no ISR race).
#define EML_PREP_MARGIN_US     400U
// Minimum lead between acquisition processing and the first locked boundary.
#define EML_LOCK_LEAD_US       600U

// Session timing in milliseconds; converted to slots per profile.
#define EML_HANDSHAKE_TIMEOUT_MS 700U
#define EML_LINK_TIMEOUT_MS      1500U
#define EML_RETRY_SLOTS          2U   // Sync repeated in every own slot
#define EML_MAX_FRAME_AGE_SLOTS  3U   // descriptor may be processed up to one pair late
#define EML_QUIET_SLOTS          5U

#define EML_FSK_SYNC_BYTES       4U
#define EML_FSK_FIFO_BYTES       64U  // SX1276 FSK FIFO (LoRa has 256)

enum EmlFskShaping_e : uint8_t { EML_FSK_SHAPING_NONE = 0, EML_FSK_SHAPING_BT_1_0 = 1, EML_FSK_SHAPING_BT_0_5 = 2, EML_FSK_SHAPING_BT_0_3 = 3 };

struct EmlMacProfile_s
{
    uint8_t phy_profile;  // EmlPhyProfile_e
    uint8_t frame_size;   // fixed physical length N (implicit header / fixed-length packet)
    uint8_t preamble;     // LoRa: symbols; FSK: bytes
    // LoRa
    uint8_t sf;           // spreading factor
    uint8_t cr_denom;     // coding rate 4/cr_denom
    uint32_t bw_hz;
    // FSK/GFSK
    uint32_t bitrate;
    uint32_t fdev_hz;
    uint32_t rxbw_hz;     // single-sideband RX/AFC filter
    uint8_t shaping;      // EmlFskShaping_e
    bool experimental;    // outside the analysed profile; never a default (P5.7)
    // timing
    uint16_t tx_start_us;
    uint16_t rx_done_us;
    uint32_t slot_us;
};

// P4.2: LoRa BW500 / SF6 / CR4/5, implicit header, hardware CRC off (EML CRC16 instead)
constexpr EmlMacProfile_s EML_PROFILE_LORA64 = {EML_PHY_LORA_500_SF6_CR45, EML_FRAME_SIZE_64, 8, 6, 5, 500000UL,
                                                0, 0, 0, 0, false, EML_TX_START_US, EML_RX_DONE_US, 17500UL};
constexpr EmlMacProfile_s EML_PROFILE_LORA32 = {EML_PHY_LORA_500_SF6_CR45, EML_FRAME_SIZE_32, 8, 6, 5, 500000UL,
                                                0, 0, 0, 0, false, EML_TX_START_US, EML_RX_DONE_US, 10400UL};
// P5.2: GFSK 250 kbit/s, BT 1.0, Fdev 90 kHz (h = 0.72), RxBw = AfcBw = 250 kHz SSB.
// Fdev + BR/2 = 215 kHz <= 250 kHz filter leaves +-35 kHz for crystal offset before AFC;
// preamble 10 B (320 us) covers detection (2 B), AFC/AGC and responder alignment.
constexpr EmlMacProfile_s EML_PROFILE_FSK64 = {EML_PHY_FSK_250, EML_FRAME_SIZE_64, 10, 0, 0, 0,
                                               250000UL, 90000UL, 250000UL, EML_FSK_SHAPING_BT_1_0, false,
                                               EML_FSK_TX_START_US, EML_FSK_RX_DONE_US, 3800UL};
constexpr EmlMacProfile_s EML_PROFILE_FSK32 = {EML_PHY_FSK_250, EML_FRAME_SIZE_32, 10, 0, 0, 0,
                                               250000UL, 90000UL, 250000UL, EML_FSK_SHAPING_BT_1_0, false,
                                               EML_FSK_TX_START_US, EML_FSK_RX_DONE_US, 2800UL};
// P5.7 experiment: 300 kbit/s is the SX1276 FSK maximum. h = 0.5 needs Fdev >= 75 kHz,
// leaving only +-25 kHz offset margin in the 250 kHz filter. It shares the FSK wire PHY
// ID, but it is a different configuration record (different configuration_id) and is
// a build-time opt-in only.
constexpr EmlMacProfile_s EML_PROFILE_FSK64_300K = {EML_PHY_FSK_250, EML_FRAME_SIZE_64, 10, 0, 0, 0,
                                                    300000UL, 75000UL, 250000UL, EML_FSK_SHAPING_BT_1_0, true,
                                                    EML_FSK_TX_START_US, EML_FSK_RX_DONE_US, 3500UL};

// ---- LoRa time on air (Semtech SX1276 datasheet §4.1.1.7 / AN1200.13) ----------------
constexpr uint32_t emlCeilDivPositive(int32_t a, int32_t b) { return a <= 0 ? 0U : (uint32_t)((a + b - 1) / b); }

constexpr uint32_t emlLoraPayloadSymbols(uint8_t sf, uint8_t crDenom, uint16_t length, bool implicitHeader, bool crc, bool lowDataRateOpt)
{
    return 8U + emlCeilDivPositive(8 * (int32_t)length - 4 * sf + 28 + (crc ? 16 : 0) - (implicitHeader ? 20 : 0),
                                   4 * (sf - (lowDataRateOpt ? 2 : 0))) * crDenom;
}

// ((preamble + 4.25) + payloadSymbols) * 2^SF / BW, rounded up to whole microseconds
constexpr uint32_t emlLoraToaUs(uint8_t sf, uint32_t bwHz, uint8_t crDenom, uint8_t preamble, uint16_t length,
                                bool implicitHeader, bool crc, bool lowDataRateOpt)
{
    return (uint32_t)(((uint64_t)(1UL << sf) * 1000000ULL *
                       (4ULL * preamble + 17ULL + 4ULL * emlLoraPayloadSymbols(sf, crDenom, length, implicitHeader, crc, lowDataRateOpt))
                       + 4ULL * bwHz - 1ULL) / (4ULL * bwHz));
}

// ---- FSK time on air (P5.4/P5.5) ----------------------------------------------------
// Fixed-length packet: preamble + sync word + N payload bytes; no length byte, no
// address, no hardware CRC, so the 64-byte FIFO holds exactly the frame.
constexpr uint32_t emlFskToaUs(uint32_t bitrate, uint8_t preambleBytes, uint8_t syncBytes, uint16_t length, uint8_t crcBytes)
{
    return (uint32_t)((8ULL * (preambleBytes + syncBytes + length + crcBytes) * 1000000ULL + bitrate - 1ULL) / bitrate);
}

constexpr bool emlIsFsk(const EmlMacProfile_s &p) { return p.phy_profile == EML_PHY_FSK_250; }

constexpr uint32_t emlProfileToaUs(const EmlMacProfile_s &p)
{
    return emlIsFsk(p) ? emlFskToaUs(p.bitrate, p.preamble, EML_FSK_SYNC_BYTES, p.frame_size, 0)
                       : emlLoraToaUs(p.sf, p.bw_hz, p.cr_denom, p.preamble, p.frame_size, true, false, false);
}

// Expected RX timestamp relative to the sender's slot boundary
constexpr uint32_t emlRxOffsetUs(const EmlMacProfile_s &p) { return p.tx_start_us + emlProfileToaUs(p) + p.rx_done_us; }

// Smallest slot that fits TX start + ToA + RX completion + processing + guard
constexpr uint32_t emlMinSlotUs(const EmlMacProfile_s &p) { return emlRxOffsetUs(p) + EML_PROC_US + EML_GUARD_US; }

// How late the responder may open RX after the coordinator boundary and still catch the
// frame: LoRa needs the last ~3 preamble symbols, FSK the 2-byte detector plus ~2 bytes
// for AFC/AGC before the sync word.
constexpr uint32_t emlRxLateToleranceUs(const EmlMacProfile_s &p)
{
    return p.tx_start_us + (emlIsFsk(p) ? (uint32_t)((8ULL * (p.preamble - 4U) * 1000000ULL) / p.bitrate)
                                        : (uint32_t)(((uint64_t)(p.preamble - 3U) << p.sf) * 1000000ULL / p.bw_hz));
}

// SX1276 FSK limits (datasheet §2.5.3, Table 7): BR <= 300 kbit/s, Fdev <= 200 kHz,
// Fdev + BR/2 <= 250 kHz, 0.5 <= h = 2*Fdev/BR <= 10, occupied Fdev + BR/2 within the
// 250 kHz maximum SSB filter, preamble long enough for detection + AFC, payload in FIFO.
constexpr bool emlFskWithinDatasheet(const EmlMacProfile_s &p)
{
    return p.bitrate >= 1200UL && p.bitrate <= 300000UL && p.fdev_hz >= 600UL && p.fdev_hz <= 200000UL &&
        p.fdev_hz + p.bitrate / 2UL <= 250000UL && 4UL * p.fdev_hz >= p.bitrate && p.fdev_hz <= 5UL * p.bitrate &&
        p.rxbw_hz <= 250000UL && p.fdev_hz + p.bitrate / 2UL <= p.rxbw_hz &&
        p.preamble >= 4 && p.frame_size <= EML_FSK_FIFO_BYTES && p.shaping <= EML_FSK_SHAPING_BT_0_3;
}

constexpr bool emlProfileValid(const EmlMacProfile_s &p)
{
    return (p.frame_size == EML_FRAME_SIZE_32 || p.frame_size == EML_FRAME_SIZE_64) &&
        p.slot_us >= emlMinSlotUs(p) &&
        // responder bias must keep its TX inside the coordinator's odd slot
        EML_RESPONDER_BIAS_US < EML_GUARD_US &&
        (emlIsFsk(p) ? emlFskWithinDatasheet(p) && (p.bitrate == 250000UL || p.experimental)
                     : p.phy_profile == EML_PHY_LORA_500_SF6_CR45 && p.sf == 6 && p.bw_hz == 500000UL &&
                           p.cr_denom == 5 && p.preamble >= 6);
}

static_assert(emlProfileToaUs(EML_PROFILE_LORA64) == 16032U, "LoRa SF6/BW500/CR4/5 64-byte ToA");
static_assert(emlProfileToaUs(EML_PROFILE_LORA32) == 8992U, "LoRa SF6/BW500/CR4/5 32-byte ToA");
static_assert(emlProfileToaUs(EML_PROFILE_FSK64) == 2496U, "FSK250 64-byte ToA");
static_assert(emlProfileToaUs(EML_PROFILE_FSK32) == 1472U, "FSK250 32-byte ToA");
static_assert(emlProfileValid(EML_PROFILE_LORA64), "EML LoRa 64-byte slot budget");
static_assert(emlProfileValid(EML_PROFILE_LORA32), "EML LoRa 32-byte slot budget");
static_assert(emlProfileValid(EML_PROFILE_FSK64), "EML FSK 64-byte slot budget");
static_assert(emlProfileValid(EML_PROFILE_FSK32), "EML FSK 32-byte slot budget");
static_assert(emlProfileValid(EML_PROFILE_FSK64_300K), "EML FSK 300k experiment budget");

constexpr uint16_t emlMsToSlots(uint32_t ms, uint32_t slotUs) { return (uint16_t)((ms * 1000UL + slotUs - 1UL) / slotUs); }

// ---- Shared slot/hop clock (ISR-owned on both sides) --------------------------------
struct EmlSlotClock_s
{
    uint32_t slot;
    uint16_t hop;
    uint16_t hop_count;
};

// Coordinator start: the first advance yields slot 0 on hop 0
__attribute__((always_inline)) inline void emlClockInit(EmlSlotClock_s &c, uint16_t hopCount)
{
    c.slot = 0xFFFFFFFFUL;
    c.hop_count = hopCount ? hopCount : 1;
    c.hop = (uint16_t)(c.hop_count - 1U);
}

__attribute__((always_inline)) inline void emlClockAdvance(EmlSlotClock_s &c)
{
    ++c.slot;
    if (!(c.slot & 1UL) && ++c.hop >= c.hop_count) c.hop = 0;
}

__attribute__((always_inline)) inline bool emlOwnSlot(uint32_t slot, EmlRole_e role) { return (slot & 1UL) == (uint32_t)role; }

// Hop used `slotsAfter` slots after an even slot on `evenHop`
inline uint16_t emlHopAfter(uint16_t evenHop, uint32_t slotsAfter, uint16_t hopCount)
{
    return (uint16_t)((evenHop + slotsAfter / 2UL) % hopCount);
}

// The transport clock: the next own TX slot strictly after `cur`. Monotonic in
// `cur`, so every transport call sees non-decreasing time.
inline uint32_t emlNextOwnSlot(uint32_t cur, EmlRole_e role)
{
    uint32_t next = cur + 1UL;
    if (!emlOwnSlot(next, role)) ++next;
    return next;
}

// ---- Responder acquisition (P4.5) -----------------------------------------------------
struct EmlAcquire_s
{
    uint32_t hello_slot;      // extended slot of the received HELLO (even)
    uint32_t first_slot;      // slot started by the first local timer boundary
    uint32_t first_boundary_us;
    uint32_t clock_slot;      // clock state to load before starting the timer
    uint16_t clock_hop;
};

/**
 * Map a HELLO heard on a rendezvous slot onto the local clock. The extended slot
 * only moves forward (16-bit wire counter, §7.1), and the first local boundary is
 * the earliest one at least EML_LOCK_LEAD_US away. A descriptor processed more
 * than EML_MAX_FRAME_AGE_SLOTS late is refused.
 */
inline bool emlAcquire(uint32_t localSlot, uint16_t helloSlot16, uint16_t helloHop, uint16_t hopCount,
                       uint32_t rxUs, uint32_t nowUs, const EmlMacProfile_s &p, EmlAcquire_s *out)
{
    if (!out || (helloSlot16 & 1U) || !hopCount || helloHop >= hopCount || !p.slot_us) return false;
    const uint32_t ext = localSlot + (uint16_t)(helloSlot16 - (uint16_t)localSlot);
    uint32_t boundary = rxUs - emlRxOffsetUs(p) - EML_RESPONDER_BIAS_US + p.slot_us;
    uint32_t k = 1;
    while ((int32_t)(boundary - nowUs) < (int32_t)EML_LOCK_LEAD_US)
    {
        if (++k > EML_MAX_FRAME_AGE_SLOTS) return false;
        boundary += p.slot_us;
    }
    out->hello_slot = ext;
    out->first_slot = ext + k;
    out->first_boundary_us = boundary;
    out->clock_slot = ext + k - 1UL;
    out->clock_hop = emlHopAfter(helloHop, k - 1UL, hopCount);
    return true;
}

// Responder timer correction for a coordinator frame received in slot `slotStartUs`:
// positive means the local boundary is too early and the timer must be delayed.
inline int32_t emlPhaseCorrection(uint32_t rxUs, uint32_t slotStartUs, const EmlMacProfile_s &p)
{
    return (int32_t)(rxUs - slotStartUs) - (int32_t)(emlRxOffsetUs(p) + EML_RESPONDER_BIAS_US);
}

// ---- Link identity and configuration record ------------------------------------------
struct EmlLinkIds_s
{
    uint32_t coordinator;
    uint32_t responder;
    uint32_t configuration;
};

struct EmlFhssRecord_s
{
    uint32_t freq_start;
    uint32_t freq_stop;
    uint32_t freq_count;
    uint32_t seed;
    uint16_t hop_count;
};

inline uint32_t emlFnv1a(const uint8_t *data, size_t length, uint32_t hash = 2166136261UL)
{
    for (size_t i = 0; i < length; ++i) { hash ^= data[i]; hash *= 16777619UL; }
    return hash;
}

inline size_t emlPutLe(uint8_t *out, size_t at, uint32_t value, uint8_t bytes)
{
    for (uint8_t i = 0; i < bytes; ++i) out[at + i] = (uint8_t)(value >> (8U * i));
    return at + bytes;
}

inline EmlTransportConfig_s emlMacTransportConfig(EmlRole_e role, const EmlLinkIds_s &ids, const EmlMacProfile_s &p, uint16_t hopCount)
{
    EmlTransportConfig_s c = {};
    c.role = role;
    c.local_id = role == EML_COORDINATOR ? ids.coordinator : ids.responder;
    c.peer_id = role == EML_COORDINATOR ? ids.responder : ids.coordinator;
    c.configuration_id = ids.configuration;
    c.phy_profile = p.phy_profile;
    c.frame_size = p.frame_size;
    c.hop_count = hopCount;
    c.retry_slots = EML_RETRY_SLOTS;
    c.handshake_timeout_slots = emlMsToSlots(EML_HANDSHAKE_TIMEOUT_MS, p.slot_us);
    c.link_timeout_slots = emlMsToSlots(EML_LINK_TIMEOUT_MS, p.slot_us);
    c.quiet_slots = EML_QUIET_SLOTS;
    c.max_frame_age_slots = EML_MAX_FRAME_AGE_SLOTS;
    return c;
}

/**
 * IDs shared by a bound pair: both sides derive them from the binding UID, so a
 * differently bound pair never completes the handshake. configuration_id hashes the
 * exact record (PHY, N, slot, FHSS table inputs, timing); any change yields a new ID.
 */
inline EmlLinkIds_s emlDeriveLinkIds(const uint8_t uid[6], const EmlMacProfile_s &p, const EmlFhssRecord_s &f)
{
    static const uint8_t tagT[4] = {'E', 'M', 'L', 'T'}, tagR[4] = {'E', 'M', 'L', 'R'};
    EmlLinkIds_s ids;
    ids.coordinator = emlFnv1a(uid, 6, emlFnv1a(tagT, 4));
    ids.responder = emlFnv1a(uid, 6, emlFnv1a(tagR, 4));
    if (!ids.coordinator) ids.coordinator = 1;
    if (!ids.responder || ids.responder == ids.coordinator) ids.responder = ids.coordinator ^ 0x5A5A5A5AUL;

    uint8_t record[96];
    size_t n = 0;
    record[n++] = 'E'; record[n++] = 'M'; record[n++] = 'L'; record[n++] = 'C';
    record[n++] = 2; // record layout version (2: FSK fields)
    record[n++] = p.phy_profile; record[n++] = p.frame_size; record[n++] = p.sf;
    record[n++] = p.cr_denom; record[n++] = p.preamble;
    n = emlPutLe(record, n, p.bw_hz, 4);
    n = emlPutLe(record, n, p.bitrate, 4);
    n = emlPutLe(record, n, p.fdev_hz, 4);
    n = emlPutLe(record, n, p.rxbw_hz, 4);
    record[n++] = p.shaping; record[n++] = EML_FSK_SYNC_BYTES; record[n++] = p.experimental ? 1 : 0;
    n = emlPutLe(record, n, p.slot_us, 4);
    n = emlPutLe(record, n, f.freq_start, 4);
    n = emlPutLe(record, n, f.freq_stop, 4);
    n = emlPutLe(record, n, f.freq_count, 4);
    n = emlPutLe(record, n, f.seed, 4);
    n = emlPutLe(record, n, f.hop_count, 2);
    n = emlPutLe(record, n, EML_RETRY_SLOTS, 1);
    n = emlPutLe(record, n, EML_HANDSHAKE_TIMEOUT_MS, 2);
    n = emlPutLe(record, n, EML_LINK_TIMEOUT_MS, 2);
    n = emlPutLe(record, n, EML_QUIET_SLOTS, 1);
    n = emlPutLe(record, n, EML_MAX_FRAME_AGE_SLOTS, 1);
    n = emlPutLe(record, n, EML_RESPONDER_BIAS_US, 2);
    n = emlPutLe(record, n, p.tx_start_us, 2);
    n = emlPutLe(record, n, p.rx_done_us, 2);
    ids.configuration = emlFnv1a(uid, 6, emlFnv1a(record, n));
    if (!ids.configuration) ids.configuration = 1;
    return ids;
}

// FSK sync word shared by a bound pair (4 bytes, never 0x00 per the SX1276 datasheet)
inline void emlFskSyncWord(const uint8_t uid[6], uint8_t out[EML_FSK_SYNC_BYTES])
{
    static const uint8_t tag[4] = {'E', 'M', 'L', 'S'};
    uint32_t h = emlFnv1a(uid, 6, emlFnv1a(tag, 4));
    for (uint8_t i = 0; i < EML_FSK_SYNC_BYTES; ++i)
    {
        uint8_t b = (uint8_t)(h >> (8U * i));
        out[i] = b ? b : (uint8_t)(0x5AU + i);
    }
}

// ---- PHY selection (P5.5) --------------------------------------------------------------
/**
 * Both sides cycle through the same ordered PHY list (index 0 preferred) while no link
 * exists, so a pair always meets on the first PHY that works:
 *  - coordinator dwell_i >= 2 rendezvous periods of PHY i (one HELLO lost is tolerated),
 *    and it never leaves a PHY with a handshake in progress or a session open;
 *  - responder dwell_i >= coordinator cycle + coordinator dwell_i, so a whole coordinator
 *    window of PHY i falls inside every responder window of PHY i.
 * A lost session (coordinator) or lost lock (responder) restarts at the preferred PHY,
 * which is how the link returns from the LoRa fallback to FSK.
 */
#define EML_PHY_MAX 2U

struct EmlPhyPlan_s
{
    uint8_t count;
    EmlMacProfile_s profiles[EML_PHY_MAX];
    uint32_t coordinator_dwell_ms[EML_PHY_MAX];
    uint32_t responder_dwell_ms[EML_PHY_MAX];
};

// The ELRS sequence visits the sync channel once per freqCount hops, one hop per slot pair
inline uint32_t emlRendezvousPeriodUs(const EmlMacProfile_s &p, uint32_t freqCount)
{
    return 2UL * freqCount * p.slot_us;
}

inline EmlPhyPlan_s emlPhyPlan(const EmlMacProfile_s *profiles, uint8_t count, uint32_t freqCount)
{
    EmlPhyPlan_s plan = {};
    plan.count = count > EML_PHY_MAX ? (uint8_t)EML_PHY_MAX : count;
    uint32_t cycle = 0;
    for (uint8_t i = 0; i < plan.count; ++i)
    {
        plan.profiles[i] = profiles[i];
        // two rendezvous plus a few slots for the OFFER that stops the coordinator cycling
        plan.coordinator_dwell_ms[i] = (2UL * emlRendezvousPeriodUs(profiles[i], freqCount) + 8UL * profiles[i].slot_us + 999UL) / 1000UL;
        cycle += plan.coordinator_dwell_ms[i];
    }
    for (uint8_t i = 0; i < plan.count; ++i)
        plan.responder_dwell_ms[i] = cycle + plan.coordinator_dwell_ms[i];
    return plan;
}

class EmlPhySelector
{
    const EmlPhyPlan_s *plan_ = nullptr;
    EmlRole_e role_ = EML_COORDINATOR;
    uint8_t index_ = 0;
    uint32_t since_ = 0;

public:
    void begin(const EmlPhyPlan_s &plan, EmlRole_e role, uint32_t nowMs) { plan_ = &plan; role_ = role; index_ = 0; since_ = nowMs; }
    uint8_t index() const { return index_; }
    const EmlMacProfile_s &profile() const { return plan_->profiles[index_]; }
    uint32_t dwellMs() const { return role_ == EML_COORDINATOR ? plan_->coordinator_dwell_ms[index_] : plan_->responder_dwell_ms[index_]; }
    /**
     * linked: session open (coordinator) / timer locked (responder) - stay.
     * mayLeave: coordinator only in HELLO, responder only while searching.
     * lost: the link of this side just ended. Returns true when the PHY must change.
     */
    bool update(uint32_t nowMs, bool linked, bool mayLeave, bool lost)
    {
        if (!plan_ || plan_->count < 2) return false;
        if (lost)
        {
            since_ = nowMs;
            if (index_ == 0) return false;
            index_ = 0;
            return true;
        }
        if (linked) { since_ = nowMs; return false; }
        if (!mayLeave || nowMs - since_ < dwellMs()) return false;
        index_ = (uint8_t)((index_ + 1U) % plan_->count);
        since_ = nowMs;
        return true;
    }
};

// ---- Link quality: received peer frames over the last 100 slot pairs (P5.5) ------------
// PHY independent (FSK has no SNR): LQ = CRC-valid peer frames / expected, PER = 100 - LQ.
class EmlLinkQuality
{
    static const uint8_t WINDOW = 100;
    uint32_t ring_[(WINDOW + 31) / 32] = {};
    uint8_t pos_ = 0, count_ = 0, good_ = 0;
    uint32_t next_ = 0, pending_ = 0;

    void push(bool ok)
    {
        const uint32_t mask = 1UL << (pos_ & 31U);
        uint32_t &word = ring_[pos_ >> 5];
        if (count_ == WINDOW) { if (word & mask) --good_; }
        else ++count_;
        if (ok) { word |= mask; ++good_; } else word &= ~mask;
        if (++pos_ == WINDOW) pos_ = 0;
    }

public:
    void reset(uint32_t pair)
    {
        for (uint32_t &w : ring_) w = 0;
        pos_ = count_ = good_ = 0;
        next_ = pair; pending_ = 0;
    }
    // A CRC-valid peer frame of slot pair `pair` arrived (late or duplicate calls are ignored)
    void received(uint32_t pair)
    {
        const uint32_t d = pair - next_;
        if (d < 32U) pending_ |= 1UL << d;
    }
    // Every pair before `pair` is complete
    void finalizeBefore(uint32_t pair)
    {
        uint32_t n = pair - next_;
        if ((int32_t)n < 0)
        {
            // just behind (acquisition) is a no-op; far behind means the counter wrapped
            if ((int32_t)n < -(int32_t)WINDOW) reset(pair);
            return;
        }
        if (n > 2U * WINDOW) { reset(pair); for (uint8_t i = 0; i < WINDOW; ++i) push(false); return; }
        while (n--) { push(pending_ & 1UL); pending_ >>= 1; ++next_; }
    }
    uint8_t lq() const { return count_ ? (uint8_t)((good_ * 100U) / count_) : 0; }
    uint8_t samples() const { return count_; }
};

// ---- ISR -> foreground "TX started" notifications ------------------------------------
// SPSC like EmlMailbox: the timer ISR pushes the exact ticket it transmitted, the
// foreground pops it and calls transport.sent() BEFORE handling any RX descriptor.
template <uint32_t Capacity>
class EmlTicketRing
{
    static_assert(Capacity && !(Capacity & (Capacity - 1U)), "power of two");
    EmlTxTicket_s entries_[Capacity] = {};
    alignas(4) uint32_t head_ = 0, tail_ = 0;

public:
    __attribute__((always_inline)) inline bool push(const EmlTxTicket_s &t)
    {
        const uint32_t head = __atomic_load_n(&head_, __ATOMIC_RELAXED);
        if (head - __atomic_load_n(&tail_, __ATOMIC_ACQUIRE) >= Capacity) return false;
        EmlTxTicket_s &e = entries_[head & (Capacity - 1U)];
        e.generation = t.generation; e.slot = t.slot; e.serial = t.serial; e.type = t.type; e.phase = t.phase;
        __atomic_store_n(&head_, head + 1U, __ATOMIC_RELEASE);
        return true;
    }
    __attribute__((always_inline)) inline bool pop(EmlTxTicket_s *out)
    {
        const uint32_t tail = __atomic_load_n(&tail_, __ATOMIC_RELAXED);
        if (tail == __atomic_load_n(&head_, __ATOMIC_ACQUIRE)) return false;
        const EmlTxTicket_s &e = entries_[tail & (Capacity - 1U)];
        out->generation = e.generation; out->slot = e.slot; out->serial = e.serial; out->type = e.type; out->phase = e.phase;
        __atomic_store_n(&tail_, tail + 1U, __ATOMIC_RELEASE);
        return true;
    }
};

#endif
