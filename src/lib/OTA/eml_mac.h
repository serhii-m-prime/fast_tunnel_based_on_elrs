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
// Slot boundary -> first preamble symbol on air: timer ISR, TX descriptor pop,
// frequency write, 64-byte FIFO burst (~53 us at 10 MHz SPI, P4.1) and SX1276
// FSTX/PLL + PA ramp.
#define EML_TX_START_US        250U
// Last payload symbol -> RX callback timestamp: demodulator latency (~1 symbol),
// DIO0 ISR, IRQ flag read and the FIFO burst read.
#define EML_RX_DONE_US         200U
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

struct EmlMacProfile_s
{
    uint8_t phy_profile;  // EmlPhyProfile_e
    uint8_t frame_size;   // fixed physical length N (implicit header)
    uint8_t sf;           // LoRa spreading factor
    uint8_t cr_denom;     // coding rate 4/cr_denom
    uint8_t preamble;     // preamble symbols
    uint32_t bw_hz;
    uint32_t slot_us;
};

// P4.2: LoRa BW500 / SF6 / CR4/5, implicit header, hardware CRC off (EML CRC16 instead)
constexpr EmlMacProfile_s EML_PROFILE_LORA64 = {EML_PHY_LORA_500_SF6_CR45, EML_FRAME_SIZE_64, 6, 5, 8, 500000UL, 17500UL};
constexpr EmlMacProfile_s EML_PROFILE_LORA32 = {EML_PHY_LORA_500_SF6_CR45, EML_FRAME_SIZE_32, 6, 5, 8, 500000UL, 10400UL};

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

constexpr uint32_t emlProfileToaUs(const EmlMacProfile_s &p)
{
    return emlLoraToaUs(p.sf, p.bw_hz, p.cr_denom, p.preamble, p.frame_size, true, false, false);
}

// Expected RX timestamp relative to the sender's slot boundary
constexpr uint32_t emlRxOffsetUs(const EmlMacProfile_s &p) { return EML_TX_START_US + emlProfileToaUs(p) + EML_RX_DONE_US; }

// Smallest slot that fits TX start + ToA + RX completion + processing + guard
constexpr uint32_t emlMinSlotUs(const EmlMacProfile_s &p) { return emlRxOffsetUs(p) + EML_PROC_US + EML_GUARD_US; }

constexpr bool emlProfileValid(const EmlMacProfile_s &p)
{
    return p.phy_profile == EML_PHY_LORA_500_SF6_CR45 && (p.frame_size == EML_FRAME_SIZE_32 || p.frame_size == EML_FRAME_SIZE_64) &&
        p.sf == 6 && p.bw_hz == 500000UL && p.cr_denom == 5 && p.preamble >= 6 && p.slot_us >= emlMinSlotUs(p) &&
        // responder bias must keep its TX inside the coordinator's odd slot
        EML_RESPONDER_BIAS_US < EML_GUARD_US;
}

static_assert(emlProfileToaUs(EML_PROFILE_LORA64) == 16032U, "LoRa SF6/BW500/CR4/5 64-byte ToA");
static_assert(emlProfileToaUs(EML_PROFILE_LORA32) == 8992U, "LoRa SF6/BW500/CR4/5 32-byte ToA");
static_assert(emlProfileValid(EML_PROFILE_LORA64), "EML 64-byte slot budget");
static_assert(emlProfileValid(EML_PROFILE_LORA32), "EML 32-byte slot budget");

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

    uint8_t record[64];
    size_t n = 0;
    record[n++] = 'E'; record[n++] = 'M'; record[n++] = 'L'; record[n++] = 'C';
    record[n++] = 1; // record layout version
    record[n++] = p.phy_profile; record[n++] = p.frame_size; record[n++] = p.sf;
    record[n++] = p.cr_denom; record[n++] = p.preamble;
    n = emlPutLe(record, n, p.bw_hz, 4);
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
    n = emlPutLe(record, n, EML_TX_START_US + EML_RX_DONE_US, 2);
    ids.configuration = emlFnv1a(uid, 6, emlFnv1a(record, n));
    if (!ids.configuration) ids.configuration = 1;
    return ids;
}

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
