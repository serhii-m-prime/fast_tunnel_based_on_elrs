#pragma once

#include "eml_mac.h"

enum : uint8_t { EML_TDD_SYMMETRIC = 0, EML_TDD_COORDINATOR = 1, EML_TDD_RESPONDER = 2 };
constexpr uint16_t EML_TDD_LEAD_SLOTS = 24;

struct EmlTddLease_s { uint32_t start, end; uint16_t token; uint8_t mode; };
struct EmlTddPlan_s { EmlTddLease_s leases[2]; };

__attribute__((always_inline)) inline uint8_t emlTddMode(uint32_t slot, const EmlTddPlan_s &plan)
{
    for (uint8_t i = 0; i < 2; ++i)
        if ((int32_t)(slot - plan.leases[i].start) >= 0 && (int32_t)(slot - plan.leases[i].end) < 0)
            return plan.leases[i].mode;
    return EML_TDD_SYMMETRIC;
}

__attribute__((always_inline)) inline EmlRole_e emlTddOwner(uint32_t slot, const EmlTddPlan_s &plan)
{
    const uint8_t mode = emlTddMode(slot, plan);
    // Invariant anchors: 4k belongs to C and 4k+3 to R in every mode.
    // A lost COMMIT cannot remove either direction's ACK/phase opportunities.
    if (mode == EML_TDD_COORDINATOR) return (slot & 3U) == 3U ? EML_RESPONDER : EML_COORDINATOR;
    if (mode == EML_TDD_RESPONDER) return (slot & 3U) == 0U ? EML_COORDINATOR : EML_RESPONDER;
    return slot & 1U ? EML_RESPONDER : EML_COORDINATOR;
}

inline uint32_t emlTddNextOwn(uint32_t slot, EmlRole_e role, const EmlTddPlan_s &plan)
{
    do { ++slot; } while (emlTddOwner(slot, plan) != role);
    return slot;
}

inline bool emlTddAnchor(uint32_t slot, EmlRole_e role) { return (slot & 3U) == (role == EML_COORDINATOR ? 0U : 3U); }

inline uint32_t emlTddEmergencyMs(const EmlMacProfile_s &p, bool adaptive)
{
    // Three invariant peer opportunities, including RX completion/phase margin.
    // LoRa64 at 3:1 has 71.2 ms between ACK opportunities, unlike symmetric TDD.
    const uint32_t budget = (12UL * p.slot_us + emlRxOffsetUs(p) + 2UL * EML_GUARD_US + 999U) / 1000U;
    return adaptive && budget > EML_EMERGENCY_MS ? budget : EML_EMERGENCY_MS;
}

struct EmlTddStats_s { uint32_t proposed = 0, committed = 0, expired = 0, timeouts = 0, rejected = 0; };

class EmlTddController
{
    EmlRole_e role_ = EML_COORDINATOR;
    EmlTddPlan_s plan_ = {};
    EmlTddStats_s stats_;
    EmlBandMessage_s pending_ = {};
    uint32_t start_ = 0, lastPeer_ = 0, lastStatus_ = 0, candidateSince_ = 0;
    uint16_t counter_ = 0, peerQueued_ = 0, endedToken_ = 0;
    uint8_t phase_ = 0, peerFlow_ = EML_FLOW_CTRL_PAUSE, candidate_ = EML_TDD_SYMMETRIC;
    bool enabled_ = false, dirty_ = false;

    void arm()
    {
        if (plan_.leases[1].token == pending_.token && plan_.leases[1].start == start_) return;
        plan_.leases[0] = plan_.leases[1];
        plan_.leases[1] = {start_, start_ + EML_TDD_LEASE_SLOTS, pending_.token, pending_.band};
        dirty_ = true; ++stats_.committed;
    }
public:
    void begin(EmlRole_e role, bool enabled, uint32_t slot)
    {
        role_ = role; enabled_ = enabled; plan_ = {}; phase_ = 0; dirty_ = true;
        lastPeer_ = slot - 2U * EML_TDD_LEASE_SLOTS; lastStatus_ = slot - 32U;
        peerQueued_ = 0; peerFlow_ = EML_FLOW_CTRL_PAUSE;
        endedToken_ = 0;
        candidate_ = EML_TDD_SYMMETRIC; candidateSince_ = slot;
    }
    bool negotiating() const { return phase_ != 0; }
    const EmlTddPlan_s &plan() const { return plan_; }
    uint16_t peerQueued() const { return peerQueued_; }
    const EmlTddStats_s &stats() const { return stats_; }
    bool takeDirty() { const bool dirty = dirty_; dirty_ = false; return dirty; }
    void tick(uint32_t slot, size_t queued, uint8_t flow, uint8_t peerFlow, uint8_t payload, bool bandBusy)
    {
        const EmlTddLease_s &latest = plan_.leases[1];
        if (latest.token && latest.token != endedToken_ && (int32_t)(slot - latest.end) >= 0)
        { endedToken_ = latest.token; ++stats_.expired; }
        if (phase_ && (int32_t)(slot - start_) >= 0)
        {
            if (latest.token != pending_.token || latest.start != start_) ++stats_.timeouts;
            phase_ = 0;
        }
        if (!enabled_ || role_ != EML_COORDINATOR || phase_ || bandBusy || slot - lastPeer_ > 64U) return;
        uint8_t desired = EML_TDD_SYMMETRIC;
        if (queued >= 2U * payload && queued > 3UL * peerQueued_ && peerFlow != EML_FLOW_CTRL_PAUSE)
            desired = EML_TDD_COORDINATOR;
        else if (peerQueued_ >= 2U * payload && peerQueued_ > 3UL * queued && flow != EML_FLOW_CTRL_PAUSE && peerFlow_ != EML_FLOW_CTRL_PAUSE)
            desired = EML_TDD_RESPONDER;
        if (desired != candidate_) { candidate_ = desired; candidateSince_ = slot; return; }
        // Confirm demand with a report newer than the observed imbalance. A
        // previously idle peer may have received UART data at the same instant.
        if (desired && (int32_t)(lastPeer_ - candidateSince_) <= 0) return;
        const EmlTddLease_s &last = plan_.leases[1];
        const uint32_t earliest = (slot + EML_TDD_LEAD_SLOTS + 3U) & ~3UL;
        // Negotiate a successor only near expiry. Current leases cannot be
        // shortened unilaterally, and every lease has a hard, shared end.
        if (last.token && (int32_t)(last.end - earliest) > 0) return;
        if (!desired && emlTddMode(slot, plan_) == EML_TDD_SYMMETRIC) return;
        start_ = earliest;
        if (++counter_ == 0) ++counter_;
        pending_ = {EML_TDD_PROPOSE, desired, EML_TDD_LEASE_SLOTS, (uint16_t)start_, counter_};
        phase_ = EML_TDD_PROPOSE; ++stats_.proposed;
    }
    bool command(uint32_t slot, size_t queued, uint8_t flow, bool pending, EmlBandMessage_s *out) const
    {
        if (!enabled_ || !emlTddAnchor(slot, role_) || (slot & 7U) != (role_ == EML_COORDINATOR ? 0U : 3U)) return false;
        if (phase_ && (int32_t)(start_ - slot) >= 2)
        { *out = pending_; out->op = phase_; return true; }
        if (role_ == EML_RESPONDER && slot - lastStatus_ >= 32U)
        {
            *out = {EML_TDD_STATUS, EML_TDD_SYMMETRIC, (uint8_t)(flow | (pending ? 4U : 0U)),
                    (uint16_t)slot, (uint16_t)(queued > 65535U ? 65535U : queued)};
            return true;
        }
        return false;
    }
    bool receive(const EmlBandMessage_s &m, uint32_t slot)
    {
        if (!enabled_ || m.op < EML_TDD_PROPOSE || !emlBandMessageValid(m)) { ++stats_.rejected; return false; }
        const EmlTddLease_s &armed = plan_.leases[1];
        if (role_ == EML_RESPONDER && m.op == EML_TDD_COMMIT && m.token == armed.token &&
            m.switch_slot == (uint16_t)armed.start && m.band == armed.mode) return true;
        if (m.op == EML_TDD_STATUS && role_ == EML_COORDINATOR && m.switch_slot == (uint16_t)slot)
        { peerQueued_ = m.token; peerFlow_ = m.mask & 3U; lastPeer_ = slot; return true; }
        const int16_t lead = (int16_t)(m.switch_slot - (uint16_t)slot);
        if (lead < 2 || lead > (int16_t)(EML_TDD_LEAD_SLOTS + 3U)) { ++stats_.rejected; return false; }
        if (role_ == EML_RESPONDER && m.op == EML_TDD_PROPOSE)
        {
            if (phase_ && (pending_.token != m.token || pending_.switch_slot != m.switch_slot || pending_.band != m.band))
            { ++stats_.rejected; return false; }
            const EmlTddLease_s &last = plan_.leases[1];
            if (last.token && (int32_t)(slot + (uint16_t)lead - last.end) < 0) { ++stats_.rejected; return false; }
            pending_ = m; start_ = slot + (uint16_t)lead; phase_ = EML_TDD_ACK;
            return true;
        }
        if (!phase_ || m.token != pending_.token || m.switch_slot != pending_.switch_slot || m.band != pending_.band)
        { ++stats_.rejected; return false; }
        if (role_ == EML_COORDINATOR && m.op == EML_TDD_ACK) { phase_ = EML_TDD_COMMIT; return true; }
        if (role_ == EML_RESPONDER && m.op == EML_TDD_COMMIT) { arm(); phase_ = 0; return true; }
        ++stats_.rejected; return false;
    }
    void sent(uint8_t op, uint32_t slot)
    {
        if (op == EML_TDD_STATUS) lastStatus_ = slot;
        if (op == EML_TDD_COMMIT && phase_ == EML_TDD_COMMIT && (int32_t)(start_ - slot) >= 2) arm();
    }
};
