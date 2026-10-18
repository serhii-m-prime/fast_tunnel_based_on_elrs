#pragma once

#include "eml_mac.h"
#include "eml_fhss.h"

constexpr uint32_t EML_EMERGENCY_MS = 150;
constexpr uint32_t EML_BAND_HOLD_MS = 2000;
constexpr uint32_t EML_BAND_PROMOTE_MS = 30000;
constexpr uint16_t EML_BAND_LEAD_SLOTS = 24;

// Foreground-only handover policy. A proposal and its ACK are retried until the
// coordinator transmits COMMIT. The responder arms only on COMMIT reception.
// Finite exchanges cannot guarantee agreement under unbounded loss: a lost set
// of COMMITs is repaired by the emergency search, never by guessing a hop.
class EmlBandController
{
    EmlRole_e role_ = EML_COORDINATOR;
    uint8_t band_ = 0, mask_ = 15, peerMask_ = 15, phase_ = 0;
    uint8_t peerLq_ = 100, peerMissed_ = 0;
    uint16_t token_ = 0;
    EmlBandMessage_s pending_ = {};
    uint32_t switch_ = 0, since_ = 0, healthy_ = 0, peerStatusMs_ = 0;
    bool armed_ = false, healthyValid_ = false;

public:
    void begin(EmlRole_e role, uint8_t band, uint8_t mask, uint32_t nowMs)
    {
        role_ = role; band_ = band; mask_ = mask; peerMask_ = 15;
        phase_ = 0; armed_ = healthyValid_ = false; since_ = nowMs;
        peerLq_ = 100; peerMissed_ = 0; peerStatusMs_ = nowMs;
    }
    bool busy() const { return phase_ != 0; }
    bool armed() const { return armed_; }
    uint32_t switchSlot() const { return switch_; }
    uint8_t target() const { return pending_.band; }
    void completed(uint8_t band, uint32_t nowMs)
    {
        band_ = band; phase_ = 0; armed_ = healthyValid_ = false; since_ = nowMs;
        peerLq_ = 100; peerMissed_ = 0;
    }
    void consider(uint32_t slot, uint32_t nowMs, uint8_t lq, uint8_t samples, uint8_t missedAck)
    {
        if (phase_ && (int32_t)(slot - switch_) >= 0 && !armed_) phase_ = 0;
        // One DATA sent and awaiting its first ACK is ordinary traffic, not a
        // lost ACK. Only a retransmission breaks the promotion hold.
        const bool good = samples >= 100 && lq >= 90 && missedAck <= 1 &&
            (nowMs - peerStatusMs_ > 2000 || (peerLq_ >= 90 && peerMissed_ <= 1));
        if (!good) healthyValid_ = false;
        else if (!healthyValid_) { healthy_ = nowMs; healthyValid_ = true; }
        if (role_ != EML_COORDINATOR || busy() || nowMs - since_ < EML_BAND_HOLD_MS) return;
        const bool bad = (samples >= 10 && lq < 60) || missedAck >= 4 ||
            (nowMs - peerStatusMs_ <= 2000 && (peerLq_ < 60 || peerMissed_ >= 4));
        const uint8_t common = (uint8_t)(mask_ & peerMask_);
        uint8_t target = band_;
        if (bad)
        {
            for (uint8_t b = band_ + 1; b < EML_BAND_COUNT; ++b)
                if (common & (1U << b)) { target = b; break; }
        }
        else if (healthyValid_ && nowMs - healthy_ >= EML_BAND_PROMOTE_MS)
        {
            for (uint8_t b = 0; b < band_; ++b)
                if (common & (1U << b)) { target = b; break; }
        }
        if (target == band_) return;
        if (++token_ == 0) ++token_;
        switch_ = (slot + EML_BAND_LEAD_SLOTS + 1U) & ~1UL;
        pending_ = {EML_BAND_PROPOSE, target, mask_, (uint16_t)switch_, token_};
        phase_ = EML_BAND_PROPOSE;
    }
    bool command(uint32_t slot, uint8_t lq, uint8_t samples, uint8_t missedAck, EmlBandMessage_s *out) const
    {
        if (phase_ && (int32_t)(switch_ - slot) > 1)
        {
            *out = pending_; out->op = phase_; out->mask = mask_;
            return true;
        }
        // Regular status provides the peer mask and reports reverse-link degradation.
        // It is separate from raw UART bytes and consumes one normal TDD slot.
        if (role_ == EML_RESPONDER && ((slot & 31U) == 1U || (samples >= 10 && lq < 60) || missedAck >= 4))
        {
            *out = {EML_BAND_STATUS, band_, mask_, (uint16_t)slot,
                    (uint16_t)((uint16_t)(samples >= 10 ? lq : 100) << 8 | missedAck)};
            return true;
        }
        return false;
    }
    bool receive(const EmlBandMessage_s &m, uint32_t slot, uint32_t nowMs)
    {
        if (!emlBandMessageValid(m) || !(mask_ & (1U << m.band))) return false;
        if (role_ == EML_COORDINATOR && m.op == EML_BAND_STATUS && m.band == band_ && m.switch_slot == (uint16_t)slot)
        {
            peerMask_ = m.mask; peerLq_ = (uint8_t)(m.token >> 8); peerMissed_ = (uint8_t)m.token;
            peerStatusMs_ = nowMs;
            return true;
        }
        const int16_t lead = (int16_t)(m.switch_slot - (uint16_t)slot);
        if (lead < 2 || lead > EML_BAND_LEAD_SLOTS + 1 || m.band == band_) return false;
        if (role_ == EML_RESPONDER && m.op == EML_BAND_PROPOSE)
        {
            if (busy() && (pending_.token != m.token || pending_.switch_slot != m.switch_slot || pending_.band != m.band)) return false;
            pending_ = m; switch_ = slot + (uint16_t)lead; phase_ = EML_BAND_ACK;
            peerMask_ = m.mask;
            return true;
        }
        if (!busy() || m.token != pending_.token || m.switch_slot != pending_.switch_slot || m.band != pending_.band) return false;
        if (role_ == EML_COORDINATOR && m.op == EML_BAND_ACK)
        {
            peerMask_ = m.mask; phase_ = EML_BAND_COMMIT;
            return true;
        }
        if (role_ == EML_RESPONDER && m.op == EML_BAND_COMMIT)
        {
            armed_ = true;
            return true;
        }
        return false;
    }
    void sent(uint8_t op) { if (phase_ == EML_BAND_COMMIT && op == EML_BAND_COMMIT) armed_ = true; }
};

// Flattened PHY x band search. Coordinator scans 0 -> 1 -> 2 -> 3 on each
// PHY; receiver dwells for a full (unpruned) coordinator cycle plus a window.
// Consequently different local PLL masks and arbitrary boot-time offsets still
// give a complete common window, provided at least one (PHY,band) works on both.
class EmlBandSearch
{
    const EmlPhyPlan_s *plan_ = nullptr;
    EmlRole_e role_ = EML_COORDINATOR;
    uint8_t masks_[EML_PHY_MAX] = {}, index_ = 0;
    uint32_t since_ = 0, cycleMs_ = 0;
    bool valid_ = false;
public:
    bool begin(const EmlPhyPlan_s &plan, EmlRole_e role, const uint8_t *masks, uint32_t nowMs)
    {
        plan_ = &plan; role_ = role; since_ = nowMs; cycleMs_ = 0; index_ = 0;
        valid_ = false;
        if (!masks || !plan.count || plan.count > EML_PHY_MAX) return false;
        for (uint8_t p = 0; p < plan.count; ++p)
        { masks_[p] = masks[p] & 15U; cycleMs_ += EML_BAND_COUNT * plan.coordinator_dwell_ms[p]; }
        for (; index_ < plan.count * EML_BAND_COUNT; ++index_)
            if (masks_[phy()] & (1U << band())) { valid_ = true; return true; }
        index_ = 0;
        return false;
    }
    uint8_t phy() const { return index_ / EML_BAND_COUNT; }
    uint8_t band() const { return index_ % EML_BAND_COUNT; }
    uint8_t mask() const { return valid_ ? masks_[phy()] : 0; }
    uint32_t dwellMs() const
    { return plan_->coordinator_dwell_ms[phy()] + (role_ == EML_RESPONDER ? cycleMs_ : 0); }
    bool update(uint32_t nowMs, bool held)
    {
        if (!valid_) return false;
        if (held) { since_ = nowMs; return false; }
        if (nowMs - since_ < dwellMs()) return false;
        const uint8_t old = index_;
        do { index_ = (uint8_t)((index_ + 1U) % (plan_->count * EML_BAND_COUNT)); }
        while (!(masks_[phy()] & (1U << band())));
        since_ = nowMs;
        return index_ != old;
    }
};
