#pragma once

#include <stdint.h>
#include "eml_transport.h"

// P9.4 automatic Master/Slave selection, foreground only.
//
// An automatic node first listens as a responder whose band search is held on its
// first (default) band, then advertises as a coordinator, alternating until a
// session starts. Each phase lasts its minimum plus k * step, k uniform in 0..7:
//  - listen >= R = band cycle + band dwell finds a coordinator that cycles its
//    bands alone: any window of that length contains one full coordinator dwell
//    (two HELLO periods) on the listener's band;
//  - advertise >= 2R lets a fixed responder meet it: the responder dwells R per
//    band, so any 2R window contains one whole responder dwell;
//  - two automatic nodes that draw different k become coordinator at least one
//    step (one coordinator dwell) apart, and a new coordinator starts on the
//    default band on which the other node is still listening.
// Equal draws are resolved by the next phases. A session or handshake in progress
// (held) keeps the current role and the phase restarts when it ends.
class EmlRoleElection
{
    uint32_t rng_ = 1, since_ = 0, phase_ = 0, listen_ = 0, advertise_ = 0, step_ = 0;
    EmlRole_e role_ = EML_COORDINATOR;
    bool automatic_ = false;

    uint32_t draw(uint32_t minimum)
    {
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        return minimum + (rng_ >> 29) * step_;
    }

public:
    static constexpr uint8_t kMaxSteps = 7;

    void begin(bool automatic, EmlRole_e fixedRole, uint32_t listenMs, uint32_t advertiseMs, uint32_t stepMs,
               uint32_t seed, uint32_t nowMs)
    {
        automatic_ = automatic;
        role_ = automatic ? EML_RESPONDER : fixedRole;
        listen_ = listenMs; advertise_ = advertiseMs; step_ = stepMs;
        rng_ = seed ? seed : 0x9E3779B9UL;
        since_ = nowMs;
        phase_ = automatic ? draw(listen_) : 0;
    }
    bool automatic() const { return automatic_; }
    EmlRole_e role() const { return role_; }
    // The listener keeps its band so a new coordinator is heard on the default band
    bool listening() const { return automatic_ && role_ == EML_RESPONDER; }
    uint32_t phaseMs() const { return phase_; }

    // Returns true when the role changed; the caller restarts the MAC in role().
    bool update(uint32_t nowMs, bool held)
    {
        if (!automatic_) return false;
        if (held) { since_ = nowMs; return false; }
        if (nowMs - since_ < phase_) return false;
        role_ = role_ == EML_RESPONDER ? EML_COORDINATOR : EML_RESPONDER;
        phase_ = draw(role_ == EML_RESPONDER ? listen_ : advertise_);
        since_ = nowMs;
        return true;
    }
};
