#pragma once

// Native-only deterministic MAC/transport measurement harness. The firmware
// classes are used directly; RF/SPI/CPU execution and UART electrical timing
// are not emulated. Accepted-byte latency ends at the model UART drain.
#include "eml_tdd.h"
#include <vector>
#include <stdexcept>
#include <algorithm>

template <uint8_t Window, size_t Capacity = 1024>
class EmlTddSimulation
{
    uint32_t random_ = 0x12345678;
    uint32_t generation_[2] = {};
public:
    EmlTransport<Capacity, Window> transport[2];
    EmlTddController scheduler[2];
    EmlMacProfile_s profile;
    uint32_t slot = 0, elapsed_slots = 0;
    unsigned loss = 0, collisions = 0, short_frames = 0, data_frames = 0;
    uint8_t drop_op = 0;
    unsigned drop_count = 0;
    bool drop_always = false, adaptive, short_ack;
    uint64_t airtime_us = 0;
    size_t accepted[2] = {}, delivered[2] = {};
    std::vector<uint64_t> accepted_at[2], latency_us[2];

    EmlTddSimulation(const EmlMacProfile_s &p, bool adapt, bool compact, uint32_t start = 0)
        : profile(p), slot(start), adaptive(adapt), short_ack(compact && emlIsFsk(p))
    {
        const uint8_t uid[] = {1,2,3,4,5,6};
        const EmlFhssRecord_s f = {emlBandStartReg(0), emlBandChannelReg(0,59), 60, 123, 240};
        const auto ids = emlDeriveLinkIds(uid, p, f, true, Window, adaptive, short_ack);
        for (uint8_t r = 0; r < 2; ++r)
        {
            const auto cfg = emlMacTransportConfig((EmlRole_e)r, ids, p, 240, Window, adaptive, short_ack);
            if (!transport[r].begin(cfg, 100U + r, slot)) throw std::runtime_error("bad simulation config");
            scheduler[r].begin((EmlRole_e)r, adaptive, slot);
            generation_[r] = transport[r].generation();
        }
    }
    uint64_t nowUs() const { return (uint64_t)elapsed_slots * profile.slot_us; }
    static uint8_t pattern(size_t index, uint8_t source) { return (uint8_t)(index * 73U + (index >> 7) + source * 19U); }
    size_t feed(uint8_t role, size_t count)
    {
        uint8_t bytes[64]; count = std::min(count, sizeof(bytes));
        for (size_t i = 0; i < count; ++i) bytes[i] = pattern(accepted[role] + i, role);
        const size_t n = transport[role].acceptUart(bytes, count);
        accepted[role] += n;
        accepted_at[role].insert(accepted_at[role].end(), n, nowUs());
        return n;
    }
    void drain(uint8_t destination, size_t limit, uint64_t offsetUs = 0)
    {
        uint8_t bytes[64]; const uint8_t source = destination ^ 1U;
        const size_t n = transport[destination].peekUart(bytes, std::min(limit, sizeof(bytes)));
        for (size_t i = 0; i < n; ++i)
        {
            if (bytes[i] != pattern(delivered[source], source)) throw std::runtime_error("duplicate, reordered or corrupt UART byte");
            latency_us[source].push_back(nowUs() + offsetUs - accepted_at[source].at(delivered[source]));
            ++delivered[source];
        }
        transport[destination].consumeUart(n);
    }
    void step(bool drainUart = true)
    {
        uint8_t frames[2][64] = {}; EmlTxTicket_s tickets[2] = {};
        bool sending[2] = {}; size_t length[2] = {};
        for (uint8_t r = 0; r < 2; ++r)
        {
            transport[r].tick(slot);
            if (generation_[r] != transport[r].generation())
            {
                generation_[r] = transport[r].generation();
                scheduler[r].begin((EmlRole_e)r, adaptive, slot);
            }
            scheduler[r].tick(slot, transport[r].queuedTx(), transport[r].flowControl(), transport[r].peerFlowControl(),
                              emlMaxPayloadForFrameSize(profile.frame_size), false);
            if (emlTddOwner(slot, scheduler[r].plan()) != r) continue;
            const bool compact = short_ack && emlShortAckSlot(slot);
            length[r] = compact ? EML_ACK_FRAME_SIZE : profile.frame_size;
            EmlBandMessage_s control;
            const bool cmd = !compact && transport[r].active() && scheduler[r].command(slot, transport[r].queuedTx(),
                                    transport[r].flowControl(), transport[r].pending(), &control);
            sending[r] = transport[r].prepare(slot, (uint16_t)((slot >> 1) % 240), frames[r], 64, &tickets[r], cmd ? &control : nullptr, compact);
            if (sending[r])
            {
                if (!transport[r].sent(tickets[r], slot)) throw std::runtime_error("unmatched simulation ticket");
                if (tickets[r].type == EML_PKT_TYPE_SYNC) scheduler[r].sent(tickets[r].phase, slot);
                EmlMacProfile_s wire = profile; wire.frame_size = (uint8_t)length[r];
                airtime_us += emlProfileToaUs(wire);
                if (compact) ++short_frames;
                if (tickets[r].type == EML_PKT_TYPE_DATA) ++data_frames;
            }
        }
        if (sending[0] && sending[1]) ++collisions;
        else for (uint8_t r = 0; r < 2; ++r) if (sending[r])
        {
            const uint8_t to = r ^ 1U;
            random_ = random_ * 1664525UL + 1013904223UL;
            bool dropped = random_ % 100U < loss;
            if (tickets[r].type == EML_PKT_TYPE_SYNC && tickets[r].phase == drop_op && (drop_always || drop_count))
            { dropped = true; if (drop_count) --drop_count; }
            if (!dropped)
            {
                const EmlRxContext_s ctx = {transport[to].generation(), slot, (uint16_t)((slot >> 1) % 240)};
                const auto result = transport[to].receive(frames[r], length[r], ctx, slot);
                if (result == EML_RX_CONTROL)
                {
                    EmlFrameView_s view; EmlBandMessage_s msg;
                    if (emlFrameDecode(frames[r], length[r], profile.frame_size, &view) == EML_FRAME_OK &&
                        emlBandMessageRead(view.payload, view.header.payload_len, &msg) && msg.op >= EML_TDD_PROPOSE)
                        scheduler[to].receive(msg, slot);
                }
            }
        }
        if (drainUart)
        {
            // End-of-slot UART service, one bounded 64-byte batch per side.
            drain(0,64,profile.slot_us); drain(1,64,profile.slot_us);
        }
        ++slot; ++elapsed_slots;
    }
    void establish()
    {
        for (unsigned i = 0; i < 500 && (!transport[0].active() || !transport[1].active()); ++i) step();
        if (!transport[0].active() || !transport[1].active()) throw std::runtime_error("simulation handshake timed out");
    }
};
