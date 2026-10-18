#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
#include <unity.h>
#include <FHSS.h>
#include "eml_mac.h"

// ---- P4.3: time on air and slot budget ------------------------------------------------

void test_toa_matches_semtech_and_elrs_tables()
{
    // ELRS ExpressLRS_AirRateRFperf (SX127x) lists these rounded to 10 us
    TEST_ASSERT_UINT32_WITHIN(5, 4380, emlLoraToaUs(6, 500000, 7, 8, 8, true, false, false));   // 200 Hz
    TEST_ASSERT_UINT32_WITHIN(5, 6690, emlLoraToaUs(6, 500000, 8, 8, 13, true, false, false));  // 100 Hz 8ch
    TEST_ASSERT_UINT32_WITHIN(5, 8770, emlLoraToaUs(7, 500000, 7, 8, 8, true, false, false));   // 100 Hz
    // Semtech payload symbol counts for SF6/CR4/5 implicit header without CRC
    TEST_ASSERT_EQUAL_UINT32(113, emlLoraPayloadSymbols(6, 5, 64, true, false, false));
    TEST_ASSERT_EQUAL_UINT32(58, emlLoraPayloadSymbols(6, 5, 32, true, false, false));
    // Explicit header and CRC only add symbols
    TEST_ASSERT_TRUE(emlLoraToaUs(6, 500000, 5, 8, 64, false, true, false) > emlProfileToaUs(EML_PROFILE_LORA64));
    TEST_ASSERT_EQUAL_UINT32(16032, emlProfileToaUs(EML_PROFILE_LORA64));
    TEST_ASSERT_EQUAL_UINT32(8992, emlProfileToaUs(EML_PROFILE_LORA32));
}

void test_toa_every_length_monotonic_and_bounded()
{
    uint32_t previous = 0;
    for (uint16_t n = 1; n <= 64; n++)
    {
        const uint32_t toa = emlLoraToaUs(6, 500000, 5, 8, n, true, false, false);
        TEST_ASSERT_TRUE(toa >= previous);
        TEST_ASSERT_EQUAL_UINT32(0, (toa - 1568U - 1024U) % 640U); // 12.25 preamble + 8 symbols, then 5-symbol blocks
        previous = toa;
    }
    TEST_ASSERT_EQUAL_UINT32(16032, previous);
}

void test_slot_budget()
{
    const EmlMacProfile_s profiles[] = {EML_PROFILE_LORA64, EML_PROFILE_LORA32};
    for (const EmlMacProfile_s &p : profiles)
    {
        TEST_ASSERT_TRUE(emlProfileValid(p));
        const uint32_t toa = emlProfileToaUs(p);
        TEST_ASSERT_EQUAL_UINT32(EML_TX_START_US + toa + EML_RX_DONE_US, emlRxOffsetUs(p));
        TEST_ASSERT_TRUE(p.slot_us >= emlMinSlotUs(p));
        // The peer frame ends (with responder bias and guard) before the next own boundary
        TEST_ASSERT_TRUE(EML_TX_START_US + toa + EML_RX_DONE_US + EML_RESPONDER_BIAS_US + EML_PREP_MARGIN_US < p.slot_us);
        // A responder this much late still opens RX inside the coordinator's preamble
        TEST_ASSERT_TRUE(EML_GUARD_US / 2U < EML_TX_START_US + (uint32_t)p.preamble * 128U / 2U);
        EmlMacProfile_s tight = p;
        tight.slot_us = emlMinSlotUs(p) - 1U;
        TEST_ASSERT_FALSE(emlProfileValid(tight));
    }
    // Net payload per direction: one frame per slot pair
    TEST_ASSERT_EQUAL_UINT32(1714, (uint32_t)(60ULL * 1000000ULL / (2ULL * EML_PROFILE_LORA64.slot_us)));
    TEST_ASSERT_EQUAL_UINT32(1346, (uint32_t)(28ULL * 1000000ULL / (2ULL * EML_PROFILE_LORA32.slot_us)));
}

// ---- P4.4/P4.5: slot clock, hop advance and transport time -----------------------------

void test_clock_symmetric_tdd_and_hops()
{
    EmlSlotClock_s c;
    emlClockInit(c, 5);
    for (uint32_t s = 0; s < 40; s++)
    {
        emlClockAdvance(c);
        TEST_ASSERT_EQUAL_UINT32(s, c.slot);
        TEST_ASSERT_EQUAL_UINT16((s / 2U) % 5U, c.hop); // a pair shares one hop
        TEST_ASSERT_EQUAL(s % 2U == 0, emlOwnSlot(s, EML_COORDINATOR));
        TEST_ASSERT_EQUAL(s % 2U == 1, emlOwnSlot(s, EML_RESPONDER));
        TEST_ASSERT_EQUAL_UINT16(c.hop, emlHopAfter(0, s, 5));
    }
    // 32-bit wrap: 0xFFFFFFFF is odd, the next slot 0 is even and hops
    c.slot = 0xFFFFFFFEUL; c.hop = 3;
    emlClockAdvance(c);
    TEST_ASSERT_EQUAL_UINT16(3, c.hop);
    emlClockAdvance(c);
    TEST_ASSERT_EQUAL_UINT32(0, c.slot);
    TEST_ASSERT_EQUAL_UINT16(4, c.hop);
    emlClockAdvance(c); emlClockAdvance(c);
    TEST_ASSERT_EQUAL_UINT16(0, c.hop);
}

void test_next_own_slot_is_monotonic()
{
    for (uint32_t role = 0; role < 2; role++)
    {
        uint32_t previous = 0;
        for (uint32_t cur = 0; cur < 20; cur++)
        {
            const uint32_t next = emlNextOwnSlot(cur, (EmlRole_e)role);
            TEST_ASSERT_TRUE(next > cur && next - cur <= 2U);
            TEST_ASSERT_TRUE(emlOwnSlot(next, (EmlRole_e)role));
            TEST_ASSERT_TRUE(next >= previous);
            previous = next;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0, emlNextOwnSlot(0xFFFFFFFFUL, EML_COORDINATOR));
}

void test_transport_config_from_profile()
{
    const uint8_t uid[6] = {1, 2, 3, 4, 5, 6};
    const EmlFhssRecord_s fhss = {903500000, 926900000, 40, 0x1234, 240};
    const EmlMacProfile_s profiles[] = {EML_PROFILE_LORA64, EML_PROFILE_LORA32};
    for (const EmlMacProfile_s &p : profiles)
    {
        const EmlLinkIds_s ids = emlDeriveLinkIds(uid, p, fhss);
        const EmlTransportConfig_s tx = emlMacTransportConfig(EML_COORDINATOR, ids, p, 240);
        const EmlTransportConfig_s rx = emlMacTransportConfig(EML_RESPONDER, ids, p, 240);
        TEST_ASSERT_TRUE(EmlTransport<1024>::validConfig(tx));
        TEST_ASSERT_TRUE(EmlTransport<1024>::validConfig(rx));
        TEST_ASSERT_EQUAL_UINT32(tx.local_id, rx.peer_id);
        TEST_ASSERT_EQUAL_UINT32(rx.local_id, tx.peer_id);
        TEST_ASSERT_TRUE(tx.link_timeout_slots * p.slot_us >= EML_LINK_TIMEOUT_MS * 1000U);
        TEST_ASSERT_TRUE((tx.link_timeout_slots - 1U) * p.slot_us < EML_LINK_TIMEOUT_MS * 1000U);
    }
}

void test_link_ids_bind_uid_and_configuration()
{
    const uint8_t uid[6] = {1, 2, 3, 4, 5, 6};
    uint8_t other[6] = {1, 2, 3, 4, 5, 7};
    const EmlFhssRecord_s fhss = {903500000, 926900000, 40, 0x1234, 240};
    const EmlLinkIds_s a = emlDeriveLinkIds(uid, EML_PROFILE_LORA64, fhss);
    const EmlLinkIds_s again = emlDeriveLinkIds(uid, EML_PROFILE_LORA64, fhss);
    TEST_ASSERT_EQUAL_UINT32(a.coordinator, again.coordinator);
    TEST_ASSERT_EQUAL_UINT32(a.configuration, again.configuration);
    TEST_ASSERT_TRUE(a.coordinator && a.responder && a.configuration && a.coordinator != a.responder);

    const EmlLinkIds_s b = emlDeriveLinkIds(other, EML_PROFILE_LORA64, fhss);
    TEST_ASSERT_NOT_EQUAL(a.coordinator, b.coordinator);
    TEST_ASSERT_NOT_EQUAL(a.configuration, b.configuration);
    TEST_ASSERT_NOT_EQUAL(a.configuration, emlDeriveLinkIds(uid, EML_PROFILE_LORA32, fhss).configuration);
    EmlFhssRecord_s f2 = fhss; f2.hop_count = 200;
    TEST_ASSERT_NOT_EQUAL(a.configuration, emlDeriveLinkIds(uid, EML_PROFILE_LORA64, f2).configuration);
    f2 = fhss; f2.freq_start += 1;
    TEST_ASSERT_NOT_EQUAL(a.configuration, emlDeriveLinkIds(uid, EML_PROFILE_LORA64, f2).configuration);
}

// ---- P4.5: acquisition and phase tracking -------------------------------------------------

void test_acquire_maps_hello_to_next_boundary()
{
    const EmlMacProfile_s &p = EML_PROFILE_LORA64;
    const uint32_t senderBoundary = 1000000;  // coordinator starts slot 0x1234 (even) here
    const uint32_t rx = senderBoundary + emlRxOffsetUs(p);
    EmlAcquire_s a;
    TEST_ASSERT_TRUE(emlAcquire(7, 0x1234, 9, 240, rx, rx + 150, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x1234, a.hello_slot);
    TEST_ASSERT_EQUAL_UINT32(0x1235, a.first_slot);
    TEST_ASSERT_EQUAL_UINT32(senderBoundary + p.slot_us - EML_RESPONDER_BIAS_US, a.first_boundary_us);
    TEST_ASSERT_EQUAL_UINT32(0x1234, a.clock_slot);
    TEST_ASSERT_EQUAL_UINT16(9, a.clock_hop);
    TEST_ASSERT_EQUAL_INT32(0, emlPhaseCorrection(rx, senderBoundary - EML_RESPONDER_BIAS_US, p));
    TEST_ASSERT_EQUAL_INT32(-50, emlPhaseCorrection(rx, senderBoundary - EML_RESPONDER_BIAS_US + 50, p)); // local late
    TEST_ASSERT_EQUAL_INT32(30, emlPhaseCorrection(rx, senderBoundary - EML_RESPONDER_BIAS_US - 30, p));  // local early

    // Late processing skips to a later boundary with the matching hop, up to the frame age limit
    const uint32_t late = senderBoundary + p.slot_us - EML_RESPONDER_BIAS_US - EML_LOCK_LEAD_US + 1;
    TEST_ASSERT_TRUE(emlAcquire(7, 0x1234, 239, 240, rx, late, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x1236, a.first_slot);
    TEST_ASSERT_EQUAL_UINT32(0x1235, a.clock_slot);
    TEST_ASSERT_EQUAL_UINT16(239, a.clock_hop);
    EmlSlotClock_s c = {a.clock_slot, a.clock_hop, 240};
    emlClockAdvance(c);
    TEST_ASSERT_EQUAL_UINT16(0, c.hop); // wraps the sequence on the even slot
    TEST_ASSERT_TRUE(emlAcquire(7, 0x1234, 1, 240, rx, late + p.slot_us, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x1237, a.first_slot);
    TEST_ASSERT_FALSE(emlAcquire(7, 0x1234, 1, 240, rx, late + 2 * p.slot_us, p, &a));

    // Odd slot or out-of-table hop is not a coordinator HELLO
    TEST_ASSERT_FALSE(emlAcquire(7, 0x1235, 1, 240, rx, rx + 150, p, &a));
    TEST_ASSERT_FALSE(emlAcquire(7, 0x1234, 240, 240, rx, rx + 150, p, &a));
    TEST_ASSERT_FALSE(emlAcquire(7, 0x1234, 1, 240, rx, rx + 150, p, nullptr));
}

void test_acquire_extended_slot_only_moves_forward()
{
    const EmlMacProfile_s &p = EML_PROFILE_LORA32;
    EmlAcquire_s a;
    const uint32_t rx = 5000000;
    TEST_ASSERT_TRUE(emlAcquire(0x0001FFF0UL, 0x0004, 0, 40, rx, rx + 100, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x00020004UL, a.hello_slot);
    TEST_ASSERT_TRUE(emlAcquire(0x00010010UL, 0x0008, 0, 40, rx, rx + 100, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x00020008UL, a.hello_slot); // behind locally -> next wrap, never backwards
    TEST_ASSERT_TRUE(emlAcquire(0x00010008UL, 0x0008, 0, 40, rx, rx + 100, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x00010008UL, a.hello_slot);
    TEST_ASSERT_TRUE(emlAcquire(0xFFFFFFF0UL, 0x0002, 0, 40, rx, rx + 100, p, &a));
    TEST_ASSERT_EQUAL_UINT32(0x00000002UL, a.hello_slot);
}

void test_ticket_ring_fifo()
{
    EmlTicketRing<4> ring;
    EmlTxTicket_s t = {7, 0, 0, EML_PKT_TYPE_DATA, 0}, out = {};
    TEST_ASSERT_FALSE(ring.pop(&out));
    for (uint32_t i = 0; i < 4; i++) { t.slot = i; t.serial = i + 1; TEST_ASSERT_TRUE(ring.push(t)); }
    TEST_ASSERT_FALSE(ring.push(t));
    for (uint32_t i = 0; i < 4; i++)
    {
        TEST_ASSERT_TRUE(ring.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(i, out.slot);
        TEST_ASSERT_EQUAL_UINT32(i + 1, out.serial);
        TEST_ASSERT_EQUAL_UINT32(7, out.generation);
    }
    TEST_ASSERT_FALSE(ring.pop(&out));
    for (uint32_t i = 0; i < 10; i++) { t.slot = i; TEST_ASSERT_TRUE(ring.push(t)); TEST_ASSERT_TRUE(ring.pop(&out)); TEST_ASSERT_EQUAL_UINT32(i, out.slot); }
}

// ---- P4.4-P4.6: MAC + transport pair over a simulated FHSS channel -----------------------
// Models the firmware schedule of rxtx_eml_bridge.cpp: shared hop per pair, coordinator
// hops blindly, responder searches on the rendezvous channel, locks from a HELLO, tracks
// losses and re-searches; TX reports precede RX; early and late frame preparation.

static uint32_t simSeed = 1;
static uint32_t nextRandom() { simSeed = simSeed * 1103515245UL + 12345UL; return simSeed >> 8; }

template <size_t Capacity>
struct SimNode
{
    EmlTransport<Capacity> t;
    EmlSlotClock_s clock;
    bool locked = false;
    uint32_t lastHeard = 0;
    std::deque<uint8_t> sent, received;
    uint32_t streamByte = 0;
    void feed(size_t n)
    {
        uint8_t buf[64];
        for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)(streamByte + i) ^ (uint8_t)((streamByte + i) >> 8);
        const size_t accepted = t.acceptUart(buf, n);
        for (size_t i = 0; i < accepted; i++) sent.push_back(buf[i]);
        streamByte += accepted;
    }
    void drain(size_t n)
    {
        uint8_t buf[64];
        const size_t got = t.peekUart(buf, n < 64 ? n : 64);
        for (size_t i = 0; i < got; i++) received.push_back(buf[i]);
        t.consumeUart(got);
    }
};

template <size_t Capacity>
struct SimLink
{
    const EmlMacProfile_s &p;
    uint8_t sequence[FHSS_SEQUENCE_LEN];
    uint16_t hops;
    const uint8_t syncChannel = 20;
    EmlTransportConfig_s ctx, crx;
    SimNode<Capacity> c, r;
    uint32_t g;        // true global slot, == coordinator slot
    uint32_t researchSlots;
    uint32_t lossPercent = 0;
    bool blackout = false;
    uint32_t earlyOnlyPercent = 0, staleTxPercent = 0;
    uint32_t acquisitions = 0, lockLosses = 0;

    explicit SimLink(const EmlMacProfile_s &profile, uint32_t startSlot = 0) : p(profile)
    {
        hops = FHSSrandomiseFHSSsequenceBuild(0x55AA1234UL, 40, syncChannel, sequence);
        const uint8_t uid[6] = {9, 8, 7, 6, 5, 4};
        const EmlFhssRecord_s f = {903500000, 926900000, 40, 0x55AA1234UL, hops};
        const EmlLinkIds_s ids = emlDeriveLinkIds(uid, p, f);
        ctx = emlMacTransportConfig(EML_COORDINATOR, ids, p, hops);
        crx = emlMacTransportConfig(EML_RESPONDER, ids, p, hops);
        researchSlots = crx.link_timeout_slots + 4U;
        emlClockInit(c.clock, hops);
        c.clock.slot = startSlot - 1U;
        g = startSlot - 1U;
        TEST_ASSERT_TRUE(c.t.begin(ctx, 0x1111U, startSlot));
        rebootResponder(0x2222U);
    }
    void rebootResponder(uint32_t seed)
    {
        emlClockInit(r.clock, hops);
        r.clock.slot = 0x12345U; // unrelated local count, odd
        r.locked = false;
        TEST_ASSERT_TRUE(r.t.begin(crx, seed, emlNextOwnSlot(r.clock.slot, EML_RESPONDER)));
    }
    bool lost() { return blackout || nextRandom() % 100U < lossPercent; }
    template <typename Node>
    bool makeFrame(Node &from, EmlRole_e role, uint32_t slot, uint16_t hop, uint8_t *frame)
    {
        // Late preparation normally wins; sometimes only the early one exists, sometimes the
        // ISR sends the early one although a late one replaced its ticket (sent() refused).
        EmlTxTicket_s early, late;
        const bool haveEarly = from.t.prepare(slot, hop, frame, 64, &early);
        const uint32_t pick = nextRandom() % 100U;
        if (pick < earlyOnlyPercent)
            return haveEarly && from.t.sent(early, slot);
        uint8_t second[64];
        const bool haveLate = from.t.prepare(slot, hop, second, 64, &late);
        if (pick < earlyOnlyPercent + staleTxPercent && haveEarly)
        {
            from.t.sent(early, slot); // refused: superseded ticket, like a late push that missed the boundary
            return true;
        }
        if (!haveLate) return false;
        memcpy(frame, second, 64);
        return from.t.sent(late, slot);
        (void)role;
    }
    void step()
    {
        ++g;
        emlClockAdvance(c.clock);
        TEST_ASSERT_EQUAL_UINT32(g, c.clock.slot);
        if (r.locked) emlClockAdvance(r.clock);
        uint8_t frame[64];
        if (!(g & 1U))
        {
            const uint16_t hop = c.clock.hop;
            const bool tx = makeFrame(c, EML_COORDINATOR, g, hop, frame);
            if (tx && !lost())
            {
                if (r.locked && r.clock.hop == hop)
                {
                    TEST_ASSERT_TRUE(emlOwnSlot(r.clock.slot + 1U, EML_RESPONDER));
                    TEST_ASSERT_EQUAL_UINT16((uint16_t)g, (uint16_t)r.clock.slot);
                    EmlFrameView_s v;
                    if (emlFrameDecode(frame, p.frame_size, p.frame_size, &v) == EML_FRAME_OK) r.lastHeard = r.clock.slot;
                    const EmlRxContext_s context = {r.t.generation(), r.clock.slot, r.clock.hop};
                    r.t.receive(frame, p.frame_size, context, emlNextOwnSlot(r.clock.slot, EML_RESPONDER));
                }
                else if (!r.locked && sequence[hop] == syncChannel)
                {
                    acquire(frame);
                }
            }
        }
        else if (r.locked)
        {
            TEST_ASSERT_EQUAL_UINT16(c.clock.hop, r.clock.hop); // FHSS agreement after lock
            if (makeFrame(r, EML_RESPONDER, r.clock.slot, r.clock.hop, frame) && !lost())
            {
                const EmlRxContext_s context = {c.t.generation(), g, c.clock.hop};
                c.t.receive(frame, p.frame_size, context, emlNextOwnSlot(g, EML_COORDINATOR));
            }
        }
        c.t.tick(emlNextOwnSlot(g, EML_COORDINATOR));
        if (r.locked)
        {
            r.t.tick(emlNextOwnSlot(r.clock.slot, EML_RESPONDER));
            if (r.clock.slot - r.lastHeard > researchSlots)
            {
                r.locked = false;
                ++lockLosses;
                r.t.reset(emlNextOwnSlot(r.clock.slot, EML_RESPONDER));
            }
        }
    }
    void acquire(const uint8_t *frame)
    {
        EmlFrameView_s v; EmlSync_s s;
        if (emlFrameDecode(frame, p.frame_size, p.frame_size, &v) != EML_FRAME_OK || v.header.pkt_type != EML_PKT_TYPE_SYNC ||
            emlSyncDecode(v.payload, v.header.payload_len, &s) != EML_SYNC_OK || s.phase != EML_SYNC_HELLO ||
            s.sender_id != crx.peer_id || s.configuration_id != crx.configuration_id)
            return;
        const uint32_t boundary = 100000U + g * p.slot_us;
        const uint32_t rx = boundary + emlRxOffsetUs(p);
        EmlAcquire_s a;
        TEST_ASSERT_TRUE(emlAcquire(r.clock.slot, s.slot_counter, s.hop_index, hops, rx, rx + 200, p, &a));
        TEST_ASSERT_EQUAL_UINT32(boundary + p.slot_us - EML_RESPONDER_BIAS_US, a.first_boundary_us);
        TEST_ASSERT_EQUAL_UINT32(1, a.first_slot - a.hello_slot);
        r.clock.slot = a.clock_slot;
        r.clock.hop = a.clock_hop;
        const EmlRxContext_s context = {r.t.generation(), a.hello_slot, s.hop_index};
        if (r.t.receive(frame, p.frame_size, context, emlNextOwnSlot(a.clock_slot, EML_RESPONDER)) != EML_RX_CONTROL) return;
        r.locked = true;
        r.lastHeard = a.hello_slot;
        ++acquisitions;
    }
    uint32_t runUntilActive(uint32_t limit)
    {
        const uint32_t start = g;
        while (!(c.t.active() && r.t.active()) && g - start < limit) step();
        return g - start;
    }
};

template <size_t Capacity>
static void checkStream(std::deque<uint8_t> &sent, std::deque<uint8_t> &received)
{
    TEST_ASSERT_TRUE(received.size() <= sent.size());
    for (size_t i = 0; i < received.size(); i++)
        TEST_ASSERT_EQUAL_HEX8(sent[i], received[i]);
}

template <size_t Capacity>
static void runTraffic(SimLink<Capacity> &link, uint32_t slots, size_t feed)
{
    for (uint32_t i = 0; i < slots; i++)
    {
        link.c.feed(feed); link.r.feed(feed);
        link.step();
        link.c.drain(64); link.r.drain(64);
    }
}

void test_mac_rendezvous_lock_and_handshake()
{
    simSeed = 7;
    SimLink<1024> link(EML_PROFILE_LORA64);
    // The responder can only lock on a rendezvous hop: at most one sequence block
    // (40 hops = 80 slots) plus the handshake and quiet interval
    const uint32_t slots = link.runUntilActive(400);
    TEST_ASSERT_TRUE(link.c.t.active() && link.r.t.active());
    TEST_ASSERT_TRUE(slots <= 80U + 2U * EML_QUIET_SLOTS + 12U);
    TEST_ASSERT_EQUAL_UINT32(1, link.acquisitions);
}

void test_mac_bidirectional_stream_with_losses()
{
    const EmlMacProfile_s profiles[] = {EML_PROFILE_LORA64, EML_PROFILE_LORA32};
    for (const EmlMacProfile_s &p : profiles)
    {
        simSeed = 11;
        SimLink<1024> link(p);
        link.runUntilActive(400);
        link.lossPercent = 15;
        link.earlyOnlyPercent = 10;
        link.staleTxPercent = 5;
        runTraffic(link, 3000, 40);
        link.lossPercent = 0; link.earlyOnlyPercent = 0; link.staleTxPercent = 0;
        for (uint32_t i = 0; i < 400; i++) { link.step(); link.c.drain(64); link.r.drain(64); }
        TEST_ASSERT_TRUE(link.c.t.active() && link.r.t.active());
        checkStream<1024>(link.c.sent, link.r.received);
        checkStream<1024>(link.r.sent, link.c.received);
        // Every accepted byte arrived exactly once after the loss-free tail
        TEST_ASSERT_EQUAL(link.c.sent.size(), link.r.received.size());
        TEST_ASSERT_EQUAL(link.r.sent.size(), link.c.received.size());
        // 15% loss per frame still moves well over half of the clean 1-frame-per-pair rate
        const size_t maxPayload = emlMaxPayloadForFrameSize(p.frame_size);
        TEST_ASSERT_TRUE(link.r.received.size() > 1500U * maxPayload / 2U);
        TEST_ASSERT_EQUAL_UINT32(0, link.lockLosses);
    }
}

void test_mac_responder_reboot_and_blackout_recover()
{
    simSeed = 23;
    SimLink<1024> link(EML_PROFILE_LORA64);
    link.runUntilActive(400);
    runTraffic(link, 200, 30);

    // Responder reboot: coordinator times out its session, responder searches the rendezvous
    link.rebootResponder(0x3333U);
    const uint32_t recover = link.runUntilActive(1000);
    TEST_ASSERT_TRUE(link.c.t.active() && link.r.t.active());
    TEST_ASSERT_TRUE(recover <= link.ctx.link_timeout_slots + 80U + 2U * EML_QUIET_SLOTS + 16U);
    TEST_ASSERT_EQUAL_UINT32(2, link.acquisitions);

    // Total blackout longer than the link timeout: responder loses lock and re-acquires
    link.blackout = true;
    for (uint32_t i = 0; i < link.researchSlots + 10U; i++) link.step();
    TEST_ASSERT_FALSE(link.c.t.active());
    TEST_ASSERT_FALSE(link.r.locked);
    TEST_ASSERT_EQUAL_UINT32(1, link.lockLosses);
    link.blackout = false;
    link.runUntilActive(400);
    TEST_ASSERT_TRUE(link.c.t.active() && link.r.t.active());
    TEST_ASSERT_EQUAL_UINT32(3, link.acquisitions);

    // A fresh session streams exactly
    link.c.sent.clear(); link.r.sent.clear(); link.c.received.clear(); link.r.received.clear();
    runTraffic(link, 1000, 50);
    for (uint32_t i = 0; i < 200; i++) { link.step(); link.c.drain(64); link.r.drain(64); }
    TEST_ASSERT_EQUAL(link.c.sent.size(), link.r.received.size());
    checkStream<1024>(link.c.sent, link.r.received);
    checkStream<1024>(link.r.sent, link.c.received);
}

void test_mac_short_outage_keeps_session()
{
    simSeed = 31;
    SimLink<1024> link(EML_PROFILE_LORA64);
    link.runUntilActive(400);
    runTraffic(link, 100, 20);
    const uint32_t opened = link.c.t.stats().sessions_opened;
    link.blackout = true;
    for (uint32_t i = 0; i < link.ctx.link_timeout_slots / 2U; i++) { link.c.feed(20); link.step(); }
    link.blackout = false;
    runTraffic(link, 300, 20);
    for (uint32_t i = 0; i < 100; i++) { link.step(); link.c.drain(64); link.r.drain(64); }
    TEST_ASSERT_EQUAL_UINT32(opened, link.c.t.stats().sessions_opened);
    TEST_ASSERT_EQUAL_UINT32(0, link.lockLosses);
    TEST_ASSERT_EQUAL(link.c.sent.size(), link.r.received.size());
    checkStream<1024>(link.c.sent, link.r.received);
}

void test_mac_slot_counter_wrap()
{
    simSeed = 41;
    SimLink<1024> link(EML_PROFILE_LORA32, 65536U - 300U);
    link.runUntilActive(400);
    runTraffic(link, 1200, 25); // crosses the 16-bit Sync counter wrap while streaming
    for (uint32_t i = 0; i < 200; i++) { link.step(); link.c.drain(64); link.r.drain(64); }
    TEST_ASSERT_TRUE(link.g > 65536U);
    TEST_ASSERT_TRUE(link.c.t.active() && link.r.t.active());
    TEST_ASSERT_EQUAL(link.c.sent.size(), link.r.received.size());
    checkStream<1024>(link.c.sent, link.r.received);
    checkStream<1024>(link.r.sent, link.c.received);
}

void setUp() {}
void tearDown() {}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_toa_matches_semtech_and_elrs_tables);
    RUN_TEST(test_toa_every_length_monotonic_and_bounded);
    RUN_TEST(test_slot_budget);
    RUN_TEST(test_clock_symmetric_tdd_and_hops);
    RUN_TEST(test_next_own_slot_is_monotonic);
    RUN_TEST(test_transport_config_from_profile);
    RUN_TEST(test_link_ids_bind_uid_and_configuration);
    RUN_TEST(test_acquire_maps_hello_to_next_boundary);
    RUN_TEST(test_acquire_extended_slot_only_moves_forward);
    RUN_TEST(test_ticket_ring_fifo);
    RUN_TEST(test_mac_rendezvous_lock_and_handshake);
    RUN_TEST(test_mac_bidirectional_stream_with_losses);
    RUN_TEST(test_mac_responder_reboot_and_blackout_recover);
    RUN_TEST(test_mac_short_outage_keeps_session);
    RUN_TEST(test_mac_slot_counter_wrap);
    UNITY_END();
    return 0;
}
