#include <unity.h>
#include <FHSS.h>
#include "eml_band.h"
#include "../../lib/SX127xDriver/SX127xPll.h"
#include <vector>
#include <cstring>

void test_four_band_grid_and_sequence()
{
    const uint32_t starts[] = {940000000, 880000000, 810000000, 750000000};
    uint8_t reference[256], sequence[256];
    for (uint32_t seed : {0U, 1U, 0x12345678U, 0xFFFFFFFFU})
    {
        memset(reference, 0xEE, sizeof(reference));
        TEST_ASSERT_EQUAL_UINT16(240, FHSSrandomiseFHSSsequenceBuild(seed, 60, 30, reference));
        for (uint8_t band = 0; band < 4; ++band)
        {
            memset(sequence, 0xEE, sizeof(sequence));
            TEST_ASSERT_EQUAL_UINT32(starts[band], emlBandEdgeHz(band));
            TEST_ASSERT_EQUAL_UINT16(240, FHSSrandomiseFHSSsequenceBuild(seed, 60, 30, sequence));
            TEST_ASSERT_EQUAL_MEMORY(reference, sequence, 256);
            for (uint16_t block = 0; block < 240; block += 60)
            {
                bool seen[60] = {};
                TEST_ASSERT_EQUAL_UINT8(30, sequence[block]);
                for (uint16_t i = block; i < block + 60; ++i)
                {
                    const uint8_t ch = sequence[i];
                    TEST_ASSERT_LESS_THAN(60, ch);
                    TEST_ASSERT_FALSE(seen[ch]); seen[ch] = true;
                    if (i) TEST_ASSERT_NOT_EQUAL(sequence[i - 1], ch);
                    const uint32_t hz = starts[band] + 500000U + ch * 1000000U;
                    const uint32_t reg = emlBandChannelReg(band, ch);
                    TEST_ASSERT_EQUAL_UINT32(emlSx1276Frf(hz), reg);
                    const uint64_t programmed = (uint64_t)reg * 32000000ULL / (1U << 19);
                    TEST_ASSERT_TRUE(hz - programmed < 62);
                    TEST_ASSERT_TRUE(programmed - 250000 >= starts[band]);
                    TEST_ASSERT_TRUE(programmed + 250000 < starts[band] + 60000000UL);
                }
            }
            TEST_ASSERT_NOT_EQUAL(sequence[0], sequence[239]);
            for (uint16_t i = 240; i < 256; ++i) TEST_ASSERT_EQUAL_HEX8(0xEE, sequence[i]);
            EmlSlotClock_s clock; emlClockInit(clock, 240);
            for (unsigned i = 0; i < 1024; ++i)
            { emlClockAdvance(clock); TEST_ASSERT_LESS_THAN(240, clock.hop); }
        }
    }
    TEST_ASSERT_EQUAL_UINT32(EML_CHANNEL_STEP_REG, emlBandChannelReg(0, 1) - emlBandChannelReg(0, 0));
    TEST_ASSERT_EQUAL_UINT8(4, emlFirstBand(0));
    TEST_ASSERT_EQUAL_UINT16(0, FHSSrandomiseFHSSsequenceBuild(0, 0, 0, sequence));
    TEST_ASSERT_EQUAL_UINT16(0, FHSSrandomiseFHSSsequenceBuild(0, 60, 60, sequence));
    TEST_ASSERT_EQUAL_UINT16(0, FHSSrandomiseFHSSsequenceBuild(0, 257, 0, sequence));
    TEST_ASSERT_EQUAL_UINT16(0, FHSSrandomiseFHSSsequenceBuild(0, 60, 30, nullptr));
}

void test_multiband_configuration_is_distinct()
{
    uint8_t uid[] = {1,2,3,4,5,6};
    const EmlFhssRecord_s f = {emlBandStartReg(0), emlBandChannelReg(0, 59), 60, 123, 240};
    auto a = emlDeriveLinkIds(uid, EML_PROFILE_FSK64, f, false);
    auto b = emlDeriveLinkIds(uid, EML_PROFILE_FSK64, f, true);
    TEST_ASSERT_EQUAL(a.coordinator, b.coordinator);
    TEST_ASSERT_NOT_EQUAL(a.configuration, b.configuration);
    TEST_ASSERT_NOT_EQUAL(b.configuration, emlDeriveLinkIds(uid, EML_PROFILE_LORA64, f, true).configuration);
}

void test_pll_poll_timeout_stale_unstable_and_wrap()
{
    for (uint32_t initial : {0U, 0xFFFFFFD0U})
    {
        uint32_t now = initial;
        auto clock = [&]() { return now; };
        auto pause = [&]() { now += 20; };
        auto never = []() { return (uint8_t)0x80; };
        TEST_ASSERT_FALSE(sx127xWaitPll(never, clock, pause, 500, true));
        TEST_ASSERT_EQUAL_UINT32(500, now - initial);
        auto stale = []() { return (uint8_t)0x90; };
        TEST_ASSERT_FALSE(sx127xWaitPll(stale, clock, pause, 500, false));
        auto brokenSpi = []() { return (uint8_t)0xFF; };
        TEST_ASSERT_FALSE(sx127xWaitPll(brokenSpi, clock, pause, 500, true));
        auto notReady = []() { return (uint8_t)0x10; };
        TEST_ASSERT_FALSE(sx127xWaitPll(notReady, clock, pause, 500, true));
        TEST_ASSERT_TRUE(sx127xWaitPll(never, clock, pause, 500, false));
        unsigned reads = 0;
        auto flicker = [&]() { return (uint8_t)((++reads % 3) ? 0x90 : 0x80); };
        TEST_ASSERT_FALSE(sx127xWaitPll(flicker, clock, pause, 500, true));
        TEST_ASSERT_TRUE(sx127xWaitPll(stale, clock, pause, 500, true));
    }
}

struct BandPair
{
    EmlTransport<512> t[2];
    EmlBandController policy[2];
    uint8_t band[2] = {}, bytes[64] = {};
    uint32_t slot;
    uint8_t size;
    unsigned switches[2] = {};
    BandPair(uint8_t frameSize, uint32_t start) : slot(start), size(frameSize)
    {
        const auto &p = size == 64 ? EML_PROFILE_LORA64 : EML_PROFILE_LORA32;
        const EmlLinkIds_s ids = {1, 2, 123};
        for (uint8_t r = 0; r < 2; ++r)
        {
            TEST_ASSERT_TRUE(t[r].begin(emlMacTransportConfig((EmlRole_e)r, ids, p, 240), 100 + r, slot));
            policy[r].begin((EmlRole_e)r, 0, 15, 0);
        }
        for (unsigned i = 0; i < 200 && (!t[0].active() || !t[1].active()); ++i) step(0, false);
        TEST_ASSERT_TRUE(t[0].active() && t[1].active());
    }
    void step(uint8_t dropOp, bool degrade)
    {
        const uint32_t ms = 3000 + slot * 5U;
        for (uint8_t r = 0; r < 2; ++r)
        {
            if (policy[r].armed() && (int32_t)(slot - policy[r].switchSlot()) >= 0)
            { band[r] = policy[r].target(); policy[r].completed(band[r], ms); ++switches[r]; }
            if (degrade && !switches[r]) policy[r].consider(slot, ms, 50, 100, 0);
            t[r].tick(slot);
        }
        const uint8_t from = slot & 1U, to = from ^ 1U;
        EmlBandMessage_s m;
        const bool control = t[from].active() && policy[from].command(slot, 100, 100, 0, &m);
        EmlTxTicket_s ticket;
        if (t[from].prepare(slot, (uint16_t)((slot >> 1) % 240), bytes, sizeof(bytes), &ticket, control ? &m : nullptr))
        {
            TEST_ASSERT_TRUE(t[from].sent(ticket, slot));
            if (ticket.type == EML_PKT_TYPE_SYNC) policy[from].sent(ticket.phase);
            if (band[from] == band[to] && (!control || m.op != dropOp))
            {
                auto result = t[to].receive(bytes, size, {t[to].generation(), slot, (uint16_t)((slot >> 1) % 240)}, slot);
                if (result == EML_RX_CONTROL && control) TEST_ASSERT_TRUE(policy[to].receive(m, slot, ms));
            }
        }
        ++slot;
    }
};

void test_graceful_handover_retries_and_preserves_uart_arq()
{
    for (uint8_t n : {32, 64}) for (uint32_t start : {0U, 65510U, 0xFFFFFFD0U})
    {
        BandPair p(n, start);
        uint8_t input[200]; for (unsigned i = 0; i < sizeof(input); ++i) input[i] = (uint8_t)(i * 37U);
        for (auto &t : p.t) TEST_ASSERT_EQUAL_UINT32(sizeof(input), t.acceptUart(input, sizeof(input)));
        const uint32_t gen[] = {p.t[0].generation(), p.t[1].generation()};
        // Lost first PROPOSE, ACK and COMMIT all retry inside the common countdown.
        for (uint8_t op : {EML_BAND_PROPOSE, EML_BAND_ACK, EML_BAND_COMMIT})
        { p.step(op, true); p.step(op, true); }
        for (unsigned i = 0; i < 150; ++i) p.step(0, true);
        TEST_ASSERT_EQUAL_UINT8(1, p.band[0]); TEST_ASSERT_EQUAL_UINT8(1, p.band[1]);
        uint8_t output[200];
        for (uint8_t r = 0; r < 2; ++r)
        {
            TEST_ASSERT_EQUAL_UINT32(1, p.switches[r]);
            TEST_ASSERT_EQUAL_UINT32(gen[r], p.t[r].generation());
            TEST_ASSERT_EQUAL_UINT32(0, p.t[r].stats().discarded_tx);
            TEST_ASSERT_EQUAL_UINT32(0, p.t[r].queuedTx());
            TEST_ASSERT_EQUAL_UINT32(sizeof(output), p.t[r].peekUart(output, sizeof(output)));
            TEST_ASSERT_EQUAL_MEMORY(input, output, sizeof(input));
        }
    }
}

void test_lost_handover_messages_have_bounded_outcomes()
{
    for (uint8_t op : {EML_BAND_PROPOSE, EML_BAND_ACK, EML_BAND_COMMIT})
    {
        BandPair p(64, 0);
        for (unsigned i = 0; i < 50; ++i) p.step(op, true);
        if (op == EML_BAND_COMMIT)
        { TEST_ASSERT_EQUAL_UINT8(1, p.band[0]); TEST_ASSERT_EQUAL_UINT8(0, p.band[1]); }
        else
        { TEST_ASSERT_EQUAL_UINT8(0, p.band[0]); TEST_ASSERT_EQUAL_UINT8(0, p.band[1]); }
    }
}

void test_control_rejects_wrong_session_crc_and_stale_descriptor()
{
    BandPair p(32, 0);
    if (p.slot & 1U) p.step(0, false);
    EmlBandMessage_s cmd = {EML_BAND_PROPOSE, 1, 15, (uint16_t)(p.slot + 24), 1};
    EmlTxTicket_s ticket;
    uint8_t frame[64];
    const uint16_t hop = (uint16_t)((p.slot >> 1) % 240);
    TEST_ASSERT_TRUE(p.t[0].prepare(p.slot, hop, frame, 64, &ticket, &cmd));
    TEST_ASSERT_TRUE(p.t[0].sent(ticket, p.slot));
    EmlRxContext_s ctx = {p.t[1].generation(), p.slot, hop};
    frame[4] ^= 1;
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, p.t[1].receive(frame, 32, ctx, p.slot));
    frame[4] ^= 1;
    // Recompute CRC around a forged sender epoch: identity/session validation still fails.
    frame[14] ^= 1;
    EmlFrameView_s view;
    EmlHeader_s header;
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderDecode(frame, &header, 28));
    TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, frame + 2, 28, frame, 64, 32));
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, p.t[1].receive(frame, 32, ctx, p.slot));
    frame[14] ^= 1;
    TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, frame + 2, 28, frame, 64, 32));
    ++ctx.generation;
    TEST_ASSERT_EQUAL(EML_RX_STALE, p.t[1].receive(frame, 32, ctx, p.slot));
    --ctx.generation;
    TEST_ASSERT_EQUAL(EML_RX_CONTROL, p.t[1].receive(frame, 32, ctx, p.slot));
    TEST_ASSERT_EQUAL_UINT32(0, p.t[1].queuedRx());
    TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameDecode(frame, 32, 32, &view));
}

void test_hysteresis_masks_and_invalid_countdown()
{
    EmlBandController c, r;
    c.begin(EML_COORDINATOR, 2, 15, 0); r.begin(EML_RESPONDER, 2, 4, 0);
    EmlBandMessage_s m;
    c.consider(0, 2000, 100, 100, 1);
    c.consider(2, 31999, 100, 100, 1); TEST_ASSERT_FALSE(c.busy());
    c.consider(4, 32000, 100, 100, 1); TEST_ASSERT_TRUE(c.busy());
    TEST_ASSERT_TRUE(c.command(4, 100, 100, 0, &m));
    TEST_ASSERT_EQUAL_UINT8(0, m.band);
    TEST_ASSERT_FALSE(r.receive(m, 4, 32000)); // failed local PLL excluded this band
    r.begin(EML_RESPONDER, 2, 15, 0);
    auto bad = m; bad.switch_slot |= 1; TEST_ASSERT_FALSE(r.receive(bad, 4, 32000));
    bad = m; bad.token = 0; TEST_ASSERT_FALSE(r.receive(bad, 4, 32000));
    bad = m; bad.switch_slot += 100; TEST_ASSERT_FALSE(r.receive(bad, 4, 32000));
    TEST_ASSERT_TRUE(r.receive(m, 4, 32000));
    bad = m; bad.op = EML_BAND_COMMIT; ++bad.token; TEST_ASSERT_FALSE(r.receive(bad, 6, 32000));
    TEST_ASSERT_FALSE(r.armed());
    // reverse-path degradation reaches the coordinator via a validated STATUS
    c.begin(EML_COORDINATOR, 0, 15, 0);
    m = {EML_BAND_STATUS, 0, 5, 3, (uint16_t)(50U << 8)};
    TEST_ASSERT_TRUE(c.receive(m, 3, 3000));
    c.consider(4, 3000, 100, 100, 0); TEST_ASSERT_TRUE(c.busy());
    TEST_ASSERT_EQUAL_UINT8(2, c.target()); // peer's PLL mask excludes band 1
}

void test_emergency_search_meets_with_different_masks_and_start_times()
{
    EmlPhyPlan_s plan = {};
    plan.count = 2; plan.coordinator_dwell_ms[0] = 30; plan.coordinator_dwell_ms[1] = 60;
    for (uint8_t cm = 1; cm < 16; ++cm) for (uint8_t rm = 1; rm < 16; ++rm)
    {
        if (!(cm & rm)) continue;
        for (uint32_t offset : {0U, 17U, 211U, 0xFFFFFFF0U})
        {
            EmlBandSearch c, r;
            const uint8_t a[] = {cm, (uint8_t)(cm ^ 15U)}, b[] = {rm, (uint8_t)(rm ^ 15U)};
            TEST_ASSERT_TRUE(c.begin(plan, EML_COORDINATOR, a, offset));
            TEST_ASSERT_TRUE(r.begin(plan, EML_RESPONDER, b, offset - 123U));
            uint32_t overlap = 0; bool met = false;
            for (uint32_t elapsed = 0; elapsed < 8000; ++elapsed)
            {
                const uint32_t now = offset + elapsed;
                c.update(now, false); r.update(now, false);
                if (c.phy() == r.phy() && c.band() == r.band()) ++overlap; else overlap = 0;
                if (overlap >= plan.coordinator_dwell_ms[c.phy()]) { met = true; break; }
            }
            TEST_ASSERT_TRUE(met);
        }
    }
    EmlBandSearch empty; uint8_t masks[] = {0,0};
    TEST_ASSERT_FALSE(empty.begin(plan, EML_COORDINATOR, masks, 0));
    TEST_ASSERT_EQUAL_UINT8(0, empty.mask());
    TEST_ASSERT_FALSE(empty.update(100000, false));
}

void setUp() {}
void tearDown() {}
int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_four_band_grid_and_sequence);
    RUN_TEST(test_multiband_configuration_is_distinct);
    RUN_TEST(test_pll_poll_timeout_stale_unstable_and_wrap);
    RUN_TEST(test_graceful_handover_retries_and_preserves_uart_arq);
    RUN_TEST(test_lost_handover_messages_have_bounded_outcomes);
    RUN_TEST(test_control_rejects_wrong_session_crc_and_stale_descriptor);
    RUN_TEST(test_hysteresis_masks_and_invalid_countdown);
    RUN_TEST(test_emergency_search_meets_with_different_masks_and_start_times);
    UNITY_END(); return 0;
}
