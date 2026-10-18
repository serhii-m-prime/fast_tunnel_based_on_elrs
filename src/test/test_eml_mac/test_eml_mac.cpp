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
    const EmlMacProfile_s profiles[] = {EML_PROFILE_LORA64, EML_PROFILE_LORA32, EML_PROFILE_FSK64, EML_PROFILE_FSK32, EML_PROFILE_FSK64_300K};
    for (const EmlMacProfile_s &p : profiles)
    {
        TEST_ASSERT_TRUE(emlProfileValid(p));
        const uint32_t toa = emlProfileToaUs(p);
        TEST_ASSERT_EQUAL_UINT32(p.tx_start_us + toa + p.rx_done_us, emlRxOffsetUs(p));
        TEST_ASSERT_TRUE(p.slot_us >= emlMinSlotUs(p));
        // The peer frame ends (with responder bias and guard) before the next own boundary
        TEST_ASSERT_TRUE(p.tx_start_us + toa + p.rx_done_us + EML_RESPONDER_BIAS_US + EML_PREP_MARGIN_US < p.slot_us);
        // A responder late by half the guard still opens RX inside the coordinator's preamble
        TEST_ASSERT_TRUE(EML_GUARD_US / 2U < emlRxLateToleranceUs(p));
        EmlMacProfile_s tight = p;
        tight.slot_us = emlMinSlotUs(p) - 1U;
        TEST_ASSERT_FALSE(emlProfileValid(tight));
    }
    TEST_ASSERT_EQUAL_UINT32(250U + 5U * 128U, emlRxLateToleranceUs(EML_PROFILE_LORA64));
    TEST_ASSERT_EQUAL_UINT32(200U + 6U * 32U, emlRxLateToleranceUs(EML_PROFILE_FSK64));
    // Net payload per direction: one frame per slot pair
    TEST_ASSERT_EQUAL_UINT32(1714, (uint32_t)(60ULL * 1000000ULL / (2ULL * EML_PROFILE_LORA64.slot_us)));
    TEST_ASSERT_EQUAL_UINT32(1346, (uint32_t)(28ULL * 1000000ULL / (2ULL * EML_PROFILE_LORA32.slot_us)));
    TEST_ASSERT_EQUAL_UINT32(7894, (uint32_t)(60ULL * 1000000ULL / (2ULL * EML_PROFILE_FSK64.slot_us)));
    TEST_ASSERT_EQUAL_UINT32(5000, (uint32_t)(28ULL * 1000000ULL / (2ULL * EML_PROFILE_FSK32.slot_us)));
}

// ---- P5.2/P5.4/P5.7: FSK profiles ------------------------------------------------------

void test_fsk_toa_and_fifo_fit()
{
    TEST_ASSERT_EQUAL_UINT32(2496, emlProfileToaUs(EML_PROFILE_FSK64)); // (10 + 4 + 64) B * 32 us
    TEST_ASSERT_EQUAL_UINT32(1472, emlProfileToaUs(EML_PROFILE_FSK32));
    TEST_ASSERT_EQUAL_UINT32(2080, emlProfileToaUs(EML_PROFILE_FSK64_300K));
    TEST_ASSERT_EQUAL_UINT32(2560, emlFskToaUs(250000, 10, 4, 64, 2)); // a hardware CRC would cost 64 us
    for (uint16_t n = 1; n <= 64; n++)
        TEST_ASSERT_EQUAL_UINT32((10U + 4U + n) * 32U, emlFskToaUs(250000, 10, 4, n, 0));
    // Fixed length without length/address/CRC bytes: N alone fills the 64-byte FIFO
    TEST_ASSERT_TRUE(EML_PROFILE_FSK64.frame_size <= EML_FSK_FIFO_BYTES);
    EmlMacProfile_s big = EML_PROFILE_FSK64;
    big.frame_size = 65;
    TEST_ASSERT_FALSE(emlFskWithinDatasheet(big));
}

void test_fsk_profile_limits()
{
    TEST_ASSERT_TRUE(emlFskWithinDatasheet(EML_PROFILE_FSK64));
    // h = 2*Fdev/BR and the occupied single sideband Fdev + BR/2 of the chosen profile
    TEST_ASSERT_EQUAL_UINT32(215000, EML_PROFILE_FSK64.fdev_hz + EML_PROFILE_FSK64.bitrate / 2U);
    TEST_ASSERT_TRUE(4U * EML_PROFILE_FSK64.fdev_hz >= EML_PROFILE_FSK64.bitrate);

    EmlMacProfile_s p = EML_PROFILE_FSK64;
    p.fdev_hz = 125000;                 // AGENTS range low end: exactly at the filter edge, no offset margin
    TEST_ASSERT_TRUE(emlFskWithinDatasheet(p));
    p.fdev_hz = 150000;                 // Fdev + BR/2 = 275 kHz > 250 kHz
    TEST_ASSERT_FALSE(emlFskWithinDatasheet(p));
    p.fdev_hz = 50000;                  // h = 0.4 < 0.5
    TEST_ASSERT_FALSE(emlFskWithinDatasheet(p));
    p = EML_PROFILE_FSK64; p.rxbw_hz = 200000; // filter narrower than the signal
    TEST_ASSERT_FALSE(emlFskWithinDatasheet(p));
    p = EML_PROFILE_FSK64; p.preamble = 3;
    TEST_ASSERT_FALSE(emlFskWithinDatasheet(p));

    // P5.7: 300 kbit/s is inside the datasheet but only as an explicit experiment
    p = EML_PROFILE_FSK64_300K;
    TEST_ASSERT_TRUE(emlFskWithinDatasheet(p));
    TEST_ASSERT_TRUE(emlProfileValid(p));
    p.experimental = false;
    TEST_ASSERT_FALSE(emlProfileValid(p));
    // 500 / 1000 kbit/s: above the 300 kbit/s FSK limit, and h >= 0.5 needs Fdev + BR/2 of
    // 375 / 750 kHz, beyond the 250 kHz maximum RX filter
    const uint32_t rates[] = {500000, 1000000};
    for (uint32_t br : rates)
    {
        p = EML_PROFILE_FSK64_300K;
        p.bitrate = br;
        p.fdev_hz = br / 4U;
        TEST_ASSERT_FALSE(emlFskWithinDatasheet(p));
        TEST_ASSERT_FALSE(emlProfileValid(p));
        TEST_ASSERT_TRUE(p.fdev_hz + br / 2U > 250000U);
    }
}

void test_fsk_sync_word_and_configuration_ids()
{
    const uint8_t uid[6] = {1, 2, 3, 4, 5, 6};
    uint8_t a[4], b[4];
    emlFskSyncWord(uid, a);
    emlFskSyncWord(uid, b);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(a, b, 4);
    for (uint8_t i = 0; i < 4; i++) TEST_ASSERT_NOT_EQUAL(0, a[i]);
    uint8_t other[6] = {1, 2, 3, 4, 5, 7};
    emlFskSyncWord(other, b);
    TEST_ASSERT_TRUE(memcmp(a, b, 4) != 0);
    // zero bytes are replaced
    for (uint32_t seed = 0; seed < 2000; seed++)
    {
        uint8_t u[6] = {(uint8_t)seed, (uint8_t)(seed >> 8), 0, 0, 0, 0};
        emlFskSyncWord(u, b);
        for (uint8_t i = 0; i < 4; i++) TEST_ASSERT_NOT_EQUAL(0, b[i]);
    }

    const EmlFhssRecord_s fhss = {903500000, 926900000, 40, 0x1234, 240};
    const uint32_t lora = emlDeriveLinkIds(uid, EML_PROFILE_LORA64, fhss).configuration;
    const uint32_t fsk = emlDeriveLinkIds(uid, EML_PROFILE_FSK64, fhss).configuration;
    const uint32_t fsk300 = emlDeriveLinkIds(uid, EML_PROFILE_FSK64_300K, fhss).configuration;
    TEST_ASSERT_NOT_EQUAL(lora, fsk);
    TEST_ASSERT_NOT_EQUAL(fsk, fsk300);
    EmlMacProfile_s p = EML_PROFILE_FSK64; p.fdev_hz += 1000;
    TEST_ASSERT_NOT_EQUAL(fsk, emlDeriveLinkIds(uid, p, fhss).configuration);
    const EmlTransportConfig_s c = emlMacTransportConfig(EML_COORDINATOR, emlDeriveLinkIds(uid, EML_PROFILE_FSK64, fhss), EML_PROFILE_FSK64, 240);
    TEST_ASSERT_TRUE(EmlTransport<1024>::validConfig(c));
    TEST_ASSERT_EQUAL_UINT8(EML_PHY_FSK_250, c.phy_profile);
    TEST_ASSERT_EQUAL_UINT16(395, c.link_timeout_slots);
}

// ---- P5.5: PHY selection and link quality -------------------------------------------------

void test_phy_plan_dwell()
{
    const EmlMacProfile_s order[] = {EML_PROFILE_FSK64, EML_PROFILE_LORA64};
    const EmlPhyPlan_s plan = emlPhyPlan(order, 2, 40);
    TEST_ASSERT_EQUAL_UINT32(304000, emlRendezvousPeriodUs(EML_PROFILE_FSK64, 40));
    TEST_ASSERT_EQUAL_UINT32(1400000, emlRendezvousPeriodUs(EML_PROFILE_LORA64, 40));
    TEST_ASSERT_EQUAL_UINT32(639, plan.coordinator_dwell_ms[0]);   // 2 x 304 ms + 8 slots
    TEST_ASSERT_EQUAL_UINT32(2940, plan.coordinator_dwell_ms[1]);
    TEST_ASSERT_EQUAL_UINT32(639 + 2940 + 639, plan.responder_dwell_ms[0]);
    TEST_ASSERT_EQUAL_UINT32(639 + 2940 + 2940, plan.responder_dwell_ms[1]);
    const EmlPhyPlan_s single = emlPhyPlan(order, 1, 40);
    EmlPhySelector s;
    s.begin(single, EML_COORDINATOR, 0);
    TEST_ASSERT_FALSE(s.update(100000, false, true, false));
    TEST_ASSERT_EQUAL_UINT8(0, s.index());
}

void test_phy_selector_rules()
{
    const EmlMacProfile_s order[] = {EML_PROFILE_FSK64, EML_PROFILE_LORA64};
    const EmlPhyPlan_s plan = emlPhyPlan(order, 2, 40);
    EmlPhySelector c;
    c.begin(plan, EML_COORDINATOR, 1000);
    TEST_ASSERT_FALSE(c.update(1000 + 638, false, true, false));
    TEST_ASSERT_FALSE(c.update(1000 + 700, false, false, false)); // handshake in progress: stay
    TEST_ASSERT_TRUE(c.update(1000 + 701, false, true, false));
    TEST_ASSERT_EQUAL_UINT8(1, c.index());
    TEST_ASSERT_EQUAL_UINT8(EML_PHY_LORA_500_SF6_CR45, c.profile().phy_profile);
    // A session holds the PHY indefinitely, its loss returns to the preferred PHY
    TEST_ASSERT_FALSE(c.update(100000, true, false, false));
    TEST_ASSERT_FALSE(c.update(200000, true, false, false));
    TEST_ASSERT_TRUE(c.update(200001, false, true, true));
    TEST_ASSERT_EQUAL_UINT8(0, c.index());
    TEST_ASSERT_FALSE(c.update(200002, false, true, true)); // already preferred
    // millis() wrap
    c.begin(plan, EML_COORDINATOR, 0xFFFFFF00UL);
    TEST_ASSERT_FALSE(c.update(0x00000100UL, false, true, false));
    TEST_ASSERT_TRUE(c.update(0xFFFFFF00UL + 639U, false, true, false));
}

// Abstract timeline: each side cycles with its own phase; the pair locks at the first
// rendezvous instant of a working PHY while both are on it. The plan must guarantee a
// meeting within one responder cycle plus one coordinator cycle.
static uint32_t meetTime(const EmlPhyPlan_s &plan, const EmlMacProfile_s *order, bool fskWorks,
                         uint32_t coordinatorStart, uint32_t responderStart, uint32_t rendezvousPhase, uint8_t *phy)
{
    EmlPhySelector c, r;
    c.begin(plan, EML_COORDINATOR, coordinatorStart);
    r.begin(plan, EML_RESPONDER, responderStart);
    uint32_t coordinatorEntered = coordinatorStart;
    const uint32_t start = coordinatorStart > responderStart ? coordinatorStart : responderStart;
    for (uint32_t t = start; t < start + 60000U; t++)
    {
        if (c.update(t, false, true, false)) coordinatorEntered = t;
        r.update(t, false, true, false);
        const uint8_t i = c.index();
        const bool works = order[i].phy_profile != EML_PHY_FSK_250 || fskWorks;
        const uint32_t period = emlRendezvousPeriodUs(order[i], 40) / 1000U;
        if (works && i == r.index() && t > responderStart && (t - coordinatorEntered + rendezvousPhase) % period == 0)
        {
            *phy = order[i].phy_profile;
            return t - start;
        }
    }
    return 0xFFFFFFFFUL;
}

void test_phy_selector_meets_on_first_working_phy()
{
    const EmlMacProfile_s order[] = {EML_PROFILE_FSK64, EML_PROFILE_LORA64};
    const EmlPhyPlan_s plan = emlPhyPlan(order, 2, 40);
    const uint32_t bound = plan.responder_dwell_ms[0] + plan.responder_dwell_ms[1];
    uint32_t worstFsk = 0, worstLora = 0;
    for (uint32_t offset = 0; offset < 12000; offset += 137)
    {
        for (uint32_t phase = 0; phase < 1400; phase += 211)
        {
            uint8_t phy = 0;
            uint32_t t = meetTime(plan, order, true, 5000 + offset, 5000, phase % 304, &phy);
            TEST_ASSERT_TRUE(t <= bound);
            if (phy == EML_PHY_FSK_250 && t > worstFsk) worstFsk = t;
            t = meetTime(plan, order, false, 5000, 5000 + offset, phase, &phy);
            TEST_ASSERT_TRUE(t <= bound);
            TEST_ASSERT_EQUAL_UINT8(EML_PHY_LORA_500_SF6_CR45, phy);
            if (t > worstLora) worstLora = t;
        }
    }
    // Both start together (power-on of a pair): FSK is found within its first dwell
    uint8_t phy = 0;
    TEST_ASSERT_TRUE(meetTime(plan, order, true, 0, 0, 100, &phy) <= plan.coordinator_dwell_ms[0]);
    TEST_ASSERT_EQUAL_UINT8(EML_PHY_FSK_250, phy);
    TEST_ASSERT_TRUE(worstLora <= bound);
    TEST_ASSERT_TRUE(worstFsk > 0);
}

void test_link_quality_window()
{
    EmlLinkQuality q;
    q.reset(10);
    TEST_ASSERT_EQUAL_UINT8(0, q.lq());
    for (uint32_t pair = 10; pair < 210; pair++)
    {
        if (pair % 4 != 0)
        {
            q.received(pair);                 // 75 %
            q.received(pair);                 // duplicate marks are harmless
        }
        q.finalizeBefore(pair);               // pair itself still open
    }
    q.finalizeBefore(210);
    TEST_ASSERT_EQUAL_UINT8(100, q.samples());
    TEST_ASSERT_EQUAL_UINT8(75, q.lq());
    q.received(5);                            // far late: ignored
    q.finalizeBefore(209);                    // slightly behind: no-op
    TEST_ASSERT_EQUAL_UINT8(75, q.lq());
    // 50 lost pairs push the window to 25 % of the last 100... (50 x 75% + 50 x 0) / 100
    q.finalizeBefore(260);
    TEST_ASSERT_EQUAL_UINT8(37, q.lq());
    // a long gap clears everything to 0 %
    q.finalizeBefore(1000);
    TEST_ASSERT_EQUAL_UINT8(0, q.lq());
    for (uint32_t pair = 1000; pair < 1100; pair++) { q.received(pair); q.finalizeBefore(pair + 1); }
    TEST_ASSERT_EQUAL_UINT8(100, q.lq());
    // counter wrap resets instead of stalling
    q.finalizeBefore(3);
    q.received(3); q.finalizeBefore(4);
    TEST_ASSERT_EQUAL_UINT8(100, q.lq());
    TEST_ASSERT_EQUAL_UINT8(1, q.samples());
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
    const EmlMacProfile_s profiles[] = {EML_PROFILE_LORA64, EML_PROFILE_LORA32, EML_PROFILE_FSK64, EML_PROFILE_FSK32};
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
    RUN_TEST(test_fsk_toa_and_fifo_fit);
    RUN_TEST(test_fsk_profile_limits);
    RUN_TEST(test_fsk_sync_word_and_configuration_ids);
    RUN_TEST(test_phy_plan_dwell);
    RUN_TEST(test_phy_selector_rules);
    RUN_TEST(test_phy_selector_meets_on_first_working_phy);
    RUN_TEST(test_link_quality_window);
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
