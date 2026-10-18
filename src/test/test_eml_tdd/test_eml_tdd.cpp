#include <unity.h>
#include "../support/eml_tdd_sim.h"
#include "eml_mailbox.h"

static_assert(sizeof(EmlTransport<1024,4>) + 2 * sizeof(EmlMailbox) < 3600, "RX transport/mailbox budget");

void test_lease_owners_fairness_and_wrap()
{
    for (uint32_t start : {32U, 65528U, 0xFFFFFFF8U}) for (uint8_t mode : {1,2})
    {
        const EmlTddPlan_s plan = {{{start, start + 64U, 1, mode}, {0,0,0,0}}};
        TEST_ASSERT_EQUAL(EML_TDD_SYMMETRIC, emlTddMode(start - 1U, plan));
        TEST_ASSERT_EQUAL(EML_TDD_SYMMETRIC, emlTddMode(start + 64U, plan));
        unsigned count[2] = {};
        for (uint32_t i = 0; i < 64; ++i)
        {
            const uint32_t slot = start + i;
            const auto owner = emlTddOwner(slot, plan); ++count[owner];
            TEST_ASSERT_TRUE(emlTddNextOwn(slot, EML_COORDINATOR, plan) - slot <= 4);
            TEST_ASSERT_TRUE(emlTddNextOwn(slot, EML_RESPONDER, plan) - slot <= 4);
            if (!(slot & 3U)) TEST_ASSERT_EQUAL(EML_COORDINATOR, owner);
            if ((slot & 3U) == 3) TEST_ASSERT_EQUAL(EML_RESPONDER, owner);
        }
        TEST_ASSERT_EQUAL_UINT32(mode == 1 ? 48 : 16, count[0]);
        TEST_ASSERT_EQUAL_UINT32(mode == 2 ? 48 : 16, count[1]);
    }
}

void test_compact_ack_vectors_crc_slot_and_mailbox()
{
    const EmlHeader_s header = {EML_PKT_TYPE_KEEPALIVE, 7, 3, 0, EML_FLOW_CTRL_HIGH_WATER};
    uint8_t bytes[8] = {}; EmlFrameView_s view;
    TEST_ASSERT_TRUE(emlAckEncode(header, bytes, 4));
    TEST_ASSERT_TRUE(emlAckDecode(bytes, 4, &view));
    TEST_ASSERT_EQUAL(3, view.header.ack_num);
    for (unsigned b = 0; b < 32; ++b)
    { bytes[b / 8] ^= 1U << (b % 8); TEST_ASSERT_FALSE(emlAckDecode(bytes,4,&view)); bytes[b / 8] ^= 1U << (b % 8); }
    TEST_ASSERT_FALSE(emlAckDecode(bytes,3,&view));
    EmlMailbox box; EmlRadioDescriptor_s d;
    TEST_ASSERT_TRUE(box.push(bytes,4,{1,8,0})); TEST_ASSERT_TRUE(box.pop(&d)); TEST_ASSERT_EQUAL(4,d.length);
    for (uint32_t slot = 0; slot < 512; ++slot) if (emlShortAckSlot(slot))
    {
        TEST_ASSERT_TRUE((slot & 3U) == 0 || (slot & 3U) == 3);
    }
    EmlMacProfile_s shortProfile = EML_PROFILE_FSK64; shortProfile.frame_size = 4;
    TEST_ASSERT_EQUAL_UINT32(576, emlProfileToaUs(shortProfile));
    TEST_ASSERT_TRUE(emlMinSlotUs(shortProfile) < EML_PROFILE_FSK32.slot_us);
    shortProfile=EML_PROFILE_FSK64_300K; shortProfile.frame_size=4;
    TEST_ASSERT_EQUAL_UINT32(481,emlProfileToaUs(shortProfile));
    TEST_ASSERT_EQUAL_UINT32(150,emlTddEmergencyMs(EML_PROFILE_FSK64,true));
    TEST_ASSERT_EQUAL_UINT32(231,emlTddEmergencyMs(EML_PROFILE_LORA64,true));
    TEST_ASSERT_EQUAL_UINT32(150,emlTddEmergencyMs(EML_PROFILE_LORA64,false));
}

void test_window_four_lost_head_reorders_once_and_acks_cumulatively()
{
    EmlTddSimulation<4> sim(EML_PROFILE_FSK64, true, true); sim.establish();
    while (sim.slot & 1U) sim.step();
    TEST_ASSERT_EQUAL_UINT32(64, sim.feed(0,64));
    TEST_ASSERT_EQUAL_UINT32(64, sim.feed(0,64));
    TEST_ASSERT_EQUAL_UINT32(64, sim.feed(0,64));
    TEST_ASSERT_EQUAL_UINT32(48, sim.feed(0,48));
    uint8_t bytes[4][64]; EmlTxTicket_s tickets[4];
    const uint32_t base = sim.slot;
    // Drop DATA0, deliver 1..3 while no ACK is available to the sender.
    for (uint8_t i = 0; i < 4; ++i)
    {
        const uint32_t slot = base + 2U * i;
        TEST_ASSERT_TRUE(sim.transport[0].prepare(slot,0,bytes[i],64,&tickets[i]));
        TEST_ASSERT_TRUE(sim.transport[0].sent(tickets[i],slot));
        if (i) TEST_ASSERT_EQUAL(EML_RX_ACCEPTED, sim.transport[1].receive(bytes[i],64,{sim.transport[1].generation(),slot,0},slot));
    }
    TEST_ASSERT_EQUAL(4,sim.transport[0].inFlight());
    TEST_ASSERT_EQUAL(0,sim.transport[1].queuedRx());
    TEST_ASSERT_EQUAL(7,sim.transport[1].acknowledgedRxSequence());
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,sim.transport[1].receive(bytes[0],64,{sim.transport[1].generation(),base+8U,0},base+8U));
    TEST_ASSERT_EQUAL(240,sim.transport[1].queuedRx());
    TEST_ASSERT_EQUAL(3,sim.transport[1].acknowledgedRxSequence());
    EmlTxTicket_s ackTicket; uint8_t ack[64];
    TEST_ASSERT_TRUE(sim.transport[1].prepare(base+9U,0,ack,64,&ackTicket));
    TEST_ASSERT_TRUE(sim.transport[1].sent(ackTicket,base+9U));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,sim.transport[0].receive(ack,64,{sim.transport[0].generation(),base+9U,0},base+9U));
    TEST_ASSERT_EQUAL(0,sim.transport[0].queuedTx());
    TEST_ASSERT_EQUAL(4,sim.transport[0].nextTxSequence());
    TEST_ASSERT_EQUAL(EML_RX_DUPLICATE,sim.transport[1].receive(bytes[0],64,{sim.transport[1].generation(),base+10U,0},base+10U));
    TEST_ASSERT_EQUAL(240,sim.transport[1].queuedRx());
}

template <uint8_t Window>
void streamCase(const EmlMacProfile_s &profile, bool adaptive, unsigned loss, uint32_t start)
{
    EmlTddSimulation<Window> sim(profile,adaptive,true,start); sim.establish(); sim.loss=loss;
    const size_t targets[] = {8193,4097};
    for (unsigned i=0;i<16000 && (sim.delivered[0]<targets[0] || sim.delivered[1]<targets[1]);++i)
    {
        for (uint8_t r=0;r<2;++r) if (sim.accepted[r]<targets[r]) sim.feed(r,std::min((size_t)64,targets[r]-sim.accepted[r]));
        sim.step(i%11<7); // long, bounded UART pauses
    }
    for (uint8_t r=0;r<2;++r)
    {
        TEST_ASSERT_EQUAL_UINT32(targets[r],sim.delivered[r]);
        TEST_ASSERT_EQUAL_UINT32(0,sim.transport[r].stats().discarded_tx);
        TEST_ASSERT_EQUAL_UINT32(0,sim.transport[r].stats().session_resets);
    }
}

void test_window_sizes_binary_streams_losses_backpressure_and_wrap()
{
    for (auto p : {EML_PROFILE_FSK64, EML_PROFILE_FSK32, EML_PROFILE_LORA64, EML_PROFILE_LORA32})
    {
        streamCase<1>(p,false,12,0);
        streamCase<2>(p,true,12,65520U);
        streamCase<3>(p,true,12,0xFFFFFFF0U);
        streamCase<4>(p,true,12,0xFFFFFFF0U);
    }
}

void test_lost_schedule_phases_expire_without_clock_or_session_reset()
{
    for (uint8_t op : {EML_TDD_PROPOSE,EML_TDD_ACK,EML_TDD_COMMIT})
    {
        EmlTddSimulation<4> sim(EML_PROFILE_FSK64,true,true); sim.establish();
        sim.drop_op=op; sim.drop_always=true;
        for (unsigned i=0;i<500;++i) { sim.feed(0,64); sim.step(); }
        TEST_ASSERT_TRUE(sim.delivered[0]>2000);
        TEST_ASSERT_EQUAL_UINT32(0,sim.transport[0].stats().session_resets+sim.transport[1].stats().session_resets);
        const uint32_t stop=sim.slot;
        // Simulate losing all future control; any existing mode must expire.
        sim.scheduler[0].begin(EML_COORDINATOR,false,stop);
        for (unsigned i=0;i<100;++i) sim.step();
        TEST_ASSERT_EQUAL(EML_TDD_SYMMETRIC,emlTddMode(sim.slot,sim.scheduler[1].plan()));
    }
}

void test_quality_counts_expected_slots_instead_of_pairs()
{
    const EmlTddPlan_s plan={{{0,256,1,EML_TDD_COORDINATOR},{0,0,0,0}}};
    EmlLinkQuality q; q.reset(0);
    for(uint32_t s=0;s<240;++s)
    {
        if(emlTddOwner(s,plan)==EML_RESPONDER) q.received(s);
        q.finalizeExpectedBefore(s, [&](uint32_t slot){return emlTddOwner(slot,plan)==EML_RESPONDER;});
    }
    TEST_ASSERT_EQUAL_UINT8(100,q.lq());
}

void test_uart_drain_releases_buffered_head_after_multiple_gaps()
{
    EmlTddSimulation<4,128> sim(EML_PROFILE_FSK64,true,false); sim.establish();
    uint32_t slot=sim.slot+2U; uint8_t payload[60]={}, frame[64];
    auto receive=[&](uint8_t seq,uint8_t n)
    {
        const EmlHeader_s header={EML_PKT_TYPE_DATA,seq,7,n,EML_FLOW_CTRL_NORMAL};
        emlFrameEncode(&header,payload,n,frame,64,64);
        slot+=2U;
        return sim.transport[1].receive(frame,64,{sim.transport[1].generation(),slot,0},slot);
    };
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,receive(0,60));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,receive(4,4));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,receive(1,60));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,receive(2,4));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,receive(3,4));
    TEST_ASSERT_EQUAL(128,sim.transport[1].queuedRx());
    TEST_ASSERT_EQUAL(3,sim.transport[1].acknowledgedRxSequence());
    TEST_ASSERT_TRUE(sim.transport[1].consumeUart(60));
    TEST_ASSERT_EQUAL(4,sim.transport[1].acknowledgedRxSequence());
    TEST_ASSERT_EQUAL(72,sim.transport[1].queuedRx());
    TEST_ASSERT_EQUAL(EML_RX_DUPLICATE,receive(4,4));
    TEST_ASSERT_EQUAL(72,sim.transport[1].queuedRx());
}

void test_cumulative_ack_rejects_unsent_tail_and_half_space()
{
    EmlTddSimulation<4> sim(EML_PROFILE_FSK64,true,false); sim.establish();
    for(unsigned i=0;i<4;++i) sim.feed(0,60);
    uint8_t bytes[64]; EmlTxTicket_s ticket;
    const uint32_t base=sim.slot;
    for(unsigned i=0;i<4;++i)
    {
        TEST_ASSERT_TRUE(sim.transport[0].prepare(base+2*i,0,bytes,64,&ticket));
        if(i<3) TEST_ASSERT_TRUE(sim.transport[0].sent(ticket,base+2*i));
    }
    EmlHeader_s header={EML_PKT_TYPE_KEEPALIVE,7,3,0,EML_FLOW_CTRL_NORMAL};
    emlFrameEncode(&header,nullptr,0,bytes,64,64);
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,sim.transport[0].receive(bytes,64,{sim.transport[0].generation(),base+9,0},base+9));
    TEST_ASSERT_EQUAL(240,sim.transport[0].queuedTx()); // DATA3 was prepared, never sent
    TEST_ASSERT_TRUE(sim.transport[0].sent(ticket,base+6));
    header.ack_num=4; // five entries, outside the entire W=4 window
    emlFrameEncode(&header,nullptr,0,bytes,64,64);
    sim.transport[0].receive(bytes,64,{sim.transport[0].generation(),base+11,0},base+11);
    TEST_ASSERT_EQUAL(240,sim.transport[0].queuedTx());
    header.ack_num=3;
    emlFrameEncode(&header,nullptr,0,bytes,64,64);
    sim.transport[0].receive(bytes,64,{sim.transport[0].generation(),base+13,0},base+13);
    TEST_ASSERT_EQUAL(0,sim.transport[0].queuedTx());
    TEST_ASSERT_EQUAL(EML_RX_STALE,sim.transport[0].receive(bytes,64,{sim.transport[0].generation(),base+15,0},base+23));
}

void test_late_or_mismatched_schedule_control_cannot_arm()
{
    EmlTddController r; r.begin(EML_RESPONDER,true,65520U);
    EmlBandMessage_s m={EML_TDD_PROPOSE,EML_TDD_COORDINATOR,EML_TDD_LEASE_SLOTS,8,7};
    TEST_ASSERT_TRUE(r.receive(m,65520U)); // common switch = 65544 across 16-bit wrap
    auto bad=m; bad.op=EML_TDD_COMMIT; ++bad.token;
    TEST_ASSERT_FALSE(r.receive(bad,65524U));
    bad=m; bad.op=EML_TDD_COMMIT; bad.band=EML_TDD_RESPONDER;
    TEST_ASSERT_FALSE(r.receive(bad,65524U));
    m.op=EML_TDD_COMMIT;
    TEST_ASSERT_FALSE(r.receive(m,65544U)); // deadline already reached
    TEST_ASSERT_EQUAL(EML_TDD_SYMMETRIC,emlTddMode(65544U,r.plan()));
    TEST_ASSERT_TRUE(r.receive(m,65524U));
    TEST_ASSERT_EQUAL(EML_TDD_COORDINATOR,emlTddMode(65544U,r.plan()));
    TEST_ASSERT_EQUAL(EML_TDD_SYMMETRIC,emlTddMode(65608U,r.plan()));
}

void test_schedule_control_piggybacks_ack_only_for_current_session()
{
    EmlTddSimulation<4> sim(EML_PROFILE_FSK64,true,false); sim.establish();
    const uint32_t slot=(sim.slot+3U)&~3UL;
    sim.feed(0,1);
    uint8_t frame[64]; EmlTxTicket_s ticket;
    TEST_ASSERT_TRUE(sim.transport[0].prepare(slot,0,frame,64,&ticket));
    TEST_ASSERT_TRUE(sim.transport[0].sent(ticket,slot));
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED,sim.transport[1].receive(frame,64,{sim.transport[1].generation(),slot,0},slot));
    const EmlBandMessage_s status={EML_TDD_STATUS,0,0,(uint16_t)(slot+3),0};
    TEST_ASSERT_TRUE(sim.transport[1].prepare(slot+3,0,frame,64,&ticket,&status));
    TEST_ASSERT_TRUE(sim.transport[1].sent(ticket,slot+3));
    EmlFrameView_s view; TEST_ASSERT_EQUAL(EML_FRAME_OK,emlFrameDecode(frame,64,64,&view));
    TEST_ASSERT_EQUAL(0,view.header.ack_num);
    const EmlHeader_s header=view.header;
    frame[14]^=1; // correct CRC, foreign sender epoch
    emlFrameEncode(&header,frame+2,28,frame,64,64);
    TEST_ASSERT_EQUAL(EML_RX_REJECTED,sim.transport[0].receive(frame,64,{sim.transport[0].generation(),slot+3,0},slot+3));
    TEST_ASSERT_EQUAL(1,sim.transport[0].queuedTx());
    frame[14]^=1;
    emlFrameEncode(&header,frame+2,28,frame,64,64);
    TEST_ASSERT_EQUAL(EML_RX_CONTROL,sim.transport[0].receive(frame,64,{sim.transport[0].generation(),slot+3,0},slot+3));
    TEST_ASSERT_EQUAL(0,sim.transport[0].queuedTx());
}

void setUp() {} void tearDown() {}
int main(int,char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_lease_owners_fairness_and_wrap);
    RUN_TEST(test_compact_ack_vectors_crc_slot_and_mailbox);
    RUN_TEST(test_window_four_lost_head_reorders_once_and_acks_cumulatively);
    RUN_TEST(test_window_sizes_binary_streams_losses_backpressure_and_wrap);
    RUN_TEST(test_lost_schedule_phases_expire_without_clock_or_session_reset);
    RUN_TEST(test_quality_counts_expected_slots_instead_of_pairs);
    RUN_TEST(test_uart_drain_releases_buffered_head_after_multiple_gaps);
    RUN_TEST(test_cumulative_ack_rejects_unsent_tail_and_half_space);
    RUN_TEST(test_late_or_mismatched_schedule_control_cannot_arm);
    RUN_TEST(test_schedule_control_piggybacks_ack_only_for_current_session);
    UNITY_END(); return 0;
}
