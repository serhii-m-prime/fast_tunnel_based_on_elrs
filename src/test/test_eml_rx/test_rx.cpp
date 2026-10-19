#include <unity.h>
#include <initializer_list>
#include "eml_rx.h"
#include "eml_mac.h"

void test_receive_gate_rejects_prearmed_and_expired_frames_across_micros_wrap()
{
    for (uint32_t start : {10000U, 0xFFFFFF00U})
    {
        EmlArmedRx_s gate = {{1, 2, 3}, start, 0, 64, true, false};
        TEST_ASSERT_FALSE(gate.accepts(start - 1400U, 4100));
        TEST_ASSERT_FALSE(gate.accepts(start - 1U, 4100));
        TEST_ASSERT_TRUE(gate.accepts(start, 4100));
        TEST_ASSERT_TRUE(gate.accepts(start + 2846U, 4100));
        TEST_ASSERT_TRUE(gate.accepts(start + 4099U, 4100));
        TEST_ASSERT_FALSE(gate.accepts(start + 4100U, 4100));
        gate.valid = false;
        TEST_ASSERT_FALSE(gate.accepts(start + 2846U, 4100));
    }
}

void test_corrupt_and_foreign_hello_do_not_consume_receive_slot()
{
    EmlTransport<1024> sender, receiver;
    const EmlLinkIds_s ids = {1,2,3};
    const EmlTransportConfig_s c = emlMacTransportConfig(EML_COORDINATOR, ids, EML_PROFILE_FSK64, 240);
    const EmlTransportConfig_s r = emlMacTransportConfig(EML_RESPONDER, ids, EML_PROFILE_FSK64, 240);
    TEST_ASSERT_TRUE(sender.begin(c, 10, 0));
    TEST_ASSERT_TRUE(receiver.begin(r, 20, 0));
    EmlArmedRx_s gate = {{receiver.generation(), 0, 0}, 10000, 0, 64, true, false};
    uint8_t bytes[64]; EmlTxTicket_s ticket;
    TEST_ASSERT_TRUE(sender.prepare(0, 0, bytes, sizeof(bytes), &ticket));
    TEST_ASSERT_TRUE(sender.sent(ticket, 0));
    bytes[10] ^= 1;
    TEST_ASSERT_TRUE(gate.accepts(12700, 4100));
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, receiver.receive(bytes, sizeof(bytes), gate.context, 1));
    TEST_ASSERT_FALSE(gate.taken);
    bytes[10] ^= 1;
    // Valid CRC but wrong link identity also leaves the gate open.
    EmlSync_s hello;
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncDecode(bytes + 2, EML_SYNC_PAYLOAD_SIZE, &hello));
    ++hello.receiver_id;
    uint8_t payload[EML_SYNC_PAYLOAD_SIZE];
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&hello, payload, sizeof(payload)));
    const EmlHeader_s header = {EML_PKT_TYPE_SYNC,7,7,EML_SYNC_PAYLOAD_SIZE,EML_FLOW_CTRL_PAUSE};
    uint8_t foreign[64];
    TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, payload, sizeof(payload), foreign, sizeof(foreign), 64));
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, receiver.receive(foreign, sizeof(foreign), gate.context, 1));
    TEST_ASSERT_FALSE(gate.taken);
    TEST_ASSERT_TRUE(gate.accepts(12846, 4100));
    TEST_ASSERT_EQUAL(EML_RX_CONTROL, receiver.receive(bytes, sizeof(bytes), gate.context, 1));
    gate.validated(gate.context);
    TEST_ASSERT_TRUE(gate.taken);
    TEST_ASSERT_FALSE(gate.accepts(12900, 4100));
}

void test_delayed_foreground_cannot_close_new_slot_or_generation()
{
    EmlArmedRx_s gate = {{2,4,3}, 10000, 0, 64, true, false};
    gate.validated({1,4,3});
    TEST_ASSERT_FALSE(gate.taken);
    gate.validated({2,2,3});
    TEST_ASSERT_FALSE(gate.taken);
    gate.validated({2,4,2});
    TEST_ASSERT_FALSE(gate.taken);
    gate.validated({2,4,3});
    TEST_ASSERT_TRUE(gate.taken);
}

void test_search_accepts_candidates_without_a_slot_clock()
{
    EmlArmedRx_s gate = {{1,0,0}, 0, EML_DESC_SEARCH, 64, true, false};
    TEST_ASSERT_TRUE(gate.accepts(0xFFFFFF00U, 4100));
    gate.validated(gate.context);
    TEST_ASSERT_FALSE(gate.taken);
    TEST_ASSERT_TRUE(gate.accepts(1000000U, 4100));
}

void setUp() {}
void tearDown() {}
int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_receive_gate_rejects_prearmed_and_expired_frames_across_micros_wrap);
    RUN_TEST(test_corrupt_and_foreign_hello_do_not_consume_receive_slot);
    RUN_TEST(test_delayed_foreground_cannot_close_new_slot_or_generation);
    RUN_TEST(test_search_accepts_candidates_without_a_slot_clock);
    return UNITY_END();
}
