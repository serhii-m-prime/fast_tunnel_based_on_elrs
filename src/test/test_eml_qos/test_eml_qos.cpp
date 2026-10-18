#include <unity.h>
#include "eml_qos.h"

static size_t mavlink2(uint8_t *out, uint8_t payloadLength, uint32_t messageId, bool signedFrame = false)
{
    const size_t length = 10U + payloadLength + 2U + (signedFrame ? 13U : 0U);
    memset(out, 0, length);
    out[0] = 0xFD;
    out[1] = payloadLength;
    out[2] = signedFrame ? 1 : 0;
    out[7] = (uint8_t)messageId;
    out[8] = (uint8_t)(messageId >> 8);
    out[9] = (uint8_t)(messageId >> 16);
    for (uint8_t i = 0; i < payloadLength; ++i) out[10U + i] = (uint8_t)(i + messageId);
    return length;
}

void test_mavlink_boundary_and_priority()
{
    EmlQosIngress<512> q;
    q.begin(EML_TRAFFIC_MAVLINK2);
    uint8_t telemetry[40], command[45], out[60];
    const size_t telemetryLength = mavlink2(telemetry, 28, 0);
    const size_t commandLength = mavlink2(command, 33, 76);
    TEST_ASSERT_TRUE(q.accept(telemetry, 7, 1));
    TEST_ASSERT_TRUE(q.accept(telemetry + 7, telemetryLength - 7, 2));
    TEST_ASSERT_TRUE(q.accept(command, commandLength, 3));

    bool high = false;
    TEST_ASSERT_EQUAL(commandLength, q.peek(out, sizeof(out), &high));
    TEST_ASSERT_TRUE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(command, out, commandLength);
    TEST_ASSERT_TRUE(q.consume(commandLength, high));
    TEST_ASSERT_EQUAL(telemetryLength, q.peek(out, sizeof(out), &high));
    TEST_ASSERT_FALSE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(telemetry, out, telemetryLength);
}

void test_long_mavlink_is_lossless_and_fragmented_at_eml_limit()
{
    EmlQosIngress<512> q;
    q.begin(EML_TRAFFIC_MAVLINK2);
    uint8_t frame[280], out[60], rebuilt[280];
    const size_t length = mavlink2(frame, 255, 33, true);
    TEST_ASSERT_EQUAL_UINT32(280, length);
    TEST_ASSERT_TRUE(q.accept(frame, length, 10));
    size_t copied = 0;
    while (copied < length)
    {
        bool high;
        const size_t n = q.peek(out, sizeof(out), &high);
        TEST_ASSERT_FALSE(high);
        TEST_ASSERT_TRUE(n > 0 && n <= EML_MAX_PAYLOAD_LEN);
        memcpy(rebuilt + copied, out, n);
        q.noteFragment(length - copied, n, high);
        TEST_ASSERT_TRUE(q.consume(n, high));
        copied += n;
    }
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, rebuilt, length);
    TEST_ASSERT_EQUAL_UINT32(4, q.stats().fragmented_frames);
}

void test_priority_never_interleaves_a_fragmented_frame()
{
    EmlQosIngress<512> q;
    q.begin(EML_TRAFFIC_MAVLINK2);
    uint8_t longFrame[280], command[45], out[60];
    const size_t longLength = mavlink2(longFrame, 255, 33, true);
    const size_t commandLength = mavlink2(command, 33, 76);
    TEST_ASSERT_TRUE(q.accept(longFrame, longLength, 1));
    bool high;
    TEST_ASSERT_EQUAL(60, q.peek(out, sizeof(out), &high));
    TEST_ASSERT_FALSE(high);
    q.noteFragment(longLength, 60, high);
    TEST_ASSERT_TRUE(q.consume(60, high));
    TEST_ASSERT_TRUE(q.accept(command, commandLength, 2));

    size_t copied = 60;
    while (copied < longLength)
    {
        const size_t remaining = q.frontSize(&high);
        const size_t n = q.peek(out, sizeof(out), &high);
        TEST_ASSERT_FALSE(high);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(longFrame + copied, out, n);
        q.noteFragment(remaining, n, high);
        TEST_ASSERT_TRUE(q.consume(n, high));
        copied += n;
    }
    TEST_ASSERT_EQUAL(commandLength, q.peek(out, sizeof(out), &high));
    TEST_ASSERT_TRUE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(command, out, commandLength);
}

void test_incomplete_candidate_times_out_without_byte_loss()
{
    EmlQosIngress<128> q;
    q.begin(EML_TRAFFIC_MAVLINK2);
    const uint8_t partial[] = {0xFD, 20, 0, 1, 2};
    uint8_t out[16];
    TEST_ASSERT_TRUE(q.accept(partial, sizeof(partial), 100));
    TEST_ASSERT_EQUAL(0, q.peek(out, sizeof(out)));
    q.flushIdle(119);
    TEST_ASSERT_EQUAL(0, q.peek(out, sizeof(out)));
    q.flushIdle(120);
    bool high;
    TEST_ASSERT_EQUAL(sizeof(partial), q.peek(out, sizeof(out), &high));
    TEST_ASSERT_FALSE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(partial, out, sizeof(partial));
    TEST_ASSERT_EQUAL_UINT32(1, q.stats().parser_timeouts);
}

void test_sbus_frame_is_high_priority_and_raw_mode_is_ordered()
{
    EmlQosIngress<256> q;
    q.begin(EML_TRAFFIC_SBUS);
    const uint8_t prefix[] = {1, 2, 3};
    uint8_t sbus[25] = {0x0F};
    sbus[24] = 0x00;
    TEST_ASSERT_TRUE(q.accept(prefix, sizeof(prefix), 1));
    TEST_ASSERT_TRUE(q.accept(sbus, sizeof(sbus), 2));
    uint8_t out[32]; bool high;
    TEST_ASSERT_EQUAL(sizeof(sbus), q.peek(out, sizeof(out), &high));
    TEST_ASSERT_TRUE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(sbus, out, sizeof(sbus));
    TEST_ASSERT_TRUE(q.consume(sizeof(sbus), high));
    TEST_ASSERT_EQUAL(sizeof(prefix), q.peek(out, sizeof(out), &high));
    TEST_ASSERT_FALSE(high);

    q.begin(EML_TRAFFIC_RAW);
    const uint8_t raw[] = {0xFD, 1, 0, 0x0F, 9, 8};
    TEST_ASSERT_TRUE(q.accept(raw, sizeof(raw), 3));
    TEST_ASSERT_EQUAL(sizeof(raw), q.peek(out, sizeof(out), &high));
    TEST_ASSERT_FALSE(high);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(raw, out, sizeof(raw));
}

void test_raw_small_batches_coalesce_without_loss()
{
    EmlQosIngress<512> q;
    q.begin(EML_TRAFFIC_RAW);
    uint8_t expected = 0, next = 0, out[60];
    size_t accepted = 0;
    // UART batches of 1..3 bytes, far more batches than packet slots
    for (uint32_t i = 0; q.free(); ++i)
    {
        uint8_t batch[3];
        const size_t n = 1U + i % 3U < q.free() ? 1U + i % 3U : q.free();
        for (size_t k = 0; k < n; ++k) batch[k] = next++;
        TEST_ASSERT_TRUE(q.accept(batch, n, i));
        accepted += n;
    }
    TEST_ASSERT_EQUAL(512, accepted);
    bool high = true;
    size_t drained = 0;
    while (size_t n = q.peek(out, sizeof(out), &high))
    {
        TEST_ASSERT_FALSE(high);
        // Every EML DATA frame is filled to the payload limit except the last
        TEST_ASSERT_TRUE(n == sizeof(out) || drained + n == accepted);
        for (size_t k = 0; k < n; ++k) TEST_ASSERT_EQUAL_UINT8(expected++, out[k]);
        q.noteFragment(q.frontSize(), n, high);
        TEST_ASSERT_TRUE(q.consume(n, high));
        drained += n;
    }
    TEST_ASSERT_EQUAL(accepted, drained);
}

void test_framing_mode_never_refuses_admitted_bytes()
{
    EmlQosIngress<512> q;
    q.begin(EML_TRAFFIC_MAVLINK2);
    uint8_t frame[12];
    const size_t length = mavlink2(frame, 0, 0); // minimal 12-byte frames, the most packets per byte
    uint8_t batch[64];
    uint32_t now = 1;
    while (q.free())
    {
        size_t n = 0;
        while (n + length <= sizeof(batch) && n + length <= q.free()) { memcpy(batch + n, frame, length); n += length; }
        if (!n) break;
        TEST_ASSERT_TRUE(q.accept(batch, n, now++));
    }
    TEST_ASSERT_EQUAL(0, q.free()); // the UART stalls instead of dropping bytes
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_mavlink_boundary_and_priority);
    RUN_TEST(test_long_mavlink_is_lossless_and_fragmented_at_eml_limit);
    RUN_TEST(test_incomplete_candidate_times_out_without_byte_loss);
    RUN_TEST(test_sbus_frame_is_high_priority_and_raw_mode_is_ordered);
    RUN_TEST(test_priority_never_interleaves_a_fragmented_frame);
    RUN_TEST(test_raw_small_batches_coalesce_without_loss);
    RUN_TEST(test_framing_mode_never_refuses_admitted_bytes);
    return UNITY_END();
}
