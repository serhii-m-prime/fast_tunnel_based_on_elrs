#include <unity.h>
#include <vector>
#include "eml_uart.h"

// Reference frames produced by pymavlink 2 (dialect common, srcSystem 51, srcComponent 68)
static const uint8_t kRadioStatusRef[] = {0xfd, 0x08, 0x00, 0x00, 0x07, 0x33, 0x44, 0x6d, 0x00, 0x00, 0xd2, 0x04, 0x38, 0x00,
                                          0xa7, 0xa5, 0x58, 0xff, 0xfd, 0x75};
static const uint8_t kNamedValueRef[] = {0xfd, 0x0e, 0x00, 0x00, 0x08, 0x33, 0x44, 0xfc, 0x00, 0x00, 0x40, 0xe2, 0x01,
                                         0x00, 0x61, 0x00, 0x00, 0x00, 0x45, 0x4d, 0x4c, 0x5f, 0x4c, 0x51, 0x7a, 0xea};
static const uint8_t kRadioStatusZeroRef[] = {0xfd, 0x01, 0x00, 0x00, 0x09, 0x33, 0x44, 0x6d, 0x00, 0x00, 0x00, 0x6d, 0x90};

void test_crc_check_value()
{
    uint16_t crc = 0xFFFF;
    for (const char *p = "123456789"; *p; ++p) crc = emlMavCrc(crc, (uint8_t)*p);
    TEST_ASSERT_EQUAL_HEX16(0x6F91, crc); // CRC-16/MCRF4XX
}

void test_frames_match_pymavlink()
{
    uint8_t out[EML_MAV_FRAME_MAX];
    const EmlRadioStatus_s s = {1234, 56, 167, 165, 88, 255, 0};
    TEST_ASSERT_EQUAL(sizeof(kRadioStatusRef), emlMavRadioStatus(s, 7, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kRadioStatusRef, out, sizeof(kRadioStatusRef));
    TEST_ASSERT_EQUAL(sizeof(kNamedValueRef), emlMavNamedValueInt(123456, "EML_LQ", 97, 8, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kNamedValueRef, out, sizeof(kNamedValueRef));
    const EmlRadioStatus_s zero = {};
    TEST_ASSERT_EQUAL(sizeof(kRadioStatusZeroRef), emlMavRadioStatus(zero, 9, out)); // one payload byte kept
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kRadioStatusZeroRef, out, sizeof(kRadioStatusZeroRef));
}

void test_sik_level_scale()
{
    TEST_ASSERT_EQUAL_UINT8(0, emlMavSikLevel(-140));
    TEST_ASSERT_EQUAL_UINT8(0, emlMavSikLevel(-127));
    TEST_ASSERT_EQUAL_UINT8(171, emlMavSikLevel(-37)); // 90 * 1.9
    TEST_ASSERT_EQUAL_UINT8(254, emlMavSikLevel(20));
}

static size_t mav2(uint8_t *out, uint8_t payloadLength, bool signedFrame, uint8_t fill)
{
    const size_t n = 12U + payloadLength + (signedFrame ? 13U : 0U);
    for (size_t i = 0; i < n; ++i) out[i] = (uint8_t)(fill + i) == 0xFD ? 0x11 : (uint8_t)(fill + i);
    out[0] = 0xFD; out[1] = payloadLength; out[2] = signedFrame ? 1 : 0;
    return n;
}

void test_boundary_tracks_v1_v2_signed_and_raw()
{
    std::vector<uint8_t> stream;
    std::vector<size_t> ends; // positions after which the tracker must be idle
    uint8_t f[300];
    stream.push_back(0x10); ends.push_back(stream.size());
    size_t n = mav2(f, 20, false, 1); stream.insert(stream.end(), f, f + n); ends.push_back(stream.size());
    n = mav2(f, 255, true, 2); stream.insert(stream.end(), f, f + n); ends.push_back(stream.size()); // 280 B wire maximum
    const uint8_t v1[] = {0xFE, 3, 0, 1, 1, 0, 0xA, 0xB, 0xC, 0x12, 0x34};
    stream.insert(stream.end(), v1, v1 + sizeof(v1)); ends.push_back(stream.size());
    stream.push_back(0x20); ends.push_back(stream.size());

    EmlMavlinkBoundary b;
    size_t e = 0;
    for (size_t i = 0; i < stream.size(); ++i)
    {
        const size_t next = b.toBoundary(stream.data() + i, stream.size() - i);
        TEST_ASSERT_EQUAL(ends[e] - i, next);
        b.feed(stream[i]);
        const bool atEnd = i + 1U == ends[e];
        TEST_ASSERT_EQUAL(atEnd, b.idle());
        if (atEnd) ++e;
    }
    TEST_ASSERT_EQUAL(ends.size(), e);
}

void test_stalled_garbage_frame_is_abandoned()
{
    EmlMavlinkBoundary b;
    const uint8_t garbage[] = {0xFD, 200};
    b.feed(garbage, sizeof(garbage), 100);
    TEST_ASSERT_FALSE(b.injectable(500));
    TEST_ASSERT_TRUE(b.injectable(1100));
}

struct FakePort
{
    std::vector<uint8_t> out;
    int space = 64;
    int availableForWrite() const { return space; }
    size_t write(const uint8_t *d, size_t n) { out.insert(out.end(), d, d + n); return n; }
};

struct FakeQueue
{
    std::vector<uint8_t> q;
    size_t head = 0;
    void lock() {}
    void unlock() {}
    size_t size() const { return q.size() - head; }
    bool peekBytes(uint8_t *d, size_t n) const { if (n > size()) return false; memcpy(d, q.data() + head, n); return true; }
    void skip(size_t n) { head += n; }
};

void test_drain_injects_only_between_frames_and_loses_nothing()
{
    FakeQueue queue;
    uint8_t f[300];
    std::vector<uint8_t> input;
    for (uint32_t k = 0; k < 40; ++k)
    {
        const size_t n = mav2(f, (uint8_t)(k * 37U % 256U), (k % 5U) == 0, (uint8_t)k);
        input.insert(input.end(), f, f + n);
        if (k % 7U == 3U) input.push_back((uint8_t)(0x30 + k)); // raw gap byte, never 0xFD/0xFE
    }
    queue.q = input;

    FakePort port;
    EmlMavlinkBoundary boundary;
    EmlUartStats_s stats;
    uint8_t telemetry[EML_MAV_FRAME_MAX];
    size_t telemetryLength = 0, injected = 0;
    uint32_t now = 0, calls = 0;
    while (queue.size() || telemetryLength)
    {
        if (!telemetryLength && (calls % 3U) == 0U && injected < 200U)
        {
            telemetryLength = emlMavRadioStatus({(uint16_t)injected, 0, 1, 2, 3, 4, 5}, (uint8_t)injected, telemetry);
            ++injected;
        }
        port.space = 20 + (int)(calls * 13U % 50U);
        emlDrainSerialTelemetry(port, queue, stats, boundary, telemetry, &telemetryLength, now, 1U + calls * 7U % 64U);
        ++calls; now += 1;
        TEST_ASSERT_TRUE(calls < 100000U);
    }

    // Split the output into frames and raw bytes; telemetry frames carry sysid 51
    std::vector<uint8_t> data;
    size_t seen = 0;
    for (size_t i = 0; i < port.out.size();)
    {
        if (port.out[i] == 0xFD)
        {
            const size_t n = 12U + port.out[i + 1] + ((port.out[i + 2] & 1U) ? 13U : 0U);
            TEST_ASSERT_TRUE(i + n <= port.out.size());
            if (port.out[i + 5] == EML_MAV_SYSID)
            {
                uint8_t ref[EML_MAV_FRAME_MAX];
                const uint8_t seq = port.out[i + 4];
                const size_t refLength = emlMavRadioStatus({(uint16_t)seq, 0, 1, 2, 3, 4, 5}, seq, ref);
                TEST_ASSERT_EQUAL(refLength, n);
                TEST_ASSERT_EQUAL_UINT8_ARRAY(ref, port.out.data() + i, n);
                ++seen;
            }
            else
                data.insert(data.end(), port.out.begin() + (long)i, port.out.begin() + (long)(i + n));
            i += n;
        }
        else
            data.push_back(port.out[i++]);
    }
    TEST_ASSERT_EQUAL(injected, seen);
    TEST_ASSERT_TRUE(injected > 10U);
    TEST_ASSERT_EQUAL(input.size(), data.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(input.data(), data.data(), input.size());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_frames_match_pymavlink);
    RUN_TEST(test_sik_level_scale);
    RUN_TEST(test_boundary_tracks_v1_v2_signed_and_raw);
    RUN_TEST(test_stalled_garbage_frame_is_abandoned);
    RUN_TEST(test_drain_injects_only_between_frames_and_loses_nothing);
    return UNITY_END();
}
