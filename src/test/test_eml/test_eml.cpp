#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "eml_frame.h"
#include "eml_sync.h"

static const uint8_t frameSizes[] = {EML_FRAME_SIZE_32, EML_FRAME_SIZE_64};

static void assertHeader(const EmlHeader_s &expected, const EmlHeader_s &actual)
{
    TEST_ASSERT_EQUAL_UINT8(expected.pkt_type, actual.pkt_type);
    TEST_ASSERT_EQUAL_UINT8(expected.seq_num, actual.seq_num);
    TEST_ASSERT_EQUAL_UINT8(expected.ack_num, actual.ack_num);
    TEST_ASSERT_EQUAL_UINT8(expected.payload_len, actual.payload_len);
    TEST_ASSERT_EQUAL_UINT8(expected.flow_ctrl, actual.flow_ctrl);
}

static void assertDecodeError(const uint8_t *frame, size_t length, size_t frameSize, EmlFrameStatus_e status)
{
    const EmlFrameView_s sentinel = {{EML_PKT_TYPE_DATA, 4, 3, 2, EML_FLOW_CTRL_PAUSE}, frame};
    EmlFrameView_s out = sentinel;
    TEST_ASSERT_EQUAL(status, emlFrameDecode(frame, length, frameSize, &out));
    assertHeader(sentinel.header, out.header);
    TEST_ASSERT_EQUAL_PTR(sentinel.payload, out.payload);
}

void test_frame_golden_vectors()
{
    // These constants are independent of the encoder; CRCs match Python crc_hqx.
    const struct
    {
        uint8_t length;
        uint8_t protectedBytes[62];
        uint8_t crc[2];
    } vectors[] = {
        {2, {0xB6, 0xC0}, {0x70, 0xE8}},
        {3, {0x01, 0x01, 0x00}, {0xC8, 0x9D}},
        {5, {0xE1, 0x03, 0x00, 0xFF, 0x7E}, {0x87, 0x2B}},
        {14, {0xFC, 0x0C, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, {0x64, 0x20}},
        {30, {0x4D, 0x5C, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
              14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27}, {0x2A, 0x1A}},
        {62, {0xE1, 0x3C, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
              14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
              30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45,
              46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59}, {0xFC, 0x65}}
    };
    for (const auto &v : vectors)
    {
        for (uint8_t frameSize : frameSizes)
        {
            if (v.length > frameSize - EML_CRC_SIZE)
                continue;
            uint8_t golden[64] = {};
            memcpy(golden, v.protectedBytes, v.length);
            memcpy(golden + frameSize - 2U, v.crc, 2U);
            EmlHeader_s header = {};
            TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderDecode(golden, &header, emlMaxPayloadForFrameSize(frameSize)));
            uint8_t encoded[64];
            memset(encoded, 0xA5, sizeof(encoded));
            TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, golden + 2U, header.payload_len,
                                                        encoded, sizeof(encoded), frameSize));
            TEST_ASSERT_EQUAL_HEX8_ARRAY(golden, encoded, frameSize);
            EmlFrameView_s decoded = {};
            TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameDecode(golden, frameSize, frameSize, &decoded));
            assertHeader(header, decoded.header);
            TEST_ASSERT_EQUAL_PTR(golden + 2U, decoded.payload);
            if (header.payload_len != 0U)
                TEST_ASSERT_EQUAL_HEX8_ARRAY(golden + 2U, decoded.payload, header.payload_len);
        }
    }
}

void test_all_payload_lengths_and_buffer_reuse()
{
    uint8_t payload[60];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = (uint8_t)(i * 73U); // Includes binary zero and high bits.
    for (uint8_t frameSize : frameSizes)
    {
        const uint8_t maximum = emlMaxPayloadForFrameSize(frameSize);
        // Descending lengths exercise reuse after a longer message.
        for (int length = maximum; length >= 0; --length)
        {
            for (uint8_t type = EML_PKT_TYPE_SYNC; type <= EML_PKT_TYPE_KEEPALIVE; ++type)
            {
                if ((length == 0) != (type == EML_PKT_TYPE_KEEPALIVE))
                    continue;
                const EmlHeader_s header = {type, (uint8_t)(length & 7), (uint8_t)((length + 3) & 7),
                                           (uint8_t)length, (uint8_t)(length & 3)};
                uint8_t guarded[66];
                memset(guarded, 0xA5, sizeof(guarded));
                // Dirty the output with a maximum payload first.
                const EmlHeader_s longer = {EML_PKT_TYPE_DATA, 0, 7, maximum, EML_FLOW_CTRL_NORMAL};
                TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&longer, payload, maximum,
                                                            guarded + 1, frameSize, frameSize));
                TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, length ? payload : nullptr, length,
                                                            guarded + 1, sizeof(guarded) - 1U, frameSize));
                TEST_ASSERT_EQUAL_HEX8(0xA5, guarded[0]);
                for (size_t i = frameSize + 1U; i < sizeof(guarded); ++i)
                    TEST_ASSERT_EQUAL_HEX8(0xA5, guarded[i]);
                for (size_t i = 2U + length; i < frameSize - 2U; ++i)
                    TEST_ASSERT_EQUAL_HEX8(0, guarded[1U + i]);
                EmlFrameView_s decoded = {};
                TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameDecode(guarded + 1, frameSize, frameSize, &decoded));
                assertHeader(header, decoded.header);
                if (length != 0)
                    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoded.payload, length);
            }
        }
    }
}

void test_encode_in_place_payload_overlap()
{
    const uint8_t offsets[] = {0, 1, 2, 3, 8};
    const uint8_t payload[] = {0x00, 0xFF, 0x7E, 0x01, 0x00, 0x80};
    const EmlHeader_s header = {EML_PKT_TYPE_DATA, 3, 2, sizeof(payload), EML_FLOW_CTRL_HIGH_WATER};
    for (uint8_t frameSize : frameSizes)
    {
        uint8_t expected[64];
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, payload, sizeof(payload), expected, sizeof(expected), frameSize));
        for (uint8_t offset : offsets)
        {
            uint8_t inPlace[64];
            memset(inPlace, 0xFF, sizeof(inPlace));
            memcpy(inPlace + offset, payload, sizeof(payload));
            TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, inPlace + offset, sizeof(payload),
                                                        inPlace, sizeof(inPlace), frameSize));
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, inPlace, frameSize);
        }
    }
}

void test_encode_rejects_bad_inputs_without_writes()
{
    const EmlHeader_s good = {EML_PKT_TYPE_DATA, 0, 7, 1, EML_FLOW_CTRL_NORMAL};
    const uint8_t payload[60] = {};
    uint8_t out[64], sentinel[64];
    memset(out, 0xA5, sizeof(out));
    memcpy(sentinel, out, sizeof(out));
    TEST_ASSERT_EQUAL(EML_FRAME_ERR_NULL_PTR, emlFrameEncode(nullptr, payload, 1, out, sizeof(out), 32));
    TEST_ASSERT_EQUAL(EML_FRAME_ERR_NULL_PTR, emlFrameEncode(&good, payload, 1, nullptr, sizeof(out), 32));
    TEST_ASSERT_EQUAL(EML_FRAME_ERR_NULL_PTR, emlFrameEncode(&good, nullptr, 1, out, sizeof(out), 32));
    const size_t badSizes[] = {0, 1, 4, 8, 16, 31, 33, 63, 65, 256, 288, (size_t)-1};
    for (size_t size : badSizes)
        TEST_ASSERT_EQUAL(EML_FRAME_ERR_FRAME_SIZE, emlFrameEncode(&good, payload, 1, out, sizeof(out), size));
    for (uint8_t frameSize : frameSizes)
    {
        for (size_t capacity = 0; capacity < frameSize; ++capacity)
            TEST_ASSERT_EQUAL(EML_FRAME_ERR_LENGTH, emlFrameEncode(&good, payload, 1, out, capacity, frameSize));
        TEST_ASSERT_EQUAL(EML_FRAME_ERR_PAYLOAD_LENGTH, emlFrameEncode(&good, payload, 0, out, sizeof(out), frameSize));
        TEST_ASSERT_EQUAL(EML_FRAME_ERR_PAYLOAD_LENGTH, emlFrameEncode(&good, payload, 2, out, sizeof(out), frameSize));
        TEST_ASSERT_EQUAL(EML_FRAME_ERR_PAYLOAD_LENGTH, emlFrameEncode(&good, payload, 257, out, sizeof(out), frameSize));
        const EmlHeader_s badHeaders[] = {
            {3, 0, 0, 1, 0}, {4, 0, 0, 1, 0}, {1, 8, 0, 1, 0}, {1, 0, 8, 1, 0},
            {1, 0, 0, 1, 4}, {1, 0, 0, 61, 0}, {1, 0, 0, 62, 0}, {1, 0, 0, 63, 0},
            {1, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {2, 0, 0, 1, 0},
            {1, 0, 0, (uint8_t)(emlMaxPayloadForFrameSize(frameSize) + 1U), 0}
        };
        for (const auto &bad : badHeaders)
            TEST_ASSERT_EQUAL(EML_FRAME_ERR_HEADER, emlFrameEncode(&bad, payload, bad.payload_len, out, sizeof(out), frameSize));
    }
    TEST_ASSERT_EQUAL_HEX8_ARRAY(sentinel, out, sizeof(out));
}

void test_decode_rejects_truncation_extra_bytes_and_bad_sizes()
{
    const EmlHeader_s header = {EML_PKT_TYPE_KEEPALIVE, 7, 7, 0, EML_FLOW_CTRL_NORMAL};
    uint8_t frame[65] = {};
    for (uint8_t frameSize : frameSizes)
    {
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, nullptr, 0, frame, sizeof(frame), frameSize));
        assertDecodeError(nullptr, frameSize, frameSize, EML_FRAME_ERR_NULL_PTR);
        TEST_ASSERT_EQUAL(EML_FRAME_ERR_NULL_PTR, emlFrameDecode(frame, frameSize, frameSize, nullptr));
        for (size_t length = 0; length < frameSize; ++length)
        {
            // Exact allocation lets AddressSanitizer catch even a header read
            // before the length check, including zero/one-byte input buffers.
            uint8_t *truncated = new uint8_t[length];
            memcpy(truncated, frame, length);
            assertDecodeError(truncated, length, frameSize, EML_FRAME_ERR_LENGTH);
            delete[] truncated;
        }
        assertDecodeError(frame, frameSize + 1U, frameSize, EML_FRAME_ERR_LENGTH);
        assertDecodeError(frame, 256U + frameSize, frameSize, EML_FRAME_ERR_LENGTH);
        assertDecodeError(frame, (size_t)-1, frameSize, EML_FRAME_ERR_LENGTH);
        assertDecodeError(frame, frameSize, frameSize == 32 ? 64 : 32, EML_FRAME_ERR_LENGTH);
    }
    const size_t badSizes[] = {0, 1, 4, 8, 16, 31, 33, 63, 65, 256, 288, (size_t)-1};
    for (size_t size : badSizes)
        assertDecodeError(frame, size, size, EML_FRAME_ERR_FRAME_SIZE);
}

void test_decode_every_invalid_wire_header()
{
    // CRC is valid for each safe candidate span: invalid headers must be rejected
    // before payload-length arithmetic, even when the CRC could otherwise match.
    for (uint8_t frameSize : frameSizes)
    {
        uint8_t frame[64] = {};
        for (uint32_t word = 0; word < 65536U; ++word)
        {
            frame[0] = (uint8_t)word;
            frame[1] = (uint8_t)(word >> 8U);
            EmlHeader_s header = {};
            if (emlHeaderDecode(frame, &header, emlMaxPayloadForFrameSize(frameSize)) == EML_HDR_OK)
                continue;
            const size_t length = 2U + (frame[1] & 0x3FU);
            const uint16_t crc = emlCrc16(frame, length < frameSize - 2U ? length : frameSize - 2U);
            frame[frameSize - 2U] = (uint8_t)(crc >> 8U);
            frame[frameSize - 1U] = (uint8_t)crc;
            assertDecodeError(frame, frameSize, frameSize, EML_FRAME_ERR_HEADER);
        }
    }
}

void test_decode_rejects_every_single_bit_error()
{
    uint8_t payload[60];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = (uint8_t)i;
    for (uint8_t frameSize : frameSizes)
    {
        const EmlHeader_s header = {EML_PKT_TYPE_DATA, 0, 7, emlMaxPayloadForFrameSize(frameSize), 0};
        uint8_t frame[64];
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, payload, header.payload_len, frame, sizeof(frame), frameSize));
        for (size_t i = 0; i < frameSize; ++i)
        {
            for (uint8_t bit = 0; bit < 8U; ++bit)
            {
                frame[i] ^= (uint8_t)(1U << bit);
                // Some type/length flips remain legal: they still must fail CRC.
                EmlHeader_s changed = {};
                const bool valid = emlHeaderDecode(frame, &changed, emlMaxPayloadForFrameSize(frameSize)) == EML_HDR_OK;
                assertDecodeError(frame, frameSize, frameSize, valid ? EML_FRAME_ERR_CRC : EML_FRAME_ERR_HEADER);
                frame[i] ^= (uint8_t)(1U << bit);
            }
        }
    }
}

void test_decode_rejects_padding_and_reversed_crc()
{
    const uint8_t payload[] = {0, 0xFF, 0x7E};
    const EmlHeader_s header = {EML_PKT_TYPE_DATA, 0, 7, sizeof(payload), 0};
    for (uint8_t frameSize : frameSizes)
    {
        uint8_t frame[64];
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, payload, sizeof(payload), frame, sizeof(frame), frameSize));
        for (size_t i = 2U + sizeof(payload); i < frameSize - 2U; ++i)
        {
            frame[i] = 1;
            assertDecodeError(frame, frameSize, frameSize, EML_FRAME_ERR_PADDING);
            frame[i] = 0;
        }
        const uint8_t high = frame[frameSize - 2U];
        frame[frameSize - 2U] = frame[frameSize - 1U];
        frame[frameSize - 1U] = high;
        assertDecodeError(frame, frameSize, frameSize, EML_FRAME_ERR_CRC);
    }
}

static EmlSync_s sampleSync()
{
    const EmlSync_s sync = {1, EML_SYNC_ACCEPT, EML_PHY_LORA_500_SF6_CR45, 32,
                            0x11223344U, 0x55667788U, 0x01020304U, 0xA1A2A3A4U,
                            0x10203040U, 0xFFFEU, 0x0123U};
    return sync;
}

void test_sync_golden_payload_and_frame()
{
    const EmlSync_s sync = sampleSync();
    const uint8_t golden[28] = {
        1, 2, 1, 32, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        1, 2, 3, 4, 0xA1, 0xA2, 0xA3, 0xA4, 0x10, 0x20, 0x30, 0x40,
        0xFF, 0xFE, 1, 0x23
    };
    uint8_t encoded[29];
    memset(encoded, 0xA5, sizeof(encoded));
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&sync, encoded, sizeof(encoded)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden, encoded, sizeof(golden));
    TEST_ASSERT_EQUAL_HEX8(0xA5, encoded[28]);
    EmlSync_s decoded = {};
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncDecode(golden, sizeof(golden), &decoded));
    uint8_t reencoded[28];
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&decoded, reencoded, sizeof(reencoded)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden, reencoded, sizeof(golden));
    for (uint8_t frameSize : frameSizes)
    {
        EmlSync_s message = sync;
        message.frame_size = frameSize;
        TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&message, encoded, sizeof(encoded)));
        const EmlHeader_s header = {EML_PKT_TYPE_SYNC, 7, 7, 28, EML_FLOW_CTRL_PAUSE};
        uint8_t frame[64];
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, encoded, 28, frame, sizeof(frame), frameSize));
        TEST_ASSERT_EQUAL_HEX8(frameSize == 32 ? 0x93 : 0xCB, frame[frameSize - 2U]);
        TEST_ASSERT_EQUAL_HEX8(frameSize == 32 ? 0x68 : 0xF0, frame[frameSize - 1U]);
        EmlFrameView_s view = {};
        TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameDecode(frame, frameSize, frameSize, &view));
        TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncDecode(view.payload, view.header.payload_len, &decoded));
        TEST_ASSERT_EQUAL_UINT8(frameSize, decoded.frame_size);
        TEST_ASSERT_EQUAL_HEX32(sync.sender_epoch, decoded.sender_epoch);
        TEST_ASSERT_EQUAL_HEX32(sync.receiver_epoch, decoded.receiver_epoch);
        TEST_ASSERT_EQUAL_HEX16(0xFFFE, decoded.slot_counter);
        TEST_ASSERT_EQUAL_HEX16(0x0123, decoded.hop_index);
    }
}

void test_sync_phases_and_slot_wrap()
{
    for (uint8_t phase = EML_SYNC_HELLO; phase <= EML_SYNC_READY; ++phase)
    {
        EmlSync_s sync = sampleSync();
        sync.phase = phase;
        sync.receiver_epoch = phase == EML_SYNC_HELLO ? 0 : 0xA1A2A3A4U;
        sync.slot_counter = (phase & 1U) ? 0xFFFFU : 0U;
        uint8_t payload[28];
        TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&sync, payload, sizeof(payload)));
        EmlSync_s decoded = {};
        TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncDecode(payload, sizeof(payload), &decoded));
        TEST_ASSERT_EQUAL_UINT8(phase, decoded.phase);
        TEST_ASSERT_EQUAL_HEX16(sync.slot_counter, decoded.slot_counter);
        sync.slot_counter ^= 1U;
        TEST_ASSERT_EQUAL(EML_SYNC_ERR_SLOT_PARITY, emlSyncValidate(&sync));
    }
    EmlSync_s fsk = sampleSync();
    fsk.phy_profile = EML_PHY_FSK_250;
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncValidate(&fsk)); // ID syntax is not hardware capability.
}

void test_sync_rejects_bad_fields_without_publishing()
{
    const EmlSync_s good = sampleSync();
    uint8_t golden[28];
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&good, golden, sizeof(golden)));
    const struct { uint8_t offset; uint8_t value; EmlSyncStatus_e error; } bad[] = {
        {0, 0, EML_SYNC_ERR_VERSION}, {0, 2, EML_SYNC_ERR_VERSION}, {1, 4, EML_SYNC_ERR_PHASE},
        {2, 0, EML_SYNC_ERR_PHY}, {2, 3, EML_SYNC_ERR_PHY}, {3, 0, EML_SYNC_ERR_FRAME_SIZE},
        {3, 33, EML_SYNC_ERR_FRAME_SIZE}, {25, 0xFF, EML_SYNC_ERR_SLOT_PARITY}
    };
    for (const auto &item : bad)
    {
        uint8_t payload[28];
        memcpy(payload, golden, sizeof(payload));
        payload[item.offset] = item.value;
        EmlSync_s out = good;
        TEST_ASSERT_EQUAL(item.error, emlSyncDecode(payload, sizeof(payload), &out));
        uint8_t unchanged[28];
        TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&out, unchanged, sizeof(unchanged)));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(golden, unchanged, sizeof(golden));
    }
    const uint8_t zeroOffsets[] = {4, 8, 12, 16, 20};
    for (uint8_t offset : zeroOffsets)
    {
        uint8_t payload[28];
        memcpy(payload, golden, sizeof(payload));
        memset(payload + offset, 0, 4);
        const EmlSyncStatus_e error = offset < 12 ? EML_SYNC_ERR_IDENTITY :
                                     offset < 20 ? EML_SYNC_ERR_EPOCH : EML_SYNC_ERR_CONFIGURATION;
        EmlSync_s out = good;
        TEST_ASSERT_EQUAL(error, emlSyncDecode(payload, sizeof(payload), &out));
        TEST_ASSERT_EQUAL_HEX32(good.sender_epoch, out.sender_epoch);
    }
    EmlSync_s invalid = good;
    invalid.receiver_id = invalid.sender_id;
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_IDENTITY, emlSyncValidate(&invalid));
    invalid = good;
    invalid.phase = EML_SYNC_HELLO;
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_EPOCH, emlSyncValidate(&invalid));
    uint8_t untouched[28];
    memset(untouched, 0xA5, sizeof(untouched));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_EPOCH, emlSyncEncode(&invalid, untouched, sizeof(untouched)));
    for (uint8_t byte : untouched)
        TEST_ASSERT_EQUAL_HEX8(0xA5, byte);
}

void test_sync_rejects_bad_lengths_and_null_pointers()
{
    EmlSync_s sync = sampleSync();
    uint8_t payload[29];
    TEST_ASSERT_EQUAL(EML_SYNC_OK, emlSyncEncode(&sync, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_NULL_PTR, emlSyncValidate(nullptr));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_NULL_PTR, emlSyncEncode(nullptr, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_NULL_PTR, emlSyncEncode(&sync, nullptr, sizeof(payload)));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_NULL_PTR, emlSyncDecode(nullptr, 28, &sync));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_NULL_PTR, emlSyncDecode(payload, 28, nullptr));
    for (size_t length = 0; length < 28; ++length)
    {
        TEST_ASSERT_EQUAL(EML_SYNC_ERR_LENGTH, emlSyncEncode(&sync, payload, length));
        TEST_ASSERT_EQUAL(EML_SYNC_ERR_LENGTH, emlSyncDecode(payload, length, &sync));
    }
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_LENGTH, emlSyncDecode(payload, 29, &sync));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_LENGTH, emlSyncDecode(payload, 284, &sync));
    TEST_ASSERT_EQUAL(EML_SYNC_ERR_LENGTH, emlSyncDecode(payload, (size_t)-1, &sync));
}

void setUp() {}
void tearDown() {}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_frame_golden_vectors);
    RUN_TEST(test_all_payload_lengths_and_buffer_reuse);
    RUN_TEST(test_encode_in_place_payload_overlap);
    RUN_TEST(test_encode_rejects_bad_inputs_without_writes);
    RUN_TEST(test_decode_rejects_truncation_extra_bytes_and_bad_sizes);
    RUN_TEST(test_decode_every_invalid_wire_header);
    RUN_TEST(test_decode_rejects_every_single_bit_error);
    RUN_TEST(test_decode_rejects_padding_and_reversed_crc);
    RUN_TEST(test_sync_golden_payload_and_frame);
    RUN_TEST(test_sync_phases_and_slot_wrap);
    RUN_TEST(test_sync_rejects_bad_fields_without_publishing);
    RUN_TEST(test_sync_rejects_bad_lengths_and_null_pointers);
    return UNITY_END();
}
