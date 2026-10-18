#ifndef H_EML_FRAME
#define H_EML_FRAME

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * EML (ExpressLRS Modified Link) header, fixed frame layout, CRC and codec.
 *
 * Explicit byte-level encoding independent of C/C++ compiler bitfield layout
 * and target endianness.
 *
 * Wire layout of the 2-byte header (offsets 0 and 1 of EML_Frame):
 *
 *   Byte 0 (offset 0):
 *     Bits [7:5] : ack_num     (3 bits, 0..7) - piggybacked inbound ACK sequence number
 *     Bits [4:2] : seq_num     (3 bits, 0..7) - outbound payload sequence number
 *     Bits [1:0] : pkt_type    (2 bits, 0..3) - frame type (Sync, Data, Keepalive, Reserved)
 *
 *   Byte 1 (offset 1):
 *     Bits [7:6] : flow_ctrl   (2 bits, 0..3) - local UART TX buffer watermark / backpressure
 *     Bits [5:0] : payload_len (6 bits, 0..60 valid; 61..63 reserved)
 */

#define EML_HEADER_SIZE             2U
#define EML_CRC_SIZE                2U
#define EML_OVERHEAD_SIZE           (EML_HEADER_SIZE + EML_CRC_SIZE)

#define EML_FRAME_SIZE_32           32U
#define EML_FRAME_SIZE_64           64U

#define EML_PAYLOAD_OFFSET          EML_HEADER_SIZE
#define EML_CRC_OFFSET_32           (EML_FRAME_SIZE_32 - EML_CRC_SIZE)
#define EML_CRC_OFFSET_64           (EML_FRAME_SIZE_64 - EML_CRC_SIZE)
#define EML_PADDING_VALUE           0x00U

// CRC-16/CCITT-FALSE: width=16, refin=false, refout=false, no augmentation.
// Covers the two header bytes followed by exactly payload_len payload bytes.
// Padding is excluded. Store the result MSB first in the final two frame bytes.
#define EML_CRC16_POLY              0x1021U
#define EML_CRC16_INIT              0xFFFFU
#define EML_CRC16_XOROUT            0x0000U

#define EML_MAX_PAYLOAD_LEN_32      (EML_FRAME_SIZE_32 - EML_OVERHEAD_SIZE) // 28 bytes
#define EML_MAX_PAYLOAD_LEN_64      (EML_FRAME_SIZE_64 - EML_OVERHEAD_SIZE) // 60 bytes
#define EML_MAX_PAYLOAD_LEN         EML_MAX_PAYLOAD_LEN_64                  // 60 bytes ceiling
#define EML_RESERVED_PAYLOAD_MIN    61U                                     // 61..63 are reserved

#define EML_SEQ_MAX                 7U
#define EML_SEQ_MODULO              8U
#define EML_SEQ_MASK                0x07U

// Byte 0 shifts and masks
#define EML_HDR_B0_PKT_TYPE_SHIFT   0U
#define EML_HDR_B0_PKT_TYPE_MASK    0x03U
#define EML_HDR_B0_SEQ_NUM_SHIFT    2U
#define EML_HDR_B0_SEQ_NUM_MASK     0x07U
#define EML_HDR_B0_ACK_NUM_SHIFT    5U
#define EML_HDR_B0_ACK_NUM_MASK     0x07U

// Byte 1 shifts and masks
#define EML_HDR_B1_PAYLOAD_LEN_SHIFT 0U
#define EML_HDR_B1_PAYLOAD_LEN_MASK  0x3FU
#define EML_HDR_B1_FLOW_CTRL_SHIFT   6U
#define EML_HDR_B1_FLOW_CTRL_MASK    0x03U

/**
 * Fixed wire storage: header | meaningful payload | zero padding | CRC16.
 * Only the first payload_len bytes of payload are meaningful; the rest must
 * be EML_PADDING_VALUE. CRC always occupies the final two bytes, independently
 * of payload_len. All fields are byte arrays, so no packed attribute or native
 * integer endianness is involved. CRC is CCITT-FALSE, high byte then low byte.
 * These types define storage; use emlFrameEncode/emlFrameDecode for the codec.
 */
template <size_t FrameSize>
struct EmlFrame_s
{
    static_assert(FrameSize == EML_FRAME_SIZE_32 || FrameSize == EML_FRAME_SIZE_64,
                  "EML frames must be exactly 32 or 64 bytes");

    uint8_t header[EML_HEADER_SIZE];
    uint8_t payload[FrameSize - EML_OVERHEAD_SIZE]; // meaningful bytes followed by padding
    uint8_t crc[EML_CRC_SIZE];
};

using EmlFrame32_s = EmlFrame_s<EML_FRAME_SIZE_32>;
using EmlFrame64_s = EmlFrame_s<EML_FRAME_SIZE_64>;

static_assert(sizeof(EmlFrame32_s) == 32U, "EML32 wire size");
static_assert(offsetof(EmlFrame32_s, header) == 0U, "EML32 header offset");
static_assert(offsetof(EmlFrame32_s, payload) == EML_PAYLOAD_OFFSET, "EML32 payload offset");
static_assert(offsetof(EmlFrame32_s, crc) == EML_CRC_OFFSET_32, "EML32 CRC offset");
static_assert(sizeof(EmlFrame64_s) == 64U, "EML64 wire size");
static_assert(offsetof(EmlFrame64_s, header) == 0U, "EML64 header offset");
static_assert(offsetof(EmlFrame64_s, payload) == EML_PAYLOAD_OFFSET, "EML64 payload offset");
static_assert(offsetof(EmlFrame64_s, crc) == EML_CRC_OFFSET_64, "EML64 CRC offset");

enum EmlPktType_e : uint8_t
{
    EML_PKT_TYPE_SYNC      = 0x00U, // 0b00: Sync / session handshake & link control frame
    EML_PKT_TYPE_DATA      = 0x01U, // 0b01: Reliable UART data frame (payload_len = 1..max)
    EML_PKT_TYPE_KEEPALIVE = 0x02U, // 0b10: Keepalive / pure ACK / idle slot (payload_len = 0)
    EML_PKT_TYPE_RESERVED  = 0x03U  // 0b11: Reserved (must be rejected on decode)
};

enum EmlFlowCtrl_e : uint8_t
{
    EML_FLOW_CTRL_NORMAL     = 0x00U, // 0b00: < 50% full (>= 4 max frames free)
    EML_FLOW_CTRL_HIGH_WATER = 0x01U, // 0b01: 50%..75% full (normal 1:1 TDD, no burst expansion)
    EML_FLOW_CTRL_CRITICAL   = 0x02U, // 0b10: >= 75% full (< 2 max frames free, throttle)
    EML_FLOW_CTRL_PAUSE      = 0x03U  // 0b11: < 1 max frame (< 60 B) free, pause new DATA frames
};

enum EmlHdrStatus_e : uint8_t
{
    EML_HDR_OK                    = 0U,
    EML_HDR_ERR_NULL_PTR          = 1U,
    EML_HDR_ERR_RESERVED_PKT_TYPE = 2U, // pkt_type == 0b11
    EML_HDR_ERR_SEQ_OUT_OF_RANGE  = 3U, // seq_num > 7 (when validating unpacked struct)
    EML_HDR_ERR_ACK_OUT_OF_RANGE  = 4U, // ack_num > 7 (when validating unpacked struct)
    EML_HDR_ERR_FLOW_OUT_OF_RANGE = 5U, // flow_ctrl > 3 (when validating unpacked struct)
    EML_HDR_ERR_RESERVED_LEN      = 6U, // payload_len in 61..63
    EML_HDR_ERR_LEN_EXCEEDS_FRAME = 7U, // payload_len > maxPayloadLen for active frame size (28 or 60)
    EML_HDR_ERR_EMPTY_DATA        = 8U, // pkt_type == DATA with payload_len == 0
    EML_HDR_ERR_NONEMPTY_KEEPALIVE = 9U,// pkt_type == KEEPALIVE with payload_len != 0
    EML_HDR_ERR_EMPTY_SYNC        = 10U // pkt_type == SYNC with payload_len == 0
};

/**
 * Unpacked logical representation of the 2-byte EML header.
 * Uses plain fixed-width integers (no C++ bitfields) for deterministic behavior.
 */
struct EmlHeader_s
{
    uint8_t pkt_type;    // EmlPktType_e (0..2 valid, 3 reserved)
    uint8_t seq_num;     // 0..7
    uint8_t ack_num;     // 0..7
    uint8_t payload_len; // 0..60 (or 0..28 for 32-byte frame)
    uint8_t flow_ctrl;   // EmlFlowCtrl_e (0..3)
};

static inline constexpr uint8_t emlSeqNext(uint8_t seq)
{
    return (uint8_t)((seq + 1U) & EML_SEQ_MASK);
}

static inline constexpr uint8_t emlMaxPayloadForFrameSize(uint8_t frameSize)
{
    return (frameSize == EML_FRAME_SIZE_32) ? EML_MAX_PAYLOAD_LEN_32 :
           (frameSize == EML_FRAME_SIZE_64) ? EML_MAX_PAYLOAD_LEN_64 : 0U;
}

/**
 * Table-free CRC-16/CCITT-FALSE reference calculation (P2.3).
 * data must point to length readable bytes; nullptr is permitted only for length=0.
 * The caller validates the frame and payload length before selecting this span.
 * Start each frame with EML_CRC16_INIT. Since xorout=0 and there is no reflection,
 * a previous result can seed the next chunk of the SAME frame (header then payload).
 * No padding, stored CRC, implicit zero bytes, UID or FHSS nonce enter this CRC.
 * This primitive does not validate a frame or establish ISR timing/placement.
 * RF integration and ISR placement remain separate tasks (P3.8 and P4).
 */
static inline uint16_t emlCrc16(const uint8_t *data, size_t length, uint16_t crc = EML_CRC16_INIT)
{
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? EML_CRC16_POLY : 0U));
        }
    }
    return (uint16_t)(crc ^ EML_CRC16_XOROUT);
}

/**
 * Validate an unpacked EmlHeader_s against protocol rules and maximum payload length.
 * @param hdr           Pointer to unpacked header structure.
 * @param maxPayloadLen Maximum allowed payload bytes for the active frame size (e.g. 60 or 28).
 */
static inline EmlHdrStatus_e emlHeaderValidate(const EmlHeader_s *hdr, uint8_t maxPayloadLen = EML_MAX_PAYLOAD_LEN_64)
{
    if (hdr == nullptr)
    {
        return EML_HDR_ERR_NULL_PTR;
    }
    if (hdr->pkt_type >= EML_PKT_TYPE_RESERVED)
    {
        return EML_HDR_ERR_RESERVED_PKT_TYPE;
    }
    if (hdr->seq_num > EML_SEQ_MAX)
    {
        return EML_HDR_ERR_SEQ_OUT_OF_RANGE;
    }
    if (hdr->ack_num > EML_SEQ_MAX)
    {
        return EML_HDR_ERR_ACK_OUT_OF_RANGE;
    }
    if (hdr->flow_ctrl > EML_FLOW_CTRL_PAUSE)
    {
        return EML_HDR_ERR_FLOW_OUT_OF_RANGE;
    }
    if (hdr->payload_len >= EML_RESERVED_PAYLOAD_MIN)
    {
        return EML_HDR_ERR_RESERVED_LEN;
    }
    if (maxPayloadLen > EML_MAX_PAYLOAD_LEN_64 || hdr->payload_len > maxPayloadLen)
    {
        return EML_HDR_ERR_LEN_EXCEEDS_FRAME;
    }
    if (hdr->pkt_type == EML_PKT_TYPE_DATA && hdr->payload_len == 0U)
    {
        return EML_HDR_ERR_EMPTY_DATA;
    }
    if (hdr->pkt_type == EML_PKT_TYPE_KEEPALIVE && hdr->payload_len != 0U)
    {
        return EML_HDR_ERR_NONEMPTY_KEEPALIVE;
    }
    if (hdr->pkt_type == EML_PKT_TYPE_SYNC && hdr->payload_len == 0U)
    {
        return EML_HDR_ERR_EMPTY_SYNC;
    }
    return EML_HDR_OK;
}

/**
 * Encode an unpacked EmlHeader_s into 2 wire bytes (out[0], out[1]).
 * Rejects reserved or out-of-range fields prior to writing.
 */
static inline EmlHdrStatus_e emlHeaderEncode(const EmlHeader_s *hdr, uint8_t out[EML_HEADER_SIZE], uint8_t maxPayloadLen = EML_MAX_PAYLOAD_LEN_64)
{
    if (out == nullptr)
    {
        return EML_HDR_ERR_NULL_PTR;
    }
    EmlHdrStatus_e status = emlHeaderValidate(hdr, maxPayloadLen);
    if (status != EML_HDR_OK)
    {
        return status;
    }

    out[0] = (uint8_t)(((hdr->ack_num & EML_HDR_B0_ACK_NUM_MASK) << EML_HDR_B0_ACK_NUM_SHIFT) |
                       ((hdr->seq_num & EML_HDR_B0_SEQ_NUM_MASK) << EML_HDR_B0_SEQ_NUM_SHIFT) |
                       ((hdr->pkt_type & EML_HDR_B0_PKT_TYPE_MASK) << EML_HDR_B0_PKT_TYPE_SHIFT));

    out[1] = (uint8_t)(((hdr->flow_ctrl & EML_HDR_B1_FLOW_CTRL_MASK) << EML_HDR_B1_FLOW_CTRL_SHIFT) |
                       ((hdr->payload_len & EML_HDR_B1_PAYLOAD_LEN_MASK) << EML_HDR_B1_PAYLOAD_LEN_SHIFT));

    return EML_HDR_OK;
}

/**
 * Decode 2 wire bytes (in[0], in[1]) into an unpacked EmlHeader_s and validate all fields.
 * The raw fields are always extracted into *hdr (when non-null) before returning validation status.
 */
static inline EmlHdrStatus_e emlHeaderDecode(const uint8_t in[EML_HEADER_SIZE], EmlHeader_s *hdr, uint8_t maxPayloadLen = EML_MAX_PAYLOAD_LEN_64)
{
    if (in == nullptr || hdr == nullptr)
    {
        return EML_HDR_ERR_NULL_PTR;
    }

    hdr->pkt_type    = (uint8_t)((in[0] >> EML_HDR_B0_PKT_TYPE_SHIFT) & EML_HDR_B0_PKT_TYPE_MASK);
    hdr->seq_num     = (uint8_t)((in[0] >> EML_HDR_B0_SEQ_NUM_SHIFT) & EML_HDR_B0_SEQ_NUM_MASK);
    hdr->ack_num     = (uint8_t)((in[0] >> EML_HDR_B0_ACK_NUM_SHIFT) & EML_HDR_B0_ACK_NUM_MASK);
    hdr->payload_len = (uint8_t)((in[1] >> EML_HDR_B1_PAYLOAD_LEN_SHIFT) & EML_HDR_B1_PAYLOAD_LEN_MASK);
    hdr->flow_ctrl   = (uint8_t)((in[1] >> EML_HDR_B1_FLOW_CTRL_SHIFT) & EML_HDR_B1_FLOW_CTRL_MASK);

    return emlHeaderValidate(hdr, maxPayloadLen);
}

enum EmlFrameStatus_e : uint8_t
{
    EML_FRAME_OK = 0U,
    EML_FRAME_ERR_NULL_PTR,
    EML_FRAME_ERR_FRAME_SIZE,
    EML_FRAME_ERR_LENGTH,
    EML_FRAME_ERR_HEADER,
    EML_FRAME_ERR_PAYLOAD_LENGTH,
    EML_FRAME_ERR_CRC,
    EML_FRAME_ERR_PADDING
};

// A borrowed, read-only payload view. It remains valid only while the input
// buffer is alive and unchanged. This view must not overlap the input buffer.
struct EmlFrameView_s
{
    EmlHeader_s header;
    const uint8_t *payload;
};

/**
 * Encode exactly frameSize (32 or 64) bytes after validating all inputs.
 * payloadLength must equal header->payload_len, and payload may be null only
 * for length zero. outCapacity may be larger than frameSize; its tail is untouched.
 * Payload/output overlap is supported, including encoding in place. All errors
 * leave output untouched. No heap, global state, ARQ or UART side effects.
 */
static inline EmlFrameStatus_e emlFrameEncode(const EmlHeader_s *header,
                                             const uint8_t *payload, size_t payloadLength,
                                             uint8_t *out, size_t outCapacity, size_t frameSize)
{
    if (header == nullptr || out == nullptr)
        return EML_FRAME_ERR_NULL_PTR;
    if (frameSize != EML_FRAME_SIZE_32 && frameSize != EML_FRAME_SIZE_64)
        return EML_FRAME_ERR_FRAME_SIZE;
    if (outCapacity < frameSize)
        return EML_FRAME_ERR_LENGTH;

    const EmlHeader_s value = *header;
    const uint8_t maximum = emlMaxPayloadForFrameSize((uint8_t)frameSize);
    if (emlHeaderValidate(&value, maximum) != EML_HDR_OK)
        return EML_FRAME_ERR_HEADER;
    if (payloadLength != value.payload_len)
        return EML_FRAME_ERR_PAYLOAD_LENGTH;
    if (payloadLength != 0U && payload == nullptr)
        return EML_FRAME_ERR_NULL_PTR;

    // Copy before touching header/padding, so payload may point anywhere in out.
    if (payloadLength != 0U)
        memmove(out + EML_PAYLOAD_OFFSET, payload, payloadLength);
    emlHeaderEncode(&value, out, maximum);
    const size_t protectedLength = EML_HEADER_SIZE + payloadLength;
    const size_t crcOffset = frameSize - EML_CRC_SIZE;
    memset(out + protectedLength, EML_PADDING_VALUE, crcOffset - protectedLength);
    const uint16_t crc = emlCrc16(out, protectedLength);
    out[crcOffset] = (uint8_t)(crc >> 8U);
    out[crcOffset + 1U] = (uint8_t)(crc & 0xFFU);
    return EML_FRAME_OK;
}

/**
 * Decode an exact physical frame. Reject length/header errors before selecting
 * the CRC span, then verify CRC and every padding byte. Publish the output only
 * on success; errors leave the previous view untouched. Success proves envelope
 * integrity, not peer/session acceptance or Sync payload semantics (eml_sync.h).
 */
static inline EmlFrameStatus_e emlFrameDecode(const uint8_t *in, size_t inLength,
                                             size_t frameSize, EmlFrameView_s *out)
{
    if (in == nullptr || out == nullptr)
        return EML_FRAME_ERR_NULL_PTR;
    if (frameSize != EML_FRAME_SIZE_32 && frameSize != EML_FRAME_SIZE_64)
        return EML_FRAME_ERR_FRAME_SIZE;
    if (inLength != frameSize)
        return EML_FRAME_ERR_LENGTH;

    EmlFrameView_s value = {};
    if (emlHeaderDecode(in, &value.header, emlMaxPayloadForFrameSize((uint8_t)frameSize)) != EML_HDR_OK)
        return EML_FRAME_ERR_HEADER;
    const size_t protectedLength = EML_HEADER_SIZE + value.header.payload_len;
    const size_t crcOffset = frameSize - EML_CRC_SIZE;
    const uint16_t received = (uint16_t)((uint16_t)in[crcOffset] << 8U) | in[crcOffset + 1U];
    if (emlCrc16(in, protectedLength) != received)
        return EML_FRAME_ERR_CRC;
    for (size_t i = protectedLength; i < crcOffset; ++i)
    {
        if (in[i] != EML_PADDING_VALUE)
            return EML_FRAME_ERR_PADDING;
    }
    value.payload = in + EML_PAYLOAD_OFFSET;
    *out = value;
    return EML_FRAME_OK;
}

#endif // H_EML_FRAME
