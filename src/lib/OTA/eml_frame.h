#ifndef H_EML_FRAME
#define H_EML_FRAME

#include <stddef.h>
#include <stdint.h>

/**
 * EML (ExpressLRS Modified Link) Frame Header Specification (P2.1)
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

#endif // H_EML_FRAME
