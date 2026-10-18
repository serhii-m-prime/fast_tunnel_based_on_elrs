#pragma once

#include "eml_frame.h"

constexpr uint8_t EML_ACK_FRAME_SIZE = 4;

// Deterministic FSK control slots, independent of any negotiated lease. Losing
// a mode message cannot change the physical length expected at these anchors.
__attribute__((always_inline)) inline bool emlShortAckSlot(uint32_t slot)
{
    return (slot & 63U) == 8U || (slot & 63U) == 59U;
}

inline bool emlAckEncode(const EmlHeader_s &header, uint8_t *out, size_t capacity)
{
    if (!out || capacity < EML_ACK_FRAME_SIZE || header.pkt_type != EML_PKT_TYPE_KEEPALIVE ||
        emlHeaderEncode(&header, out, 0) != EML_HDR_OK) return false;
    const uint16_t crc = emlCrc16(out, 2);
    out[2] = (uint8_t)(crc >> 8); out[3] = (uint8_t)crc;
    return true;
}

inline bool emlAckDecode(const uint8_t *bytes, size_t length, EmlFrameView_s *out)
{
    if (!bytes || !out || length != EML_ACK_FRAME_SIZE) return false;
    EmlFrameView_s view = {};
    if (emlHeaderDecode(bytes, &view.header, 0) != EML_HDR_OK || view.header.pkt_type != EML_PKT_TYPE_KEEPALIVE ||
        emlCrc16(bytes, 2) != (uint16_t)((uint16_t)bytes[2] << 8 | bytes[3])) return false;
    *out = view;
    return true;
}
