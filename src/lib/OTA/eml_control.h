#pragma once

#include "eml_sync.h"

// Session-bound link control carried in a normal fixed-length SYNC frame.
// 28 bytes fits EML32. Handshake phases 0..3 retain their existing layout.
enum : uint8_t { EML_BAND_PROPOSE = 4, EML_BAND_ACK = 5, EML_BAND_COMMIT = 6, EML_BAND_STATUS = 7 };
enum : uint8_t { EML_TDD_PROPOSE = 8, EML_TDD_ACK = 9, EML_TDD_COMMIT = 10, EML_TDD_STATUS = 11 };
enum : uint8_t { EML_LINK_STATUS_COORDINATOR = 12, EML_LINK_STATUS_RESPONDER = 13 };
enum : uint8_t { EML_LINK_HAS_SNR = 0x01 };
constexpr uint8_t EML_TDD_LEASE_SLOTS = 64;
struct EmlBandMessage_s
{
    uint8_t op, band, mask;
    uint16_t switch_slot, token; // STATUS: observed slot, (LQ << 8) | missed ACK count
};

inline bool emlBandMessageValid(const EmlBandMessage_s &m)
{
    if (m.op == EML_LINK_STATUS_COORDINATOR || m.op == EML_LINK_STATUS_RESPONDER)
        return (m.band == EML_PHY_LORA_500_SF6_CR45 || m.band == EML_PHY_FSK_250) &&
            (m.mask & (uint8_t)~EML_LINK_HAS_SNR) == 0 &&
            (m.switch_slot >> 8) <= 100 && (uint8_t)m.switch_slot <= 100 &&
            (m.switch_slot >> 8) + (uint8_t)m.switch_slot == 100;
    if (m.op >= EML_TDD_PROPOSE && m.op <= EML_TDD_STATUS)
        return m.band <= 2 && (m.op == EML_TDD_STATUS ? m.mask <= 7 :
            m.mask == EML_TDD_LEASE_SLOTS && !(m.switch_slot & 3U) && m.token);
    return m.op >= EML_BAND_PROPOSE && m.op <= EML_BAND_STATUS && m.band < 4 &&
        m.mask && !(m.mask & 0xF0U) && (m.mask & (1U << m.band)) &&
        (m.op == EML_BAND_STATUS ? (m.token >> 8) <= 100 : m.token && !(m.switch_slot & 1U));
}

struct EmlLinkReport_s
{
    uint8_t phy, lq, per;
    int8_t rssi, snr;
    bool snr_valid;
};

inline EmlBandMessage_s emlLinkMessage(uint8_t role, const EmlLinkReport_s &r)
{
    return {(uint8_t)(role == 0 ? EML_LINK_STATUS_COORDINATOR : EML_LINK_STATUS_RESPONDER),
        r.phy, (uint8_t)(r.snr_valid ? EML_LINK_HAS_SNR : 0),
        (uint16_t)((uint16_t)r.lq << 8 | r.per),
        (uint16_t)((uint16_t)(uint8_t)r.rssi << 8 | (uint8_t)r.snr)};
}

inline bool emlLinkReport(const EmlBandMessage_s &m, EmlLinkReport_s *out)
{
    if (!out || (m.op != EML_LINK_STATUS_COORDINATOR && m.op != EML_LINK_STATUS_RESPONDER) ||
        !emlBandMessageValid(m)) return false;
    *out = {m.band, (uint8_t)(m.switch_slot >> 8), (uint8_t)m.switch_slot,
        (int8_t)(m.token >> 8), (int8_t)m.token, (m.mask & EML_LINK_HAS_SNR) != 0};
    return true;
}

inline void emlBandMessageWrite(const EmlBandMessage_s &m, uint8_t *out)
{
    out[0] = EML_PROTOCOL_VERSION; out[1] = m.op; out[2] = m.band; out[3] = m.mask;
    out[24] = (uint8_t)(m.switch_slot >> 8); out[25] = (uint8_t)m.switch_slot;
    out[26] = (uint8_t)(m.token >> 8); out[27] = (uint8_t)m.token;
}

inline bool emlBandMessageRead(const uint8_t *in, size_t length, EmlBandMessage_s *out)
{
    if (!in || !out || length != EML_SYNC_PAYLOAD_SIZE || in[0] != EML_PROTOCOL_VERSION) return false;
    EmlBandMessage_s m = {in[1], in[2], in[3], (uint16_t)((uint16_t)in[24] << 8 | in[25]),
                                            (uint16_t)((uint16_t)in[26] << 8 | in[27])};
    if (!emlBandMessageValid(m)) return false;
    *out = m;
    return true;
}
