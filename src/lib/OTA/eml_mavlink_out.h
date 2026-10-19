#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Bridge self-telemetry on the local UART: MAVLink2 RADIO_STATUS (#109) like SiK/RFD
// radios, plus NAMED_VALUE_INT (#252) for the link quality RADIO_STATUS lacks. Opt-in and
// only in the MAVLink2 traffic mode, between complete frames of the output stream, so
// the transparent Raw transport never carries bridge metrics (P8.3). Nothing is sent
// over the air.
constexpr uint8_t EML_MAV_SYSID = 51;  // SiK radio convention
constexpr uint8_t EML_MAV_COMPID = 68; // MAV_COMP_ID_TELEMETRY_RADIO
constexpr uint32_t EML_MAV_MSG_RADIO_STATUS = 109;
constexpr uint8_t EML_MAV_CRC_RADIO_STATUS = 185;
constexpr uint32_t EML_MAV_MSG_NAMED_VALUE_INT = 252;
constexpr uint8_t EML_MAV_CRC_NAMED_VALUE_INT = 44;
constexpr size_t EML_MAV_FRAME_MAX = 12U + 18U; // largest payload sent here: NAMED_VALUE_INT
constexpr uint8_t EML_MAV_UNKNOWN = 255;

// CRC-16/MCRF4XX (MAVLink X.25 accumulate)
inline uint16_t emlMavCrc(uint16_t crc, uint8_t byte)
{
    uint8_t t = (uint8_t)(byte ^ (uint8_t)crc);
    t = (uint8_t)(t ^ (uint8_t)(t << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4));
}

// Unsigned MAVLink2 frame with trailing-zero payload truncation; returns its length
inline size_t emlMavlink2Frame(uint32_t msgid, uint8_t crcExtra, const uint8_t *payload, uint8_t length,
                               uint8_t seq, uint8_t *out)
{
    while (length > 1 && payload[length - 1U] == 0) --length;
    out[0] = 0xFD; out[1] = length; out[2] = 0; out[3] = 0; out[4] = seq;
    out[5] = EML_MAV_SYSID; out[6] = EML_MAV_COMPID;
    out[7] = (uint8_t)msgid; out[8] = (uint8_t)(msgid >> 8); out[9] = (uint8_t)(msgid >> 16);
    memcpy(out + 10, payload, length);
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 10U + length; ++i) crc = emlMavCrc(crc, out[i]);
    crc = emlMavCrc(crc, crcExtra);
    out[10U + length] = (uint8_t)crc;
    out[11U + length] = (uint8_t)(crc >> 8);
    return 12U + length;
}

struct EmlRadioStatus_s
{
    uint16_t rxerrors, fixed;                // wrapping counters
    uint8_t rssi, remrssi, txbuf, noise, remnoise;
};

inline size_t emlMavRadioStatus(const EmlRadioStatus_s &s, uint8_t seq, uint8_t *out)
{
    const uint8_t payload[9] = {(uint8_t)s.rxerrors, (uint8_t)(s.rxerrors >> 8), (uint8_t)s.fixed, (uint8_t)(s.fixed >> 8),
                                s.rssi, s.remrssi, s.txbuf, s.noise, s.remnoise};
    return emlMavlink2Frame(EML_MAV_MSG_RADIO_STATUS, EML_MAV_CRC_RADIO_STATUS, payload, sizeof(payload), seq, out);
}

inline size_t emlMavNamedValueInt(uint32_t timeMs, const char *name, int32_t value, uint8_t seq, uint8_t *out)
{
    uint8_t payload[18] = {};
    for (uint8_t i = 0; i < 4; ++i)
    {
        payload[i] = (uint8_t)(timeMs >> (8U * i));
        payload[4U + i] = (uint8_t)((uint32_t)value >> (8U * i));
    }
    for (uint8_t i = 0; i < 10 && name[i]; ++i) payload[8U + i] = (uint8_t)name[i];
    return emlMavlink2Frame(EML_MAV_MSG_NAMED_VALUE_INT, EML_MAV_CRC_NAMED_VALUE_INT, payload, sizeof(payload), seq, out);
}

// SiK RSSI/noise units, as Mission Planner shows them: dBm = value / 1.9 - 127
inline uint8_t emlMavSikLevel(int16_t dbm)
{
    int32_t v = ((int32_t)dbm + 127) * 19 / 10;
    if (v < 0) v = 0;
    if (v > 254) v = 254;
    return (uint8_t)v;
}

// Tracks MAVLink1/2 frame boundaries in a byte stream. Bytes outside a frame (raw
// gaps) keep it idle. A frame stalled for idleResetMs (garbage 0xFD/0xFE, never
// completed) is abandoned so telemetry cannot be blocked forever; real fragmented
// frames resume within a few slots.
class EmlMavlinkBoundary
{
    uint16_t remaining_ = 0;
    uint8_t header_ = 0, version_ = 0, length_ = 0;
    uint32_t lastMs_ = 0;

public:
    bool idle() const { return header_ == 0 && remaining_ == 0; }
    void reset() { remaining_ = 0; header_ = version_ = length_ = 0; }
    void feed(uint8_t b)
    {
        if (remaining_) { --remaining_; return; }
        if (!header_)
        {
            if (b == 0xFD || b == 0xFE) { version_ = b == 0xFD ? 2 : 1; header_ = 1; }
            return;
        }
        ++header_;
        if (header_ == 2)
        {
            length_ = b;
            if (version_ == 1) { remaining_ = (uint16_t)(length_ + 8U - 2U); header_ = 0; }
            return;
        }
        // MAVLink2 byte 2: incompatibility flags (bit 0: 13-byte signature)
        remaining_ = (uint16_t)(12U + length_ + ((b & 0x01U) ? 13U : 0U) - 3U);
        header_ = 0;
    }
    void feed(const uint8_t *data, size_t n, uint32_t nowMs)
    {
        if (n) lastMs_ = nowMs;
        for (size_t i = 0; i < n; ++i) feed(data[i]);
    }
    // Idle, or a frame abandoned after idleResetMs without output
    bool injectable(uint32_t nowMs, uint32_t idleResetMs = 1000U)
    {
        if (!idle() && nowMs - lastMs_ >= idleResetMs) reset();
        return idle();
    }
    // Bytes of data[0..n) up to the next boundary (n if none is reached)
    size_t toBoundary(const uint8_t *data, size_t n) const
    {
        EmlMavlinkBoundary copy = *this;
        for (size_t i = 0; i < n; ++i)
        {
            copy.feed(data[i]);
            if (copy.idle()) return i + 1U;
        }
        return n;
    }
};
