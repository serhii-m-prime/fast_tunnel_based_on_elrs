#pragma once

#include <stdint.h>

// FSK/GFSK packet-mode parameters for SX127xDriver::ConfigFsk() (P5.1). Pure
// register arithmetic, no Arduino dependencies, so it is native-testable.

#define SX127X_FXOSC_HZ 32000000UL

struct SX127xFskParams_s
{
    uint32_t bitrate;        // bit/s
    uint32_t fdev_hz;        // frequency deviation
    uint32_t rxbw_hz;        // single-sideband receiver (and AFC) filter, one of the RegRxBw steps
    uint8_t shaping;         // SX127X_FSK_SHAPING_*
    uint8_t preamble_bytes;  // 0xAA.. preamble
    uint8_t sync_size;       // 1..8 bytes, no 0x00 byte
    uint8_t sync[8];
};

// RegBitrate (15:0) and RegBitRateFrac: BitRate = FXOSC / (reg + frac/16)
struct SX127xFskBitrateReg_s
{
    uint16_t reg;
    uint8_t frac;
};

constexpr uint32_t sx127xFskBitrateX16(uint32_t bitrate)
{
    return (uint32_t)(((uint64_t)SX127X_FXOSC_HZ * 16ULL + bitrate / 2U) / bitrate);
}

inline SX127xFskBitrateReg_s sx127xFskBitrateReg(uint32_t bitrate)
{
    const uint32_t x16 = sx127xFskBitrateX16(bitrate);
    return {(uint16_t)(x16 >> 4), (uint8_t)(x16 & 0x0FU)};
}

// Bitrate actually produced by the register setting
constexpr uint32_t sx127xFskActualBitrate(uint32_t bitrate)
{
    return (uint32_t)(((uint64_t)SX127X_FXOSC_HZ * 16ULL + sx127xFskBitrateX16(bitrate) / 2U) / sx127xFskBitrateX16(bitrate));
}

// RegFdev (13:0): Fdev = Fstep * reg, Fstep = FXOSC / 2^19
constexpr uint16_t sx127xFskFdevReg(uint32_t fdevHz)
{
    return (uint16_t)((((uint64_t)fdevHz << 19) + SX127X_FXOSC_HZ / 2U) / SX127X_FXOSC_HZ);
}

constexpr uint32_t sx127xFskActualFdev(uint32_t fdevHz)
{
    return (uint32_t)(((uint64_t)sx127xFskFdevReg(fdevHz) * SX127X_FXOSC_HZ + (1UL << 18)) >> 19);
}

// RegRxBw / RegAfcBw mantissa/exponent: RxBw = FXOSC / (Mant * 2^(Exp + 2)), Mant 16/20/24.
// Only exact steps are accepted (2.6 kHz .. 250 kHz).
inline bool sx127xFskBwReg(uint32_t bwHz, uint8_t *reg)
{
    static const uint8_t mantissa[3] = {16, 20, 24};
    for (uint8_t exp = 0; exp < 8; exp++)
    {
        for (uint8_t m = 0; m < 3; m++)
        {
            const uint32_t divider = (uint32_t)mantissa[m] << (exp + 2U);
            if (SX127X_FXOSC_HZ % divider == 0 && SX127X_FXOSC_HZ / divider == bwHz)
            {
                if (reg) *reg = (uint8_t)((m << 3) | exp);
                return true;
            }
        }
    }
    return false;
}

inline bool sx127xFskSyncValid(const SX127xFskParams_s &p)
{
    if (p.sync_size < 1 || p.sync_size > 8) return false;
    for (uint8_t i = 0; i < p.sync_size; i++)
        if (p.sync[i] == 0x00) return false; // datasheet: 0x00 sync bytes are not allowed
    return true;
}

// Datasheet limits (SX1276 §2.5.3/§3.4.7 Table 7): BR <= 300 kbps, Fdev 0.6..200 kHz,
// Fdev + BR/2 <= 250 kHz, modulation index 0.5 <= 2*Fdev/BR <= 10, and the
// occupied bandwidth (Carson, single sideband Fdev + BR/2) must fit the RX filter.
inline bool sx127xFskParamsValid(const SX127xFskParams_s &p)
{
    return p.bitrate >= 1200U && p.bitrate <= 300000U &&
        p.fdev_hz >= 600U && p.fdev_hz <= 200000U &&
        p.fdev_hz + p.bitrate / 2U <= 250000U &&
        4U * p.fdev_hz >= p.bitrate && p.fdev_hz <= 5U * p.bitrate &&
        p.fdev_hz + p.bitrate / 2U <= p.rxbw_hz && sx127xFskBwReg(p.rxbw_hz, nullptr) &&
        p.preamble_bytes >= 3 && sx127xFskSyncValid(p);
}
