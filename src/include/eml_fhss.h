#pragma once

#include <stdint.h>

// Band limits are edges of sixty 1 MHz cells. Centres sit 500 kHz inside
// the edges; even a BW500 signal stays inside its band. All arithmetic here
// is integer SX1276 FRF arithmetic, including on native (which otherwise uses Hz).
constexpr uint8_t EML_BAND_COUNT = 4;
constexpr uint8_t EML_CHANNEL_COUNT = 60;
constexpr uint8_t EML_SYNC_CHANNEL = 30;
constexpr uint16_t EML_HOP_COUNT = 240; // four permutations of 60, fits uint8_t index
constexpr uint32_t EML_CHANNEL_STEP_HZ = 1000000UL;
constexpr uint32_t EML_CHANNEL_STEP_REG = 16384UL;
constexpr uint32_t emlBandEdgeHz(uint8_t band)
{
    return band == 0 ? 940000000UL : band == 1 ? 880000000UL : band == 2 ? 810000000UL : 750000000UL;
}
constexpr uint32_t emlBandEndHz(uint8_t band) { return emlBandEdgeHz(band) + 60000000UL; }
constexpr uint32_t emlSx1276Frf(uint32_t hz) { return (uint32_t)(((uint64_t)hz << 19) / 32000000ULL); }
constexpr uint32_t emlBandStartReg(uint8_t band) { return emlSx1276Frf(emlBandEdgeHz(band) + 500000UL); }
constexpr uint32_t emlBandChannelReg(uint8_t band, uint8_t channel)
{
    return emlBandStartReg(band) + (uint32_t)channel * EML_CHANNEL_STEP_REG;
}
static_assert(EML_HOP_COUNT <= 256, "FHSSptr must represent the complete sequence");
static_assert(emlSx1276Frf(1000000UL) == EML_CHANNEL_STEP_REG, "exact 1 MHz FRF step");

inline uint8_t emlFirstBand(uint8_t mask)
{
    for (uint8_t b = 0; b < EML_BAND_COUNT; ++b) if (mask & (1U << b)) return b;
    return EML_BAND_COUNT;
}
