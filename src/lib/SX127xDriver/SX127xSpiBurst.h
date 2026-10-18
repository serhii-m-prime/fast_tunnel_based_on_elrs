#pragma once

#include <stdint.h>

#include "SX127xRegs.h"

// SPIEx copies each transaction through the controller data buffer: 16 words
// (W0..W15) on ESP32 and ESP8266, so one transaction carries at most 64 bytes
// including the SX127x address byte. Longer bursts (64-byte EML frames) are
// split into several transactions; hardware CS is released between them.
#define SX127X_SPI_MAX_TRANSFER_BYTES   64U
#define SX127X_SPI_MAX_BURST_DATA_BYTES (SX127X_SPI_MAX_TRANSFER_BYTES - 1U)

// Data bytes carried by the next transaction of a burst with `remaining` bytes left.
__attribute__((always_inline)) static inline uint8_t sx127xSpiChunkLen(uint8_t remaining)
{
    return remaining > SX127X_SPI_MAX_BURST_DATA_BYTES ? (uint8_t)SX127X_SPI_MAX_BURST_DATA_BYTES : remaining;
}

// Register address that continues a burst after `offset` data bytes. RegFifo
// accesses advance RegFifoAddrPtr inside the radio and keep address 0x00; any
// other burst auto-increments the register address.
__attribute__((always_inline)) static inline uint8_t sx127xSpiChunkAddr(uint8_t reg, uint8_t offset)
{
    return reg == SX127X_REG_FIFO ? reg : (uint8_t)(reg + offset);
}

// Burst read/write through an SPIEx-like `spi` (read/write(csMask, wordAlignedBuf, size)).
// Forced inline so the loop lands in the IRAM caller (radio ISR path); copies are plain
// byte loops. numBytes == 0 still issues the single address-only transaction as before.
template <typename Spi>
__attribute__((always_inline)) inline void sx127xSpiBurstRead(Spi &spi, uint8_t csMask, uint8_t reg, uint8_t *data, uint8_t numBytes)
{
    alignas(4) uint8_t buf[SX127X_SPI_MAX_TRANSFER_BYTES];
    uint8_t offset = 0;
    do
    {
        const uint8_t chunk = sx127xSpiChunkLen((uint8_t)(numBytes - offset));
        buf[0] = sx127xSpiChunkAddr(reg, offset) | SPI_READ;

        spi.read(csMask, buf, chunk + 1U);

        for (uint8_t i = 0; i < chunk; i++)
        {
            data[offset + i] = buf[i + 1];
        }
        offset += chunk;
    } while (offset < numBytes);
}

// spi.write() must copy buf into the controller before returning (SPIEx does), so buf is
// refilled for the next chunk while the previous one may still be clocking out.
template <typename Spi>
__attribute__((always_inline)) inline void sx127xSpiBurstWrite(Spi &spi, uint8_t csMask, uint8_t reg, const uint8_t *data, uint8_t numBytes)
{
    alignas(4) uint8_t buf[SX127X_SPI_MAX_TRANSFER_BYTES];
    uint8_t offset = 0;
    do
    {
        const uint8_t chunk = sx127xSpiChunkLen((uint8_t)(numBytes - offset));
        buf[0] = sx127xSpiChunkAddr(reg, offset) | SPI_WRITE;
        for (uint8_t i = 0; i < chunk; i++)
        {
            buf[i + 1] = data[offset + i];
        }

        spi.write(csMask, buf, chunk + 1U);
        offset += chunk;
    } while (offset < numBytes);
}
