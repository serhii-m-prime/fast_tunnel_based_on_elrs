#pragma once

#include "SX127xFsk.h"
#include "SX127xRegs.h"

// Program the FSK packet engine in Standby. The driver and native register-map
// tests use the same writes; no Arduino, heap or ISR work is involved here.
template <typename Hal>
bool sx127xFskConfigurePacketEngine(Hal &hal, const SX127xFskParams_s &params,
                                  uint8_t payloadLength, uint8_t radioNumber)
{
    uint8_t bwReg;
    if (!sx127xFskParamsValid(params) || !sx127xFskBwReg(params.rxbw_hz, &bwReg) ||
        !payloadLength || payloadLength > SX127X_FSK_FIFO_SIZE)
        return false;

    const SX127xFskBitrateReg_s br = sx127xFskBitrateReg(params.bitrate);
    hal.writeRegister(SX127X_FSK_REG_BITRATE_MSB, (uint8_t)(br.reg >> 8), radioNumber);
    hal.writeRegister(SX127X_FSK_REG_BITRATE_LSB, (uint8_t)br.reg, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_BITRATE_FRAC, br.frac, radioNumber);
    const uint16_t fdev = sx127xFskFdevReg(params.fdev_hz);
    hal.writeRegister(SX127X_FSK_REG_FDEV_MSB, (uint8_t)((fdev >> 8) & 0x3F), radioNumber);
    hal.writeRegister(SX127X_FSK_REG_FDEV_LSB, (uint8_t)fdev, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PA_RAMP, params.shaping | SX127X_FSK_PA_RAMP_40_US, radioNumber);
    // Establish a documented initial gain before the preamble-triggered AGC runs.
    hal.writeRegister(SX127X_REG_LNA, SX127X_LNA_GAIN_1 | SX127X_LNA_BOOST_ON, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_RX_CONFIG, SX127X_FSK_RX_CONFIG_DEFAULT, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_RSSI_CONFIG, SX127X_FSK_RSSI_SMOOTHING_32, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_RX_BW, bwReg, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_AFC_BW, bwReg, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_AFC_FEI, SX127X_FSK_AFC_AUTO_CLEAR_ON, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PREAMBLE_DETECT, SX127X_FSK_PREAMBLE_DETECT_2B, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PREAMBLE_MSB, 0, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PREAMBLE_LSB, params.preamble_bytes, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_SYNC_CONFIG,
        SX127X_FSK_AUTO_RESTART_RX | SX127X_FSK_SYNC_ON | (uint8_t)(params.sync_size - 1U), radioNumber);
    alignas(4) uint8_t sync[8];
    for (uint8_t i = 0; i < params.sync_size; ++i) sync[i] = params.sync[i];
    hal.writeRegister(SX127X_FSK_REG_SYNC_VALUE1, sync, params.sync_size, radioNumber);
    // Fixed-length payload, whitening, no address/length byte or hardware CRC.
    hal.writeRegister(SX127X_FSK_REG_PACKET_CONFIG1, SX127X_FSK_PACKET_FIXED | SX127X_FSK_DCFREE_WHITENING, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PACKET_CONFIG2, SX127X_FSK_DATA_MODE_PACKET, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_PAYLOAD_LENGTH, payloadLength, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_FIFO_THRESH, SX127X_FSK_TX_START_FIFO_NOT_EMPTY | 0x0F, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_SEQ_CONFIG1, 0x00, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_DIO_MAPPING1, SX127X_FSK_DIO0_PAYLOAD_PACKET, radioNumber);
    return true;
}
