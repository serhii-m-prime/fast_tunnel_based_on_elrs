#pragma once

#include "targets.h"

#if defined(EML_BRIDGE)
#include "OTA.h"

/**
 * EML bridge MAC (P4.2-P4.6). In EML_BRIDGE builds this replaces the stock ELRS
 * RF link: radio profile, timer, FHSS, Sync handshake, ARQ and the AirPort UART
 * queues. ES900TX is the coordinator, ES900RX the responder.
 */

// After Radio.Begin() succeeded: configure the EML PHY, install radio/timer
// callbacks, start the transport and the slot clock (coordinator) or the
// rendezvous search (responder).
void emlBridgeBegin();

// Foreground work, once per main loop: TX/RX descriptors, transport timeouts,
// frame preparation, acquisition/lock tracking and connectionState.
void emlBridgeLoop();

// While suspended the slot clock keeps running but no SPI traffic is issued
// (EEPROM/config commits). Frames are lost and recovered by ARQ.
void emlBridgeSuspend(bool suspend);

// UART <-> transport, bounded batches (see eml_uart.h)
void emlBridgeReadSerial(Stream &port);
void emlBridgeDrainSerial(Stream &port, uint32_t maxBytes = 64);

bool emlBridgeActive();
const EmlTransportStats_s &emlBridgeTransportStats();
const EmlUartStats_s &emlBridgeUartStats();

struct EmlMacStats_s
{
    uint32_t tx_frames, tx_no_frame, tx_overrun, rx_frames, rx_dropped;
    uint32_t acquisitions, lock_losses, late_prepares, sent_unmatched, missed_boundaries;
    uint32_t band_handovers, emergency_searches;
};
const EmlMacStats_s &emlBridgeMacStats();

// P5.5: PHY-independent link metrics over the last 100 slot pairs (FSK has no SNR)
struct EmlLinkStats_s
{
    uint8_t phy_profile;  // EmlPhyProfile_e in use
    uint8_t band, pll_band_mask;
    uint8_t lq;           // % of expected peer frames received intact
    uint8_t per;          // 100 - lq
    int8_t rssi;          // dBm, averaged
    uint32_t fsk_fifo_overruns;
};
EmlLinkStats_s emlBridgeLinkStats();

#endif
