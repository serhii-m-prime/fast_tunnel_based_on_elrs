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
};
const EmlMacStats_s &emlBridgeMacStats();

#endif
