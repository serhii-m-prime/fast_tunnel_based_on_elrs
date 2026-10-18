#pragma once

#include "targets.h"

#if defined(EML_BRIDGE)
#include "OTA.h"
#include "eml_tdd.h"
#include "eml_qos.h"

/**
 * EML bridge MAC (P4.2-P4.6). In EML_BRIDGE builds this replaces the stock ELRS
 * RF link: radio profile, timer, FHSS, Sync handshake, ARQ and the AirPort UART
 * queues. The coordinator (Master) owns the slot clock, the responder (Slave)
 * follows it; the role comes from options (P9.4), Auto elects it at run time.
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
EmlRole_e emlBridgeRole(); // current, possibly elected, MAC role

struct EmlBridgeDiag_s
{
    uint8_t role, mac_mode, session_state, pll_mask, search_band;
    uint16_t hops;
    uint32_t slot_us, configuration_id, local_id, peer_id, last_hello_configuration_id;
    uint32_t election_phase_ms;
    // Responder search: RSSI range, the last undecodable frame and the last HELLO
    // rejected for its hop (bench diagnostics)
    uint32_t search_frames_seen;
    int8_t search_rssi_min, search_rssi_max, last_bad_rssi, last_off_hop_rssi;
    uint8_t last_bad_length, last_off_hop_channel, sync_channel;
    uint16_t last_off_hop;
    uint8_t last_bad_frame[64];
    uint8_t last_tx_sync_length;
    uint8_t last_tx_sync_frame[64]; // coordinator: last frame prepared for the rendezvous channel
    // Locked mode: last frame the transport rejected, and RX-done time after slot start
    uint8_t last_invalid_frame[64];
    uint8_t last_invalid_length;
    int8_t last_invalid_rssi;
    int32_t last_invalid_offset_us, last_valid_offset_us;
    uint8_t fast_turnaround;
};
EmlBridgeDiag_s emlBridgeDiag();
const EmlTransportStats_s &emlBridgeTransportStats();
const EmlUartStats_s &emlBridgeUartStats();

struct EmlMacStats_s
{
    uint32_t tx_frames, tx_no_frame, tx_overrun, rx_frames, rx_dropped;
    uint32_t acquisitions, lock_losses, late_prepares, sent_unmatched, missed_boundaries;
    uint32_t band_handovers, emergency_searches, role_changes;
    // Bench diagnostics: radio IRQs without a valid packet, and why rendezvous
    // candidates were not accepted by the responder search
    uint32_t rx_irq_errors, search_frames, search_bad_frame, search_not_hello;
    uint32_t search_id_mismatch, search_cfg_mismatch, search_late, search_off_hop;
    uint32_t tx_sync_channel; // frames transmitted on the rendezvous channel
    uint32_t emergency_handover, emergency_timeout, emergency_session_lost;
    uint32_t pump_ok, pump_control, pump_unissued, pump_window, pump_flow, pump_empty, late_missed;
};
const EmlMacStats_s &emlBridgeMacStats();
const EmlTddStats_s &emlBridgeTddStats();
const EmlQosStats_s &emlBridgeQosStats();

// P5.5: PHY-independent link metrics over the last 100 slot pairs (FSK has no SNR)
struct EmlLinkStats_s
{
    uint8_t phy_profile;  // EmlPhyProfile_e in use
    uint8_t band, pll_band_mask;
    uint8_t tdd_mode, arq_window, in_flight;
    uint8_t lq;           // % of expected peer frames received intact
    uint8_t per;          // 100 - lq
    int8_t rssi;          // dBm, averaged
    int8_t snr;           // dB, averaged; invalid for FSK
    bool snr_valid;
    uint8_t peer_lq, peer_per;
    int8_t peer_rssi, peer_snr;
    bool peer_snr_valid;
    uint32_t fsk_fifo_overruns, fsk_pll_waits, fsk_pll_timeouts;
};
EmlLinkStats_s emlBridgeLinkStats();

// Bench diagnostics of the Arduino loop (bridge_main.cpp): one entry per section
enum : uint8_t { EML_LOOP_DRAIN = 0, EML_LOOP_DEVICES, EML_LOOP_READ, EML_LOOP_MAC, EML_LOOP_COMMIT, EML_LOOP_SECTIONS };
struct EmlLoopStats_s
{
    uint32_t loops, commits, total_max_us, heap_min;
    uint32_t section_max_us[EML_LOOP_SECTIONS];
    uint64_t total_sum_us;
};
extern EmlLoopStats_s emlLoopStats;
size_t emlBridgeQosBuffered();
uint32_t emlBridgeTddPeerQueued();

#endif
