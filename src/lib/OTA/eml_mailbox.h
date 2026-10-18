#ifndef H_EML_MAILBOX
#define H_EML_MAILBOX

#include "eml_transport.h"

struct EmlRadioDescriptor_s
{
    uint8_t bytes[EML_FRAME_SIZE_64];
    uint8_t length;
    EmlRxContext_s context;
    EmlTxTicket_s ticket; // zero for RX, exact prepare() ticket for a TX handoff
    uint32_t rx_us;         // RX: micros() at the RX-done callback
    uint32_t slot_start_us; // RX: local boundary of context.slot (MAC phase tracking)
    uint8_t flags;          // RX: EML_DESC_* MAC flags
    int8_t rssi;            // RX: packet RSSI in dBm (LoRa packet RSSI, FSK RSSI at PayloadReady)
};

#define EML_DESC_SEARCH 0x01U // received while the responder was unlocked; context.slot/hop unknown

/**
 * Two-entry SPSC handoff for ISR <-> foreground (one mailbox per direction).
 * All storage and caller buffers must be in DRAM. Always-inline bounded byte
 * copies avoid flash memcpy; aligned 32-bit acquire/release loads/stores do
 * not require libatomic on Xtensa. Producer owns head, consumer owns tail.
 * Caller stamps the RX generation when arming reception; never relabel an old
 * callback using the current generation. Full means reject the NEW frame.
 */
class EmlMailbox
{
    EmlRadioDescriptor_s entries_[2] = {};
    alignas(4) uint32_t head_ = 0, tail_ = 0;

public:
    __attribute__((always_inline)) inline bool push(const uint8_t *bytes, size_t length, const EmlRxContext_s &context,
                                                   const EmlTxTicket_s *ticket = nullptr,
                                                   uint32_t rxUs = 0, uint32_t slotStartUs = 0, uint8_t flags = 0,
                                                   int8_t rssi = 0)
    {
        if (!bytes || (length != 32 && length != 64)) return false;
        if (ticket && (!ticket->serial || ticket->generation != context.generation || ticket->slot != context.slot)) return false;
        const uint32_t head = __atomic_load_n(&head_, __ATOMIC_RELAXED);
        const uint32_t tail = __atomic_load_n(&tail_, __ATOMIC_ACQUIRE);
        if (head - tail >= 2U) return false;
        EmlRadioDescriptor_s &entry = entries_[head & 1U];
        volatile uint8_t *out = entry.bytes;
        for (size_t i = 0; i < length; ++i) out[i] = bytes[i];
        entry.length = (uint8_t)length;
        entry.context.generation = context.generation;
        entry.context.slot = context.slot;
        entry.context.hop = context.hop;
        entry.ticket.generation = ticket ? ticket->generation : 0;
        entry.ticket.slot = ticket ? ticket->slot : 0;
        entry.ticket.serial = ticket ? ticket->serial : 0;
        entry.ticket.type = ticket ? ticket->type : 0;
        entry.ticket.phase = ticket ? ticket->phase : 0;
        entry.rx_us = rxUs;
        entry.slot_start_us = slotStartUs;
        entry.flags = flags;
        entry.rssi = rssi;
        __atomic_store_n(&head_, head + 1U, __ATOMIC_RELEASE);
        return true;
    }
    __attribute__((always_inline)) inline bool pop(EmlRadioDescriptor_s *out)
    {
        if (!out) return false;
        const uint32_t tail = __atomic_load_n(&tail_, __ATOMIC_RELAXED);
        const uint32_t head = __atomic_load_n(&head_, __ATOMIC_ACQUIRE);
        if (tail == head) return false;
        const EmlRadioDescriptor_s &entry = entries_[tail & 1U];
        const volatile uint8_t *in = entry.bytes;
        volatile uint8_t *dest = out->bytes;
        for (size_t i = 0; i < entry.length; ++i) dest[i] = in[i];
        out->length = entry.length;
        out->context.generation = entry.context.generation;
        out->context.slot = entry.context.slot;
        out->context.hop = entry.context.hop;
        out->ticket.generation = entry.ticket.generation;
        out->ticket.slot = entry.ticket.slot;
        out->ticket.serial = entry.ticket.serial;
        out->ticket.type = entry.ticket.type;
        out->ticket.phase = entry.ticket.phase;
        out->rx_us = entry.rx_us;
        out->slot_start_us = entry.slot_start_us;
        out->flags = entry.flags;
        out->rssi = entry.rssi;
        __atomic_store_n(&tail_, tail + 1U, __ATOMIC_RELEASE);
        return true;
    }
    // Only after stopping the producer/consumer and quiescing radio IRQs.
    void clearQuiesced()
    {
        __atomic_store_n(&head_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&tail_, 0U, __ATOMIC_RELEASE);
    }
};

#endif
