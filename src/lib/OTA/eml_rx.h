#pragma once

#include "eml_mailbox.h"

// ISR/foreground shared receive gate. Foreground calls validated() only after
// codec AND transport validation, with radio/timer interrupts excluded.
struct EmlArmedRx_s
{
    EmlRxContext_s context;
    uint32_t start_us;
    uint8_t flags;
    uint8_t length;
    bool valid;
    bool taken;

    __attribute__((always_inline)) inline bool accepts(uint32_t rxUs, uint32_t slotUs) const
    {
        if (!valid || taken) return false;
        if (flags & EML_DESC_SEARCH) return true; // no slot clock until HELLO acquisition
        const int32_t offset = (int32_t)(rxUs - start_us);
        return offset >= 0 && (uint32_t)offset < slotUs;
    }

    __attribute__((always_inline)) inline void validated(const EmlRxContext_s &received)
    {
        // Foreground may finish an old descriptor after an ISR has armed a new
        // slot/session. That result must never close the new receive window.
        if (valid && !(flags & EML_DESC_SEARCH) && context.generation == received.generation &&
            context.slot == received.slot && context.hop == received.hop)
            taken = true;
    }
};
