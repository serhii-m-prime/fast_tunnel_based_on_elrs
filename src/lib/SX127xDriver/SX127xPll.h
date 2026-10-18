#pragma once

#include <stdint.h>

// Startup-only bounded polling; injected I/O also exercises timeouts and stale
// lock bits on native. FSK RegIrqFlags1: ModeReady bit7, PllLock bit4.
template <typename Read, typename Clock, typename Pause>
bool sx127xWaitPll(Read read, Clock clock, Pause pause, uint32_t timeoutUs, bool locked)
{
    const uint32_t start = clock();
    uint32_t stable = start;
    bool seen = false;
    do
    {
        const uint8_t flags = read();
        const uint32_t now = clock();
        const bool ok = flags != 0xFF && (locked ? (flags & 0x90U) == 0x90U : !(flags & 0x10U));
        if (!ok) seen = false;
        else if (!locked) return true;
        else if (!seen) { stable = now; seen = true; }
        else if (now - stable >= 100U) return true;
        pause();
    } while (clock() - start < timeoutUs);
    return false;
}
