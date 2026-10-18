#ifndef H_EML_UART
#define H_EML_UART

#include "eml_transport.h"

struct EmlUartStats_s
{
    uint32_t read = 0, written = 0, discarded_disconnected = 0, refused = 0;
    uint32_t input_stalls = 0, output_stalls = 0;
};

// Port readBytes must be nonblocking (bridge UART timeout=0, sole reader).
// Lock only the bounded memory copy, never serial I/O. One batch per loop.
template <typename Port, typename Queue>
size_t emlReadSerial(Port &port, Queue &queue, bool admitting, EmlUartStats_s &stats, size_t budget = 64)
{
    const int available = port.available();
    if (available <= 0 || !budget) return 0;
    size_t count = (size_t)available < budget ? (size_t)available : budget;
    if (count > 64) count = 64;
    if (admitting && count > queue.free()) count = queue.free();
    if (!count) { ++stats.input_stalls; return 0; }
    uint8_t bytes[64];
    size_t read = port.readBytes(bytes, count);
    if (read > count) read = count;
    stats.read += read;
    if (!admitting) { stats.discarded_disconnected += read; return read; }
    queue.lock();
    const bool accepted = queue.tryPushBytes(bytes, read);
    queue.unlock();
    if (!accepted) stats.refused += read;
    return read;
}

template <typename Port, typename Queue>
size_t emlDrainSerial(Port &port, Queue &queue, EmlUartStats_s &stats, size_t budget = 64)
{
    if (!queue.size() || !budget) return 0;
    const int writable = port.availableForWrite();
    if (writable <= 0) { ++stats.output_stalls; return 0; }
    size_t count = queue.size() < budget ? queue.size() : budget;
    if (count > (size_t)writable) count = (size_t)writable;
    if (count > 64) count = 64;
    uint8_t bytes[64];
    queue.lock();
    const bool copied = queue.peekBytes(bytes, count);
    queue.unlock();
    if (!copied) return 0;
    size_t written = port.write(bytes, count);
    if (written > count) written = count;
    queue.lock(); queue.skip(written); queue.unlock();
    stats.written += written;
    return written;
}

// Adapts the foreground transport to the same batching helpers as legacy FIFO.
template <size_t Capacity, uint8_t WindowSize = 1>
class EmlTransportUart
{
    EmlTransport<Capacity, WindowSize> &transport_;
public:
    explicit EmlTransportUart(EmlTransport<Capacity, WindowSize> &transport) : transport_(transport) {}
    void lock() {} void unlock() {}
    size_t free() const { return transport_.uartSpace(); }
    size_t size() const { return transport_.queuedRx(); }
    bool tryPushBytes(const uint8_t *data, size_t n) { return transport_.acceptUart(data, n) == n; }
    bool peekBytes(uint8_t *data, size_t n) const { return transport_.peekUart(data, n) == n; }
    void skip(size_t n) { transport_.consumeUart(n); }
};

#endif
