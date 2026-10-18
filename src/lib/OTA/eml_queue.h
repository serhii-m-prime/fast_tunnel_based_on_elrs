#ifndef H_EML_QUEUE
#define H_EML_QUEUE

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Foreground-owned byte ring. Exact-capacity, no implicit discard on overflow.
// The transport is single-owner; ISR handoff uses EmlMailbox, never this ring.
template <size_t Capacity>
class EmlByteQueue
{
    static_assert(Capacity > 0 && Capacity <= 65535U, "EML queue capacity");
    uint8_t bytes_[Capacity] = {};
    size_t head_ = 0;
    size_t used_ = 0;

public:
    size_t size() const { return used_; }
    size_t free() const { return Capacity - used_; }
    void clear() { head_ = used_ = 0; }
    bool push(const uint8_t *data, size_t length)
    {
        if (length > free() || (length && !data)) return false;
        size_t tail = (head_ + used_) % Capacity;
        const size_t first = length < Capacity - tail ? length : Capacity - tail;
        if (first) memcpy(bytes_ + tail, data, first);
        if (length > first) memcpy(bytes_, data + first, length - first);
        used_ += length;
        return true;
    }
    size_t peek(uint8_t *out, size_t limit, size_t offset = 0) const
    {
        if (!out || offset > used_) return 0;
        const size_t head = (head_ + offset) % Capacity;
        const size_t count = limit < used_ - offset ? limit : used_ - offset;
        const size_t first = count < Capacity - head ? count : Capacity - head;
        if (first) memcpy(out, bytes_ + head, first);
        if (count > first) memcpy(out + first, bytes_, count - first);
        return count;
    }
    bool consume(size_t length)
    {
        if (length > used_) return false;
        head_ = (head_ + length) % Capacity;
        used_ -= length;
        return true;
    }
};

#endif
