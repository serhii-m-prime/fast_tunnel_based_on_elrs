#ifndef H_EML_QOS
#define H_EML_QOS

#include "eml_frame.h"
#include "eml_queue.h"

// Protocol-aware ingress is deliberately opt-in. RAW retains the original
// byte-for-byte ordering contract. MAVLink/SBUS modes preserve every byte but
// may move a complete high-priority frame ahead of queued normal frames.
enum EmlTrafficMode_e : uint8_t
{
    EML_TRAFFIC_RAW = 0,
    EML_TRAFFIC_MAVLINK2 = 1,
    EML_TRAFFIC_SBUS = 2,
};

struct EmlQosStats_s
{
    uint32_t normal_frames = 0;
    uint32_t priority_frames = 0;
    uint32_t priority_spills = 0;
    uint32_t fragmented_frames = 0;
    uint32_t parser_timeouts = 0;
};

template <size_t Capacity, uint8_t MaxPackets>
class EmlPacketLane
{
    EmlByteQueue<Capacity> bytes_;
    uint16_t lengths_[MaxPackets] = {};
    uint8_t head_ = 0, count_ = 0;

public:
    size_t size() const { return bytes_.size(); }
    size_t free() const { return bytes_.free(); }
    bool empty() const { return count_ == 0; }
    size_t frontSize() const { return empty() ? 0 : lengths_[head_]; }
    bool full() const { return count_ == MaxPackets; }
    size_t slotsFree() const { return MaxPackets - count_; }
    void clear() { bytes_.clear(); head_ = count_ = 0; }

    bool push(const uint8_t *data, size_t length)
    {
        if (!length || length > UINT16_MAX || full() || length > free()) return false;
        if (!bytes_.push(data, length)) return false;
        lengths_[(head_ + count_) % MaxPackets] = (uint16_t)length;
        ++count_;
        return true;
    }

    // Raw streams have no boundaries: extend the newest packet instead of
    // spending one packet slot (and one EML frame) per UART batch.
    bool append(const uint8_t *data, size_t length)
    {
        if (empty()) return push(data, length);
        uint16_t &tail = lengths_[(head_ + count_ - 1U) % MaxPackets];
        if (!length || length > free() || (size_t)tail + length > UINT16_MAX) return push(data, length);
        if (!bytes_.push(data, length)) return false;
        tail = (uint16_t)(tail + length);
        return true;
    }

    size_t peek(uint8_t *out, size_t maximum) const
    {
        if (empty() || !out || !maximum) return 0;
        const size_t count = lengths_[head_] < maximum ? lengths_[head_] : maximum;
        return bytes_.peek(out, count);
    }

    bool consume(size_t count)
    {
        if (empty() || !count || count > lengths_[head_] || !bytes_.consume(count)) return false;
        lengths_[head_] = (uint16_t)(lengths_[head_] - count);
        if (!lengths_[head_])
        {
            head_ = (uint8_t)((head_ + 1U) % MaxPackets);
            --count_;
        }
        return true;
    }
};

// Capacity is a logical admission limit. The normal lane can hold the full
// limit. Both lanes can independently hold that limit, so any admitted byte can
// always enter its classified lane; aggregate admission still never exceeds
// Capacity.
template <size_t Capacity>
class EmlQosIngress
{
    static_assert(Capacity >= 128, "EML QoS ingress capacity");
    static constexpr size_t MAVLINK2_MAX_FRAME = 280;
    static constexpr size_t SBUS_FRAME = 25;
    static constexpr size_t PRIORITY_CAPACITY = Capacity;
    static constexpr uint8_t NORMAL_PACKETS = (uint8_t)(Capacity / 12U + 2U);
    static constexpr uint8_t PRIORITY_PACKETS = (uint8_t)(PRIORITY_CAPACITY / 12U + 2U);
    // Packets one 64-byte UART batch can create in a framing mode (12-byte minimum
    // MAVLink2 frame plus unframed head/tail)
    static constexpr size_t BATCH_PACKETS = 64U / 12U + 2U;

    EmlPacketLane<Capacity, NORMAL_PACKETS> normal_;
    EmlPacketLane<PRIORITY_CAPACITY, PRIORITY_PACKETS> priority_;
    uint8_t staged_[MAVLINK2_MAX_FRAME] = {};
    size_t staged_length_ = 0, expected_length_ = 0;
    uint32_t last_input_ms_ = 0;
    EmlTrafficMode_e mode_ = EML_TRAFFIC_RAW;
    bool candidate_ = false;
    bool fragment_locked_ = false, fragment_high_ = false;
    EmlQosStats_s stats_;

    size_t buffered() const { return normal_.size() + priority_.size() + staged_length_; }

    bool pushPacket(const uint8_t *data, size_t length, bool high)
    {
        if (!length) return true;
        bool queued = false;
        if (high) queued = priority_.push(data, length);
        if (!queued)
        {
            if (high) ++stats_.priority_spills;
            queued = normal_.push(data, length);
        }
        if (!queued) return false; // admission accounting makes this unreachable
        if (high) ++stats_.priority_frames;
        else ++stats_.normal_frames;
        return true;
    }

    bool flushStage(bool high)
    {
        if (!staged_length_) return true;
        const bool ok = pushPacket(staged_, staged_length_, high);
        staged_length_ = expected_length_ = 0;
        candidate_ = false;
        return ok;
    }

    bool mavlinkCritical() const
    {
        if (staged_length_ < 10) return false;
        const uint32_t messageId = (uint32_t)staged_[7] | ((uint32_t)staged_[8] << 8) |
                                   ((uint32_t)staged_[9] << 16);
        return messageId == 69U || messageId == 76U; // MANUAL_CONTROL, COMMAND_LONG
    }

    bool sbusEndValid() const
    {
        if (staged_length_ != SBUS_FRAME) return false;
        const uint8_t end = staged_[SBUS_FRAME - 1U];
        return end == 0x00 || end == 0x04 || end == 0x14 || end == 0x24 || end == 0x34;
    }

    bool acceptMavlinkByte(uint8_t byte)
    {
        if (!candidate_)
        {
            if (byte == 0xFDU)
            {
                if (!flushStage(false)) return false;
                candidate_ = true;
                staged_[staged_length_++] = byte;
            }
            else
            {
                staged_[staged_length_++] = byte;
                if (staged_length_ == EML_MAX_PAYLOAD_LEN && !flushStage(false)) return false;
            }
            return true;
        }

        staged_[staged_length_++] = byte;
        if (staged_length_ == 3U)
        {
            expected_length_ = 10U + staged_[1] + 2U + ((staged_[2] & 0x01U) ? 13U : 0U);
        }
        if (expected_length_ && staged_length_ == expected_length_)
            return flushStage(mavlinkCritical());
        return true;
    }

    bool acceptSbusByte(uint8_t byte)
    {
        if (!candidate_)
        {
            if (byte == 0x0FU)
            {
                if (!flushStage(false)) return false;
                candidate_ = true;
                expected_length_ = SBUS_FRAME;
                staged_[staged_length_++] = byte;
            }
            else
            {
                staged_[staged_length_++] = byte;
                if (staged_length_ == EML_MAX_PAYLOAD_LEN && !flushStage(false)) return false;
            }
            return true;
        }
        staged_[staged_length_++] = byte;
        if (staged_length_ == expected_length_)
            return flushStage(sbusEndValid());
        return true;
    }

public:
    void begin(EmlTrafficMode_e mode)
    {
        clear();
        stats_ = {};
        mode_ = mode <= EML_TRAFFIC_SBUS ? mode : EML_TRAFFIC_RAW;
    }

    void clear()
    {
        normal_.clear(); priority_.clear();
        staged_length_ = expected_length_ = 0;
        candidate_ = fragment_locked_ = fragment_high_ = false;
    }

    size_t size() const { return buffered(); }
    // Admission also needs packet slots: the UART must stall rather than read
    // bytes that accept() could not queue.
    size_t queued() const { return buffered(); }
    size_t free() const
    {
        if (mode_ != EML_TRAFFIC_RAW && normal_.slotsFree() < BATCH_PACKETS) return 0;
        if (mode_ == EML_TRAFFIC_RAW && normal_.full()) return 0;
        return buffered() < Capacity ? Capacity - buffered() : 0;
    }
    EmlTrafficMode_e mode() const { return mode_; }
    const EmlQosStats_s &stats() const { return stats_; }

    bool accept(const uint8_t *data, size_t length, uint32_t nowMs)
    {
        if ((length && !data) || length > free()) return false;
        last_input_ms_ = nowMs;
        if (mode_ == EML_TRAFFIC_RAW)
        {
            if (!length) return true;
            if (!normal_.append(data, length)) return false;
            ++stats_.normal_frames;
            return true;
        }
        for (size_t i = 0; i < length; ++i)
        {
            const bool ok = mode_ == EML_TRAFFIC_MAVLINK2 ? acceptMavlinkByte(data[i]) : acceptSbusByte(data[i]);
            if (!ok) return false;
        }
        return true;
    }

    // Flush unframed raw bytes immediately. An incomplete protocol candidate is
    // retained briefly across UART batches, then released intact to avoid a
    // malformed/truncated frame blocking the bridge indefinitely.
    void flushIdle(uint32_t nowMs, uint32_t candidateTimeoutMs = 20U)
    {
        if (!staged_length_) return;
        if (!candidate_ || nowMs - last_input_ms_ >= candidateTimeoutMs)
        {
            if (candidate_) ++stats_.parser_timeouts;
            flushStage(false);
        }
    }

    size_t peek(uint8_t *out, size_t maximum, bool *high = nullptr) const
    {
        const bool useHigh = fragment_locked_ ? fragment_high_ : !priority_.empty();
        if (high) *high = useHigh;
        return useHigh ? priority_.peek(out, maximum) : normal_.peek(out, maximum);
    }

    size_t frontSize(bool *high = nullptr) const
    {
        const bool useHigh = fragment_locked_ ? fragment_high_ : !priority_.empty();
        if (high) *high = useHigh;
        return useHigh ? priority_.frontSize() : normal_.frontSize();
    }

    bool consume(size_t count, bool high)
    {
        EmlPacketLane<PRIORITY_CAPACITY, PRIORITY_PACKETS> *highLane = &priority_;
        const size_t remaining = high ? highLane->frontSize() : normal_.frontSize();
        const bool consumed = high ? highLane->consume(count) : normal_.consume(count);
        if (consumed && fragment_locked_ && high == fragment_high_ && count == remaining)
            fragment_locked_ = false;
        return consumed;
    }

    void noteFragment(size_t remainingBefore, size_t sent, bool high)
    {
        if (mode_ != EML_TRAFFIC_RAW && sent < remainingBefore)
        {
            ++stats_.fragmented_frames;
            fragment_locked_ = true;
            fragment_high_ = high;
        }
    }
};

template <size_t Capacity>
class EmlQosUart
{
    EmlQosIngress<Capacity> &ingress_;
    uint32_t now_ms_ = 0;

public:
    explicit EmlQosUart(EmlQosIngress<Capacity> &ingress) : ingress_(ingress) {}
    void setNow(uint32_t nowMs) { now_ms_ = nowMs; }
    void lock() {} void unlock() {}
    size_t free() const { return ingress_.free(); }
    bool tryPushBytes(const uint8_t *data, size_t length) { return ingress_.accept(data, length, now_ms_); }
};

#endif
