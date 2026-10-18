#ifndef H_EML_TRANSPORT
#define H_EML_TRANSPORT

#include "eml_queue.h"
#include "eml_sync.h"
#include "eml_control.h"

enum EmlRole_e : uint8_t { EML_COORDINATOR = 0, EML_RESPONDER = 1 };
enum EmlSessionState_e : uint8_t
{
    EML_LISTENING, EML_HELLO, EML_OFFER, EML_ACCEPT, EML_WAIT_FIRST, EML_ACTIVE
};

struct EmlTransportConfig_s
{
    EmlRole_e role;
    uint32_t local_id;
    uint32_t peer_id;
    uint32_t configuration_id;
    uint8_t phy_profile;
    uint8_t frame_size;
    uint16_t hop_count;
    uint16_t retry_slots;
    uint16_t handshake_timeout_slots;
    uint16_t link_timeout_slots;
    uint16_t quiet_slots;
    uint8_t max_frame_age_slots;
};

// Captured by the MAC when a receive is armed, not when a delayed callback runs.
// slot is the extended local schedule, not merely the 16-bit Sync wire field.
struct EmlRxContext_s
{
    uint32_t generation;
    uint32_t slot;
    uint16_t hop;
};

struct EmlTxTicket_s
{
    uint32_t generation;
    uint32_t slot;
    uint32_t serial;
    uint8_t type;
    uint8_t phase;
};

struct EmlTransportStats_s
{
    uint32_t uart_accepted = 0, uart_refused = 0, uart_delivered = 0;
    uint32_t discarded_tx = 0, discarded_rx = 0;
    uint32_t transmitted = 0, retransmitted = 0, received = 0;
    uint32_t duplicates = 0, backpressure = 0, invalid = 0, stale = 0;
    uint32_t ignored_ack = 0, session_resets = 0, sessions_opened = 0;
};

enum EmlReceiveResult_e : uint8_t
{
    EML_RX_CONTROL, EML_RX_ACCEPTED, EML_RX_DUPLICATE, EML_RX_BACKPRESSURE,
    EML_RX_REJECTED, EML_RX_STALE
};

/**
 * One foreground owner, two independent stop-and-wait directions. No ISR,
 * Arduino, heap, timers, SPI, UART or RF calls. The MAC owns the slot/hop clock
 * and supplies freshness metadata; a successful codec alone cannot bind DATA
 * to a session. Configuration/epoch generation must be provisioned by caller.
 */
template <size_t QueueCapacity>
class EmlTransport
{
    static_assert(QueueCapacity >= EML_MAX_PAYLOAD_LEN, "Queue must fit a maximum DATA");
    EmlByteQueue<QueueCapacity> tx_, rx_;
    EmlTransportConfig_s config_ = {};
    EmlTransportStats_s stats_;
    EmlSessionState_e state_ = EML_LISTENING;
    bool configured_ = false, session_ = false, pending_ = false, pendingSent_ = false;
    bool receivedData_ = false, haveRxSlot_ = false, syncSent_ = false, readyReply_ = false;
    uint32_t generation_ = 0, epochCounter_ = 0, txEpoch_ = 0, rxEpoch_ = 0;
    uint32_t candidateTx_ = 0, candidateRx_ = 0, attemptStarted_ = 0, quietStarted_ = 0;
    uint32_t lastGood_ = 0, lastRxSlot_ = 0, lastSyncSent_ = 0, firstSentSlot_ = 0;
    uint32_t serial_ = 0;
    EmlTxTicket_s ticket_ = {};
    uint8_t nextTx_ = 0, pendingSeq_ = 0, pendingLength_ = 0, lastIssued_ = 7;
    uint8_t expectedRx_ = 0, lastAck_ = 7, peerFlow_ = EML_FLOW_CTRL_NORMAL;
    uint8_t phasesSent_ = 0;

    static bool newer(uint32_t a, uint32_t b) { const uint32_t d = a - b; return d && d < 0x80000000UL; }
    uint32_t nextEpoch() { if (++epochCounter_ == 0) ++epochCounter_; return epochCounter_; }
    void newGeneration() { if (++generation_ == 0) ++generation_; ticket_ = {}; haveRxSlot_ = false; }
    bool quiet(uint32_t now) const { return now - quietStarted_ < config_.quiet_slots; }
    void discardTransport()
    {
        stats_.discarded_tx += tx_.size();
        stats_.discarded_rx += rx_.size();
        tx_.clear(); rx_.clear();
        pending_ = pendingSent_ = receivedData_ = false;
        nextTx_ = expectedRx_ = 0;
        lastIssued_ = lastAck_ = 7;
        peerFlow_ = EML_FLOW_CTRL_NORMAL;
    }
    void startAttempt(uint32_t now)
    {
        candidateTx_ = config_.role == EML_COORDINATOR ? nextEpoch() : 0;
        candidateRx_ = 0;
        state_ = config_.role == EML_COORDINATOR ? EML_HELLO : EML_LISTENING;
        attemptStarted_ = quietStarted_ = now;
        syncSent_ = readyReply_ = false; phasesSent_ = 0;
    }
    void commit(uint32_t now, EmlSessionState_e state)
    {
        if (session_) ++stats_.session_resets;
        discardTransport();
        newGeneration();
        txEpoch_ = candidateTx_; rxEpoch_ = candidateRx_;
        session_ = true;
        state_ = state;
        lastGood_ = attemptStarted_ = now;
        syncSent_ = readyReply_ = false; phasesSent_ = 0;
        ++stats_.sessions_opened;
    }
    bool sameKey(const EmlSync_s &s, uint32_t t, uint32_t r) const
    {
        return config_.role == EML_COORDINATOR ? s.sender_epoch == r && s.receiver_epoch == t :
                                               s.sender_epoch == t && s.receiver_epoch == r;
    }
    EmlReceiveResult_e reject() { ++stats_.invalid; return EML_RX_REJECTED; }
    EmlReceiveResult_e sync(const EmlFrameView_s &frame, const EmlRxContext_s &context, uint32_t now)
    {
        EmlSync_s s = {};
        if (frame.header.payload_len != EML_SYNC_PAYLOAD_SIZE || frame.header.seq_num != 7 ||
            frame.header.ack_num != 7 || frame.header.flow_ctrl != EML_FLOW_CTRL_PAUSE ||
            emlSyncDecode(frame.payload, frame.header.payload_len, &s) != EML_SYNC_OK ||
            s.sender_id != config_.peer_id || s.receiver_id != config_.local_id ||
            s.configuration_id != config_.configuration_id || s.phy_profile != config_.phy_profile ||
            s.frame_size != config_.frame_size || s.hop_index != context.hop ||
            s.slot_counter != (uint16_t)context.slot)
            return reject();
        if (config_.role == EML_COORDINATOR)
        {
            if (s.phase == EML_SYNC_OFFER && s.receiver_epoch == candidateTx_ &&
                (state_ == EML_HELLO || state_ == EML_ACCEPT) && (phasesSent_ & (1U << EML_SYNC_HELLO)))
            {
                if (state_ == EML_ACCEPT && candidateRx_ != s.sender_epoch) return reject();
                candidateRx_ = s.sender_epoch;
                state_ = EML_ACCEPT; syncSent_ = false; ticket_.serial = 0;
            }
            else if (s.phase == EML_SYNC_READY && state_ == EML_ACCEPT &&
                     sameKey(s, candidateTx_, candidateRx_) && !quiet(now) && (phasesSent_ & (1U << EML_SYNC_ACCEPT)))
                commit(now, EML_ACTIVE);
            else if (!(session_ && s.phase == EML_SYNC_READY && sameKey(s, txEpoch_, rxEpoch_)))
                return reject();
        }
        else
        {
            if (s.phase == EML_SYNC_HELLO)
            {
                if (session_ && s.sender_epoch == txEpoch_) return EML_RX_CONTROL;
                if (state_ != EML_OFFER || candidateTx_ != s.sender_epoch)
                {
                    candidateTx_ = s.sender_epoch; candidateRx_ = nextEpoch();
                    state_ = EML_OFFER; attemptStarted_ = quietStarted_ = now;
                    phasesSent_ = 0;
                }
                syncSent_ = false; ticket_.serial = 0;
            }
            else if (s.phase == EML_SYNC_ACCEPT && state_ == EML_OFFER &&
                     sameKey(s, candidateTx_, candidateRx_) && !quiet(now) && (phasesSent_ & (1U << EML_SYNC_OFFER)))
                commit(now, EML_WAIT_FIRST);
            else if (s.phase == EML_SYNC_ACCEPT && session_ && sameKey(s, txEpoch_, rxEpoch_) &&
                     (state_ == EML_WAIT_FIRST || state_ == EML_ACTIVE))
            {
                readyReply_ = state_ == EML_ACTIVE;
                syncSent_ = false; ticket_.serial = 0;
            }
            else return reject();
        }
        return EML_RX_CONTROL;
    }

public:
    static bool validConfig(const EmlTransportConfig_s &c)
    {
        return c.role <= EML_RESPONDER && c.local_id && c.peer_id && c.local_id != c.peer_id &&
            c.configuration_id && (c.phy_profile == EML_PHY_LORA_500_SF6_CR45 || c.phy_profile == EML_PHY_FSK_250) &&
            (c.frame_size == 32 || c.frame_size == 64) && c.hop_count && c.retry_slots >= 2 &&
            c.max_frame_age_slots && c.max_frame_age_slots < 2U * EML_SEQ_MODULO &&
            c.quiet_slots > c.max_frame_age_slots && c.handshake_timeout_slots > c.quiet_slots + c.retry_slots &&
            c.link_timeout_slots > c.handshake_timeout_slots && c.link_timeout_slots < 32768;
    }
    bool begin(const EmlTransportConfig_s &config, uint32_t freshEpochSeed, uint32_t now)
    {
        if (!validConfig(config) || !freshEpochSeed) return false;
        if (session_) ++stats_.session_resets;
        discardTransport();
        config_ = config; configured_ = true; session_ = false;
        epochCounter_ = freshEpochSeed - 1U;
        newGeneration(); startAttempt(now);
        return true;
    }
    void reset(uint32_t now)
    {
        if (!configured_) return;
        if (session_) ++stats_.session_resets;
        discardTransport(); session_ = false;
        newGeneration(); startAttempt(now);
    }
    void tick(uint32_t now)
    {
        if (!configured_) return;
        if (session_ && now - lastGood_ >= config_.link_timeout_slots) { reset(now); return; }
        if (state_ != EML_ACTIVE && state_ != EML_LISTENING &&
            now - attemptStarted_ >= config_.handshake_timeout_slots)
        {
            if (session_ && state_ == EML_OFFER)
            {
                state_ = EML_ACTIVE; candidateTx_ = txEpoch_; candidateRx_ = rxEpoch_;
                syncSent_ = false; ticket_.serial = 0;
            }
            else reset(now);
        }
    }
    EmlSessionState_e state() const { return state_; }
    bool active() const { return state_ == EML_ACTIVE; }
    uint32_t generation() const { return generation_; }
    uint8_t nextTxSequence() const { return nextTx_; }
    uint8_t expectedRxSequence() const { return expectedRx_; }
    uint8_t acknowledgedRxSequence() const { return lastAck_; }
    bool pending() const { return pending_; }
    size_t queuedTx() const { return tx_.size(); }
    size_t queuedRx() const { return rx_.size(); }
    size_t uartSpace() const { return active() ? tx_.free() : 0; }
    const EmlTransportStats_s &stats() const { return stats_; }
    size_t acceptUart(const uint8_t *data, size_t length)
    {
        if (!data && length) return 0;
        const size_t n = active() ? (length < tx_.free() ? length : tx_.free()) : 0;
        tx_.push(data, n);
        stats_.uart_accepted += n; stats_.uart_refused += length - n;
        return n;
    }
    size_t peekUart(uint8_t *out, size_t limit) const { return rx_.peek(out, limit); }
    bool consumeUart(size_t count)
    {
        if (!rx_.consume(count)) return false;
        stats_.uart_delivered += count;
        return true;
    }
    uint8_t flowControl() const
    {
        const size_t maximum = emlMaxPayloadForFrameSize(config_.frame_size);
        if (rx_.free() < maximum) return EML_FLOW_CTRL_PAUSE;
        if (rx_.free() < 2U * maximum || rx_.size() * 4U >= QueueCapacity * 3U) return EML_FLOW_CTRL_CRITICAL;
        if (rx_.free() < 4U * maximum || rx_.size() * 2U >= QueueCapacity) return EML_FLOW_CTRL_HIGH_WATER;
        return EML_FLOW_CTRL_NORMAL;
    }
    bool prepare(uint32_t slot, uint16_t hop, uint8_t *out, size_t capacity, EmlTxTicket_s *ticket,
                 const EmlBandMessage_s *control = nullptr)
    {
        tick(slot);
        if (!configured_ || !out || !ticket || capacity < config_.frame_size ||
            hop >= config_.hop_count || (slot & 1U) != config_.role) return false;
        EmlHeader_s header = {EML_PKT_TYPE_KEEPALIVE, lastIssued_, lastAck_, 0, flowControl()};
        uint8_t phase = 0;
        if (!active() || readyReply_)
        {
            if (state_ == EML_LISTENING ||
                (syncSent_ && slot - lastSyncSent_ < config_.retry_slots)) return false;
            phase = state_ == EML_HELLO ? EML_SYNC_HELLO : state_ == EML_OFFER ? EML_SYNC_OFFER :
                    state_ == EML_ACCEPT ? EML_SYNC_ACCEPT : EML_SYNC_READY;
            if (phase == EML_SYNC_READY && quiet(slot)) return false;
            EmlSync_s s = {EML_PROTOCOL_VERSION, phase, config_.phy_profile, config_.frame_size,
                config_.local_id, config_.peer_id,
                config_.role == EML_COORDINATOR ? candidateTx_ : candidateRx_,
                phase == EML_SYNC_HELLO ? 0U : config_.role == EML_COORDINATOR ? candidateRx_ : candidateTx_,
                config_.configuration_id, (uint16_t)slot, hop};
            if (emlSyncEncode(&s, out + 2, capacity - 2) != EML_SYNC_OK) return false;
            header = {EML_PKT_TYPE_SYNC, 7, 7, EML_SYNC_PAYLOAD_SIZE, EML_FLOW_CTRL_PAUSE};
        }
        else if (control && emlBandMessageValid(*control) && (control->op & 1U) == config_.role)
        {
            phase = control->op;
            emlBandMessageWrite(*control, out + 2);
            emlSyncWrite32(out + 6, config_.local_id);
            emlSyncWrite32(out + 10, config_.peer_id);
            emlSyncWrite32(out + 14, config_.role == EML_COORDINATOR ? txEpoch_ : rxEpoch_);
            emlSyncWrite32(out + 18, config_.role == EML_COORDINATOR ? rxEpoch_ : txEpoch_);
            emlSyncWrite32(out + 22, config_.configuration_id);
            header = {EML_PKT_TYPE_SYNC, 7, 7, EML_SYNC_PAYLOAD_SIZE, EML_FLOW_CTRL_PAUSE};
        }
        else if (!quiet(slot) && (pending_ || (tx_.size() && peerFlow_ != EML_FLOW_CTRL_PAUSE)))
        {
            const uint8_t length = pending_ ? pendingLength_ : (uint8_t)(tx_.size() < emlMaxPayloadForFrameSize(config_.frame_size) ? tx_.size() : emlMaxPayloadForFrameSize(config_.frame_size));
            tx_.peek(out + 2, length);
            header = {EML_PKT_TYPE_DATA, pending_ ? pendingSeq_ : nextTx_, lastAck_, length, flowControl()};
        }
        if (emlFrameEncode(&header, out + 2, header.payload_len, out, capacity, config_.frame_size) != EML_FRAME_OK)
            return false;
        if (header.pkt_type == EML_PKT_TYPE_DATA && !pending_)
        {
            pending_ = true; pendingSent_ = false;
            pendingSeq_ = nextTx_; pendingLength_ = header.payload_len; lastIssued_ = pendingSeq_;
        }
        if (++serial_ == 0) ++serial_;
        ticket_ = {generation_, slot, serial_, header.pkt_type, phase};
        *ticket = ticket_;
        return true;
    }
    bool canSend(const EmlTxTicket_s &ticket, uint32_t slot) const
    {
        return configured_ && ticket.generation == generation_ && ticket.serial == ticket_.serial &&
            ticket.serial && ticket.slot == slot && ticket.slot == ticket_.slot && ticket.type == ticket_.type &&
            ticket.phase == ticket_.phase && (ticket.type != EML_PKT_TYPE_DATA || (active() && pending_));
    }
    bool sent(const EmlTxTicket_s &ticket, uint32_t slot)
    {
        if (!canSend(ticket, slot)) return false;
        ++stats_.transmitted;
        if (ticket.type == EML_PKT_TYPE_SYNC && ticket.phase <= EML_SYNC_READY)
        {
            syncSent_ = true; lastSyncSent_ = slot; readyReply_ = false;
            phasesSent_ |= (uint8_t)(1U << ticket.phase);
        }
        if (ticket.type == EML_PKT_TYPE_DATA)
        {
            if (pendingSent_) ++stats_.retransmitted;
            else { pendingSent_ = true; firstSentSlot_ = slot; }
        }
        ticket_.serial = 0; // completion is exactly once
        return true;
    }
    EmlReceiveResult_e receive(const uint8_t *bytes, size_t length, const EmlRxContext_s &context, uint32_t now)
    {
        tick(now);
        if (!configured_) return reject();
        const uint32_t age = now - context.slot;
        if (context.generation != generation_ || age > config_.max_frame_age_slots ||
            (haveRxSlot_ && !newer(context.slot, lastRxSlot_)))
        { ++stats_.stale; return EML_RX_STALE; }
        if (context.hop >= config_.hop_count || (context.slot & 1U) == config_.role) return reject();
        EmlFrameView_s frame = {};
        if (emlFrameDecode(bytes, length, config_.frame_size, &frame) != EML_FRAME_OK) return reject();
        if (frame.header.pkt_type == EML_PKT_TYPE_SYNC)
        {
            if (frame.header.payload_len == EML_SYNC_PAYLOAD_SIZE && frame.payload[1] >= EML_BAND_PROPOSE)
            {
                EmlBandMessage_s m;
                const uint8_t *p = frame.payload;
                if (!active() || quiet(now) || frame.header.seq_num != 7 || frame.header.ack_num != 7 ||
                    frame.header.flow_ctrl != EML_FLOW_CTRL_PAUSE || !emlBandMessageRead(p, frame.header.payload_len, &m) ||
                    (m.op & 1U) == config_.role || emlSyncRead32(p + 4) != config_.peer_id ||
                    emlSyncRead32(p + 8) != config_.local_id || emlSyncRead32(p + 20) != config_.configuration_id ||
                    emlSyncRead32(p + 12) != (config_.role == EML_COORDINATOR ? rxEpoch_ : txEpoch_) ||
                    emlSyncRead32(p + 16) != (config_.role == EML_COORDINATOR ? txEpoch_ : rxEpoch_)) return reject();
                lastRxSlot_ = context.slot; haveRxSlot_ = true; lastGood_ = now; ++stats_.received;
                return EML_RX_CONTROL;
            }
            const uint32_t generationBefore = generation_;
            const EmlReceiveResult_e result = sync(frame, context, now);
            if (result == EML_RX_CONTROL && generationBefore == generation_)
            { lastRxSlot_ = context.slot; haveRxSlot_ = true; }
            return result;
        }
        if (state_ == EML_WAIT_FIRST && config_.role == EML_RESPONDER && (phasesSent_ & (1U << EML_SYNC_READY)) &&
            frame.header.ack_num == 7 && frame.header.seq_num == (frame.header.pkt_type == EML_PKT_TYPE_DATA ? 0 : 7))
        { state_ = EML_ACTIVE; lastGood_ = now; }
        if (!active() || quiet(now)) return reject();
        const bool data = frame.header.pkt_type == EML_PKT_TYPE_DATA;
        const bool duplicate = data && receivedData_ && frame.header.seq_num == lastAck_;
        if (data && !duplicate && frame.header.seq_num != expectedRx_) return reject();
        lastRxSlot_ = context.slot; haveRxSlot_ = true; lastGood_ = now; ++stats_.received;
        peerFlow_ = frame.header.flow_ctrl;
        if (pending_ && pendingSent_ && frame.header.ack_num == pendingSeq_ && newer(context.slot, firstSentSlot_))
        {
            tx_.consume(pendingLength_); nextTx_ = emlSeqNext(pendingSeq_);
            pending_ = pendingSent_ = false;
        }
        else if (pending_ && frame.header.ack_num != ((nextTx_ - 1U) & EML_SEQ_MASK)) ++stats_.ignored_ack;
        if (duplicate) { ++stats_.duplicates; return EML_RX_DUPLICATE; }
        if (data)
        {
            if (!rx_.push(frame.payload, frame.header.payload_len))
            { ++stats_.backpressure; return EML_RX_BACKPRESSURE; }
            lastAck_ = frame.header.seq_num; expectedRx_ = emlSeqNext(expectedRx_); receivedData_ = true;
        }
        return EML_RX_ACCEPTED;
    }
};

#endif
