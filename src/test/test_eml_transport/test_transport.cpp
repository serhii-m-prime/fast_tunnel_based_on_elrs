#include <unity.h>
#include <algorithm>
#include <vector>
#include <deque>
#include "eml_transport.h"
#include "eml_mailbox.h"
#include "eml_uart.h"

static_assert(sizeof(EmlTransport<4096>) < 9000, "ESP32 transport budget");
static_assert(sizeof(EmlTransport<1024>) + 2 * sizeof(EmlMailbox) < 3000, "ESP8285 transport/mailbox budget");

static EmlTransportConfig_s config(EmlRole_e role, uint8_t frameSize = 64)
{
    const EmlTransportConfig_s c = {role, role == EML_COORDINATOR ? 1U : 2U,
        role == EML_COORDINATOR ? 2U : 1U, 0x10203040U, EML_PHY_LORA_500_SF6_CR45,
        frameSize, 7, 2, 40, 100, 3, 2};
    return c;
}

template <size_t Capacity = 128>
struct Pair
{
    EmlTransport<Capacity> tx, rx;
    uint32_t slot;
    uint8_t last[64] = {};
    size_t length = 64;
    EmlReceiveResult_e result = EML_RX_REJECTED;
    explicit Pair(uint8_t frameSize = 64, uint32_t start = 0) : slot(start), length(frameSize)
    {
        TEST_ASSERT_TRUE(tx.begin(config(EML_COORDINATOR, frameSize), 0x11111111, slot));
        TEST_ASSERT_TRUE(rx.begin(config(EML_RESPONDER, frameSize), 0x22222222, slot));
    }
    bool step(bool drop = false, bool corrupt = false)
    {
        tx.tick(slot); rx.tick(slot);
        auto &from = slot & 1U ? rx : tx;
        auto &to = slot & 1U ? tx : rx;
        EmlTxTicket_s ticket = {};
        const uint16_t hop = (uint16_t)((slot / 2U) % 7U);
        const EmlRxContext_s context = {to.generation(), slot, hop};
        const bool built = from.prepare(slot, hop, last, sizeof(last), &ticket);
        if (built)
        {
            TEST_ASSERT_TRUE(from.sent(ticket, slot));
            TEST_ASSERT_FALSE(from.sent(ticket, slot));
            if (corrupt) last[length - 1U] ^= 0x80;
            if (!drop) result = to.receive(last, length, context, slot);
        }
        ++slot;
        return built;
    }
    void establish()
    {
        for (unsigned i = 0; i < 300 && (!tx.active() || !rx.active()); ++i) step();
        TEST_ASSERT_TRUE(tx.active()); TEST_ASSERT_TRUE(rx.active());
    }
    void untilEven() { if (slot & 1U) step(); }
};

void test_handshake_and_every_lost_phase()
{
    for (uint8_t size : {32, 64})
    {
        for (uint8_t phase = 0; phase < 4; ++phase)
        {
            Pair<> p(size);
            bool lost = false;
            for (unsigned i = 0; i < 300 && (!p.tx.active() || !p.rx.active()); ++i)
            {
                auto &from = p.slot & 1U ? p.rx : p.tx;
                auto &to = p.slot & 1U ? p.tx : p.rx;
                EmlTxTicket_s ticket = {};
                uint8_t bytes[64];
                const uint16_t hop = (uint16_t)((p.slot / 2U) % 7U);
                if (from.prepare(p.slot, hop, bytes, sizeof(bytes), &ticket))
                {
                    TEST_ASSERT_TRUE(from.sent(ticket, p.slot));
                    if (!lost && ticket.type == EML_PKT_TYPE_SYNC && ticket.phase == phase) lost = true;
                    else to.receive(bytes, size, {to.generation(), p.slot, hop}, p.slot);
                }
                ++p.slot;
            }
            TEST_ASSERT_TRUE(lost);
            TEST_ASSERT_TRUE(p.tx.active()); TEST_ASSERT_TRUE(p.rx.active());
            TEST_ASSERT_EQUAL(0, p.tx.nextTxSequence()); TEST_ASSERT_EQUAL(0, p.rx.expectedRxSequence());
            TEST_ASSERT_EQUAL(7, p.tx.acknowledgedRxSequence());
        }
    }
}

void test_pending_bytes_survive_data_and_ack_loss()
{
    Pair<64> p; p.establish(); p.untilEven();
    uint8_t bytes[60];
    for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;
    TEST_ASSERT_EQUAL(60, p.tx.acceptUart(bytes, sizeof(bytes)));
    p.step(true); // DATA lost
    TEST_ASSERT_TRUE(p.tx.pending()); TEST_ASSERT_EQUAL(60, p.tx.queuedTx());
    p.step(); p.step(); // heartbeat, then retry DATA received
    TEST_ASSERT_EQUAL(60, p.rx.queuedRx());
    TEST_ASSERT_EQUAL(0, p.rx.acknowledgedRxSequence());
    p.step(true); // ACK lost while RX delivery queue is PAUSE/full
    p.step(); // duplicate retry must be accepted without requiring 60 more bytes
    TEST_ASSERT_EQUAL(EML_RX_DUPLICATE, p.result);
    TEST_ASSERT_EQUAL(60, p.rx.queuedRx()); TEST_ASSERT_EQUAL(60, p.tx.queuedTx());
    p.step();
    TEST_ASSERT_FALSE(p.tx.pending()); TEST_ASSERT_EQUAL(0, p.tx.queuedTx());
    uint8_t out[60]; TEST_ASSERT_EQUAL(60, p.rx.peekUart(out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(bytes, out, sizeof(out));
    TEST_ASSERT_TRUE(p.rx.consumeUart(60));
    TEST_ASSERT_EQUAL(1, p.rx.stats().duplicates);
    TEST_ASSERT_GREATER_THAN(0, p.tx.stats().retransmitted);
}

void test_unsent_and_wrong_ack_cannot_release_bytes()
{
    Pair<> p; p.establish(); p.untilEven();
    const uint8_t bytes[] = {0, 0xFF, 0x7E};
    p.tx.acceptUart(bytes, sizeof(bytes));
    EmlTxTicket_s ticket;
    uint8_t data[64];
    TEST_ASSERT_TRUE(p.tx.prepare(p.slot, (p.slot / 2U) % 7U, data, sizeof(data), &ticket));
    TEST_ASSERT_FALSE(p.tx.sent(ticket, p.slot + 2U));
    uint8_t ack[64];
    EmlHeader_s header = {EML_PKT_TYPE_KEEPALIVE, 7, 0, 0, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_FRAME_OK, emlFrameEncode(&header, nullptr, 0, ack, sizeof(ack), 64));
    ++p.slot;
    p.tx.receive(ack, 64, {p.tx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot);
    TEST_ASSERT_EQUAL(sizeof(bytes), p.tx.queuedTx()); // no actual DATA transmission
    ++p.slot;
    TEST_ASSERT_TRUE(p.tx.prepare(p.slot, (p.slot / 2U) % 7U, data, sizeof(data), &ticket));
    TEST_ASSERT_TRUE(p.tx.sent(ticket, p.slot));
    ++p.slot;
    header.ack_num = 3;
    emlFrameEncode(&header, nullptr, 0, ack, sizeof(ack), 64);
    p.tx.receive(ack, 64, {p.tx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot);
    TEST_ASSERT_EQUAL(sizeof(bytes), p.tx.queuedTx());
    p.slot += 2;
    header.ack_num = 0;
    emlFrameEncode(&header, nullptr, 0, ack, sizeof(ack), 64);
    p.tx.receive(ack, 64, {p.tx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot);
    TEST_ASSERT_EQUAL(0, p.tx.queuedTx()); TEST_ASSERT_EQUAL(1, p.tx.nextTxSequence());
}

void test_rx_backpressure_does_not_ack_unstored_payload()
{
    Pair<64> p; p.establish(); p.untilEven();
    uint8_t bytes[60] = {};
    p.tx.acceptUart(bytes, sizeof(bytes)); p.step(); p.step();
    TEST_ASSERT_EQUAL(60, p.rx.queuedRx());
    TEST_ASSERT_EQUAL(EML_FLOW_CTRL_PAUSE, p.rx.flowControl());
    p.tx.acceptUart(bytes, sizeof(bytes));
    for (unsigned i = 0; i < 4; ++i) p.step();
    TEST_ASSERT_FALSE(p.tx.pending()); // remote PAUSE prevents a new window
    TEST_ASSERT_EQUAL(60, p.tx.queuedTx());
    p.untilEven();
    uint8_t forged[64];
    const EmlHeader_s header = {EML_PKT_TYPE_DATA, 1, 7, 60, EML_FLOW_CTRL_NORMAL};
    emlFrameEncode(&header, bytes, 60, forged, sizeof(forged), 64);
    const EmlRxContext_s context = {p.rx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)};
    TEST_ASSERT_EQUAL(EML_RX_BACKPRESSURE, p.rx.receive(forged, 64, context, p.slot));
    TEST_ASSERT_EQUAL(0, p.rx.acknowledgedRxSequence());
    TEST_ASSERT_EQUAL(1, p.rx.expectedRxSequence());
    TEST_ASSERT_EQUAL(60, p.rx.queuedRx());
    p.rx.consumeUart(60);
    p.slot += 2;
    TEST_ASSERT_EQUAL(EML_RX_ACCEPTED, p.rx.receive(forged, 64,
        {p.rx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot));
    TEST_ASSERT_EQUAL(1, p.rx.acknowledgedRxSequence());
}

void test_stale_generation_age_and_out_of_order_descriptors()
{
    Pair<> p; p.establish(); p.untilEven();
    const uint8_t bytes[] = {1, 2, 3};
    p.tx.acceptUart(bytes, sizeof(bytes));
    uint8_t frame[64]; EmlTxTicket_s ticket;
    p.tx.prepare(p.slot, (p.slot / 2U) % 7U, frame, 64, &ticket);
    const EmlRxContext_s saved = {p.rx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)};
    TEST_ASSERT_EQUAL(EML_RX_STALE, p.rx.receive(frame, 64, saved, p.slot + 3U));
    TEST_ASSERT_EQUAL(0, p.rx.queuedRx());
    p.rx.reset(p.slot);
    TEST_ASSERT_EQUAL(EML_RX_STALE, p.rx.receive(frame, 64, saved, p.slot));
    TEST_ASSERT_EQUAL(0, p.rx.queuedRx());
    p.tx.reset(p.slot); p.establish(); p.untilEven();
    p.tx.acceptUart(bytes, sizeof(bytes)); p.step();
    const uint8_t ackBefore = p.rx.acknowledgedRxSequence();
    const EmlRxContext_s old = {p.rx.generation(), p.slot - 1U, (uint16_t)(((p.slot - 1U) / 2U) % 7U)};
    TEST_ASSERT_EQUAL(EML_RX_STALE, p.rx.receive(p.last, 64, old, p.slot));
    TEST_ASSERT_EQUAL(ackBefore, p.rx.acknowledgedRxSequence());
}

static uint32_t randomState = 1;
static uint32_t random32() { randomState = randomState * 1664525U + 1013904223U; return randomState; }

void test_bidirectional_binary_streams_under_noise_and_uart_pauses()
{
    for (uint8_t frameSize : {32, 64})
    {
        Pair<> p(frameSize); p.establish();
        std::vector<uint8_t> sourceTx(4096), sourceRx(3073), deliveredTx, deliveredRx;
        for (size_t i = 0; i < sourceTx.size(); ++i) sourceTx[i] = (uint8_t)(i * 73U);
        for (size_t i = 0; i < sourceRx.size(); ++i) sourceRx[i] = (uint8_t)(i ^ (i >> 3U));
        size_t acceptedTx = 0, acceptedRx = 0;
        randomState = 0x10203040U + frameSize;
        for (unsigned i = 0; i < 30000 && (deliveredRx.size() < sourceTx.size() || deliveredTx.size() < sourceRx.size()); ++i)
        {
            const size_t nt = std::min(sourceTx.size() - acceptedTx, (size_t)(random32() % 49U));
            const size_t nr = std::min(sourceRx.size() - acceptedRx, (size_t)(random32() % 37U));
            acceptedTx += p.tx.acceptUart(sourceTx.data() + acceptedTx, nt);
            acceptedRx += p.rx.acceptUart(sourceRx.data() + acceptedRx, nr);
            p.step(random32() % 7U == 0, random32() % 17U == 0);
            // Long output pauses fill RAM queues; the opposite stream must progress.
            uint8_t batch[64];
            if (i % 97U > 25U)
            {
                const size_t n = p.rx.peekUart(batch, random32() % 53U);
                deliveredRx.insert(deliveredRx.end(), batch, batch + n); p.rx.consumeUart(n);
            }
            if (i % 113U > 45U)
            {
                const size_t n = p.tx.peekUart(batch, random32() % 41U);
                deliveredTx.insert(deliveredTx.end(), batch, batch + n); p.tx.consumeUart(n);
            }
        }
        TEST_ASSERT_EQUAL(sourceTx.size(), acceptedTx); TEST_ASSERT_EQUAL(sourceRx.size(), acceptedRx);
        TEST_ASSERT_EQUAL(sourceTx.size(), deliveredRx.size()); TEST_ASSERT_EQUAL(sourceRx.size(), deliveredTx.size());
        TEST_ASSERT_EQUAL_HEX8_ARRAY(sourceTx.data(), deliveredRx.data(), sourceTx.size());
        TEST_ASSERT_EQUAL_HEX8_ARRAY(sourceRx.data(), deliveredTx.data(), sourceRx.size());
        TEST_ASSERT_GREATER_THAN(8, p.tx.stats().transmitted);
        TEST_ASSERT_GREATER_THAN(0, p.tx.stats().retransmitted + p.rx.stats().retransmitted);
        TEST_ASSERT_GREATER_THAN(0, p.tx.stats().duplicates + p.rx.stats().duplicates);
        TEST_ASSERT_EQUAL(0, p.tx.stats().session_resets + p.rx.stats().session_resets);
    }
}

void test_timeout_reboot_and_no_replay_of_ambiguous_tail()
{
    Pair<> p; p.establish(); p.untilEven();
    const uint8_t old[] = {0x11, 0x22, 0x33};
    p.tx.acceptUart(old, sizeof(old)); p.step(); p.step(true); // delivered, ACK lost
    TEST_ASSERT_TRUE(p.tx.pending());
    uint8_t out[16]; TEST_ASSERT_EQUAL(sizeof(old), p.rx.peekUart(out, sizeof(out)));
    p.rx.consumeUart(sizeof(old));
    TEST_ASSERT_TRUE(p.rx.begin(config(EML_RESPONDER), 0x98765432U, p.slot));
    // Sender eventually times out, discards the unconfirmed old tail and starts fresh.
    for (unsigned i = 0; i < 500 && (p.tx.stats().session_resets == 0 || !p.tx.active() || !p.rx.active()); ++i) p.step();
    TEST_ASSERT_TRUE(p.tx.active()); TEST_ASSERT_TRUE(p.rx.active());
    TEST_ASSERT_EQUAL(sizeof(old), p.tx.stats().discarded_tx);
    TEST_ASSERT_EQUAL(0, p.rx.queuedRx()); TEST_ASSERT_EQUAL(0, p.tx.queuedTx());
    const uint8_t fresh[] = {0, 0xFF, 0x7E};
    p.tx.acceptUart(fresh, sizeof(fresh));
    for (unsigned i = 0; i < 12 && p.rx.queuedRx() < sizeof(fresh); ++i) p.step();
    TEST_ASSERT_EQUAL(sizeof(fresh), p.rx.peekUart(out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(fresh, out, sizeof(fresh));
}

void test_probe_does_not_clear_active_queues_or_reset_sequences()
{
    Pair<> p; p.establish(); p.untilEven();
    const uint8_t bytes[] = {4, 5, 6};
    p.rx.acceptUart(bytes, sizeof(bytes));
    const uint32_t generation = p.rx.generation();
    EmlSync_s hello = {1, EML_SYNC_HELLO, EML_PHY_LORA_500_SF6_CR45, 64,
        1, 2, 0xABCDEF12U, 0, 0x10203040U, (uint16_t)p.slot, (uint16_t)((p.slot / 2U) % 7U)};
    uint8_t payload[28], frame[64];
    emlSyncEncode(&hello, payload, sizeof(payload));
    const EmlHeader_s header = {EML_PKT_TYPE_SYNC, 7, 7, 28, EML_FLOW_CTRL_PAUSE};
    emlFrameEncode(&header, payload, 28, frame, sizeof(frame), 64);
    TEST_ASSERT_EQUAL(EML_RX_CONTROL, p.rx.receive(frame, 64,
        {generation, p.slot, hello.hop_index}, p.slot));
    TEST_ASSERT_EQUAL(EML_OFFER, p.rx.state());
    TEST_ASSERT_EQUAL(sizeof(bytes), p.rx.queuedTx());
    for (unsigned i = 0; i < 60 && !p.rx.active(); ++i) p.step();
    TEST_ASSERT_TRUE(p.rx.active()); TEST_ASSERT_EQUAL(generation, p.rx.generation());
    TEST_ASSERT_EQUAL(0, p.rx.stats().session_resets);
    for (unsigned i = 0; i < 10 && p.tx.queuedRx() < sizeof(bytes); ++i) p.step();
    uint8_t out[8]; TEST_ASSERT_EQUAL(sizeof(bytes), p.tx.peekUart(out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(bytes, out, sizeof(bytes));
}

void test_config_rejection_and_wraparound()
{
    EmlTransport<64> endpoint;
    auto c = config(EML_COORDINATOR);
    TEST_ASSERT_FALSE(endpoint.begin(c, 0, 0));
    c.max_frame_age_slots = 16; TEST_ASSERT_FALSE(endpoint.begin(c, 1, 0));
    c = config(EML_COORDINATOR); c.quiet_slots = 2; TEST_ASSERT_FALSE(endpoint.begin(c, 1, 0));
    c = config(EML_COORDINATOR); c.local_id = c.peer_id; TEST_ASSERT_FALSE(endpoint.begin(c, 1, 0));
    Pair<> mismatch;
    c = config(EML_RESPONDER); ++c.configuration_id;
    mismatch.rx.begin(c, 1, 0);
    for (unsigned i = 0; i < 50; ++i) mismatch.step();
    TEST_ASSERT_FALSE(mismatch.tx.active()); TEST_ASSERT_FALSE(mismatch.rx.active());
    const uint8_t byte = 1;
    TEST_ASSERT_EQUAL(0, mismatch.tx.acceptUart(&byte, 1));
    for (uint32_t start : {0xFFF8U, 0xFFFFFFF8U})
    {
        Pair<> p(64, start); p.establish();
        for (unsigned i = 0; i < 24; ++i)
        {
            TEST_ASSERT_EQUAL(1, p.tx.acceptUart(&byte, 1));
            for (unsigned j = 0; j < 8 && (p.tx.queuedTx() || p.rx.queuedRx() == 0); ++j) p.step();
            TEST_ASSERT_EQUAL(1, p.rx.queuedRx());
            p.rx.consumeUart(1);
            TEST_ASSERT_EQUAL((i + 1U) & 7U, p.tx.nextTxSequence());
        }
    }
}

struct FakeSerial
{
    std::deque<uint8_t> input;
    std::vector<uint8_t> output;
    size_t room = 64, shortWrite = 64, shortRead = 64;
    int available() { return (int)input.size(); }
    int availableForWrite() { return (int)room; }
    size_t readBytes(uint8_t *out, size_t n)
    {
        TEST_ASSERT_LESS_OR_EQUAL(input.size(), n);
        n = std::min(n, shortRead);
        for (size_t i = 0; i < n; ++i) { out[i] = input.front(); input.pop_front(); }
        return n;
    }
    size_t write(const uint8_t *in, size_t n)
    {
        TEST_ASSERT_LESS_OR_EQUAL(room, n);
        n = std::min(n, shortWrite);
        output.insert(output.end(), in, in + n);
        return n;
    }
};

void test_uart_batches_partial_writes_and_capacity()
{
    Pair<64> p; p.establish(); p.untilEven();
    EmlTransportUart<64> tx(p.tx), rx(p.rx);
    EmlUartStats_s txStats, rxStats;
    FakeSerial source, sink;
    for (unsigned i = 0; i < 128; ++i) source.input.push_back((uint8_t)i);
    TEST_ASSERT_EQUAL(64, emlReadSerial(source, tx, true, txStats));
    TEST_ASSERT_EQUAL(0, emlReadSerial(source, tx, true, txStats));
    TEST_ASSERT_EQUAL(64, source.input.size());
    p.step(); p.step();
    sink.room = 0;
    TEST_ASSERT_EQUAL(0, emlDrainSerial(sink, rx, rxStats));
    TEST_ASSERT_EQUAL(60, p.rx.queuedRx());
    sink.room = 8; sink.shortWrite = 3;
    TEST_ASSERT_EQUAL(3, emlDrainSerial(sink, rx, rxStats));
    TEST_ASSERT_EQUAL(57, p.rx.queuedRx());
    for (unsigned i = 0; i < 19; ++i) emlDrainSerial(sink, rx, rxStats);
    TEST_ASSERT_EQUAL(0, p.rx.queuedRx()); TEST_ASSERT_EQUAL(60, sink.output.size());
    for (size_t i = 0; i < sink.output.size(); ++i) TEST_ASSERT_EQUAL_HEX8(i, sink.output[i]);
    const size_t before = source.input.size();
    TEST_ASSERT_EQUAL(10, emlReadSerial(source, tx, false, txStats, 10));
    TEST_ASSERT_EQUAL(before - 10, source.input.size());
    TEST_ASSERT_EQUAL(10, txStats.discarded_disconnected);
    TEST_ASSERT_EQUAL(1, txStats.input_stalls);
    TEST_ASSERT_EQUAL(1, rxStats.output_stalls);
}

void test_mailbox_boundaries_and_generation_stamp()
{
    EmlMailbox box;
    EmlRadioDescriptor_s out = {};
    uint8_t bytes[64]; for (unsigned i = 0; i < 64; ++i) bytes[i] = (uint8_t)i;
    TEST_ASSERT_FALSE(box.pop(&out));
    TEST_ASSERT_FALSE(box.push(nullptr, 64, {1, 2, 3}));
    TEST_ASSERT_FALSE(box.push(bytes, 63, {1, 2, 3}));
    for (unsigned cycle = 0; cycle < 1000; ++cycle)
    {
        TEST_ASSERT_TRUE(box.push(bytes, 64, {7, cycle, 6}));
        TEST_ASSERT_TRUE(box.push(bytes, 32, {8, cycle + 1, 2}));
        TEST_ASSERT_FALSE(box.push(bytes, 64, {9, cycle + 2, 0}));
        TEST_ASSERT_TRUE(box.pop(&out));
        TEST_ASSERT_EQUAL(7, out.context.generation); TEST_ASSERT_EQUAL(cycle, out.context.slot);
        TEST_ASSERT_EQUAL(64, out.length); TEST_ASSERT_EQUAL_HEX8_ARRAY(bytes, out.bytes, 64);
        TEST_ASSERT_TRUE(box.pop(&out)); TEST_ASSERT_EQUAL(8, out.context.generation);
        TEST_ASSERT_EQUAL(32, out.length); TEST_ASSERT_FALSE(box.pop(&out));
    }
    TEST_ASSERT_TRUE(box.push(bytes, 64, {3, 4, 1}));
    box.clearQuiesced(); TEST_ASSERT_FALSE(box.pop(&out));
    const EmlTxTicket_s ticket = {5, 10, 123, EML_PKT_TYPE_DATA, 0};
    TEST_ASSERT_FALSE(box.push(bytes, 64, {5, 12, 1}, &ticket));
    TEST_ASSERT_TRUE(box.push(bytes, 64, {5, 10, 1}, &ticket));
    TEST_ASSERT_TRUE(box.pop(&out));
    TEST_ASSERT_EQUAL(123, out.ticket.serial); TEST_ASSERT_EQUAL(5, out.ticket.generation);
}

void test_coordinator_reboot_replaces_session_once()
{
    Pair<> p; p.establish();
    const uint8_t bytes[] = {9, 8, 7};
    p.rx.acceptUart(bytes, sizeof(bytes));
    const uint32_t oldGeneration = p.rx.generation();
    p.tx.begin(config(EML_COORDINATOR), 0x99887766, p.slot);
    p.establish();
    TEST_ASSERT_NOT_EQUAL(oldGeneration, p.rx.generation());
    TEST_ASSERT_EQUAL(sizeof(bytes), p.rx.stats().discarded_tx);
    TEST_ASSERT_EQUAL(1, p.rx.stats().session_resets);
    TEST_ASSERT_EQUAL(0, p.rx.nextTxSequence());
    TEST_ASSERT_EQUAL(0, p.rx.queuedTx());
    for (unsigned i = 0; i < 20; ++i) p.step();
    TEST_ASSERT_EQUAL(1, p.rx.stats().session_resets);
}

void test_premature_ready_and_duplicate_ready()
{
    Pair<> p;
    p.step(); p.step(); // HELLO/OFFER, coordinator has not sent ACCEPT yet.
    TEST_ASSERT_EQUAL(EML_ACCEPT, p.tx.state());
    EmlSync_s ready = {1, EML_SYNC_READY, EML_PHY_LORA_500_SF6_CR45, 64,
        2, 1, 0x22222222U, 0x11111111U, 0x10203040U, 3, 1};
    uint8_t payload[28], frame[64];
    emlSyncEncode(&ready, payload, sizeof(payload));
    const EmlHeader_s header = {EML_PKT_TYPE_SYNC, 7, 7, 28, EML_FLOW_CTRL_PAUSE};
    emlFrameEncode(&header, payload, 28, frame, sizeof(frame), 64);
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, p.tx.receive(frame, 64, {p.tx.generation(), 3, 1}, 3));
    TEST_ASSERT_FALSE(p.tx.active());
    p.slot = 4; p.establish();
    if (!(p.slot & 1U)) p.step();
    const uint8_t byte = 0xAA;
    p.tx.acceptUart(&byte, 1);
    const uint32_t generation = p.tx.generation();
    ready.slot_counter = (uint16_t)p.slot; ready.hop_index = (p.slot / 2U) % 7U;
    emlSyncEncode(&ready, payload, sizeof(payload));
    emlFrameEncode(&header, payload, 28, frame, sizeof(frame), 64);
    TEST_ASSERT_EQUAL(EML_RX_CONTROL, p.tx.receive(frame, 64, {generation, p.slot, ready.hop_index}, p.slot));
    TEST_ASSERT_EQUAL(generation, p.tx.generation()); TEST_ASSERT_EQUAL(1, p.tx.queuedTx());
    TEST_ASSERT_EQUAL(0, p.tx.nextTxSequence());
}

void test_invalid_data_or_crc_cannot_apply_piggyback_ack()
{
    Pair<> p; p.establish(); p.untilEven();
    const uint8_t byte = 0x55;
    p.tx.acceptUart(&byte, 1);
    p.step(); // outbound DATA0 sent and accepted, leave ACK undelivered.
    EmlHeader_s header = {EML_PKT_TYPE_DATA, 3, 0, 1, EML_FLOW_CTRL_PAUSE};
    uint8_t frame[64];
    emlFrameEncode(&header, &byte, 1, frame, sizeof(frame), 64);
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, p.tx.receive(frame, 64,
        {p.tx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot));
    TEST_ASSERT_EQUAL(1, p.tx.queuedTx()); TEST_ASSERT_EQUAL(0, p.tx.queuedRx());
    header.pkt_type = EML_PKT_TYPE_KEEPALIVE; header.payload_len = 0;
    emlFrameEncode(&header, nullptr, 0, frame, sizeof(frame), 64);
    frame[63] ^= 1;
    TEST_ASSERT_EQUAL(EML_RX_REJECTED, p.tx.receive(frame, 64,
        {p.tx.generation(), p.slot, (uint16_t)((p.slot / 2U) % 7U)}, p.slot));
    TEST_ASSERT_EQUAL(1, p.tx.queuedTx()); TEST_ASSERT_TRUE(p.tx.pending());
}

void setUp() {} void tearDown() {}
int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_handshake_and_every_lost_phase);
    RUN_TEST(test_pending_bytes_survive_data_and_ack_loss);
    RUN_TEST(test_unsent_and_wrong_ack_cannot_release_bytes);
    RUN_TEST(test_rx_backpressure_does_not_ack_unstored_payload);
    RUN_TEST(test_stale_generation_age_and_out_of_order_descriptors);
    RUN_TEST(test_bidirectional_binary_streams_under_noise_and_uart_pauses);
    RUN_TEST(test_timeout_reboot_and_no_replay_of_ambiguous_tail);
    RUN_TEST(test_probe_does_not_clear_active_queues_or_reset_sequences);
    RUN_TEST(test_config_rejection_and_wraparound);
    RUN_TEST(test_uart_batches_partial_writes_and_capacity);
    RUN_TEST(test_mailbox_boundaries_and_generation_stamp);
    RUN_TEST(test_coordinator_reboot_replaces_session_once);
    RUN_TEST(test_premature_ready_and_duplicate_ready);
    RUN_TEST(test_invalid_data_or_crc_cannot_apply_piggyback_ack);
    return UNITY_END();
}
