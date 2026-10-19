#include <unity.h>
#include <cstring>
#include <initializer_list>
#include "../../lib/SX127xDriver/SX127xFskConfig.h"
#include "../../lib/SX127xDriver/SX127xPll.h"

struct MockFskHal
{
    uint8_t regs[128], masks[128];
    unsigned writes = 0;
    MockFskHal() { memset(regs, 0xFF, sizeof(regs)); memset(masks, 0, sizeof(masks)); }
    void writeRegister(uint8_t address, uint8_t value, uint8_t radio)
    {
        regs[address] = value; masks[address] = radio; ++writes;
    }
    void writeRegister(uint8_t address, uint8_t *bytes, uint8_t length, uint8_t radio)
    {
        for (uint8_t i = 0; i < length; ++i) writeRegister(address + i, bytes[i], radio);
    }
};

static SX127xFskParams_s profile250()
{
    return {250000, 90000, 250000, SX127X_FSK_SHAPING_BT_1_0, 10, 4, {0x12, 0x34, 0x56, 0x78}};
}

void test_packet_engine_matches_sx1276_register_map()
{
    for (uint8_t length : {uint8_t(4), uint8_t(32), uint8_t(64)})
    {
        MockFskHal hal;
        TEST_ASSERT_TRUE(sx127xFskConfigurePacketEngine(hal, profile250(), length, 3));
        // Independent datasheet fixture: 32 MHz / 128 = 250000 bit/s;
        // round(90000 / Fstep) = 0x05C3; RxBw mantissa 16, exponent 1.
        const uint8_t addresses[] = {0x02,0x03,0x5D,0x04,0x05,0x0A,0x0C,0x0D,0x0E,
            0x12,0x13,0x1A,0x1F,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x30,0x31,0x35,0x36,0x40};
        const uint8_t expected[] = {0x00,0x80,0x00,0x05,0xC3,0x29,0x23,0x1E,0x04,
            0x01,0x01,0x01,0xAA,0x00,0x0A,0x53,0x12,0x34,0x56,0x78,0x40,0x40,0x8F,0x00,0x00};
        for (unsigned i = 0; i < sizeof(addresses); ++i)
        {
            TEST_ASSERT_EQUAL_HEX8(expected[i], hal.regs[addresses[i]]);
            TEST_ASSERT_EQUAL_HEX8(3, hal.masks[addresses[i]]);
        }
        TEST_ASSERT_EQUAL_UINT8(length, hal.regs[0x32]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, hal.regs[0x00]); // never feed a length byte into the FIFO
    }
}

void test_bandwidth_reserved_fields_and_unsupported_settings()
{
    const uint32_t widths[] = {250000,200000,125000,100000,62500,50000};
    const uint8_t expected[] = {0x01,0x09,0x02,0x0A,0x03,0x0B};
    for (unsigned i = 0; i < sizeof(expected); ++i)
    {
        SX127xFskParams_s p = profile250();
        p.bitrate = 50000; p.fdev_hz = 12500; p.rxbw_hz = widths[i];
        MockFskHal hal;
        TEST_ASSERT_TRUE(sx127xFskConfigurePacketEngine(hal, p, 64, 1));
        TEST_ASSERT_EQUAL_HEX8(expected[i], hal.regs[0x12]);
        TEST_ASSERT_EQUAL_HEX8(expected[i], hal.regs[0x13]);
    }
    for (uint32_t width : {500000U,400000U,333333U,300000U,0U})
    {
        uint8_t out = 0xA5;
        TEST_ASSERT_FALSE(sx127xFskBwReg(width, &out));
        TEST_ASSERT_EQUAL_HEX8(0xA5, out);
        SX127xFskParams_s p = profile250(); p.rxbw_hz = width;
        MockFskHal hal;
        TEST_ASSERT_FALSE(sx127xFskConfigurePacketEngine(hal, p, 64, 1));
        TEST_ASSERT_EQUAL_UINT32(0, hal.writes);
    }
    for (uint8_t length : {uint8_t(0),uint8_t(65)})
    {
        MockFskHal hal;
        TEST_ASSERT_FALSE(sx127xFskConfigurePacketEngine(hal, profile250(), length, 1));
        TEST_ASSERT_EQUAL_UINT32(0, hal.writes);
    }
}

void test_runtime_pll_requires_mode_ready_and_rejects_broken_spi()
{
    for (uint32_t initial : {0U, 0xFFFFFF80U})
    {
        for (uint8_t flags : {uint8_t(0x10),uint8_t(0x80),uint8_t(0x00),uint8_t(0xFF)})
        {
            uint32_t now = initial;
            auto read = [&]() { now += 5; return flags; };
            auto clock = [&]() { return now; };
            TEST_ASSERT_FALSE(sx127xWaitPllReady(read, clock, 250));
            TEST_ASSERT_EQUAL_UINT32(250, now - initial);
        }
        uint32_t now = initial;
        unsigned reads = 0;
        auto read = [&]() { now += 5; return (uint8_t)(++reads < 10 ? 0x10 : 0x90); };
        auto clock = [&]() { return now; };
        TEST_ASSERT_TRUE(sx127xWaitPllReady(read, clock, 250));
        TEST_ASSERT_EQUAL_UINT32(50, now - initial);
        TEST_ASSERT_EQUAL_UINT32(10, reads);
        auto late = [&]() { now += 250; return (uint8_t)0x90; };
        TEST_ASSERT_FALSE(sx127xWaitPllReady(late, clock, 250));
    }
}

void setUp() {}
void tearDown() {}
int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_packet_engine_matches_sx1276_register_map);
    RUN_TEST(test_bandwidth_reserved_fields_and_unsupported_settings);
    RUN_TEST(test_runtime_pll_requires_mode_ready_and_rejects_broken_spi);
    return UNITY_END();
}
