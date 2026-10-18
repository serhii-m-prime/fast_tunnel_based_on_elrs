#include <cstdint>
#include <cstring>
#include <vector>
#include <unity.h>

// SX127xDriver is lib_ignored on native; the burst helpers are header-only.
#include "../../lib/SX127xDriver/SX127xSpiBurst.h"

// Models the SX127x side of SPIEx transactions: one address byte (bit 7 = write)
// then data. RegFifo accesses go through RegFifoAddrPtr, which persists between
// transactions and wraps at 256; other registers auto-increment per byte.
class MockSx127xSpi
{
public:
    uint8_t regs[0x80];
    uint8_t fifo[256];
    std::vector<uint32_t> sizes;
    std::vector<uint8_t> addrs;
    bool oversize = false;
    bool misaligned = false;

    MockSx127xSpi() { reset(); }

    void reset()
    {
        memset(regs, 0, sizeof(regs));
        memset(fifo, 0, sizeof(fifo));
        sizes.clear();
        addrs.clear();
        oversize = misaligned = false;
    }

    void read(uint8_t, uint8_t *data, uint32_t size) { transfer(data, size, false); }
    void write(uint8_t, uint8_t *data, uint32_t size) { transfer(data, size, true); }

private:
    void transfer(uint8_t *buf, uint32_t size, bool writing)
    {
        sizes.push_back(size);
        oversize |= size == 0 || size > SX127X_SPI_MAX_TRANSFER_BYTES;
        misaligned |= (reinterpret_cast<uintptr_t>(buf) & 3U) != 0;
        if (size == 0) return;
        const uint8_t first = buf[0];
        TEST_ASSERT_EQUAL(writing, (first & SPI_WRITE) != 0);
        uint8_t addr = first & 0x7F;
        addrs.push_back(addr);
        for (uint32_t i = 1; i < size; i++)
        {
            uint8_t *cell;
            if (addr == SX127X_REG_FIFO)
            {
                cell = &fifo[regs[SX127X_REG_FIFO_ADDR_PTR]];
                regs[SX127X_REG_FIFO_ADDR_PTR]++;
            }
            else
            {
                cell = &regs[addr & 0x7F];
                addr++;
            }
            if (writing) *cell = buf[i];
            else buf[i] = *cell;
        }
    }
};

static MockSx127xSpi spi;

static void fill(uint8_t *data, unsigned len, uint8_t seed)
{
    for (unsigned i = 0; i < len; i++) data[i] = (uint8_t)(seed + i * 7U);
}

void test_chunk_helpers()
{
    TEST_ASSERT_EQUAL_UINT8(0, sx127xSpiChunkLen(0));
    TEST_ASSERT_EQUAL_UINT8(13, sx127xSpiChunkLen(13));
    TEST_ASSERT_EQUAL_UINT8(63, sx127xSpiChunkLen(63));
    TEST_ASSERT_EQUAL_UINT8(63, sx127xSpiChunkLen(64));
    TEST_ASSERT_EQUAL_UINT8(63, sx127xSpiChunkLen(255));
    TEST_ASSERT_EQUAL_HEX8(SX127X_REG_FIFO, sx127xSpiChunkAddr(SX127X_REG_FIFO, 63));
    TEST_ASSERT_EQUAL_HEX8(SX127X_REG_FRF_MSB + 2, sx127xSpiChunkAddr(SX127X_REG_FRF_MSB, 2));
}

// Stock ELRS packets (8/13 bytes) and short register bursts keep one transaction
void test_short_bursts_single_transaction()
{
    const unsigned lens[] = {1, 3, 8, 13, 32, 63};
    for (unsigned len : lens)
    {
        spi.reset();
        uint8_t in[255], out[255];
        fill(in, len, (uint8_t)len);
        sx127xSpiBurstWrite(spi, 1, SX127X_REG_FIFO, in, (uint8_t)len);
        TEST_ASSERT_EQUAL(1, spi.sizes.size());
        TEST_ASSERT_EQUAL(len + 1, spi.sizes[0]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(in, spi.fifo, len);
        spi.regs[SX127X_REG_FIFO_ADDR_PTR] = 0;
        sx127xSpiBurstRead(spi, 1, SX127X_REG_FIFO, out, (uint8_t)len);
        TEST_ASSERT_EQUAL(2, spi.sizes.size());
        TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, len);
        TEST_ASSERT_FALSE(spi.oversize);
        TEST_ASSERT_FALSE(spi.misaligned);
    }
}

// Every FIFO burst length round-trips, never exceeds one SPIEx buffer per transaction,
// and starts at the FIFO pointer, including a wrap past FIFO address 255
void test_fifo_burst_all_lengths()
{
    const uint8_t starts[] = {0x00, 0x80, 0xE0};
    for (uint8_t start : starts)
    {
        for (unsigned len = 0; len <= 255; len++)
        {
            spi.reset();
            uint8_t in[255] = {0}, out[255] = {0};
            fill(in, len, (uint8_t)(len ^ start));
            spi.regs[SX127X_REG_FIFO_ADDR_PTR] = start;
            sx127xSpiBurstWrite(spi, 1, SX127X_REG_FIFO, in, (uint8_t)len);
            const size_t writes = spi.sizes.size();
            TEST_ASSERT_EQUAL(len == 0 ? 1 : (len + 62) / 63, writes);
            TEST_ASSERT_EQUAL_UINT8((uint8_t)(start + len), spi.regs[SX127X_REG_FIFO_ADDR_PTR]);
            for (unsigned i = 0; i < len; i++)
            {
                TEST_ASSERT_EQUAL_HEX8(in[i], spi.fifo[(uint8_t)(start + i)]);
            }

            spi.regs[SX127X_REG_FIFO_ADDR_PTR] = start;
            sx127xSpiBurstRead(spi, 1, SX127X_REG_FIFO, out, (uint8_t)len);
            TEST_ASSERT_EQUAL(2 * writes, spi.sizes.size());
            if (len) TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, len);
            for (uint8_t addr : spi.addrs) TEST_ASSERT_EQUAL_HEX8(SX127X_REG_FIFO, addr);
            TEST_ASSERT_FALSE(spi.oversize);
            TEST_ASSERT_FALSE(spi.misaligned);
        }
    }
}

// The 64-byte EML frame: two transactions of 64 + 2 bytes, bytes outside the frame untouched
void test_eml64_frame_split()
{
    spi.reset();
    memset(spi.fifo, 0xA5, sizeof(spi.fifo));
    uint8_t frame[64], out[64 + 1];
    fill(frame, sizeof(frame), 0x31);
    sx127xSpiBurstWrite(spi, 3, SX127X_REG_FIFO, frame, sizeof(frame));
    TEST_ASSERT_EQUAL(2, spi.sizes.size());
    TEST_ASSERT_EQUAL(64, spi.sizes[0]);
    TEST_ASSERT_EQUAL(2, spi.sizes[1]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(frame, spi.fifo, sizeof(frame));
    TEST_ASSERT_EQUAL_HEX8(0xA5, spi.fifo[64]);

    memset(out, 0x5A, sizeof(out));
    spi.regs[SX127X_REG_FIFO_ADDR_PTR] = 0;
    sx127xSpiBurstRead(spi, 3, SX127X_REG_FIFO, out, sizeof(frame));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(frame, out, sizeof(frame));
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[64]);
}

// Non-FIFO bursts continue at the next register address in later chunks
void test_register_burst_continues_address()
{
    spi.reset();
    uint8_t in[70], out[70];
    fill(in, sizeof(in), 3);
    sx127xSpiBurstWrite(spi, 1, 0x01, in, sizeof(in));
    TEST_ASSERT_EQUAL(2, spi.addrs.size());
    TEST_ASSERT_EQUAL_HEX8(0x01, spi.addrs[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01 + 63, spi.addrs[1]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, &spi.regs[0x01], sizeof(in));
    sx127xSpiBurstRead(spi, 1, 0x01, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, sizeof(out));

    const uint8_t frf[3] = {0xF0, 0x00, 0x00};
    spi.reset();
    sx127xSpiBurstWrite(spi, 1, SX127X_REG_FRF_MSB, frf, sizeof(frf));
    TEST_ASSERT_EQUAL(1, spi.sizes.size());
    TEST_ASSERT_EQUAL(4, spi.sizes[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(frf, &spi.regs[SX127X_REG_FRF_MSB], sizeof(frf));
}

void test_zero_length_keeps_address_only_transaction()
{
    spi.reset();
    uint8_t dummy = 0x77;
    sx127xSpiBurstRead(spi, 1, SX127X_REG_VERSION, &dummy, 0);
    TEST_ASSERT_EQUAL(1, spi.sizes.size());
    TEST_ASSERT_EQUAL(1, spi.sizes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x77, dummy);
}

void setUp() {}
void tearDown() {}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_chunk_helpers);
    RUN_TEST(test_short_bursts_single_transaction);
    RUN_TEST(test_fifo_burst_all_lengths);
    RUN_TEST(test_eml64_frame_split);
    RUN_TEST(test_register_burst_continues_address);
    RUN_TEST(test_zero_length_keeps_address_only_transaction);
    UNITY_END();

    return 0;
}
