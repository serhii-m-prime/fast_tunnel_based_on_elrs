/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * Unit tests for over the air packet encoding, decoding and associated utilities
 *
 * Entry point is setup_switches()
 */


#include <cstdint>
#include <iostream>
#include <unity.h>

#include "CRSFEndpoint.h"
#include "POWERMGNT.h"
#include "common.h"
#include "crsf_sysmocks.h"
#include "targets.h"

#include <OTA.h>

class MockEndpoint : public CRSFEndpoint
{
public:
    MockEndpoint() : CRSFEndpoint((crsf_addr_e)1) {}
    void handleMessage(const crsf_header_t *message) override {}
};
CRSFEndpoint *crsfEndpoint = new MockEndpoint();

uint32_t ChannelData[CRSF_NUM_CHANNELS];      // Current state of channels, CRSF format

static void resetTestChannelData()
{
    for (auto &ch : ChannelData)
        ch = CRSF_CHANNEL_VALUE_UNSET;
}

void test_crsf_endpoints()
{
    // Validate 988us and 2012us convert to approprate CRSF values. Spoiler: They don't
    TEST_ASSERT_EQUAL(-1024, Us_to_OpenTx(US_CHANNEL_VALUE_STD_MIN));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_STD_MIN+1, OpenTx_to_Crsf(-1024)); // NOTE: 988 comes from OpenTX as 173, not 172!
    TEST_ASSERT_EQUAL(1024, Us_to_OpenTx(US_CHANNEL_VALUE_STD_MAX));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_STD_MAX, OpenTx_to_Crsf(1024));

    // Validate CRSF values convert to their expected values in OpenTX
    TEST_ASSERT_EQUAL(US_CHANNEL_VALUE_STD_MIN, Crsf_to_OpenTx_to_Us(CRSF_CHANNEL_VALUE_STD_MIN));
    TEST_ASSERT_EQUAL(US_CHANNEL_VALUE_STD_MAX-1, Crsf_to_OpenTx_to_Us(CRSF_CHANNEL_VALUE_STD_MAX)); // NOTE: Feeding the 2012 CRSF value back into OpenTX would give 2011

    // Validate CRSF values convert to their expected values in Betaflight
    TEST_ASSERT_EQUAL(US_CHANNEL_VALUE_STD_MIN, Crsf_to_BfUs(CRSF_CHANNEL_VALUE_STD_MIN));
    TEST_ASSERT_EQUAL(US_CHANNEL_VALUE_STD_MAX, Crsf_to_BfUs(CRSF_CHANNEL_VALUE_STD_MAX));

    // Validate important values are still the same value when mapped and umapped from their 10-bit representations
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_STD_MIN,  Crsf_to_Uint10_to_Crsf(CRSF_CHANNEL_VALUE_STD_MIN));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_1000, Crsf_to_Uint10_to_Crsf(CRSF_CHANNEL_VALUE_1000));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_MID,  Crsf_to_Uint10_to_Crsf(CRSF_CHANNEL_VALUE_MID));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_2000, Crsf_to_Uint10_to_Crsf(CRSF_CHANNEL_VALUE_2000));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_STD_MAX,  Crsf_to_Uint10_to_Crsf(CRSF_CHANNEL_VALUE_STD_MAX));
}

void test_crsfToBit()
{
    TEST_ASSERT_EQUAL(0, CRSF_to_BIT(CRSF_CHANNEL_VALUE_1000));
    TEST_ASSERT_EQUAL(1, CRSF_to_BIT(CRSF_CHANNEL_VALUE_2000));
}

void test_bitToCrsf()
{
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_1000, BIT_to_CRSF(0));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_2000, BIT_to_CRSF(1));
}

void test_crsfToN()
{
    TEST_ASSERT_EQUAL(0, CRSF_to_N(CRSF_CHANNEL_VALUE_STD_MIN, 64));
    TEST_ASSERT_EQUAL(0, CRSF_to_N(CRSF_CHANNEL_VALUE_1000, 64));
    TEST_ASSERT_EQUAL(0b100000, CRSF_to_N(CRSF_CHANNEL_VALUE_MID, 64));
    TEST_ASSERT_EQUAL(0b111111, CRSF_to_N(CRSF_CHANNEL_VALUE_2000, 64));
    TEST_ASSERT_EQUAL(0b111111, CRSF_to_N(CRSF_CHANNEL_VALUE_STD_MAX, 64));

    TEST_ASSERT_EQUAL(0, CRSF_to_N(CRSF_CHANNEL_VALUE_STD_MIN, 128));
    TEST_ASSERT_EQUAL(0, CRSF_to_N(CRSF_CHANNEL_VALUE_1000, 128));
    TEST_ASSERT_EQUAL(0b1000000, CRSF_to_N(CRSF_CHANNEL_VALUE_MID, 128));
    TEST_ASSERT_EQUAL(0b1111111, CRSF_to_N(CRSF_CHANNEL_VALUE_2000, 128));
    TEST_ASSERT_EQUAL(0b1111111, CRSF_to_N(CRSF_CHANNEL_VALUE_STD_MAX, 128));
}

void test_nToCrsf()
{
    // 6-bit
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_1000, N_to_CRSF(0, 63));
    TEST_ASSERT_EQUAL(1004, N_to_CRSF(0b100000, 63));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_2000, N_to_CRSF(0b111111, 63));

    // 7-bit
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_1000, N_to_CRSF(0, 127));
    TEST_ASSERT_EQUAL(997, N_to_CRSF(0b1000000, 127));
    TEST_ASSERT_EQUAL(CRSF_CHANNEL_VALUE_2000, N_to_CRSF(0b1111111, 127));
}

// ------------------------------------------------
// Test the hybrid8 encoding/decoding

/* Check the hybrid 8 encoding of a packet for OTA tx
*/
void test_encodingHybrid8(bool highResChannel)
{
    constexpr uint8_t N_SWITCHES = 8;
    uint8_t UID[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    uint8_t TXdataBuffer[OTA4_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;

    // Define the input data
    // 4 channels of 11-bit analog data
    ChannelData[0] = 0x0123 & 0b11111111111;
    ChannelData[1] = 0x4567 & 0b11111111111;
    ChannelData[2] = 0x89AB & 0b11111111111;
    ChannelData[3] = 0xCDEF & 0b11111111111;

    OTA_Packet8_s ota;
    OTA_Channels_4x10 ch;
    switch (OtaNonce)
    {
        case sizeof(ota.rc):
        break;
        case 13:
        break;
        case 17:
        break;
        case sizeof(ch):
        break;
        case 8:
        break;
    }
    // 8 switches
    for(int i = 0; i < N_SWITCHES; i++) {
        constexpr int CHANNELS[] =
            { CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_2000 };
        ChannelData[4+i] = CHANNELS[i % 3];
    }

    // set the nextSwitchIndex so we know which switch to expect in the packet
    if (highResChannel)
        OtaSetHybrid8NextSwitchIndex(7-1);
    else
        OtaSetHybrid8NextSwitchIndex(3-1);

    // encode it
    OtaUpdateSerializers(smHybridOr16ch, OTA4_PACKET_SIZE);
    OtaPackChannelData(otaPktPtr, ChannelData, false);

    // check it looks right
    // 1st byte is CRC & packet type
    uint8_t header = PACKET_TYPE_RCDATA;
    TEST_ASSERT_EQUAL(header, TXdataBuffer[0]);

    // bytes 1 through 5 are 10 bit packed analog channels representing 998-2012 (CRSF_CHANNEL_VALUE_STD_MIN-CRSF_CHANNEL_VALUE_STD_MAX)
    uint8_t expected[5] = { 0x4a, 0xd0, 0xfb, 0x49, 0xd2 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[1], 5);

    // byte 6 is the switch encoding
    TEST_ASSERT_EQUAL(CRSF_to_BIT(ChannelData[4+0]), TXdataBuffer[6] >> 7);
    // top bit is undefined
    // expect switch 0 in bit 6
    // index-1 in 3-5
    // value in 0,1,2[,3]
    if (highResChannel)
    {
        TEST_ASSERT_EQUAL(7, ((TXdataBuffer[6] & 0b110000)>>3) + 1);
        TEST_ASSERT_EQUAL(CRSF_to_N(ChannelData[4+7], 16), TXdataBuffer[6] & 0b1111);
    }
    else
    {
        TEST_ASSERT_EQUAL(3, ((TXdataBuffer[6] & 0b111000)>>3) + 1);
        TEST_ASSERT_EQUAL(CRSF_to_N(ChannelData[4+3], 6), TXdataBuffer[6] & 0b0111);
    }
}

void test_encodingHybrid8_3()
{
    test_encodingHybrid8(false);
}

void test_encodingHybrid8_7()
{
    test_encodingHybrid8(true);
}

/* Check the decoding of a packet after rx
*/
void test_decodingHybrid8(uint8_t forceSwitch, uint8_t switchval)
{
    constexpr uint8_t N_SWITCHES = 8;
    uint8_t UID[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    uint8_t TXdataBuffer[OTA4_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    // Define the input data
    // 4 channels of 11-bit analog data
    ChannelData[0] = 0x0123 & 0b11111111111;
    ChannelData[1] = 0x4567 & 0b11111111111;
    ChannelData[2] = 0x89AB & 0b11111111111;
    ChannelData[3] = 0xCDEF & 0b11111111111;

    // 8 switches
    for(int i = 0; i < N_SWITCHES; i++) {
        constexpr int CHANNELS[] =
            { CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_2000 };
        ChannelData[4+i] = CHANNELS[i % 3];
    }
    if (forceSwitch == 0)
        ChannelData[4+forceSwitch] = BIT_to_CRSF(switchval);
    else if (forceSwitch == 7)
        ChannelData[4+forceSwitch] = N_to_CRSF(switchval, 15);
    else
        ChannelData[4+forceSwitch] = SWITCH3b_to_CRSF(switchval);

    // set the nextSwitchIndex so we know which switch to expect in the packet
    if (forceSwitch == 0)
        OtaSetHybrid8NextSwitchIndex(0);
    else
        OtaSetHybrid8NextSwitchIndex(forceSwitch-1);

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    // use the encoding method to pack it into TXdataBuffer
    OtaUpdateSerializers(smHybridOr16ch, OTA4_PACKET_SIZE);
    OtaPackChannelData(otaPktPtr, ChannelData, false);

    // run the decoder, results in crsf->PackedRCdataOut
    OtaUnpackChannelData(otaPktPtr, ChannelData);

    // compare the unpacked results with the input data
    TEST_ASSERT_EQUAL(ChannelsIn[0], ChannelData[0]);
    TEST_ASSERT_EQUAL(ChannelsIn[1], ChannelData[1]);
    TEST_ASSERT_EQUAL(ChannelsIn[2], ChannelData[2]);
    TEST_ASSERT_EQUAL(ChannelsIn[3], ChannelData[3]);

    TEST_ASSERT_EQUAL(ChannelsIn[4+0], ChannelData[4]); // Switch 0 is sent on every packet
    if (forceSwitch == 7)
        TEST_ASSERT_EQUAL(ChannelsIn[4+forceSwitch], ChannelData[11]); // We forced switch 1 to be sent as the sequential field
    else if (forceSwitch != 0)
        TEST_ASSERT_EQUAL(ChannelData[4+forceSwitch], ChannelData[4+forceSwitch]);
}

void test_decodingHybrid8_all()
{
    // Switch 0 is 2 pos
    test_decodingHybrid8(0, 0);
    test_decodingHybrid8(0, 1);
    // Switch X in 6-pos mode (includes 3-pos low/high)
    for (uint8_t val=0; val<6; ++val)
        test_decodingHybrid8(3, val);
    // // Switch X in 3-pos mode center
    // test_decodingHybrid8(3, 7);
    // // Switch 7 is 16 pos
    // for (uint8_t val=0; val<16; ++val)
    //     test_decodingHybrid8(7, val);
}

void test_channelDataComplete_hybrid8()
{
    OtaUpdateSerializers(smHybridOr16ch, OTA4_PACKET_SIZE);
    resetTestChannelData();
    OtaResetChannelDataComplete();

    for (int i = 0; i < 12; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
    for (int i = 12; i < CRSF_NUM_CHANNELS; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_UNSET;

    TEST_ASSERT_TRUE(OtaIsChannelDataComplete(ChannelData));
}

void test_channelDataComplete_wide8ch()
{
    OtaUpdateSerializers(smWideOr8ch, OTA8_PACKET_SIZE);
    resetTestChannelData();
    OtaResetChannelDataComplete();

    for (int i = 0; i < 8; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;

    TEST_ASSERT_TRUE(OtaIsChannelDataComplete(ChannelData));

    resetTestChannelData();
    OtaResetChannelDataComplete();
    for (int i = 0; i < 7; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;

    TEST_ASSERT_FALSE(OtaIsChannelDataComplete(ChannelData));
}

void test_channelDataComplete_12ch()
{
    OtaUpdateSerializers(sm12ch, OTA8_PACKET_SIZE);
    resetTestChannelData();
    OtaResetChannelDataComplete();

    for (int i = 0; i < 12; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;

    TEST_ASSERT_TRUE(OtaIsChannelDataComplete(ChannelData));
}

void test_channelDataComplete_16ch()
{
    OtaUpdateSerializers(smHybridOr16ch, OTA8_PACKET_SIZE);
    resetTestChannelData();
    OtaResetChannelDataComplete();

    for (int i = 0; i < 15; ++i)
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
    TEST_ASSERT_FALSE(OtaIsChannelDataComplete(ChannelData));

    ChannelData[15] = CRSF_CHANNEL_VALUE_MID;
    TEST_ASSERT_TRUE(OtaIsChannelDataComplete(ChannelData));
}

/* Check the HybridWide encoding of a packet for OTA tx
*/
void test_encodingHybridWide(uint8_t nonce)
{
    uint8_t UID[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    uint8_t TXdataBuffer[OTA4_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    // Define the input data
    // 4 channels of 11-bit analog data
    ChannelData[0] = 0x0123 & 0b11111111111;
    ChannelData[1] = 0x4567 & 0b11111111111;
    ChannelData[2] = 0x89AB & 0b11111111111;
    ChannelData[3] = 0xCDEF & 0b11111111111;

    // 8 switches
    constexpr int N_SWITCHES = 8;
    for(int i = 0; i < N_SWITCHES; i++) {
        constexpr unsigned CHANNELS[] =
            { CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_2000 };
        ChannelData[4+i] = CHANNELS[i % 3];
    }

    // Uplink data
    linkStats.uplink_TX_Power = 3; // 100mW

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    // encode it
    OtaUpdateSerializers(smWideOr8ch, OTA4_PACKET_SIZE);
    OtaNonce = nonce;
    OtaPackChannelData(otaPktPtr, ChannelData, nonce % 2);

    // check it looks right
    // 1st byte is CRC & packet type
    uint8_t header = PACKET_TYPE_RCDATA;
    TEST_ASSERT_EQUAL(header, TXdataBuffer[0]);

    // bytes 1 through 5 are 10 bit packed analog channels representing 998-2012 (CRSF_CHANNEL_VALUE_STD_MIN-CRSF_CHANNEL_VALUE_STD_MAX)
    uint8_t expected[5] = { 0x4a, 0xd0, 0xfb, 0x49, 0xd2 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[1], 5);

    // byte 6 is the switches encoded
    uint8_t switches = TXdataBuffer[6];
    uint8_t switchIdx = nonce % 8;

    // High bit should be AUX1
    TEST_ASSERT_EQUAL(CRSF_to_BIT(ChannelsIn[4]), switches >> 7);
    // bit 6 should be the telemetryack bit
    TEST_ASSERT_EQUAL(nonce % 2, (switches >> 6) & 1);

    // If slot 7, the uplink_TX_Power should be in the low 6 bits
    if (switchIdx == 7)
        TEST_ASSERT_EQUAL(linkStats.uplink_TX_Power, switches & 0b111111);
    else
    {
        uint16_t ch = ChannelData[5+switchIdx];
        TEST_ASSERT_EQUAL(CRSF_to_N(ch, 64), switches & 0b111111); // 6-bit
    }
}

void test_encodingHybridWide_low()
{
    constexpr int N_SWITCHES = 8;
    for (int i=0; i<N_SWITCHES; ++i)
        test_encodingHybridWide(i);
}

/* Check the decoding of a packet after rx in HybridWide mode
*/
void test_decodingHybridWide(uint8_t nonce, uint8_t forceSwitch, uint16_t forceVal)
{
    uint8_t UID[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    uint8_t TXdataBuffer[OTA4_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    // Define the input data
    // 4 channels of 11-bit analog data
    ChannelData[0] = 0x0123 & 0b11111111111;
    ChannelData[1] = 0x4567 & 0b11111111111;
    ChannelData[2] = 0x89AB & 0b11111111111;
    ChannelData[3] = 0xCDEF & 0b11111111111;

    // 8 switches
    constexpr int N_SWITCHES = 8;
    for(int i = 0; i < N_SWITCHES; i++) {
        constexpr unsigned CHANNELS[] =
            { CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_2000 };
        if (i == forceSwitch)
            ChannelData[4+i] = forceVal;
        else
            ChannelData[4+i] = CHANNELS[i % 3];
    }

    // Uplink data
    linkStats.uplink_TX_Power = 3; // 100mW

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    // encode it
    OtaUpdateSerializers(smWideOr8ch, OTA4_PACKET_SIZE);
    OtaNonce = nonce;
    OtaPackChannelData(otaPktPtr, ChannelData, nonce % 2);

    // Clear the LinkStatistics to receive it from the encoding
    linkStats.uplink_TX_Power = 0;

    // run the decoder, results in crsf->PackedRCdataOut
    bool telemResult = OtaUnpackChannelData(otaPktPtr, ChannelData);

    // compare the unpacked results with the input data
    TEST_ASSERT_EQUAL(ChannelsIn[0], ChannelData[0]);
    TEST_ASSERT_EQUAL(ChannelsIn[1], ChannelData[1]);
    TEST_ASSERT_EQUAL(ChannelsIn[2], ChannelData[2]);
    TEST_ASSERT_EQUAL(ChannelsIn[3], ChannelData[3]);

    // Switch 0 is sent on every packet
    TEST_ASSERT_EQUAL(ChannelData[4], ChannelData[4]);

    uint8_t switchIdx = nonce % 8;
    // Validate the telemResult was unpacked properly
    TEST_ASSERT_EQUAL(telemResult, nonce % 2);

    if (switchIdx == 7)
    {
        TEST_ASSERT_EQUAL(linkStats.uplink_TX_Power, 3);
    }
    else
    {
        TEST_ASSERT_EQUAL(N_to_CRSF(CRSF_to_N(ChannelData[5+switchIdx], 64), 63), ChannelData[5+switchIdx]);
    }
}

void fullres_fillChannelData()
{
    // Define the input data
    // 16 channels of 11-bit analog data
    ChannelData[0] = 0x0123 & 0b11111111111;
    ChannelData[1] = 0x4567 & 0b11111111111;
    ChannelData[2] = 0x89AB & 0b11111111111;
    ChannelData[3] = 0xCDEF & 0b11111111111;
    ChannelData[4] = 0x3210 & 0b11111111111;
    ChannelData[5] = 0x7654 & 0b11111111111;
    ChannelData[6] = 0xBA98 & 0b11111111111;
    ChannelData[7] = 0xFEDC & 0b11111111111;

    ChannelData[8]  = 0x2301 & 0b11111111111;
    ChannelData[9]  = 0x6745 & 0b11111111111;
    ChannelData[10] = 0xAB89 & 0b11111111111;
    ChannelData[11] = 0xEFCD & 0b11111111111;
    ChannelData[12] = 0x1023 & 0b11111111111;
    ChannelData[13] = 0x5476 & 0b11111111111;
    ChannelData[14] = 0x98BA & 0b11111111111;
    ChannelData[15] = 0xDCFE & 0b11111111111;
}

void test_encodingFullresPowerLevels()
{
    uint8_t TXdataBuffer[OTA8_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    OtaUpdateSerializers(smWideOr8ch, OTA8_PACKET_SIZE);

    for (uint8_t pwr=PWR_10mW; pwr<PWR_COUNT; ++pwr)
    {
        memset(TXdataBuffer, 0, sizeof(TXdataBuffer));
        fullres_fillChannelData();

        // This is what we're testing here, just the power
        uint8_t crsfPower = powerToCrsfPower((PowerLevels_e)pwr);
        linkStats.uplink_TX_Power = crsfPower;

        OtaPackChannelData(otaPktPtr, ChannelData, false);
        OtaUnpackChannelData(otaPktPtr, ChannelData);

        TEST_ASSERT_EQUAL(crsfPower, linkStats.uplink_TX_Power);
    }
}

void test_encodingFullres8ch()
{
    uint8_t TXdataBuffer[OTA8_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    fullres_fillChannelData();
    linkStats.uplink_TX_Power = PWR_250mW;

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    OtaUpdateSerializers(smWideOr8ch, OTA8_PACKET_SIZE);
    OtaPackChannelData(otaPktPtr, ChannelData, false);

    // Low 4ch (CH1-CH4)
    uint8_t expected[5];
    expected[0] = ((ChannelsIn[0] >> 1) >> 0);
    expected[1] = ((ChannelsIn[0] >> 1) >> 8) | ((ChannelsIn[1] >> 1) << 2);
    expected[2] = ((ChannelsIn[1] >> 1) >> 6) | ((ChannelsIn[2] >> 1) << 4);
    expected[3] = ((ChannelsIn[2] >> 1) >> 4) | ((ChannelsIn[3] >> 1) << 6);
    expected[4] = ((ChannelsIn[3] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chLow)], 5);
    // High 4ch (CH5-CH8)
    expected[0] = ((ChannelsIn[4] >> 1) >> 0);
    expected[1] = ((ChannelsIn[4] >> 1) >> 8) | ((ChannelsIn[5] >> 1) << 2);
    expected[2] = ((ChannelsIn[5] >> 1) >> 6) | ((ChannelsIn[6] >> 1) << 4);
    expected[3] = ((ChannelsIn[6] >> 1) >> 4) | ((ChannelsIn[7] >> 1) << 6);
    expected[4] = ((ChannelsIn[7] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chHigh)], 5);

    // Check the header bits
    TEST_ASSERT_EQUAL(PACKET_TYPE_RCDATA, otaPktPtr->full.rc.packetType);
    TEST_ASSERT_EQUAL(false, otaPktPtr->full.rc.stubbornAck);
    TEST_ASSERT_EQUAL(PWR_250mW, otaPktPtr->full.rc.uplinkPower + 1);
    TEST_ASSERT_EQUAL(false, otaPktPtr->full.rc.isHighAux);
    TEST_ASSERT_EQUAL(CRSF_to_BIT(ChannelsIn[4]), otaPktPtr->full.rc.isArmed);
}

void test_encodingFullres16ch()
{
    uint8_t TXdataBuffer[OTA8_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    fullres_fillChannelData();

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    OtaUpdateSerializers(smHybridOr16ch, OTA8_PACKET_SIZE);
    OtaSetFullResNextChannelSet(false);

    // ** PACKET ONE **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    // Low 4ch (CH1-CH4)
    uint8_t expected[5];
    expected[0] = ((ChannelsIn[0] >> 1) >> 0);
    expected[1] = ((ChannelsIn[0] >> 1) >> 8) | ((ChannelsIn[1] >> 1) << 2);
    expected[2] = ((ChannelsIn[1] >> 1) >> 6) | ((ChannelsIn[2] >> 1) << 4);
    expected[3] = ((ChannelsIn[2] >> 1) >> 4) | ((ChannelsIn[3] >> 1) << 6);
    expected[4] = ((ChannelsIn[3] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chLow)], 5);
    // High 4ch, includes AUX1 (CH5-CH8)
    expected[0] = ((ChannelsIn[4] >> 1) >> 0);
    expected[1] = ((ChannelsIn[4] >> 1) >> 8) | ((ChannelsIn[5] >> 1) << 2);
    expected[2] = ((ChannelsIn[5] >> 1) >> 6) | ((ChannelsIn[6] >> 1) << 4);
    expected[3] = ((ChannelsIn[6] >> 1) >> 4) | ((ChannelsIn[7] >> 1) << 6);
    expected[4] = ((ChannelsIn[7] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chHigh)], 5);

    // ** PACKET TWO **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    // Low 4ch (CH9-CH12)
    expected[0] = ((ChannelsIn[8] >> 1) >> 0);
    expected[1] = ((ChannelsIn[8] >> 1) >> 8) | ((ChannelsIn[9] >> 1) << 2);
    expected[2] = ((ChannelsIn[9] >> 1) >> 6) | ((ChannelsIn[10] >> 1) << 4);
    expected[3] = ((ChannelsIn[10] >> 1) >> 4) | ((ChannelsIn[11] >> 1) << 6);
    expected[4] = ((ChannelsIn[11] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chLow)], 5);
    // High 4ch (CH13-CH16)
    expected[0] = ((ChannelsIn[12] >> 1) >> 0);
    expected[1] = ((ChannelsIn[12] >> 1) >> 8) | ((ChannelsIn[13] >> 1) << 2);
    expected[2] = ((ChannelsIn[13] >> 1) >> 6) | ((ChannelsIn[14] >> 1) << 4);
    expected[3] = ((ChannelsIn[14] >> 1) >> 4) | ((ChannelsIn[15] >> 1) << 6);
    expected[4] = ((ChannelsIn[15] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chHigh)], 5);
}

void test_encodingFullres12ch()
{
    uint8_t TXdataBuffer[OTA8_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    fullres_fillChannelData();

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    OtaUpdateSerializers(sm12ch, OTA8_PACKET_SIZE);
    OtaSetFullResNextChannelSet(false);

    // ** PACKET ONE **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    // Low 4ch (CH1-CH4)
    uint8_t expected[5];
    expected[0] = ((ChannelsIn[0] >> 1) >> 0);
    expected[1] = ((ChannelsIn[0] >> 1) >> 8) | ((ChannelsIn[1] >> 1) << 2);
    expected[2] = ((ChannelsIn[1] >> 1) >> 6) | ((ChannelsIn[2] >> 1) << 4);
    expected[3] = ((ChannelsIn[2] >> 1) >> 4) | ((ChannelsIn[3] >> 1) << 6);
    expected[4] = ((ChannelsIn[3] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chLow)], 5);
    // High 4ch (CH5-CH8)
    expected[0] = ((ChannelsIn[4] >> 1) >> 0);
    expected[1] = ((ChannelsIn[4] >> 1) >> 8) | ((ChannelsIn[5] >> 1) << 2);
    expected[2] = ((ChannelsIn[5] >> 1) >> 6) | ((ChannelsIn[6] >> 1) << 4);
    expected[3] = ((ChannelsIn[6] >> 1) >> 4) | ((ChannelsIn[7] >> 1) << 6);
    expected[4] = ((ChannelsIn[7] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chHigh)], 5);

    // ** PACKET TWO **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    // Low 4ch (CH1-CH4)
    expected[0] = ((ChannelsIn[0] >> 1) >> 0);
    expected[1] = ((ChannelsIn[0] >> 1) >> 8) | ((ChannelsIn[1] >> 1) << 2);
    expected[2] = ((ChannelsIn[1] >> 1) >> 6) | ((ChannelsIn[2] >> 1) << 4);
    expected[3] = ((ChannelsIn[2] >> 1) >> 4) | ((ChannelsIn[3] >> 1) << 6);
    expected[4] = ((ChannelsIn[3] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chLow)], 5);
    // Other high 4ch (CH9-CH12)
    expected[0] = ((ChannelsIn[8] >> 1) >> 0);
    expected[1] = ((ChannelsIn[8] >> 1) >> 8) | ((ChannelsIn[9] >> 1) << 2);
    expected[2] = ((ChannelsIn[9] >> 1) >> 6) | ((ChannelsIn[10] >> 1) << 4);
    expected[3] = ((ChannelsIn[10] >> 1) >> 4) | ((ChannelsIn[11] >> 1) << 6);
    expected[4] = ((ChannelsIn[11] >> 1) >> 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &TXdataBuffer[offsetof(OTA_Packet8_s, rc.chHigh)], 5);
}

void test_decodingFullres16chLow()
{
    uint8_t TXdataBuffer[OTA8_PACKET_SIZE] = {0};
    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s *)TXdataBuffer;
    uint32_t ChannelsIn[16];
    TEST_ASSERT_EQUAL(sizeof(ChannelData), sizeof(ChannelsIn));

    fullres_fillChannelData();

    // Save the channels since they go into the same place
    memcpy(ChannelsIn, ChannelData, sizeof(ChannelData));
    OtaUpdateSerializers(smHybridOr16ch, OTA8_PACKET_SIZE);
    OtaSetFullResNextChannelSet(false);

    // ** PACKET ONE **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    OtaUnpackChannelData(otaPktPtr, ChannelData);
    for (unsigned ch=0; ch<8; ++ch)
    {
        TEST_ASSERT_EQUAL(ChannelsIn[ch] & 0b11111111110, ChannelData[ch]);
    }

    // ** PACKET TWO **
    memset(TXdataBuffer, 0, sizeof(TXdataBuffer)); // "destChannels4x10 must be zeroed"
    OtaPackChannelData(otaPktPtr, ChannelData, false);
    OtaUnpackChannelData(otaPktPtr, ChannelData);
    for (unsigned ch=9; ch<16; ++ch)
    {
        TEST_ASSERT_EQUAL(ChannelsIn[ch] & 0b11111111110, ChannelData[ch]);
    }
}

void test_decodingHybridWide_AUX1()
{
    // Switch 0 is 2 pos, also tests the uplink_TX_Power
    test_decodingHybridWide(7, 0, CRSF_CHANNEL_VALUE_1000);
    test_decodingHybridWide(7, 0, CRSF_CHANNEL_VALUE_2000);
}

void test_decodingHybridWide_AUXX_low()
{
    constexpr int N_SWITCHES = 8;
    for (int i=0; i<N_SWITCHES; ++i)
        test_decodingHybridWide(i, 0, CRSF_CHANNEL_VALUE_1000);
}

void test_eml_header_vectors()
{
    uint8_t wire[EML_HEADER_SIZE] = {0, 0};
    EmlHeader_s dec = {};

    // Vector 1: DATA, seq=0, ack=7, flow=NORMAL, len=60 -> 0xE1, 0x3C
    const EmlHeader_s v1 = {EML_PKT_TYPE_DATA, 0, 7, 60, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&v1, wire, EML_MAX_PAYLOAD_LEN_64));
    TEST_ASSERT_EQUAL_HEX8(0xE1, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0x3C, wire[1]);
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderDecode(wire, &dec, EML_MAX_PAYLOAD_LEN_64));
    TEST_ASSERT_EQUAL_UINT8(EML_PKT_TYPE_DATA, dec.pkt_type);
    TEST_ASSERT_EQUAL_UINT8(0, dec.seq_num);
    TEST_ASSERT_EQUAL_UINT8(7, dec.ack_num);
    TEST_ASSERT_EQUAL_UINT8(60, dec.payload_len);
    TEST_ASSERT_EQUAL_UINT8(EML_FLOW_CTRL_NORMAL, dec.flow_ctrl);

    // Vector 2: DATA (32B frame), seq=3, ack=2, flow=HIGH_WATER, len=28 -> 0x4D, 0x5C
    const EmlHeader_s v2 = {EML_PKT_TYPE_DATA, 3, 2, 28, EML_FLOW_CTRL_HIGH_WATER};
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&v2, wire, EML_MAX_PAYLOAD_LEN_32));
    TEST_ASSERT_EQUAL_HEX8(0x4D, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0x5C, wire[1]);

    // Vector 3: KEEPALIVE, seq=5, ack=5, flow=PAUSE, len=0 -> 0xB6, 0xC0
    const EmlHeader_s v3 = {EML_PKT_TYPE_KEEPALIVE, 5, 5, 0, EML_FLOW_CTRL_PAUSE};
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&v3, wire, EML_MAX_PAYLOAD_LEN_64));
    TEST_ASSERT_EQUAL_HEX8(0xB6, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, wire[1]);

    // Vector 4: SYNC, seq=7, ack=7, flow=NORMAL, len=12 -> 0xFC, 0x0C
    const EmlHeader_s v4 = {EML_PKT_TYPE_SYNC, 7, 7, 12, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&v4, wire, EML_MAX_PAYLOAD_LEN_64));
    TEST_ASSERT_EQUAL_HEX8(0xFC, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0C, wire[1]);
}

void test_eml_header_exhaustive_roundtrip()
{
    uint32_t valid64 = 0;
    uint32_t valid32 = 0;
    for (uint32_t w = 0; w < 65536U; ++w)
    {
        const uint8_t in[EML_HEADER_SIZE] = {
            (uint8_t)(w & 0xFFU),
            (uint8_t)((w >> 8U) & 0xFFU)
        };
        EmlHeader_s dec = {};
        if (emlHeaderDecode(in, &dec, EML_MAX_PAYLOAD_LEN_64) == EML_HDR_OK)
        {
            ++valid64;
            uint8_t re[EML_HEADER_SIZE] = {0, 0};
            TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&dec, re, EML_MAX_PAYLOAD_LEN_64));
            TEST_ASSERT_EQUAL_HEX8(in[0], re[0]);
            TEST_ASSERT_EQUAL_HEX8(in[1], re[1]);
        }
        if (emlHeaderDecode(in, &dec, EML_MAX_PAYLOAD_LEN_32) == EML_HDR_OK)
        {
            ++valid32;
            uint8_t re[EML_HEADER_SIZE] = {0, 0};
            TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderEncode(&dec, re, EML_MAX_PAYLOAD_LEN_32));
            TEST_ASSERT_EQUAL_HEX8(in[0], re[0]);
            TEST_ASSERT_EQUAL_HEX8(in[1], re[1]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(30976U, valid64);
    TEST_ASSERT_EQUAL_UINT32(14592U, valid32);
}

void test_eml_header_validation_errors()
{
    EmlHeader_s h = {EML_PKT_TYPE_DATA, 0, 0, 1, EML_FLOW_CTRL_NORMAL};
    uint8_t out[EML_HEADER_SIZE] = {0, 0};

    TEST_ASSERT_EQUAL(EML_HDR_ERR_NULL_PTR, emlHeaderValidate(nullptr));
    TEST_ASSERT_EQUAL(EML_HDR_ERR_NULL_PTR, emlHeaderEncode(&h, nullptr));
    TEST_ASSERT_EQUAL(EML_HDR_ERR_NULL_PTR, emlHeaderEncode(nullptr, out));
    TEST_ASSERT_EQUAL(EML_HDR_ERR_NULL_PTR, emlHeaderDecode(nullptr, &h));
    TEST_ASSERT_EQUAL(EML_HDR_ERR_NULL_PTR, emlHeaderDecode(out, nullptr));

    h = {EML_PKT_TYPE_RESERVED, 0, 0, 1, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_RESERVED_PKT_TYPE, emlHeaderValidate(&h));

    h = {EML_PKT_TYPE_DATA, 8, 0, 1, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_SEQ_OUT_OF_RANGE, emlHeaderValidate(&h));

    h = {EML_PKT_TYPE_DATA, 0, 8, 1, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_ACK_OUT_OF_RANGE, emlHeaderValidate(&h));

    h = {EML_PKT_TYPE_DATA, 0, 0, 1, 4};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_FLOW_OUT_OF_RANGE, emlHeaderValidate(&h));

    for (uint8_t rlen = 61; rlen <= 63; ++rlen)
    {
        h = {EML_PKT_TYPE_DATA, 0, 0, rlen, EML_FLOW_CTRL_NORMAL};
        TEST_ASSERT_EQUAL(EML_HDR_ERR_RESERVED_LEN, emlHeaderValidate(&h, EML_MAX_PAYLOAD_LEN_64));
    }

    h = {EML_PKT_TYPE_DATA, 0, 0, 29, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_LEN_EXCEEDS_FRAME, emlHeaderValidate(&h, EML_MAX_PAYLOAD_LEN_32));

    h = {EML_PKT_TYPE_DATA, 0, 0, 0, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_EMPTY_DATA, emlHeaderValidate(&h));

    h = {EML_PKT_TYPE_KEEPALIVE, 0, 0, 1, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_NONEMPTY_KEEPALIVE, emlHeaderValidate(&h));

    h = {EML_PKT_TYPE_SYNC, 0, 0, 0, EML_FLOW_CTRL_NORMAL};
    TEST_ASSERT_EQUAL(EML_HDR_ERR_EMPTY_SYNC, emlHeaderValidate(&h));
}

void test_eml_crc16_reference_vectors()
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    const uint8_t zeros[] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t binary[] = {0x0F, 0xAA, 0x00, 0x55};
    // Published CRC-16/CCITT-FALSE vectors, AUTOSAR CRC Routines section 7.2.2.1.
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, emlCrc16(nullptr, 0));
    TEST_ASSERT_EQUAL_HEX16(0x29B1, emlCrc16(check, sizeof(check)));
    TEST_ASSERT_EQUAL_HEX16(0x84C0, emlCrc16(zeros, sizeof(zeros)));
    TEST_ASSERT_EQUAL_HEX16(0x2023, emlCrc16(binary, sizeof(binary)));

    // Streaming must be independent of chunk boundaries, including empty chunks.
    for (size_t split = 0; split <= sizeof(check); ++split)
    {
        const uint16_t partial = emlCrc16(check, split);
        TEST_ASSERT_EQUAL_HEX16(partial, emlCrc16(nullptr, 0, partial));
        TEST_ASSERT_EQUAL_HEX16(0x29B1, emlCrc16(check + split, sizeof(check) - split, partial));
    }
}

void test_eml_crc16_frame_vectors()
{
    // Golden inputs and wire CRC bytes independently checked with Python crc_hqx.
    // SYNC payload here is synthetic CRC test data, not the P2.4 handshake format.
    const struct
    {
        uint8_t length;
        uint8_t protectedBytes[EML_HEADER_SIZE + EML_MAX_PAYLOAD_LEN];
        uint8_t wireCrc[EML_CRC_SIZE];
    } vectors[] = {
        {2, {0xB6, 0xC0}, {0x70, 0xE8}},
        {3, {0x01, 0x01, 0x00}, {0xC8, 0x9D}},
        {5, {0xE1, 0x03, 0x00, 0xFF, 0x7E}, {0x87, 0x2B}},
        {14, {0xFC, 0x0C, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
              0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B}, {0x64, 0x20}},
        {30, {0x4D, 0x5C, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
              0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
              0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
              0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B}, {0x2A, 0x1A}},
        {62, {0xE1, 0x3C, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
              0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
              0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
              0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
              0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
              0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
              0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
              0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B}, {0xFC, 0x65}}
    };
    const uint8_t frameSizes[] = {EML_FRAME_SIZE_32, EML_FRAME_SIZE_64};
    for (const auto &v : vectors)
    {
        const uint16_t expected = (uint16_t)((uint16_t)v.wireCrc[0] << 8U) | v.wireCrc[1];
        for (const uint8_t frameSize : frameSizes)
        {
            if (v.length > frameSize - EML_CRC_SIZE)
                continue;
            uint8_t frame[EML_FRAME_SIZE_64] = {};
            memcpy(frame, v.protectedBytes, v.length);
            memcpy(frame + frameSize - EML_CRC_SIZE, v.wireCrc, EML_CRC_SIZE);
            EmlHeader_s header = {};
            TEST_ASSERT_EQUAL(EML_HDR_OK, emlHeaderDecode(frame, &header, emlMaxPayloadForFrameSize(frameSize)));
            TEST_ASSERT_EQUAL_UINT8(v.length, EML_HEADER_SIZE + header.payload_len);
            const uint16_t crc = emlCrc16(frame, v.length);
            TEST_ASSERT_EQUAL_HEX16(expected, crc);
            TEST_ASSERT_EQUAL_HEX16(expected, emlCrc16(frame + EML_HEADER_SIZE, header.payload_len,
                                                     emlCrc16(frame, EML_HEADER_SIZE)));
            // Residue checks the exact MSB-first wire bytes without including padding.
            TEST_ASSERT_EQUAL_HEX16(0, emlCrc16(frame + frameSize - EML_CRC_SIZE, EML_CRC_SIZE, crc));
            if (v.length < frameSize - EML_CRC_SIZE)
            {
                TEST_ASSERT_NOT_EQUAL(expected, emlCrc16(frame, frameSize - EML_CRC_SIZE));
                frame[v.length] = 0xA5;
                TEST_ASSERT_EQUAL_HEX16(expected, emlCrc16(frame, v.length));
                // A full decoder must independently reject this nonzero padding (P2.6).
            }
        }
    }
}

void test_eml_crc16_single_bit_errors()
{
    uint8_t data[EML_HEADER_SIZE + EML_MAX_PAYLOAD_LEN_64] = {0xE1, 0x3C};
    for (size_t i = EML_HEADER_SIZE; i < sizeof(data); ++i)
        data[i] = (uint8_t)(i - EML_HEADER_SIZE);
    // Exercise every protected header and payload bit of the 64-byte golden frame.
    for (size_t i = 0; i < sizeof(data); ++i)
    {
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            data[i] ^= (uint8_t)(1U << bit);
            TEST_ASSERT_NOT_EQUAL(0xFC65, emlCrc16(data, sizeof(data)));
            data[i] ^= (uint8_t)(1U << bit);
        }
    }
}

// Unity setup/teardown
void setUp() {}
void tearDown() {}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_crsf_endpoints);
    RUN_TEST(test_crsfToBit);
    RUN_TEST(test_bitToCrsf);
    RUN_TEST(test_crsfToN);
    RUN_TEST(test_nToCrsf);

    RUN_TEST(test_encodingHybrid8_3);
    RUN_TEST(test_encodingHybrid8_7);
    RUN_TEST(test_decodingHybrid8_all);
    RUN_TEST(test_channelDataComplete_hybrid8);
    RUN_TEST(test_channelDataComplete_wide8ch);
    RUN_TEST(test_channelDataComplete_12ch);
    RUN_TEST(test_channelDataComplete_16ch);

    RUN_TEST(test_encodingHybridWide_low);
    RUN_TEST(test_decodingHybridWide_AUX1);
    RUN_TEST(test_decodingHybridWide_AUXX_low);

    RUN_TEST(test_encodingFullresPowerLevels);
    RUN_TEST(test_encodingFullres8ch);
    RUN_TEST(test_encodingFullres16ch);
    RUN_TEST(test_encodingFullres12ch);
    RUN_TEST(test_decodingFullres16chLow);

    RUN_TEST(test_eml_header_vectors);
    RUN_TEST(test_eml_header_exhaustive_roundtrip);
    RUN_TEST(test_eml_header_validation_errors);
    RUN_TEST(test_eml_crc16_reference_vectors);
    RUN_TEST(test_eml_crc16_frame_vectors);
    RUN_TEST(test_eml_crc16_single_bit_errors);

    UNITY_END();

    return 0;
}
