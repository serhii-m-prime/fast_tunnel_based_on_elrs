#include <unity.h>
#include "eml_power.h"
#include "../../lib/SX127xDriver/SX127xRegs.h"

static EmlLinkReport_s report(uint8_t lq, int8_t rssi, bool snrValid = false, int8_t snr = 0)
{
    return {EML_PHY_FSK_250, lq, (uint8_t)(100U - lq), rssi, snr, snrValid};
}

void test_link_report_round_trip_and_snr_validity()
{
    const EmlLinkReport_s fsk = report(83, -91);
    const EmlBandMessage_s coordinator = emlLinkMessage(0, fsk);
    TEST_ASSERT_EQUAL_UINT8(EML_LINK_STATUS_COORDINATOR, coordinator.op);
    TEST_ASSERT_TRUE(emlBandMessageValid(coordinator));
    EmlLinkReport_s decoded = {};
    TEST_ASSERT_TRUE(emlLinkReport(coordinator, &decoded));
    TEST_ASSERT_EQUAL_UINT8(83, decoded.lq);
    TEST_ASSERT_EQUAL_UINT8(17, decoded.per);
    TEST_ASSERT_EQUAL_INT8(-91, decoded.rssi);
    TEST_ASSERT_FALSE(decoded.snr_valid); // FSK must never imply LoRa SNR
    EmlBandMessage_s inconsistent = coordinator;
    ++inconsistent.switch_slot;
    TEST_ASSERT_FALSE(emlBandMessageValid(inconsistent));

    const EmlLinkReport_s lora = {EML_PHY_LORA_500_SF6_CR45, 97, 3, -78, -4, true};
    const EmlBandMessage_s responder = emlLinkMessage(1, lora);
    TEST_ASSERT_EQUAL_UINT8(EML_LINK_STATUS_RESPONDER, responder.op);
    TEST_ASSERT_TRUE(emlLinkReport(responder, &decoded));
    TEST_ASSERT_TRUE(decoded.snr_valid);
    TEST_ASSERT_EQUAL_INT8(-4, decoded.snr);
}

void test_dynamic_power_hysteresis_limits_and_feedback_loss()
{
    EmlDynamicPower power;
    power.begin(true, 0, 6, 2, 60, 0);
    TEST_ASSERT_FALSE(power.receive(report(40, -110), 100)); // rate limiter
    TEST_ASSERT_TRUE(power.receive(report(40, -110), 300));
    TEST_ASSERT_EQUAL_UINT8(3, power.power());
    TEST_ASSERT_TRUE(power.receive(report(40, -110), 600));
    TEST_ASSERT_EQUAL_UINT8(4, power.power());

    TEST_ASSERT_FALSE(power.receive(report(100, -60), 1000));
    TEST_ASSERT_FALSE(power.receive(report(100, -60), 5999));
    TEST_ASSERT_TRUE(power.receive(report(100, -60), 6000));
    TEST_ASSERT_EQUAL_UINT8(3, power.power());

    TEST_ASSERT_FALSE(power.tick(7999));
    TEST_ASSERT_TRUE(power.tick(8000));
    TEST_ASSERT_EQUAL_UINT8(6, power.power());
    TEST_ASSERT_FALSE(power.receive(report(20, -120), 9000)); // already clamped
    TEST_ASSERT_EQUAL_UINT8(6, power.power());
}

void test_dynamic_power_can_be_disabled()
{
    EmlDynamicPower power;
    power.begin(false, 0, 6, 3, 60, 0);
    TEST_ASSERT_FALSE(power.receive(report(0, -120), 10000));
    TEST_ASSERT_FALSE(power.tick(20000));
    TEST_ASSERT_EQUAL_UINT8(3, power.power());
}

void test_es900rx_20dbm_register_policy()
{
    TEST_ASSERT_EQUAL_HEX8(0x8f, SX127X_PA_SELECT_BOOST | 0x0f);
    TEST_ASSERT_EQUAL_HEX8(0x87, SX127X_PA_DAC_20DBM);
    TEST_ASSERT_EQUAL_HEX8(0x84, SX127X_PA_DAC_DEFAULT);
    TEST_ASSERT_EQUAL_HEX8(0x32, SX127X_OCP_ON | SX127X_OCP_150MA);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_link_report_round_trip_and_snr_validity);
    RUN_TEST(test_dynamic_power_hysteresis_limits_and_feedback_loss);
    RUN_TEST(test_dynamic_power_can_be_disabled);
    RUN_TEST(test_es900rx_20dbm_register_policy);
    return UNITY_END();
}
