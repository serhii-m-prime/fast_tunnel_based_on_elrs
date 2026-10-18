#include "targets.h"

#if defined(EML_BRIDGE)

#include "rxtx_common.h"
#include "eml_bridge.h"
#include "devButton.h"
#include "devLED.h"
#include "devWIFI.h"
#include <LittleFS.h>

#if defined(PLATFORM_ESP32)
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_system.h>
#include "devThermal.h"
#else
#include <user_interface.h>
#include <uart.h>
#endif

// EML owns both byte queues, ARQ, PHY and the timer. No handset, RC device,
// telemetry parser or serial protocol wrapper is constructed in this entry point.
#if defined(TARGET_TX)
TxConfig config;
#else
RxConfig config;
static ELRS_EEPROM eeprom;
#endif

static device_affinity_t bridgeDevices[] = {
    {&LED_device, 0},
    {&RGB_device, 0},
    {&WIFI_device, 0},
    {&Button_device, 0},
#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
    {&Thermal_device, 0},
#endif
};
static bool uartReady = false;
static bool radioReady = false;

#if defined(PLATFORM_ESP8266)
// Preserve the RX boot-time calibration policy; it avoids a Wi-Fi calibration
// current spike before recovery is even requested on small receiver supplies.
RF_PRE_INIT()
{
#if defined(RF_CAL_MODE)
    system_phy_set_powerup_option(RF_CAL_MODE);
#else
    system_phy_set_powerup_option(2);
#endif
}
#endif

static void setupSerial()
{
    firmwareOptions.uart_baud = normalizeBridgeBaud(firmwareOptions.uart_baud);
#if defined(PLATFORM_ESP32)
    Serial.end();
    esp_log_level_set("*", ESP_LOG_NONE);
    Serial.begin(firmwareOptions.uart_baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM, false);
    uart_set_line_inverse(0, UART_SIGNAL_INV_DISABLE);
#else
    uart_set_debug(UART_NO);
    system_set_os_print(0);
    Serial.begin(firmwareOptions.uart_baud, SERIAL_8N1, SERIAL_FULL, 1, false);
#endif
    Serial.setDebugOutput(false);
    Serial.setTimeout(0);
    // Drain only the bytes already buffered at startup, even if a host is flooding
    // the UART. Later disconnected input is bounded by emlBridgeReadSerial().
    for (int remaining = Serial.available(); remaining > 0; --remaining)
        Serial.read();
    uartReady = true;
}

static void applyPower()
{
    PowerLevels_e power = (PowerLevels_e)config.GetPower();
#if defined(TARGET_RX)
    // PWR_MATCH_TX requires the removed CRSF telemetry. EML power feedback is
    // P8.3/P8.4; retain the old startup fallback for this persisted setting.
    if (power == PWR_MATCH_TX)
        power = POWERMGNT::getDefaultPower();
#endif
    POWERMGNT::setPower(power);
}

static void resetConfigAndReboot()
{
    emlBridgeSuspend(true);
    config.SetDefaults(true);
    yield();
    LittleFS.format();
    yield();
    LittleFS.begin();
    options_SetTrueDefaults();
    ESP.restart();
}

#if defined(TARGET_TX)
static void cyclePower()
{
    if (connectionState < MODE_STATES)
    {
        if (POWERMGNT::currPower() == POWERMGNT::getMaxPower())
            POWERMGNT::setPower(POWERMGNT::getMinPower());
        else
            POWERMGNT::incPower();
    }
}
#endif

void setup()
{
    if (!options_init())
    {
        // No hardware layout is safe to drive. Keep the recovery-only boot path.
        static device_affinity_t recoveryDevices[] = {{&WIFI_device, 0}};
        devicesRegister(recoveryDevices, ARRAY_SIZE(recoveryDevices));
        devicesInit();
        setConnectionState(hardwareUndefined);
        devicesStart();
        return;
    }

    setupSerial();
    if (GPIO_PIN_BUTTON == UNDEF_PIN)
        pinMode(0, INPUT_PULLUP); // UART bootloader entry must remain available
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_CTRL, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_CTRL, LOW);
    }
#if defined(TARGET_RX)
    if (GPIO_PIN_ANT_GROUP != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_GROUP, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_GROUP, LOW);
    }
    eeprom.Begin();
    config.SetStorageProvider(&eeprom);
#endif
    config.Load();

#if defined(TARGET_TX)
    if (firmwareOptions.hasUID)
        memcpy(UID, firmwareOptions.uid, UID_LEN);
    else
    {
#if defined(PLATFORM_ESP32)
        esp_read_mac(UID, ESP_MAC_WIFI_STA);
#else
        wifi_get_macaddr(STATION_IF, UID);
#endif
    }
#else
    if (config.GetIsBound())
        memcpy(UID, config.GetUID(), UID_LEN);
#endif
    // Keep the same pair identity and FHSS seed as before the cleanup.
    FHSSrandomiseFHSSsequence(OtaGetUidSeed());
    setupTargetCommon();
    devicesRegister(bridgeDevices, ARRAY_SIZE(bridgeDevices));
    devicesInit();

    Radio.currFreq = FHSSgetInitialFreq();
    if (GPIO_PIN_SCK == UNDEF_PIN || !Radio.Begin())
        setConnectionState(radioFailed);
    else
    {
        POWERMGNT::init();
        applyPower();
        emlBridgeBegin();
        radioReady = connectionState != radioFailed;
    }

    // Air binding is absent in EML. Wi-Fi registers its recovery action itself.
    registerButtonFunction(ACTION_RESET_REBOOT, resetConfigAndReboot);
#if defined(TARGET_TX)
    registerButtonFunction(ACTION_INCREASE_POWER, cyclePower);
#endif
    devicesStart();
}

void loop()
{
    const unsigned long now = millis();
    if (uartReady)
        emlBridgeDrainSerial(Serial);
    devicesUpdate(now);
    checkRebootTime(now);
    executeDeferredFunction(micros());
    if (uartReady)
        emlBridgeReadSerial(Serial);

    if (connectionState > MODE_STATES)
        return;
    emlBridgeLoop();
    if (config.IsModified())
    {
        emlBridgeSuspend(true);
        const uint32_t changes = config.Commit();
        if (radioReady)
            applyPower();
        emlBridgeSuspend(false);
        devicesTriggerEvent(changes);
    }
}

#endif // EML_BRIDGE
