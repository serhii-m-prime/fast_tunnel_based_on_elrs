#pragma once

#include "targets.h"

extern const unsigned char target_name[];
extern const uint8_t target_name_size;
extern const char commit[];
extern const char version[];

extern const char *wifi_hostname;
extern const char *wifi_ap_ssid;
extern const char *wifi_ap_password;
extern const char *wifi_ap_address;

enum BuzzerMode {
    buzzerQuiet,
    buzzerOne,
    buzzerTune
};

typedef struct _options {
    uint8_t     _magic_[8];     // this is the magic constant so the configurator can find this options block
    uint16_t    _version_;      // the version of this structure
    uint8_t     domain;         // depends on radio chip
    uint8_t     hasUID;
    uint8_t     uid[6];         // MY_UID derived from MY_BINDING_PHRASE
    uint32_t    flash_discriminator;    // Discriminator value used to determine if the device has been reflashed and therefore
                                        // the SPIFSS settings are obsolete and the flashed settings should be used in preference
    uint32_t    fan_min_runtime;
    int32_t     wifi_auto_on_interval;
    char        home_wifi_ssid[33];
    char        home_wifi_password[65];
#if defined(TARGET_RX)
    uint32_t    uart_baud;
    bool        _unused1:1; // invert_tx
    bool        lock_on_first_connection:1;
    bool        dji_permanently_armed:1;
    bool        is_airport:1;
#endif
#if defined(TARGET_TX) || defined(UNIT_TEST)
    uint32_t    tlm_report_interval;
    bool        _unused1:1;
    bool        unlock_higher_power:1;
    bool        is_airport:1;
    uint32_t    uart_baud;              // only use for airport
#endif
#if defined(EML_BRIDGE)
    uint8_t     bridge_default_band;
    uint8_t     bridge_lq_threshold;
    uint8_t     bridge_phy;
    uint8_t     bridge_tdd_mode;
    uint8_t     bridge_role;
    uint8_t     bridge_traffic_mode;
    uint8_t     bridge_power;
    uint8_t     bridge_fan_threshold;
    bool        bridge_adaptive_fallback;
    bool        bridge_uart_inverted;
    bool        bridge_dynamic_power;
#endif
} __attribute__((packed)) firmware_options_t;

// Layout is PRODUCTNAME DEVICENAME OPTIONS HARDWARE
constexpr size_t ELRSOPTS_PRODUCTNAME_SIZE = 128;
constexpr size_t ELRSOPTS_DEVICENAME_SIZE = 16;
constexpr size_t ELRSOPTS_OPTIONS_SIZE = 512;
constexpr size_t ELRSOPTS_HARDWARE_SIZE = 2048;

constexpr uint32_t BRIDGE_UART_BAUD_DEFAULT = 460800U;
constexpr uint32_t BRIDGE_SUPPORTED_BAUDS[] = {
    57600U,
    115200U,
    230400U,
    420000U,
    460800U,
};

enum : uint8_t
{
    EML_OPTION_PHY_GFSK_250 = 0,
    // The SX1276 packet modem is specified only up to 300 kbit/s. Keep the
    // requested UI value reserved, but reject it until P5.7 is proven on hardware.
    EML_OPTION_PHY_GFSK_500 = 1,
    EML_OPTION_PHY_LORA_SF6 = 2,
};
enum : uint8_t { EML_OPTION_TDD_SYMMETRIC = 0, EML_OPTION_TDD_ADAPTIVE = 1 };
// Auto listens for a Master at boot and otherwise becomes one (P9.4)
enum : uint8_t { EML_OPTION_ROLE_MASTER = 0, EML_OPTION_ROLE_SLAVE = 1, EML_OPTION_ROLE_AUTO = 2 };
enum : uint8_t { EML_OPTION_TRAFFIC_RAW = 0, EML_OPTION_TRAFFIC_MAVLINK2 = 1, EML_OPTION_TRAFFIC_SBUS = 2 };

inline bool isSupportedBridgePower(uint8_t power)
{
    return power <= 6; // PowerLevels_e PWR_10mW .. PWR_1000mW
}

inline bool isSupportedBridgeBaud(uint32_t baud)
{
    for (size_t i = 0; i < sizeof(BRIDGE_SUPPORTED_BAUDS) / sizeof(BRIDGE_SUPPORTED_BAUDS[0]); ++i)
    {
        if (BRIDGE_SUPPORTED_BAUDS[i] == baud)
        {
            return true;
        }
    }
    return false;
}

inline uint32_t normalizeBridgeBaud(uint32_t baud)
{
    return isSupportedBridgeBaud(baud) ? baud : BRIDGE_UART_BAUD_DEFAULT;
}

extern char device_name[];
extern firmware_options_t firmwareOptions;
extern bool options_init();

#if !defined(UNIT_TEST)
extern char product_name[];
extern uint32_t logo_image;
extern String& getOptions();
extern String& getHardware();
extern void saveOptions();
void setOptions(String &options);

#include "EspFlashStream.h"
bool options_HasStringInFlash(EspFlashStream &strmFlash);
void options_SetTrueDefaults();
#endif
