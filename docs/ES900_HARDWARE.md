# Апаратна карта ES900TX / ES900RX для UART-мосту

Результат P1.1, 2026-10-17. Карта зіставляє штатні ExpressLRS targets із потрібними інтерфейсами мосту. GPIO нижче — номери виводів MCU, а TX/RX у назвах UART — напрямки відносно модуля.

Джерела конфігурації: firmware `af67d57069a692262568a9639c5bbd08c524ba4b`, targets `51a697b3220f069baf3adf2c5353b710df70bbe0`. Профілі й код перевірено статично; прозвонювання конкретних плат, осцилограми UART/IRQ та вимірювання RF залишаються апаратними перевірками P1.7, P5.6, P6.6 та P8.6. Маркування TX `HP037.0184-FCC / 2D20017` надане в [AGENTS.md](../AGENTS.md); окремого профілю для цієї ревізії в targets немає.

## Вибір профілів

| Плата | Ключ у `targets.json` | Layout | Базове PlatformIO environment |
| --- | --- | --- | --- |
| HappyModel ES900TX, ESP32 + SX1276 | `happymodel.tx_900.es900` | `TX/HappyModel ES900.json` | `Unified_ESP32_900_TX_via_UART` |
| HappyModel ES900RX Nano, ESP8285 + SX1276 | `happymodel.rx_900.es900` | `RX/Generic 900.json` | `Unified_ESP8285_900_RX_via_UART` |

Відповідність ключів і layout зафіксована в [targets.json](https://github.com/ExpressLRS/targets/blob/51a697b3220f069baf3adf2c5353b710df70bbe0/targets.json). ES900RX Nano використовує саме `Generic 900.json`; `es900_dual` — інша плата з ESP32 і двома радіочипами. ES915 на STM32 також має окремі профілі.

Локальні файли `src/hardware/` завантажуються окремо й ігноруються Git. Посилання на зафіксовані layout: [ES900TX](https://github.com/ExpressLRS/targets/blob/51a697b3220f069baf3adf2c5353b710df70bbe0/TX/HappyModel%20ES900.json), [ES900RX](https://github.com/ExpressLRS/targets/blob/51a697b3220f069baf3adf2c5353b710df70bbe0/RX/Generic%20900.json). В обох записах немає `overlay`.

## GPIO та призначення

| Сигнал / поле JSON | ES900TX | ES900RX | Використання у прошивці |
| --- | --- | --- | --- |
| SPI SCK / `radio_sck` | GPIO18 | GPIO14 | Тактування SX1276 |
| SPI MISO / `radio_miso` | GPIO19 | GPIO12 | SX1276 → MCU |
| SPI MOSI / `radio_mosi` | GPIO23 | GPIO13 | MCU → SX1276 |
| SPI NSS / `radio_nss` | GPIO5 | GPIO15 | Вибір SX1276, активний LOW |
| Reset / `radio_rst` | GPIO14 | GPIO2 | Reset радіо, LOW, потім вхід |
| IRQ / `radio_dio0` | GPIO26 | GPIO4 | Основне переривання радіо |
| `radio_dio1` | Не задано | GPIO5 | Поточний SX127x HAL не підключає ISR до DIO1 |
| `radio_busy` | Не задано | Не задано | Окремого BUSY GPIO профілі не задають |
| RX enable / `power_rxen` | GPIO13 | Не задано | У TX активний HIGH під час приймання |
| TX enable / `power_txen` | GPIO12 | Не задано | У TX активний HIGH під час передачі |
| PA enable / `power_enable` | Не задано | Не задано | Окремий GPIO дозволу PA відсутній у профілях |
| APC / `power_apc2` | GPIO25, DAC | Не задано | Аналогове керування зовнішнім PA TX |
| Fan enable / `misc_fan_en` | GPIO17 | Не задано | Вентилятор TX, активний HIGH |
| Fan PWM / tacho | Не задано | Не задано | Профілі не дають керування обертами або зворотного зв'язку |
| Кнопка / `button` | Не задано | GPIO0 | Вхід кнопки RX; також вивід вибору boot mode |
| LED | GPIO27, RGB GRB | GPIO16 | Індикація стану |
| Штатний `serial_rx` | GPIO2 | GPIO3 | Вхід даних від зовнішнього пристрою |
| Штатний `serial_tx` | GPIO2 | GPIO1 | Вихід даних до зовнішнього пристрою |
| `debug_backpack_rx` | GPIO3 | Не задано | Спільні з USB-UART контакти TX |
| `debug_backpack_tx` | GPIO1 | Не задано | Спільні з USB-UART контакти TX |

«Не задано» означає відсутнє поле layout. Для цілих полів [hardware.cpp](../src/lib/OPTIONS/hardware.cpp) задає `-1` (`UNDEF_PIN`), для прапорців — `false`. Це не доводить фізичну відсутність доріжки або компонента. Зокрема, не можна призначити TX DIO1 або кнопку без окремої перевірки плати. Другого радіо й керування перемиканням антен ці профілі не задають.

Поля JSON переходять у `GPIO_PIN_*` через [Unified_ESP32_TX.h](../src/include/target/Unified_ESP32_TX.h) та [Unified_ESP_RX.h](../src/include/target/Unified_ESP_RX.h). [SX127xHal.cpp](../src/lib/SX127xDriver/SX127xHal.cpp) використовує SPI 10 МГц: на ESP32 призначає контакти з профілю, на ESP8285 викликає апаратний `SPIEx.begin()` зі штатними HSPI-контактами, які збігаються з таблицею. HAL підключає переривання RISING лише до DIO0; reset утримує LOW 50 мс і потім переводить GPIO у вхід.

Для TX [RFAMP_hal.cpp](../src/lib/RFAMP/RFAMP_hal.cpp) задає такі стани програмного керування RF-трактом:

| Стан | GPIO12 TX enable | GPIO13 RX enable |
| --- | --- | --- |
| Передача | HIGH | LOW |
| Приймання | LOW | HIGH |
| RF вимкнено | LOW | LOW |

На RX відповідних GPIO немає в layout, тому HAL не керує зовнішнім FEM через ці сигнали. Вибір `PA_BOOST` усередині SX1276 не є GPIO enable.

## UART і USB ES900TX

Штатні `serial_rx = serial_tx = 2` описують спільну сигнальну лінію інтерфейсу пульта. Для PC-мосту потрібна інша пара: GPIO3 — RX MCU, GPIO1 — TX MCU, тобто штатні контакти UART0 ESP32. На цій парі також описано debug/backpack, `use_backpack = true`, `debug_backpack_baud = 460800`.

У [офіційній інструкції ExpressLRS для ES900TX](https://www.expresslrs.org/quick-start/transmitters/es900tx/) USB визначається як CP210x, а підключення до основного MCU обирається DIP `5,6 = ON`, `1–4 = OFF`. Це відповідає наданому в AGENTS.md підключенню CP2102. Для роботи PC-мосту слід зберігати цю маршрутизацію. Режим `1,2 = ON`, решта `OFF`, описаний тією ж інструкцією для роботи з пультом, має інше призначення.

Очікуваний шлях байтів:

| Напрямок | Підключення |
| --- | --- |
| PC → модуль | USB → CP2102 TX → DIP → ESP32 GPIO3 RX |
| Модуль → PC | ESP32 GPIO1 TX → DIP → CP2102 RX → USB |

У [tx_main.cpp, `setupSerial()`](../src/src/tx_main.cpp) у режимі `is_airport` потік користувача `TxUSB` закріплено безпосередньо за апаратним контролером `UART0` (`Serial`) на виводах `U0RXD_GPIO_NUM` (`GPIO3`) / `U0TXD_GPIO_NUM` (`GPIO1`) у форматі `SERIAL_8N1` із явно вимкненою інверсією (`uart_set_line_inverse(0, UART_SIGNAL_INV_DISABLE)`, `invert = false`) і нульовим таймаутом читання (`Serial.setTimeout(0)`). Якщо `GPIO_PIN_DEBUG_RX` / `GPIO_PIN_DEBUG_TX` збігаються з `GPIO3` / `GPIO1` (як у ES900TX), `BackpackOrLogStrm` спрямовується в `NullStream`. У [devHandset.cpp](../src/lib/Handset/devHandset.cpp) запуск `CRSFHandset` / `AutoDetect` у режимі `is_airport` вимкнено на всіх платформах (`return DURATION_NEVER`). [devBackpack.cpp](../src/lib/Backpack/devBackpack.cpp) також пропускає ініціалізацію backpack у режимі AirPort.

## UART ES900RX

Профіль уже задає потрібну повнодуплексну пару UART0: `serial_rx = 3`, `serial_tx = 1`. У [rx_main.cpp, `setupSerial()`](../src/src/rx_main.cpp) для `firmwareOptions.is_airport` виділено окрему гілку на початку функції, яка ігнорує `config.GetSerialProtocol()` в EEPROM і явно конфігурує `Serial` (`UART0`) у режимі `SERIAL_8N1`, `SERIAL_FULL`, `invert = false`, `Serial.setTimeout(0)`, а `BackpackOrLogStrm` встановлює в `NullStream`.

| Контакт ES900RX | Підключення до віддаленого UART |
| --- | --- |
| TX / GPIO1 | RX зовнішнього пристрою |
| RX / GPIO3 | TX зовнішнього пристрою |
| GND | Спільна земля |
| VCC / 5V | Живлення RX 5 В |

Живлення 5 В для RX вказано на [сторінці виробника](https://www.happymodel.cn/index.php/2021/02/19/expresslrs-module-es915tx-long-range-915mhz-transmitter-and-es915rx-receiver/). UART з'єднується на логічних рівнях MCU 3,3 В; значення живлення VCC не означає допустимість 5 В на сигнальних GPIO. Фізичний порядок майданчиків звіряти з написами конкретної плати; таблиця описує сигнали, а не їх геометричний порядок.

## Підтримувані швидкості UART0 (P1.4)

- **Підтримувані значення (`BRIDGE_SUPPORTED_BAUDS` у [options.h](../src/lib/OPTIONS/options.h)):** `115200`, `230400`, `420000`, `460800` біт/с (формат `8N1`, без інверсії, повний дуплекс).
- **Початкове значення (`BRIDGE_UART_BAUD_DEFAULT`):** `460800` біт/с для обох таргетів (`HappyModel_ES900TX_Bridge` і `HappyModel_ES900RX_Bridge`). При завантаженні опцій невалідні значення нормалізуються функцією `normalizeBridgeBaud()` до `460800`.
- **Способи зміни швидкості:**
  1. **Під час збірки:** через `-DUSE_AIRPORT_AT_BAUD=<baud>` у `PLATFORMIO_BUILD_FLAGS` або `src/user_defines.txt` (перевіряється в [build_flags.py](../src/python/build_flags.py)).
  2. **Через Wi-Fi WebUI (`POST /options.json` + `/reboot`):** поле `"airport-uart-baud"` (TX) або `"rcvr-uart-baud"` (RX) зберігається у `/options.json` у LittleFS після перевірки `isSupportedBridgeBaud()` у [devWIFI.cpp](../src/lib/WIFI/devWIFI.cpp) і застосовується під час перезапуску.
  3. **Оновленням конфігурації у `.bin` без перекомпіляції:** через [UnifiedConfiguration.py](../src/python/UnifiedConfiguration.py) із JSON-опцією `"airport-uart-baud"` / `"rcvr-uart-baud"`.

## PA, рівні потужності й охолодження

| Параметр layout | ES900TX | ES900RX |
| --- | --- | --- |
| `power_control` | `3`, DAC на GPIO25 | `0`, керування SX1276 напряму |
| `power_min` / `power_max` | `0` / `6` | `0` / `2` |
| `power_default` | `2` | `2` |
| `power_values` | `[41, 60, 73, 90, 110, 132, 190]` | `[120, 124, 127]` |
| `power_high` у JSON | `6` | `2` |
| `radio_rfo_hf` | Відсутнє → `false` | Відсутнє → `false` |

За enum у [POWERMGNT.h](../src/lib/POWERMGNT/POWERMGNT.h) індекси TX `0..6` відповідають номінальним 10/25/50/100/250/500/1000 мВт, із початковим рівнем 50 мВт. [POWERMGNT.cpp](../src/lib/POWERMGNT/POWERMGNT.cpp) передає значення таблиці TX у `dacWrite(GPIO25, value)`; це коди DAC, а не dBm і не виміряні мВт. `power_values2` не задано, тому базовий рівень самого SX1276 встановлюється драйвером під час `Begin()`.

У RX доступні лише індекси `0..2` (номінально 10/25/50 мВт). Значення `120/124/127` передаються в `SX127xDriver::SetOutputPower()`, який маскує біти й додає вибір `PA_BOOST`. Значення enum не підтверджують фактичну RF-потужність. Виробник для штатного ES900RX вказує telemetry power `<17 dBm` на [сторінці виробу](https://www.happymodel.cn/index.php/2021/02/19/expresslrs-module-es915tx-long-range-915mhz-transmitter-and-es915rx-receiver/). Вимога 100 мВт потребує окремої реалізації й перевірки PA_CONFIG / PA_DAC / OCP у P8.2; збільшення `power_max` саме по собі її не виконує.

`power_high` присутнє в обох layout, але відсутнє в таблиці полів поточного `hardware.cpp`, тому цей ключ не впливає на runtime. Використовуються `power_min`, `power_max`, `power_default` і таблиці значень.

У [devThermal.cpp](../src/lib/THERMAL/devThermal.cpp) GPIO17 вмикається рівнем HIGH. Початковий поріг у [config.cpp](../src/lib/CONFIG/config.cpp) — `PWR_250mW`; поточна логіка чекає чотири цикли перевірки, а вимкнення враховує `fan_min_runtime` (початково 30 с у [options.cpp](../src/lib/OPTIONS/options.cpp)). Профіль не задає `thermal_lm75a`, I²C, PWM або tacho, тому доступність датчика температури чи контролю обертів не підтверджено. Перевірка запуску вентилятора й охолодження залишається P8.5–P8.6.

## Ізоляція UART0 та поведінка під час завантаження (P1.5)

### Усунення програмних конфліктів на UART0 (`GPIO3` RX / `GPIO1` TX)

1. **Debug / системне логування (`logging`, ESP-IDF, NonOS SDK):**
   - У [logging.cpp](../src/lib/logging/logging.cpp) глобальний вказівник `BackpackOrLogStrm` ініціалізується статичним екземпляром `NullStream` (замість `nullptr`), а рання ініціалізація `debugCreateInitLogger()` (`DEBUG_INIT`) вимкнена при визначеному `USE_AIRPORT_AT_BAUD`.
   - На `ESP32` (`ES900TX`) у [tx_main.cpp](../src/src/tx_main.cpp) на початку `setup()` та в `setupSerial()` викликається `esp_log_level_set("*", ESP_LOG_NONE)` і `Serial.setDebugOutput(false)`, що блокує вивід повідомлень ESP-IDF / FreeRTOS у `UART0`.
   - На `ESP8285` (`ES900RX`) Arduino-ядро (`init_done()` у `core_esp8266_main.cpp`) перед входом у `setup()` вмикає `system_set_os_print(1)`. Оскільки `Serial.setDebugOutput(false)` перевіряє лише `uart_get_debug() == _uart_nr`, у [rx_main.cpp](../src/src/rx_main.cpp) на початку `setup()` і в `setupSerial()` явно викликаються `uart_set_debug(UART_NO)` (перенаправляє `ets_install_putc1` на `uart_ignore_char`) та `system_set_os_print(0)`, а також заблоковано `debugRcvrLinkstats()` і ранній `Serial.begin()` для `DEBUG_LOG`.
2. **Backpack (`devBackpack`):**
   - У [devBackpack.cpp](../src/lib/Backpack/devBackpack.cpp) окрім пропуску реєстрації пристрою (`initialize()`), функції `checkBackpackUpdate()` (яка викликається напряму з `loop()` у `tx_main.cpp`), `feedUSB()`, `sendCRSFTelemetryToBackpack()` та `sendMAVLinkTelemetryToBackpack()` негайно повертають керування при `firmwareOptions.is_airport == true`. Це унеможливлює перехоплення `UART0`, passthrough-скидання через `GPIO0` або приведення `BackpackOrLogStrm` (`NullStream`) до `HardwareSerial*`.
3. **CRSF / Handset / Lua / MAVLink / I²C:**
   - У [devHandset.cpp](../src/lib/Handset/devHandset.cpp), [devTXLUA.cpp](../src/lib/tx-crsf/devTXLUA.cpp) та [devRXLUA.cpp](../src/lib/rx-crsf/devRXLUA.cpp) робота CRSF handset та Lua-пристроїв вимкнена для `firmwareOptions.is_airport`.
   - У [SerialAirPort.cpp](../src/src/rx-serial/SerialAirPort.cpp) `sendRCFrame()` повертає `DURATION_NEVER`, щоб `devSerialIO` не генерував події відправлення RC-кадрів.
   - У [tx_main.cpp](../src/src/tx_main.cpp) примусово встановлюється `config.SetLinkMode(TX_NORMAL_MODE)`, а `UARTconnected()` у режимі `is_airport` переводить стан напряму в `disconnected` (оминаючи стан `awaitingModelId`, який у штатному ELRS затримує початок RF-передачі на `3–6 с` в очікуванні CRSF `ModelID` від пульта) та викликається лише якщо `connectionState < FAILURE_STATES` (щоб не блокувати автозапуск Wi-Fi при `radioFailed`).
   - У [rx_main.cpp](../src/src/rx_main.cpp) у режимі `is_airport` заблоковано перемикання `config.SetSerialProtocol()` із `SYNC`-пакетів (`TX_MAVLINK_MODE`), вимкнено `setupSerial1()`, `crsfRouter`, `checkSendLinkStatsToFc()`, опитування `otaConnector` та гілку `PROTOCOL_MAVLINK` (яка приводила `serialIO` до `SerialMavlink*`), а `reconfigureSerial()` лише очищує черги `AirPort` без перестворення `serialIO`.
   - У [rxtx_common.cpp](../src/src/rxtx_common.cpp) `setupWire()` ігнорує будь-яку конфігурацію `SDA`/`SCL`, якщо вона вказує на `U0RXD_GPIO_NUM` (`GPIO3`) або `U0TXD_GPIO_NUM` (`GPIO1`).

### Поведінка boot-ROM виводу та приймання байтів під час завантаження

| Фаза | TX (`ESP32` + `CP2102`, `GPIO1` TX / `GPIO3` RX) | RX (`ESP8285`, `GPIO1` TX / `GPIO3` RX) |
| --- | --- | --- |
| **Апаратний ROM-завантажувач (до `setup()`)** | ROM 1-го рівня та bootloader 2-го рівня видають службовий лог на `GPIO1` зі швидкістю **`115200` біт/с (`8N1`)** протягом `~250–300 мс` після подачі живлення або скидання (`EN`). | ROM-завантажувач та `eboot` видають стартовий рядок на `GPIO1` зі швидкістю **`74880` біт/с (`8N1`, кварц 26 МГц)** протягом `~50–100 мс` після подачі живлення або скидання. |
| **Ініціалізація прошивки (`setup()` / `setupSerial()`)** | Вимикається логування ESP-IDF (`ESP_LOG_NONE`) і `Serial.setDebugOutput(false)`, налаштовується `UART0` на робочий baud мосту (`8N1`, без інверсії), після чого апаратний RX FIFO (`Serial`) та буфери `apInputBuffer`/`apOutputBuffer`/`uartInputBuffer` повністю очищуються. | Вимикаються OS-принти NonOS SDK (`uart_set_debug(UART_NO)`, `system_set_os_print(0)`) і `Serial.setDebugOutput(false)`, налаштовується `UART0` на робочий baud мосту (`8N1`, `SERIAL_FULL`, без інверсії), після чого апаратний RX FIFO та буфери `apInputBuffer`/`apOutputBuffer` очищуються. |
| **Очікування RF-з'єднання (`connectionState != connected`)** | Вхідні байти з `UART0` (`TxUSB`) у `HandleUARTin()` безперервно вичитуються з апаратного FIFO й **відкидаються** (не накопичуються в `apInputBuffer` і не передаються у `feedUSB()`). Запис у `UART0` не здійснюється. | Вхідні байти з `UART0` у `SerialAirPort::processBytes()` вичитуються й **відкидаються**; у `ProcessRfPacket_DataUl()` пакети `AirPort` у стані `tentative` ігноруються. Запис у `UART0` не здійснюється. |
| **Момент встановлення RF-з'єднання (`connected`)** | Під час переходу у `connected` (`ConnectionUpdate()`) черги `apInputBuffer` та `apOutputBuffer` очищуються; починається двобічна передача байтів. | Під час переходу у `connected` (`GotConnection()`) черги `apInputBuffer` та `apOutputBuffer` очищуються; починається двобічна передача байтів. |

> **Примітка для хоста / зовнішнього контролера:** оскільки апаратний boot-ROM обох чипів (`ESP32` на `115200` біт/с, `ESP8285` на `74880` біт/с) видає байти на лінію `GPIO1` (`TX`) до старту `setup()`, на робочій швидкості `460800` біт/с ці імпульси виглядають як короткий пакет некоректних байтів або помилок фреймінгу в перші `~100–300 мс` після скидання модуля. Протокол верхнього рівня на боці хоста або вежі має ігнорувати вхідні байти без валідної кадрової синхронізації під час перезапуску живлення, а передачу даних починати після встановлення RF-лінку (або покладатися на те, що байти, надіслані до `connected`, детерміновано відкидаються).

## Висновки для наступних завдань

| Завдання | Зафіксована передумова |
| --- | --- |
| P1.2–P1.3 | Використати зазначені ключі targets і ролі ESP32 TX / ESP8285 RX; зберегти SPI, RF enable, DAC, fan та LED. Додано environments `HappyModel_ES900TX_Bridge` та `HappyModel_ES900RX_Bridge`. |
| P1.4–P1.5 | Виділено `GPIO1`/`GPIO3` (`UART0`) обох плат для UART-мосту (`8N1`, без інверсії, підтримувані швидкості `115200`/`230400`/`420000`/`460800`, типова `460800`). Повністю ізольовано `UART0` від debug/OS-логів, backpack, CRSF/Lua та MAVLink; зафіксовано поведінку boot-ROM і відкидання вхідних байтів до переходу в `connected`. |
| P1.6–P1.7 | Зберегти шлях UART bootloader через GPIO0 і Wi-Fi recovery. RX має `button = 0`; TX application button у layout не задано. Фізичний доступ до BOOT/EN, DIP і працездатність відновлення перевірити на платах. |
| P5.3–P5.4 | FSK на TX не може вимагати DIO1 без підтвердження підключення; у поточному HAL ISR встановлено лише для DIO0. |
| P8.1–P8.2 | Зберегти сім кодів DAC TX; штатний RX не реалізує цільові 100 мВт. Потрібні окремі перевірки регістрів і вимірювання. |
| P6.6, P8.6 | Ці 915-МГц профілі не підтверджують працездатність RF-тракту або калібрування PA у 960–1020 МГц. |

P1.1–P1.5 не змінюють розмір пакета, PHY, FHSS або період таймера, тому новий розрахунок ToA для них не потрібен. Результати контрольних збірок і використання пам'яті записуються в [журнал плану](../IMPLEMENTATION_PLAN.md#журнал-виконання).
