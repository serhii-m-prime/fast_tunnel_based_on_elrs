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

У поточному [tx_main.cpp, `setupSerial()`](../src/src/tx_main.cpp) AirPort уже спрямовує `HardwareSerial(1)` на `U0RXD_GPIO_NUM` / `U0TXD_GPIO_NUM`. Для ES900TX `TxUSB` посилається на той самий `BackpackOrLogStrm`, бо debug-контакти збігаються. Отже, назва контакту UART0 не означає, що в AirPort використано саме контролер UART0: зараз це UART1, спрямований на GPIO3/GPIO1. Остаточне виділення UART0 мосту належить до P1.4–P1.5.

Bridge-конфігурація має явно закріпити GPIO3/GPIO1 за потоком користувача та виключити їх повторну ініціалізацію debug/backpack/CRSF. Простого переписування `serial_rx/tx` недостатньо: [devHandset.cpp](../src/lib/Handset/devHandset.cpp) та `setupSerial()` також визначають власників UART. [devBackpack.cpp](../src/lib/Backpack/devBackpack.cpp) уже пропускає ініціалізацію backpack у режимі AirPort. Це потрібно зберегти й перевірити в P1.5.

## UART ES900RX

Профіль уже задає потрібну повнодуплексну пару UART0: `serial_rx = 3`, `serial_tx = 1`. [rx_main.cpp, `setupSerial()`](../src/src/rx_main.cpp) на ESP8285 використовує `Serial`, а в AirPort бере швидкість із `firmwareOptions.uart_baud`. Параметри інверсії й формату ще залежать від налаштування serial protocol; P1.4 має явно забезпечити 8N1 без інверсії для мосту.

| Контакт ES900RX | Підключення до віддаленого UART |
| --- | --- |
| TX / GPIO1 | RX зовнішнього пристрою |
| RX / GPIO3 | TX зовнішнього пристрою |
| GND | Спільна земля |
| VCC / 5V | Живлення RX 5 В |

Живлення 5 В для RX вказано на [сторінці виробника](https://www.happymodel.cn/index.php/2021/02/19/expresslrs-module-es915tx-long-range-915mhz-transmitter-and-es915rx-receiver/). UART з'єднується на логічних рівнях MCU 3,3 В; значення живлення VCC не означає допустимість 5 В на сигнальних GPIO. Фізичний порядок майданчиків звіряти з написами конкретної плати; таблиця описує сигнали, а не їх геометричний порядок.

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

## Висновки для наступних завдань

| Завдання | Зафіксована передумова |
| --- | --- |
| P1.2–P1.3 | Використати зазначені ключі targets і ролі ESP32 TX / ESP8285 RX; зберегти SPI, RF enable, DAC, fan та LED. Додано environments `HappyModel_ES900TX_Bridge` та `HappyModel_ES900RX_Bridge`. |
| P1.4–P1.5 | Виділити GPIO1/GPIO3 обох плат для UART-мосту; у TX відокремити його від штатної лінії GPIO2 і спільного debug/backpack. Зафіксувати контролер UART, 8N1, інверсію, baud та поведінку під час boot. |
| P1.6–P1.7 | Зберегти шлях UART bootloader через GPIO0 і Wi-Fi recovery. RX має `button = 0`; TX application button у layout не задано. Фізичний доступ до BOOT/EN, DIP і працездатність відновлення перевірити на платах. |
| P5.3–P5.4 | FSK на TX не може вимагати DIO1 без підтвердження підключення; у поточному HAL ISR встановлено лише для DIO0. |
| P8.1–P8.2 | Зберегти сім кодів DAC TX; штатний RX не реалізує цільові 100 мВт. Потрібні окремі перевірки регістрів і вимірювання. |
| P6.6, P8.6 | Ці 915-МГц профілі не підтверджують працездатність RF-тракту або калібрування PA у 960–1020 МГц. |

P1.1 не змінює пакет, PHY, FHSS або таймер, тому новий розрахунок ToA для нього не потрібен. Результати контрольних збірок і використання пам'яті записуються в [журнал плану](../IMPLEMENTATION_PLAN.md#журнал-виконання).
