# Аудит залежностей і очищення Bridge-збірок (P6.0)

## Межі зміни

Аудит виконано для `HappyModel_ES900TX_Bridge` (ESP32) та
`HappyModel_ES900RX_Bridge` (ESP8285), вихідна ревізія `cfe3d754`.
Виключення діють лише з `EML_BRIDGE`. Штатні ELRS-таргети зберігають свої
точки входу, бібліотеки й формати конфігурації. Файли Lua на SD-карті передавача
не є інтерпретатором у MCU: тут вилучається саме firmware-обслуговування Lua/CRSF
параметрів (`devTXLUA` / `devRXLUA`).

Перед змінами перевірено реєстрацію пристроїв, глобальні конструктори, запуск,
основні цикли, Wi-Fi routes та залежності бібліотек. Перевірка лише runtime
`firmwareOptions.is_airport` залишала RC-об'єкти у збірці: PlatformIO LDF бачить
заголовки навіть у неактивних preprocessor-гілках, а конструктори й масиви
пристроїв утримують код незалежно від того, чи використовується протокол.

## Матриця залежностей

| Компонент | Що утримувало його до очищення | Дія для Bridge |
| --- | --- | --- |
| TX/RX Lua-параметри | `ui_devices` у `tx_main.cpp` / `rx_main.cpp`, CRSF endpoints і router | Штатні main виключено з source filter; `tx-crsf` / `rx-crsf` і `CrsfProtocol` ігноруються LDF |
| RC-канали, switch modes, legacy OTA CRC | Глобальний `ChannelData[16]`, serializer callbacks і пакування в `OTA.cpp` | RC-реалізацію виключено через `!EML_BRIDGE`; збережено UID, перевірку bound UID та незмінний FHSS seed |
| CRSF handset, MSP/MAVLink та stubborn telemetry | Глобальні router/endpoint/parser/MSP/ARQ-об'єкти в main, RX `SerialIO` та протокольні wrapper-и | Власна точка входу не створює ці об'єкти; `tx-*`, `rx-*`, serial wrappers і `dynpower.cpp` не компілюються |
| VTX administrator і VTX SPI/MSP | Реєстрація VTX devices, Lua callbacks, RX serial protocols | Вилучено devices та бібліотеки VTX/MSPVTX/VTXSPI; значення persisted enums/schema збережено |
| BLE joystick | `BLE_device`, NimBLE і ESP32-BLE-Gamepad в успадкованих `lib_deps` | Device відсутній; Bridge має власний список зовнішніх залежностей без BLE |
| Backpack | `Backpack_device`, MSP-парсер і перевірка оновлення в TX loop | Не реєструється й не компілюється; `BackpackOrLogStrm` залишається назвою спільного null-логера, без Backpack-обробки |
| Wi-Fi joystick | `devWIFI.cpp` → `wifiJoystick.cpp` → handset і ChannelData; `/udpcontrol`, UDP advertisement | Виключено для Bridge, включно з UDP/mDNS сервісом джойстика |
| MSP через Wi-Fi | RX `devWIFI.cpp` → `TcpMspConnector` → CRSF2MSP → router | Виключено TCP 5761 і WebSocket `/msp`; HTTP recovery не залежить від цього сервісу |
| ADC, servo/PWM, barometer, GPS, екран і motion | Пристрої та serial wrappers у штатних main, `/gps` і `/voltage-sample` | Відсутні в Bridge; GPS/voltage capabilities у JSON вимкнено; графічних залежностей у `lib_deps` немає |

Заголовки з persisted структурами й номерами enum залишаються спільними.
Зокрема, `TxConfig`/`RxConfig` та HTTP import/export поки зберігають RC/VTX поля
для сумісності даних NVS/EEPROM; вони не запускають вилучені протоколи. Переробка
WebUI та його стиснених assets належить P8.7.

## Збережені залежності й поведінка

- [`bridge_main.cpp`](../src/src/bridge_main.cpp) запускає options/hardware,
  рольову конфігурацію, UART, пристрої, FHSS і радіо. Залишені devices:
  LED/RGB, Wi-Fi, кнопка та THERMAL на ESP32. Немає глобальних RC-конструкторів.
- UART0 працює в 8N1 без інверсії, ESP32 GPIO3/GPIO1, ESP8285 штатні RX/TX.
  `normalizeBridgeBaud()` і початкові 460800 бод збережено. SDK debug-вивід
  вимкнено; спільний логер лишається `NullStream`. Boot-ROM вивід до `setup()`
  залишається апаратним обмеженням. Стартове очищення UART обмежене кількістю
  вже буферизованих байтів, тому безперервний вхід не затримує старт recovery.
- UART напряму викликає `emlBridgeReadSerial()` / `emlBridgeDrainSerial()`:
  ті самі обмежені порції по 64 байти та транспортні черги 4096/1024 B на
  напрямок. RC, MSP, MAVLink чи XON/XOFF не розбираються й не домішуються.
- [`OPTIONS`](../src/lib/OPTIONS/options.cpp), LittleFS, hardware layout,
  NVS/EEPROM та формат options.json збережені. У Bridge `is-airport` примусово
  увімкнено навіть після завантаження старого options.json із `false`: вилучений
  RC-стек неможливо активувати параметром у файлі. Зміна UART baud потребує reboot,
  як і раніше в AirPort.
- `devWIFI` зберігає Wi-Fi auto-start, AP/STA, captive portal, `/update`,
  `/firmware.bin`, `/hardware.json`, `/options.json`, `/config`, `/reboot`, `/reset`.
  Відсутній layout запускає лише Wi-Fi; помилка `Radio.Begin()` дає `radioFailed`
  та Wi-Fi recovery. GPIO0 залишається входом для UART bootloader, якщо немає
  апаратної кнопки. RX `RF_PRE_INIT` зберігає попередню політику калібрування Wi-Fi.
- [`rxtx_eml_bridge.cpp`](../src/src/rxtx_eml_bridge.cpp), wire codec, Sync,
  ARQ, PHY, FHSS, RFAMP, SPI та апаратний таймер не змінено. EML ISR не викликає
  жоден із вилучених модулів. Розмір кадра, ToA та інтервали залишаються попередніми.
- `POWERMGNT` зберігає калібрування, APC/DAC і обмеження потужності, THERMAL —
  GPIO вентилятора й поріг із конфігурації (типово 250 мВт). Прибрано лише запис
  потужності у CRSF link statistics. На RX `PWR_MATCH_TX` дає hardware default,
  як під час попереднього старту без CRSF-телеметрії. EML Dynamic Power — P8.4.
  RX power-on counter для входу в повітряний bind більше не оновлюється:
  EML і до очищення не підтримував цей bind; фраза задається через options/config.

## Відтворювані перевірки

Повний native suite перевіряє codec, ARQ, дублікати/втрати, бінарні потоки,
backpressure, reconnect, MAC, PHY-профілі, ToA та FIFO/SPI. Для перевірки результату
лінкування є [`audit_bridge_build.py`](../src/python/test_tools/audit_bridge_build.py):
він відхиляє RC/Lua/BLE/MSP/VTX/Backpack символи й бібліотеки, перевіряє вибір main,
присутність EML ISR/LittleFS/recovery routes, вбудовані defaults та бюджет IRAM.
На ESP8285 додатково перевіряються адреси EML ISR в IRAM. Це не повний доказ усіх
можливих ISR call graph і не вимірювання поведінки фізичної плати.

```bash
export PLATFORMIO_CORE_DIR="$PWD/.venv/platformio"
.venv/bin/pio test --project-dir src -e native

# Збирати послідовно: copy_html генерує спільний include/WebContent.h.
export PLATFORMIO_BUILD_DIR="$PWD/.venv/build-p6.0-final-tx"
.venv/bin/pio run --project-dir src -e HappyModel_ES900TX_Bridge -j 4
.venv/bin/python src/python/test_tools/audit_bridge_build.py \
  "$PLATFORMIO_BUILD_DIR/HappyModel_ES900TX_Bridge" --role tx \
  --core "$PLATFORMIO_CORE_DIR" --report /tmp/eml-tx-audit.json

export PLATFORMIO_BUILD_DIR="$PWD/.venv/build-p6.0-final-rx"
.venv/bin/pio run --project-dir src -e HappyModel_ES900RX_Bridge -j 4
.venv/bin/python src/python/test_tools/audit_bridge_build.py \
  "$PLATFORMIO_BUILD_DIR/HappyModel_ES900RX_Bridge" --role rx \
  --core "$PLATFORMIO_CORE_DIR" --report /tmp/eml-rx-audit.json
```

Числові результати, порівняння з контрольною збіркою та шляхи локальних
артефактів наведено в журналі P6.0 у [плані](../IMPLEMENTATION_PLAN.md).
Апаратні UART/RF/IRQ/thermal/recovery випробування залишаються P1.7/P4.7/P5.6;
компіляція й native-модель не підтверджують їх проходження.
