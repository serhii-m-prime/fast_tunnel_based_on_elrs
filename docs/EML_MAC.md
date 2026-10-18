# EML MAC: LoRa PHY, симетричний TDD і інтеграція мосту (P4.2–P4.6)

MAC зв'язує транспорт P3 ([EML_TRANSPORT.md](EML_TRANSPORT.md)) з радіо SX1276, апаратним таймером і FHSS. Чиста логіка (профілі, ToA, бюджет слота, годинник слотів/hop, захоплення, ID пари) — у [`eml_mac.h`](../src/lib/OTA/eml_mac.h), прошивкова частина — у [`rxtx_eml_bridge.cpp`](../src/src/rxtx_eml_bridge.cpp) з API [`eml_bridge.h`](../src/include/eml_bridge.h). MAC вмикається прапорцем `EML_BRIDGE`, який мають лише `HappyModel_ES900TX_Bridge` і `HappyModel_ES900RX_Bridge`. У цих збірках штатний RF-канал ELRS (RC/telemetry, ELRS sync, PFD, rate cycling, bind over the air) не працює; Wi-Fi recovery, кнопка, LED, конфігурація, керування потужністю та вентилятором лишаються.

ES900TX — coordinator, ES900RX — responder (ролі §6 [протоколу](EML_PROTOCOL.md)).

## 1. PHY-профіль (P4.2)

| Параметр | Значення |
| --- | --- |
| Модуляція | LoRa, BW 500 кГц, SF6, CR 4/5 |
| Заголовок | implicit (обов'язковий для SF6), фізична довжина фіксована: N = 64 (типово) або 32 (`-DEML_FRAME_SIZE=32` на **обох** модулях) |
| CRC | апаратний CRC вимкнено (`SetCRCMode(false)` у гілці SF6); цілісність — EML CRC16 |
| Preamble | 8 символів |
| Sync word / IQ | штатні ELRS: sync word за замовчуванням, InvertIQ за `UID[5] & 1` (драйвер SX127x його не застосовує) |
| Errata | §2.1 BW500 (`0x36=0x02`, `0x3A=0x64` високий діапазон / `0x7F` низький) та §2.3 (`DetectOptimize` bit7) — наявний код `SetBandwidthCodingRate()`; SF6 detection optimize/threshold — `SetSpreadingFactor()` |

`emlBridgeBegin()` викликає `Radio.Config(BW500, SF6, CR4/5, sync channel, 8, IQ, N)`. Порядок у драйвері (SF до BW/CR) гарантує, що implicit header і errata застосовані. Після конфігурації перевіряється `Radio.PayloadLength == N` (P4.1 відхиляє довжину понад буфер), інакше стан `radioFailed`.

## 2. Time-on-air і бюджет слота (P4.3)

ToA за формулою Semtech (SX1276 datasheet §4.1.1.7): `Tsym = 2^SF/BW`, `Tpreamble = (Npre + 4.25)·Tsym`, `Npayload = 8 + max(ceil((8·PL − 4·SF + 28 + 16·CRC − 20·IH) / (4·(SF − 2·DE)))·(CR + 4), 0)`. `emlLoraToaUs()` обчислює це цілочисельно (`constexpr`); для штатних профілів ELRS результат збігається з таблицею `ExpressLRS_AirRateRFperf` з точністю до округлення 10 мкс (4384/6688/8768 проти 4380/6690/8770).

| N | Payload-символів | ToA, мкс |
| ---: | ---: | ---: |
| 32 | 58 | 8 992 |
| 64 | 113 | 16 032 |

ToA для кожної довжини 1–64 перевіряється в `test_eml_mac`.

Складові слота (мкс):

| Складова | Значення | Що входить |
| --- | ---: | --- |
| `EML_TX_START_US` | 250 | timer ISR, pop TX-дескриптора, запис частоти, FIFO burst 64 B (~53 мкс при SPI 10 МГц, P4.1), FSTX/PLL + PA ramp SX1276 |
| ToA | 16 032 / 8 992 | §1 |
| `EML_RX_DONE_US` | 200 | затримка демодулятора (~1 символ), DIO0 ISR, читання IRQ flags і FIFO (~53 мкс) |
| `EML_PROC_US` | 600 | foreground `receive()` + `prepare()` + push перед наступною власною межею |
| `EML_GUARD_US` | 300 | залишкова похибка вирівнювання responder, джитер таймера/ISR |
| **Мінімум** | 17 382 / 10 342 | `emlMinSlotUs()` |
| **Слот** | **17 500 / 10 400** | `static_assert(emlProfileValid())` |

Пара слотів — 35,0 мс (N=64) або 20,8 мс (N=32); корисне навантаження на напрямок — 60 B за пару ≈ 1714 B/s (N=64) або 28 B ≈ 1346 B/s (N=32), без урахування втрат і повторів. Для порівняння, штатний AirPort 200 Гц із TLM 1:2 переносить 5 B за 10 мс на напрямок (~500 B/s). Ці числа розрахункові; виміряна швидкість — P4.7.

Затримки `EML_TX_START_US` і `EML_RX_DONE_US` — оцінки з даташиту й кількості SPI-транзакцій, а не виміри. Від них залежить фазова прив'язка responder (§5), тому їх потрібно виміряти в P4.7. Допустиме зміщення межі responder відносно coordinator (δ, мінус — раніше) для N=64: δ > −1,4 мс (coordinator ще в RX після свого TX done о ~16,3 мс) і δ < ~+0,5 мс (responder має увімкнути RX у перші ~3 з 8 символів preamble). Ціль — δ = −`EML_RESPONDER_BIAS_US` = −100 мкс.

## 3. Симетричний TDD і FHSS (P4.4)

Розклад не має зв'язку з telemetry ratio ELRS: немає RC- чи telemetry-кадрів, кожен напрямок отримує рівно один слот у парі.

| Слот | Передає | Приймає | Hop |
| --- | --- | --- | --- |
| парний 2k | coordinator | responder | h(k) |
| непарний 2k+1 | responder | coordinator | h(k) |

`EmlSlotClock_s` у timer ISR: `slot` збільшується на кожній межі, `hop` — на кожному парному слоті modulo довжини послідовності, **незалежно від приймання** (§6.2 протоколу). Coordinator після власного TX done одразу вмикає RX на тому самому hop; responder на парній межі ставить частоту нового hop і RX, на непарній передає.

Частоти — штатна ELRS-послідовність `FHSSsequence`, згенерована з UID на обох модулях; `hop_count = FHSSgetSequenceCount()` (FCC915: 40 каналів, 240 елементів). Частота hop: `freq_start + freq_spread·FHSSsequence[h]/256 − FreqCorrection`; `freq_start/freq_spread` копіюються в DRAM при старті, щоб ISR не читав const-таблицю доменів. FreqCorrection не підлаштовується (±20 ppm кварцу ≈ ±18 кГц при BW500 у межах допуску LoRa).

Транспорт отримує монотонний час `now = emlNextOwnSlot(cur)` — наступний власний слот після поточного. Ним користуються `prepare/receive/tick`, тому таймаути й вік дескрипторів (`max_frame_age_slots = 3`) рахуються в слотах.

## 4. Таймінги сесії

| Параметр | мс | N=64, слотів | N=32, слотів |
| --- | ---: | ---: | ---: |
| `retry_slots` | — | 2 (Sync у кожному власному слоті) | 2 |
| `max_frame_age_slots` | — | 3 | 3 |
| `quiet_slots` | — | 5 | 5 |
| `handshake_timeout_slots` | 700 | 40 | 68 |
| `link_timeout_slots` | 1500 | 86 | 145 |
| Повторний пошук responder | link + 4 слоти | 90 | 149 |

## 5. Rendezvous, захоплення й утримання (P4.5)

**Пошук.** Responder без прив'язки зупиняє таймер і слухає sync channel ELRS (`FHSSgetInitialFreq()`). Цей канал стоїть на початку кожного блоку з 40 hop, тож coordinator, який передає HELLO в кожному власному слоті, буває на ньому раз на 80 слотів (1,4 с для N=64, 0,83 с для N=32). Дескриптори пошуку мають прапорець `EML_DESC_SEARCH`. Захоплення приймає лише валідний HELLO з очікуваними peer/receiver/configuration ID, `hop_index` у межах таблиці й `FHSSsequence[hop] == sync_channel`.

**Захоплення.** `emlAcquire()` відновлює розширений слот HELLO лише вперед (16-бітний лічильник, §7.1), оцінює межу слота coordinator як `rx_us − (TX_START + ToA + RX_DONE)` і ставить першу локальну межу на `EML_RESPONDER_BIAS_US` раніше, щонайменше за `EML_LOCK_LEAD_US` (600 мкс) від поточного часу. Дескриптор, оброблений більш ніж на `max_frame_age` слотів пізніше, відкидається. Далі HELLO передається в `transport.receive()` (стан OFFER), OFFER готується для першого власного слота, foreground чекає до межі (ESP8266 `hwTimer::resume()` спрацьовує через 20 мкс) і запускає таймер. Якщо межу пропущено, responder повертається в пошук.

**Фазове підстроювання.** Для кожного CRC-валідного кадра coordinator у парному слоті `emlPhaseCorrection = (rx_us − slot_start_us) − (RX offset + bias)`. Половина корекції (обмежена ±слот/4) іде в `hwTimer::phaseShift()` і застосовується після наступної межі. Частотний offset таймера не змінюється: дрейф ±40 ppm дає ≤0,7 мкс на слот.

**Keepalive.** Транспорт формує KEEPALIVE у кожному власному слоті, де немає DATA/Sync, тож обидва напрямки щопари несуть ACK/flow control і фазову опору.

**Втрати.** Обидві сторони продовжують рахувати слоти й hop без приймання. Coordinator після `link_timeout` знову надсилає HELLO. Responder, який `link_timeout + 4` слотів не чув CRC-валідного кадра coordinator, скидає транспорт і повертається в пошук. Коротка пауза (< link timeout) зберігає сесію й ARQ. Після reboot responder coordinator відпускає стару сесію через link timeout; гірший час відновлення ≈ 1,5 с + 1,4 с + handshake (N=64).

## 6. ISR/foreground-контракт

| Подія | Контекст | Дія |
| --- | --- | --- |
| Межа власного слота | timer ISR (IRAM) | забрати з TX mailbox найновіший дескриптор саме цього слота **і** поточної generation, частота hop, `TXnb`, ticket у `sentRing`; без кадра coordinator вмикає RX |
| Межа слота peer | timer ISR | responder: частота нового hop, RX, arm контексту `{generation, slot, hop, start_us}`; coordinator уточнює `start_us` |
| TX done | DIO ISR | coordinator: RX на тому ж hop, arm наступного слота |
| RX done | DIO ISR | один кадр на armed слот (пошук приймає всі), копія ≤64 B у RX mailbox із контекстом, `rx_us`, `slot_start_us` |
| `emlBridgeLoop()` | foreground | 1) усі `sent()` з `sentRing`; 2) RX mailbox → `receive()` / захоплення / фаза; 3) `tick()`, нагляд за lock; 4) `prepare()` наступного власного слота; 5) `connectionState` |

- Generation стампується **при arm** (`macGeneration` = `transport.generation()` після кожного виклику транспорту); ISR не передає кадри з іншою generation або для іншого слота.
- Номер слота публікується через seqlock **після** TX-звіту, тому foreground не побачить новий слот без його `sent()`.
- Кадр готується двічі: «рано» (одразу після власної межі, зі старим ACK — на випадок утраченого кадра peer) і «пізно» (після обробки кадра peer попереднього слота). Готування й push виконуються, лише якщо до межі лишилося більше `EML_PREP_MARGIN_US` (400 мкс), тож ISR не зустрічає напівготового стану. Якщо ISR усе ж передав ранній кадр, якого transport уже не визнає (`sent()` → false), ACK для нього ігнорується й дані повторюються — безпечна деградація, яку покриває симуляція.
- Під час `config.Commit()` (`emlBridgeSuspend`) ISR лише рахує слоти, без SPI.
- Зміна generation (reset, link timeout, нова сесія) ізолює старі кадри без окремого очищення радіо: ISR не передає TX-дескриптори іншої generation, кожен RX-слот переозброюється з поточною generation, а RX-дескриптори старої generation transport відкидає як STALE. FIFO SX1276 перезаписується кожним кадром, інших відкладених callback немає. Під час переходу responder у пошук таймер зупиняється, а частота й RX змінюються в критичній секції.

**Cache-off.** ESP8285: усі ISR моста, SX127x HAL/драйвер, SPIEx, RFAMP і `hardware_pin/flag/int` (тепер `ICACHE_RAM_ATTR`) лежать в IRAM; `micros()` в IRAM, `memcpy` у ROM. ESP32: Arduino-ESP32 зібрано без `CONFIG_ARDUINO_ISR_IRAM`, тож timer і GPIO ISR реєструються без `ESP_INTR_FLAG_IRAM` і маскуються на час cache-off, як і в штатному ELRS; виклики flash (`micros`, `spiEnableSSPins`) у них допустимі. Замасковані межі coordinator надолужує: таймер зберігає сітку, ISR додає пропущені слоти за `micros()` і пропускає радіо на межі, обслуженій пізніше за guard.

## 7. UART і ARQ (P4.6)

- Кільця `EmlTransport<AP_MAX_BUF_LEN>` (4096 B на напрямок на ESP32 TX, 1024 B на ESP8285 RX) **замінюють** legacy AirPort FIFO: у `EML_BRIDGE` `apInputBuffer/apOutputBuffer` у `tx_main.cpp` і статичні FIFO `SerialAirPort` не компілюються.
- TX: `HandleUARTin/out` → `emlBridgeReadSerial/DrainSerial(*TxUSB)`; RX: `SerialAirPort::processSerialInput/sendQueuedData` → ті самі функції з UART0 і `maxBytesToSend`.
- Дані приймаються лише в ACTIVE; поза сесією не більше 64 B за loop відкидаються з лічильником (P3.7). Повна черга лишає байти в драйвері UART.
- `connectionState` = `connected`, коли транспорт ACTIVE, інакше `disconnected`; перехід у `connected` ставить `webserverPreventAutoStart`, тож Wi-Fi recovery автоматично стартує лише без робочого зв'язку.

## 8. Ідентичність пари

`emlDeriveLinkIds()` хешує (FNV-1a) binding UID у coordinator/responder ID і повний запис конфігурації в `configuration_id`: версія запису, PHY, N, SF/CR/preamble/BW, слот, параметри FHSS-домену, seed послідовності, `hop_count`, таймінги сесії й константи затримок. Будь-яка розбіжність збірок дає інший ID, і handshake не завершується. Свіжий epoch seed: `esp_random()` (ESP32) / `ESP.random()` (ESP8266), змішаний з `micros()`; на ESP8266 при вимкненому Wi-Fi джерело RNG не гарантує криптографічної ентропії (обмеження §7.3 протоколу).

## 9. Обмеження

- Bind over the air не підтримується: пара визначається binding phrase (`EnterBindingMode` у `EML_BRIDGE` — no-op).
- Dynamic Power не працює (потрібні EML-метрики, P8.3–P8.4): TX використовує фіксовану потужність із конфігурації, вентилятор вмикається за порогом від неї (THERMAL device).
- Rate/TLM-налаштування ELRS не впливають на EML. N задається під час збірки.
- Затримки §2 оцінені, а не виміряні; PER, latency, throughput, тривала стабільність і фазова прив'язка на реальних SX1276 — P4.7.

## 10. Перевірки

`test_eml_mac` (native): ToA проти Semtech/ELRS і для всіх довжин, бюджет слота обох профілів, годинник/hop/32-бітний wrap, монотонний час транспорту, конфігурація з профілю, ID пари, захоплення (зокрема пізня обробка й 16/32-бітний wrap), `EmlTicketRing`. Симуляція пари з реальною FHSS-послідовністю ELRS (40 каналів, 240 hop) і транспортом P3 перевіряє rendezvous ≤ 80 слотів + handshake, двобічні потоки при 15% втрат кадрів із пізнім/раннім готуванням та відхиленим `sent()`, reboot responder, blackout понад link timeout із повторним пошуком, коротку паузу без нової сесії й wrap 16-бітного Sync-лічильника. Модель не відтворює ISR-таймінги, SPI чи радіоканал — це P4.7.

```bash
export PLATFORMIO_CORE_DIR="$PWD/.venv/platformio"
.venv/bin/pio test -d src -e native
export PLATFORMIO_BUILD_DIR="$PWD/.venv/build-p4"
.venv/bin/pio run -d src -e HappyModel_ES900TX_Bridge -j 4
.venv/bin/pio run -d src -e HappyModel_ES900RX_Bridge -j 4
```
