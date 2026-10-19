# EML MAC: PHY, TDD і інтеграція мосту (P4–P7)

MAC зв'язує транспорт P3 ([EML_TRANSPORT.md](EML_TRANSPORT.md)) з радіо SX1276, апаратним таймером і FHSS. Чиста логіка (профілі, ToA, бюджет слота, годинник слотів/hop, захоплення, ID пари) — у [`eml_mac.h`](../src/lib/OTA/eml_mac.h), прошивкова частина — у [`rxtx_eml_bridge.cpp`](../src/src/rxtx_eml_bridge.cpp) з API [`eml_bridge.h`](../src/include/eml_bridge.h). MAC вмикається прапорцем `EML_BRIDGE`, який мають лише `HappyModel_ES900TX_Bridge` і `HappyModel_ES900RX_Bridge`. У цих збірках штатний RF-канал ELRS (RC/telemetry, ELRS sync, PFD, rate cycling, bind over the air) не працює; Wi-Fi recovery, кнопка, LED, конфігурація, керування потужністю та вентилятором лишаються.

ES900TX — coordinator, ES900RX — responder (ролі §6 [протоколу](EML_PROTOCOL.md)). Поточні бенди, startup PLL, формат керування та аварійний пошук описано в [EML_MULTIBAND.md](EML_MULTIBAND.md).

P7 додає [адаптивний TDD та ARQ-вікно](EML_ADAPTIVE_TDD.md): типово W=4, 1:1/3:1/1:3 із lease, детерміновані короткі FSK ACK. Таблиця симетричного TDD нижче описує базовий режим; тривалість слотів і FHSS-крок не змінюються. Для default adaptive `max_frame_age_slots=7`, `quiet_slots=9`; повні часові бюджети й фізичні обмеження — у новому описі.

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
| `EML_TURNAROUND_US` | 500 | межа місткості P6 замість номінальних 250 мкс TX start; фазова оцінка зберігає номінал |
| **Мінімум** | 17 632 / 10 592 | `emlMinSlotUs()` |
| **Слот** | **17 800 / 10 600** | `static_assert(emlProfileValid())`, строго більше мінімуму |

Пара слотів — 35,6 мс (N=64) або 21,2 мс (N=32); корисне навантаження на напрямок — 60 B за пару ≈ 1685 B/s (N=64) або 28 B ≈ 1320 B/s (N=32), без урахування втрат і повторів. Для порівняння, штатний AirPort 200 Гц із TLM 1:2 переносить 5 B за 10 мс на напрямок (~500 B/s). Ці числа розрахункові; виміряна швидкість — P4.7.

Затримки `EML_TX_START_US` і `EML_RX_DONE_US` — оцінки з даташиту й кількості SPI-транзакцій, а не виміри. Від них залежить фазова прив'язка responder (§5), тому їх потрібно виміряти в P4.7. Допустиме зміщення межі responder відносно coordinator (δ, мінус — раніше) для N=64: δ > −1,4 мс (coordinator ще в RX після свого TX done о ~16,3 мс) і δ < ~+0,5 мс (responder має увімкнути RX у перші ~3 з 8 символів preamble). Ціль — δ = −`EML_RESPONDER_BIAS_US` = −100 мкс.

## 3. Симетричний TDD і FHSS (P4.4)

Розклад не має зв'язку з telemetry ratio ELRS: немає RC- чи telemetry-кадрів, кожен напрямок отримує рівно один слот у парі.

| Слот | Передає | Приймає | Hop |
| --- | --- | --- | --- |
| парний 2k | coordinator | responder | h(k) |
| непарний 2k+1 | responder | coordinator | h(k) |

`EmlSlotClock_s` у timer ISR: `slot` збільшується на кожній межі, `hop` — на кожному парному слоті modulo довжини послідовності, **незалежно від приймання** (§6.2 протоколу). Coordinator після власного TX done одразу вмикає RX на тому самому hop; responder на парній межі ставить частоту нового hop і RX, на непарній передає.

Частоти Bridge — сітка EML4B (P6): 60 каналів на бенд, одна `FHSSsequence` із 240 елементів, згенерована з UID на обох модулях; `hop_count = FHSSgetSequenceCount()`. Штатні домени ELRS поза Bridge збережено. Частота hop: `freq_start + freq_spread·FHSSsequence[h]/256 − FreqCorrection`; `freq_start/freq_spread` копіюються в DRAM при старті, щоб ISR не читав const-таблицю доменів. FreqCorrection не підлаштовується (±20 ppm кварцу ≈ ±18 кГц при BW500 у межах допуску LoRa).

Транспорт отримує монотонний час `now = emlNextOwnSlot(cur)` — наступний власний слот після поточного. Ним користуються `prepare/receive/tick`, тому таймаути й вік дескрипторів (`max_frame_age_slots = 3`) рахуються в слотах.

## 4. Таймінги сесії

| Параметр | мс | N=64, слотів | N=32, слотів |
| --- | ---: | ---: | ---: |
| `retry_slots` | — | 2 (Sync у кожному власному слоті) | 2 |
| `max_frame_age_slots` | — | 3 | 3 |
| `quiet_slots` | — | 5 | 5 |
| `handshake_timeout_slots` | 700 | 40 | 67 |
| `link_timeout_slots` | 1500 | 85 | 142 |
| Повторний пошук responder без ACTIVE | link + 4 слоти | 89 | 146 |

Для P7 adaptive замість 3/5 використовуються 7/9 слотів age/quiet: наступний власний слот може бути на 4 слоти попереду. Handshake/link timeout не змінено.

## 5. Rendezvous, захоплення й утримання (P4.5)

**Пошук.** Responder без прив'язки зупиняє таймер і слухає sync channel ELRS (`FHSSgetInitialFreq()`). Цей канал стоїть на початку кожного блоку з 60 hop, тож coordinator, який передає HELLO в кожному власному слоті, буває на ньому раз на 120 слотів (2,136 с для LoRa N=64, 1,272 с для N=32). Бенд та PHY обирає карусель P6. Дескриптори пошуку мають прапорець `EML_DESC_SEARCH`. Захоплення приймає лише валідний HELLO з очікуваними peer/receiver/configuration ID, `hop_index` у межах таблиці й `FHSSsequence[hop] == sync_channel`.

**Захоплення.** `emlAcquire()` відновлює розширений слот HELLO лише вперед (16-бітний лічильник, §7.1), оцінює межу слота coordinator як `rx_us − (TX_START + ToA + RX_DONE)` і ставить першу локальну межу на `EML_RESPONDER_BIAS_US` раніше, щонайменше за `EML_LOCK_LEAD_US` (600 мкс) від поточного часу. Дескриптор, оброблений більш ніж на `max_frame_age` слотів пізніше, відкидається. Далі HELLO передається в `transport.receive()` (стан OFFER), OFFER готується для першого власного слота, foreground чекає до межі (ESP8266 `hwTimer::resume()` спрацьовує через 20 мкс) і запускає таймер. Якщо межу пропущено, responder повертається в пошук.

**Фазове підстроювання.** Для кожного session/configuration-valid кадра coordinator у парному слоті `emlPhaseCorrection = (rx_us − slot_start_us) − (RX offset + bias)`. Половина корекції (обмежена ±слот/4) іде в `hwTimer::phaseShift()` і застосовується після наступної межі. Частотний offset таймера не змінюється: дрейф ±40 ppm дає ≤0,7 мкс на слот.

**Keepalive.** Транспорт формує KEEPALIVE у кожному власному слоті, де немає DATA/Sync, тож обидва напрямки щопари несуть ACK/flow control і фазову опору.

**Втрати.** Обидві сторони продовжують рахувати слоти й hop без приймання. Після P7 adaptive поріг emergency для LoRa64 — 231 мс (щонайменше три peer-можливості), для інших робочих профілів — 150 мс. Коротші паузи зберігають ARQ. Під час незавершеного handshake залишаються transport timeout та `link_timeout + 4` для lock responder. Межі втрати RAM-даних при reconnect і тривалість пошуку — [EML_MULTIBAND.md](EML_MULTIBAND.md).

## 6. ISR/foreground-контракт

| Подія | Контекст | Дія |
| --- | --- | --- |
| Межа власного слота | timer ISR (IRAM) | забрати з TX mailbox найновіший дескриптор саме цього слота **і** поточної generation, частота hop, `TXnb`; ticket у `sentRing` лише після успішного старту TX; без кадра coordinator вмикає RX |
| Межа слота peer | timer ISR | обидві ролі: частота hop, відома фізична довжина, RX, arm `{generation, slot, hop, start_us}` або уточнення вже озброєного прийому |
| TX done | DIO ISR | обидві ролі: якщо наступний слот належить peer, завчасно поставити його hop/довжину, RX та arm; перед наступним власним слотом лишитися в Standby |
| RX done | DIO ISR | кандидати лише в часових межах armed слота (пошук без часової прив'язки), копія ≤64 B у bounded RX mailbox із контекстом, `rx_us`, `slot_start_us`; без CRC в ISR |
| `emlBridgeLoop()` | foreground | 1) усі `sent()` з `sentRing`; 2) RX mailbox → `receive()` / захоплення / фаза; 3) `tick()`, нагляд за lock; 4) `prepare()` наступного власного слота; 5) `connectionState` |

- Generation стампується **при arm** (`macGeneration` = `transport.generation()` після кожного виклику транспорту); ISR не передає кадри з іншою generation або для іншого слота.
- Ранній RX після TX done не приймає кадри, що завершилися до початку наступного слота; кінець вікна — його наступна межа. Віднімання `micros()` підтримує wrap. Лише успішна перевірка CRC, сесії та freshness у foreground закриває вікно (`taken`): битий або чужий кадр не забирає можливість прийняти валідний. Закриття виконується в критичній секції, лише якщо generation/slot/hop дескриптора досі збігаються з armed-контекстом.
- Номер слота публікується через seqlock **після** TX-звіту, тому foreground не побачить новий слот без його `sent()`.
- Кадр готується двічі: «рано» (одразу після власної межі, зі старим ACK — на випадок утраченого кадра peer) і «пізно» (після обробки кадра peer попереднього слота). Готування й push виконуються, лише якщо до межі лишилося більше `EML_PREP_MARGIN_US` (400 мкс), тож ISR не зустрічає напівготового стану. Якщо ISR усе ж передав ранній кадр, якого transport уже не визнає (`sent()` → false), ACK для нього ігнорується й дані повторюються — безпечна деградація, яку покриває симуляція.
- Під час `config.Commit()` (`emlBridgeSuspend`) ISR лише рахує слоти, без SPI.
- Зміна generation (reset, link timeout, нова сесія) ізолює старі кадри без окремого очищення радіо: ISR не передає TX-дескриптори іншої generation, кожен RX-слот переозброюється з поточною generation, а RX-дескриптори старої generation transport відкидає як STALE. FIFO SX1276 перезаписується кожним кадром, інших відкладених callback немає. Під час переходу responder у пошук таймер зупиняється, а частота й RX змінюються в критичній секції.

**Cache-off.** ESP8285: усі ISR моста, SX127x HAL/драйвер, SPIEx, RFAMP і `hardware_pin/flag/int` (тепер `ICACHE_RAM_ATTR`) лежать в IRAM; `micros()` в IRAM, `memcpy` у ROM. ESP32: Arduino-ESP32 зібрано без `CONFIG_ARDUINO_ISR_IRAM`, тож timer і GPIO ISR реєструються без `ESP_INTR_FLAG_IRAM` і маскуються на час cache-off, як і в штатному ELRS; виклики flash (`micros`, `spiEnableSSPins`) у них допустимі. Замасковані межі coordinator надолужує: таймер зберігає сітку, ISR додає пропущені слоти за `micros()` і пропускає радіо на межі, обслуженій пізніше за guard.

## 7. UART і ARQ (P4.6)

- Кільця `EmlTransport<AP_MAX_BUF_LEN>` (4096 B на напрямок на ESP32 TX, 1024 B на ESP8285 RX) **замінюють** legacy AirPort FIFO: у `EML_BRIDGE` `apInputBuffer/apOutputBuffer` у `tx_main.cpp` і статичні FIFO `SerialAirPort` не компілюються.
- Після очищення P6.0 обидві Bridge-збірки використовують окремий [`bridge_main.cpp`](../src/src/bridge_main.cpp): його `loop()` напряму викликає `emlBridgeReadSerial/DrainSerial(Serial)` для UART0. Штатні `tx_main.cpp`, `rx_main.cpp`, `SerialAirPort` та RC-пристрої виключені зі збірки. Аудит залежностей і збереження recovery — [BRIDGE_DEPENDENCY_AUDIT.md](BRIDGE_DEPENDENCY_AUDIT.md).
- Дані приймаються лише в ACTIVE; поза сесією не більше 64 B за loop відкидаються з лічильником (P3.7). Повна черга лишає байти в драйвері UART.
- `connectionState` = `connected`, коли транспорт ACTIVE, інакше `disconnected`; перехід у `connected` ставить `webserverPreventAutoStart`, тож Wi-Fi recovery автоматично стартує лише без робочого зв'язку.

## 8. Ідентичність пари

`emlDeriveLinkIds()` хешує (FNV-1a) binding UID у coordinator/responder ID і повний запис конфігурації в `configuration_id`: версія запису, PHY, N, SF/CR/preamble/BW, слот, параметри FHSS-домену, seed послідовності, `hop_count`, таймінги сесії й константи затримок. Будь-яка розбіжність збірок дає інший ID, і handshake не завершується. Свіжий epoch seed: `esp_random()` (ESP32) / `ESP.random()` (ESP8266), змішаний з `micros()`; на ESP8266 при вимкненому Wi-Fi джерело RNG не гарантує криптографічної ентропії (обмеження §7.3 протоколу).

## 9. Обмеження

- Bind over the air не підтримується: пара визначається binding phrase. Після P6.0 Bridge не реєструє callback повітряного bind.
- Dynamic Power не працює (потрібні EML-метрики, P8.3–P8.4): TX використовує фіксовану потужність із конфігурації, вентилятор вмикається за порогом від неї (THERMAL device).
- Rate/TLM-налаштування ELRS не впливають на EML. N, `EML_PHY_MODE`, `EML_ARQ_WINDOW`, `EML_ADAPTIVE_TDD` та `EML_SHORT_ACK` задаються під час збірки.
- Затримки §2 оцінені, а не виміряні; PER, latency, throughput, тривала стабільність і фазова прив'язка на реальних SX1276 — P4.7.

## 10. FSK/GFSK і вибір PHY (P5.1–P5.5)

Підтримка FSK вмикається `SX127X_FSK_SUPPORT` (є в обох Bridge-оточеннях); без нього FSK-гілки не включаються. Регістрова арифметика — у [`SX127xFsk.h`](../src/lib/SX127xDriver/SX127xFsk.h), профілі й вибір PHY — у `eml_mac.h`.

### 10.1. Драйвер (P5.1, P5.3)

`SX127xDriver::ConfigFsk()` переводить модем через Sleep (біт LongRangeMode змінюється лише в Sleep), записує bitrate/frac, Fdev, shaping + PA ramp 40 мкс, RxBw = AfcBw, RxConfig (AFC і AGC на кожному виявленні preamble), preamble 10 B, детектор preamble 2 B, sync word 4 B з UID, fixed-length packet mode, whitening, DIO0, частоту sync channel і запускає калібрування Image/RSSI на робочому діапазоні (з POR воно виконане лише на 434 МГц; таймаут 20 мс). `Config()` повертає LoRa: модем знову через Sleep, кеш SF/BW/CR/preamble скидається, щоб усі LoRa-регістри (зокрема BW500 errata) записалися повторно, а `0x27/0x33/0x3B`, на які FSK-сторінка записала SyncConfig/NodeAdrs/ImageCal, повертаються до LoRa-значень за замовчуванням (PPM 0, InvertIQ 0x27, InvertIQ2 0x1D).

Адреси 0x0D–0x3F у FSK мають інший зміст, тож LoRa-шляхи драйвера в FSK не виконуються: TX без `FifoAddrPtr` (FIFO очищається записом FifoOverrun, заповнюється в Standby, `TxStartCondition = FifoNotEmpty`), окремий ISR, а `GetIrqFlags/ClearIrqFlags/SetPPMoffsetReg/GetFrequencyError` у FSK нічого не роблять.

Виправлення ревю 2026-10-19: для 250 кГц SSB `RegRxBw=RegAfcBw=0x01`.
Старші біти цих регістрів SX1276 зарезервовані; поля DCC від SX1231 сюди не
переносяться. Exponent=0 також зарезервований (Table 40). Початковий
`RegLna=0x23` задає G1 та HF boost перед запуском AGC; gain=000 не використовується.
Реальні записи packet engine винесено в `SX127xFskConfig.h` і перевірено native
HAL-моделлю проти окремого регістрового вектора.

При fast turnaround очікуються **обидва** `ModeReady` та `PllLock`; `0xFF` від SPI
не приймається як успіх. Дедлайн — 250 мкс, із підтримкою wrap `micros()`.
При таймауті драйвер повертає Standby, вимикає RFAMP і повертає `false` з
`TXnb/RXnb`: MAC не публікує TX ticket і не arm-ить невдалий RX. Дані залишаються
для повторної спроби. Ліміт очікування входить у збережений reserve turnaround
500 мкс; ToA, preamble та slot interval не змінені. Фактичні затримки й результат
FSK-парування залишаються стендовою перевіркою P5.6/P9.1.

| Подія DIO0 (RegDioMapping1 = 0) | Обробка |
| --- | --- |
| PacketSent | FSK лишається в TX: Standby, RFAMP off, `TXnbISR()` (commit потужності, TX done) |
| PayloadReady | RSSI (`RegRssiValue`, згладжування 32 вибірки) → `RXdataBuffer` ← N байтів FIFO → RX done; AutoRestartRx відкриває наступний прийом |
| FifoOverrun | лічильник `FskFifoOverruns`, запис FifoOverrun очищає FIFO, RestartRxWithoutPllLock |

Використовується лише DIO0: на ES900TX DIO1 не розведено (P1.1). Окремого RX timeout немає: MAC переозброює RX на кожній межі слота через Standby, що скидає незавершений прийом.

### 10.2. Профіль 250 кбіт/с (P5.2) і формат кадра (P5.4)

| Параметр | Значення | Обґрунтування |
| --- | --- | --- |
| Bitrate | 250 000 біт/с (reg 0x0080, frac 0) | ціль P5; FSK-межа SX1276 — 300 кбіт/с |
| Shaping | GFSK BT 1.0 | найменша міжсимвольна інтерференція; BT 0.5/0.3 звузили б спектр ціною ISI при h = 0,72 |
| Fdev | 90 кГц (reg 1475 = 90,03 кГц), h = 0,72 | h ≥ 0,5 (≥ 62,5 кГц); Fdev + BR/2 = 215 кГц |
| RxBw / AfcBw | 250 кГц SSB (максимум SX1276) | запас ±35 кГц на відхилення частоти кварців до AFC (±20 ppm на кожному модулі при 915 МГц ≈ ±37 кГц у гіршому випадку, тож межу треба перевірити на стенді) |
| Preamble | 10 B (320 мкс) | детектор 2 B + AFC/AGC + допуск запізнення responder 392 мкс |
| Sync | 4 B з UID без 0x00 | розділення пар; 0x00 заборонено датащитом |
| Пакет | fixed length N, без length/address, без апаратного CRC, whitening | рівно N байтів у 64-байтовому FIFO, цілісність — EML CRC16 |

Fdev 125 кГц (нижня межа побажання AGENTS) формально допустима (Fdev + BR/2 = 250 кГц), але не лишає запасу на частотне відхилення; 150 кГц і більше при 250 кбіт/с виходять за обмеження датащиту. `emlFskWithinDatasheet()` і `sx127xFskParamsValid()` відхиляють такі комбінації.

N = 64 займає FIFO повністю, тож обслуговувати FIFO під час пакета не потрібно; N = 32 — половину. Апаратний CRC додав би 2 B (64 мкс) без нових гарантій.

| N | ToA, мкс | Мінімум слота | Слот | На напрямок |
| ---: | ---: | ---: | ---: | ---: |
| 64 | 2 496 | 4 046 | 4 100 | ≈ 7 317 B/s |
| 32 | 1 472 | 3 022 | 3 100 | ≈ 4 516 B/s |

Бюджет місткості FSK після P6: turnaround 500 + ToA + `EML_FSK_RX_DONE_US` 150 (PayloadReady, ISR, FIFO) + обробка 600 + guard 300. Номінальний `EML_FSK_TX_START_US` 200 (частота, FIFO burst, ramp) лишається для фазової оцінки. Обидві затримки — оцінки для P5.6. Допуск фази responder для N=64: δ ∈ (−1,1; +0,39) мс.

### 10.3. Метрики й вибір PHY (P5.5)

- **RSSI:** LoRa — `PktRssiValue`, FSK — `RegRssiValue`, узятий одразу після PayloadReady (не латчиться, тому це оцінка кінця кадра). SNR у FSK немає.
- **LQ/PER:** `EmlLinkQuality` рахує кадри peer, які пройшли перевірки транспорту, за останні 100 очікуваних peer TX-можливостей (PER = 100 − LQ), незалежно від PHY. Це враховує квоту P7 1/4. `emlBridgeLinkStats()` повертає PHY, бенд, PLL-маску, TDD mode, W/in-flight, LQ, PER, середній RSSI і `FskFifoOverruns`.
- **Режим:** `EML_PHY_MODE` 0 — лише LoRa, 1 — лише FSK, 2 (типово) — FSK з fallback на LoRa. N і режим мають збігатися на обох модулях; кожен PHY має власний `configuration_id`.
- **Узгоджений вибір після P6:** карусель `(PHY,band)` замінила окремий PHY-selector. Coordinator проходить бенди 0→1→2→3 кожного PHY, responder має довше вікно, розраховане за повним циклом coordinator. PLL-маски можуть відрізнятися. Формули, реальні вікна й межі відновлення — [EML_MULTIBAND.md](EML_MULTIBAND.md).
- **Повернення:** плановий перехід на вищий бенд після 30 с доброї якості зберігає сесію. Зміна PHY відбувається лише під час пошуку нової сесії.
- **Перемикання** (`applyPhy`): MAC зупиняється, таймер зупиняється, радіо переходить у Standby й переконфігуровується, `slot_us` і timeouts беруться з нового профілю, транспорт починає нову спробу зі свіжим epoch; лічильник слотів і generation тривають.

### 10.4. Межі 300/500/1000 кбіт/с (P5.7, лише аналіз)

| Bitrate | Статус | Причина |
| --- | --- | --- |
| 300 кбіт/с | експеримент, `-DEML_FSK_EXPERIMENTAL_300K`, лише N=64 | межа датащиту; h = 0,5 потребує Fdev ≥ 75 кГц, лишаючи ±25 кГц на відхилення; ToA 2 081 мкс з урахуванням RegBitrateFrac, слот 3 700 мкс |
| 500 кбіт/с | недосяжно в межах специфікації | понад 300 кбіт/с; h ≥ 0,5 дає Fdev + BR/2 ≥ 375 кГц > 250 кГц фільтра |
| 1000 кбіт/с | недосяжно в межах специфікації | Fdev + BR/2 ≥ 750 кГц; також понад межу bitrate |

Жоден із цих режимів не є типовим; висновки «нестабільний» чи «підтверджений» можливі лише за вимірюваннями (P5.6/P5.7).

## 11. Перевірки

`test_eml_mac` (native, 22 тести): ToA проти Semtech/ELRS і для всіх довжин LoRa/FSK, бюджет слота всіх профілів, межі FSK (зокрема відхилення 500/1000 кбіт/с і неекспериментальних 300 кбіт/с), FSK sync word і `configuration_id`, план і селектор PHY та гарантія зустрічі, вікно LQ/PER, годинник/hop/32-бітний wrap, монотонний час транспорту, конфігурація з профілю, ID пари, захоплення (зокрема пізня обробка й 16/32-бітний wrap), `EmlTicketRing`. Симуляція пари з реальною FHSS-послідовністю ELRS (40 каналів, 240 hop) і транспортом P3 (профілі LoRa і FSK) перевіряє rendezvous ≤ 80 слотів + handshake, двобічні потоки при 15% втрат кадрів із пізнім/раннім готуванням та відхиленим `sent()`, reboot responder, blackout понад link timeout із повторним пошуком, коротку паузу без нової сесії й wrap 16-бітного Sync-лічильника. Модель не відтворює ISR-таймінги, SPI чи радіоканал — це P4.7.

```bash
export PLATFORMIO_CORE_DIR="$PWD/.venv/platformio"
.venv/bin/pio test -d src -e native
export PLATFORMIO_BUILD_DIR="$PWD/.venv/build-p4"
.venv/bin/pio run -d src -e HappyModel_ES900TX_Bridge -j 4
.venv/bin/pio run -d src -e HappyModel_ES900RX_Bridge -j 4
```
