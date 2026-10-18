# EML transport, UART і ISR handoff (P3)

P3 реалізує foreground-транспорт у [`eml_transport.h`](../src/lib/OTA/eml_transport.h), bounded queues у [`eml_queue.h`](../src/lib/OTA/eml_queue.h), ISR handoff у [`eml_mailbox.h`](../src/lib/OTA/eml_mailbox.h) та UART batching у [`eml_uart.h`](../src/lib/OTA/eml_uart.h). Wire-формат і межі сесії визначено в [EML_PROTOCOL.md](EML_PROTOCOL.md).

Актуальне розширення Bridge P7 — [EML_ADAPTIVE_TDD.md](EML_ADAPTIVE_TDD.md): `EmlTransport<N,W>` із W=1…4 (default Bridge W=4), кумулятивні ACK, статичний RX reorder buffer та короткі FSK ACK. Опис одного непідтвердженого кадра нижче залишається базовим контрактом W=1 і старих regression-тестів. Для W=4 є окремі перевірки порядку, backpressure, wrap і memory budget <3600 B на RX разом із двома mailbox.

З P4.2–P4.6 Bridge-збірки (`EML_BRIDGE`) передають цей транспорт через радіо: LoRa-профіль, симетричний TDD, FHSS, захоплення та ISR-інтеграцію описано в [EML_MAC.md](EML_MAC.md). Legacy AirPort (OTA 8/13 байтів, FIFO нижче) лишається тільки для збірок з `USE_AIRPORT_AT_BAUD` без `EML_BRIDGE`; його відкинуті RF-кадри лише рахуються й не отримують гарантій доставки ARQ. Штатні RC/CRSF-пакети не змінено.

## Власність стану та буферів

`EmlTransport<N>` має рівно одного foreground-власника. Його методи не викликаються паралельно чи з ISR. Кожен екземпляр містить дві статичні `EmlByteQueue<N>`: UART→RF та RF→UART. Payload не алокується в heap, не має семантики MAVLink/CRSF і не копіюється у додаткове ARQ-вікно: непідтверджений префікс лишається в UART→RF ring до ACK.

Для ESP32 передбачено `N=4096` на напрямок, для ESP8285 — `N=1024`. Native `static_assert` обмежують розмір transport ESP32 <9000 B, а transport ESP8285 разом із двома mailbox <3000 B. Це бюджет бібліотеки, не вимірювання вільного стека/heap під RF-навантаженням. У `EML_BRIDGE` (P4.6) transport rings **замінюють** legacy AirPort FIFO: ті не компілюються.

`AP_MAX_BUF_LEN` у Bridge-збірках становить 4096 на ESP32 TX і 1024 на RX і задає місткість transport rings. Legacy AirPort без `EML_BRIDGE` використовує ту саму місткість для FIFO (TX — глобальні, RX — static DRAM за посиланням із `SerialAirPort`). Інші конфігурації без `USE_AIRPORT_AT_BAUD` зберігають 64 B. Радіодескриптори EML мають дві фіксовані комірки по максимум 64 B на mailbox; MAC (`rxtx_eml_bridge.cpp`) використовує окремі RX і TX mailbox та `EmlTicketRing` для звітів про старт TX.

## Налаштування та Sync/reconnect

`begin(config, freshEpochSeed, nowSlot)` перевіряє конфігурацію, відкриває нову локальну generation й починає HELLO (coordinator) або пошук (responder). Caller заздалегідь перевіряє capability реального PHY, точний спільний configuration record, peer ID та таблицю FHSS. `validConfig()` перевіряє синтаксис/межі, а не апаратну підтримку FSK.

`freshEpochSeed` має бути свіжим для кожного boot у межах життя старих кадрів. Наступні спроби в одному boot використовують ненульовий лічильник від цього seed; передавати той самий seed після reboot без джерела свіжості не можна. Отримання seed із платформи та прив'язка IDs/config до конкретної пари залишаються частиною P4-інтеграції.

Час API вимірюється розширеним 32-бітним номером слота з modulo wrap. `slot_counter` Sync — молодші 16 бітів того самого слота. MAC задає правильний `hop` і виконує незалежне просування FHSS при втратах за §6 протоколу. Transport не обчислює радіочастоти й не змінює таймери.

Обмеження конфігурації:

- `retry_slots >= 2`;
- `1 <= max_frame_age_slots < 8` — кадр не може пережити мінімальний повний оберт 8 sequence при адаптивних послідовних TX-слотах;
- `quiet_slots > max_frame_age_slots`;
- `handshake_timeout_slots > quiet_slots + retry_slots`;
- `link_timeout_slots > handshake_timeout_slots`, `<32768`.

Це вимоги в слотах, а не затверджений RF-профіль. Вектори симулятора використовують 2/3/40/100 слотів для max-age/quiet/handshake/link timeout; конкретні фізичні параметри та ToA визначаються у P4.

`tick(nowSlot)` обробляє таймаути; його також викликають `prepare/receive`. HELLO/OFFER/ACCEPT/READY повторюються ідемпотентно. Coordinator приймає OFFER лише після фактично переданого HELLO, READY — після переданого ACCEPT. Responder підтверджує ACCEPT лише після OFFER та quiet interval, а перший DATA/KEEPALIVE — лише після фактично переданого READY. До цього UART-вхід не приймається, ACK Sync ігнорується.

Нова HELLO-спроба в ACTIVE призупиняє потік і створює кандидата, не очищаючи старий стан. ACCEPT із точним новим challenge підтверджує заміну сесії. Таймаут непідтвердженого кандидата відновлює стару активну сесію, якщо її link timeout ще не минув. Повторні READY/ACCEPT поточної сесії не скидають sequence чи черги. WAIT_FIRST має власний handshake timeout від моменту commit.

Link timeout, явний `reset()` чи підтверджена заміна session key відкидають старий непідтверджений хвіст та чергу доставки, рахують відкинуті байти, змінюють generation й повертають seq/ACK до `0/7`. Автоматичного перенесення payload через session reset немає. Лічильники статистики зберігаються, мають modulo-32 арифметику й не пишуть службові символи у UART.

## ARQ, ACK та backpressure

`acceptUart(data,n)` приймає тільки префікс, що поміщається, і повертає його довжину. Решта лишається відповідальністю caller; `uart_refused` рахує неприйняті спроби, не доводить фізичну втрату байтів. Нові дані не приймаються поза ACTIVE.

`prepare(slot,hop,out,capacity,&ticket)` у власному TDD-слоті формує Sync, DATA або KEEPALIVE. DATA використовує один непідтверджений префікс і сталий `seq/payload` до ACK. Новий payload не починається при peer PAUSE; повтор уже непідтвердженого DATA дозволений, щоб утрачений ACK можна було відновити навіть за повного peer delivery ring. Header ACK/flow і CRC при повторі перераховуються.

Підготовка не означає фізичної передачі. Безпосередньо перед RF TX MAC зобов'язаний перевірити `canSend(ticket,actualSlot)`, а після фактичного старту передачі в цьому слоті один раз викликати `sent(ticket,actualSlot)`. Прострочений ticket, інша generation, інший слот чи повторний `sent` відхиляються. ACK не звільняє байти, поки DATA не позначено реально переданим; timestamp ACK також має бути пізнішим за перший TX цієї спроби. Caller не може повідомляти `sent` для скасованої/непочатої передачі.

`receive(bytes,n,context,nowSlot)` спершу перевіряє локальну generation, вік дескриптора, порядок capture-slot, роль/парність і межі hop. Потім — повний envelope/CRC/padding, Sync-семантику або стан активної сесії. Прострочений, повторно оброблений чи невпорядкований descriptor відкидається без ACK/UART-ефектів. Context — метадані MAC, а не поля з неперевіреного заголовка.

Вхідний DATA з очікуваним seq підтверджується тільки після атомарного прийняття всього payload у RF→UART ring. Нестача місця повертає `EML_RX_BACKPRESSURE`, не змінює вхідний ACK/expected seq і не очищає старих байтів. Незалежний piggyback ACK зустрічного потоку при цьому може завершити власну передачу. Дублікат останнього прийнятого DATA не додає байтів і не потребує повторного вільного місця. Інший seq відхиляється разом із його ACK/flow. Початковий `seq=7` не є дублікатом до прийому першого DATA.

Flow control враховує відсоток зайнятості та запас на повні кадри: free<1 maximum payload → PAUSE, free<2 або occupancy≥75% → CRITICAL, free<4 або occupancy≥50% → HIGH_WATER, інакше NORMAL. Обидва напрямки мають незалежні pending/expected/ACK та черги. Вікно поки завжди 1.

`peekUart()` позичає через копію обмежену порцію із delivery ring; `consumeUart(written)` видаляє лише фактично записані у драйвер UART байти. ACK означає зберігання в RAM ring, а не фізичне завершення UART TX чи збереження після знеструмлення.

## UART batching та перевантаження

`emlReadSerial` і `emlDrainSerial` роблять не більше однієї порції до 64 B на loop. UART bridge налаштований із timeout=0. Read обмежується `available()` і вільним місцем; за повної черги байти не виймаються з драйвера. Поза підключенням за один loop відкидається не більше 64 B із лічильником, щоб незупинний UART не заблокував RF/recovery.

Write обмежується `availableForWrite()`, бюджетом caller та доступними байтами. Перед записом копіюється до 64 B без видалення; після часткового write видаляється тільки повернута кількість. За нульової місткості драйвера черга зберігається. Serial I/O виконується **поза** critical section; блокування захищає лише короткі копії ring. RX `maxBytesToSend` враховується. `EmlTransportUart<N>` підключає ті самі helper до нового foreground transport для P4.

Legacy AirPort використовує нові `FIFO::tryPushBytes/atomicTryPushBytes/peekBytes/tryPopBytes`. Вони повертають bool і не очищають чергу на overflow/underflow. Старі packet-oriented FIFO API інших протоколів збережено. OTA count обмежується фізичними 5/10 B **до** перетворення в uint8; некоректний count або повний delivery FIFO відкидає новий пакет із `radioRejectedFrames`. Legacy OTA не переносить EML backpressure.

ES900 UART0/CP2102 у поточному підключенні не має налаштованих RTS/CTS. XON/XOFF у raw потік не вставляється. Стримування UART→RF на рівні ring не гарантує, що необмежений хост не переповнить скінченний UART driver FIFO. Хост має обмежувати швидкість/паузи або використовувати власний протокол; baud rate не дорівнює RF goodput. `EmlUartStats_s` рахує read/written, disconnected discard, відмови та stalls; апаратні overrun driver потрібно враховувати окремо при стендових тестах.

## ISR та інтеграційний контракт

Потрібні два `EmlMailbox`: один producer ISR → consumer foreground для RX і один producer foreground → consumer ISR для готових TX. Один mailbox не допускає другого producer/consumer. Capacity=2, overflow повертає false без перезапису старого пакета. Всі об'єкти/буфери мають бути в DRAM. Acquire/release на вирівняних uint32 забезпечують публікацію descriptor; bounded always-inline byte loop не викликає flash `memcpy` або libatomic на перевірених Xtensa-збірках. TX descriptor зберігає точний `EmlTxTicket_s`.

ISR-entry має бути `IRAM_ATTR`. Він копіює максимум 64 B із уже отриманого RF-буфера й stamp, публікує descriptor та повертається. CRC, ARQ, Sync, UART I/O, логування, RNG та алокації в цей шлях не входять. Методи transport і byte ring виконуються тільки foreground. Передача нових CRC/transport методів у ISR без нового audit заборонена.

MAC фіксує RX generation **при arm**, разом із прив'язкою timestamp/hop до конкретного буфера. Після reset не можна приписати старому IRQ/callback поточну generation. Перед `begin/reset`, зміною сесії та початком quiet interval MAC зупиняє старі TX/retry, очищає radio FIFO/IRQ та queued callbacks, переозброює RX у новому контексті. `clearQuiesced()` mailbox викликається тільки після зупинки обох сторін handoff. Транспорт відкидає стару generation, але не може сам очистити периферію.

Для поточного AirPort pack/unpack і нові FIFO try-helper примусово вбудовуються (`always_inline`) у `IRAM_ATTR` radio callers; немає overflow logging у їх ISR-гілці. ELF audit виявив, що одного section attribute на weak template з цими toolchain/LTO недостатньо: окремі інстанціації потрапляли у flash, тому лише позначки `IRAM_ATTR` не є доказом. ESP8285 FIFO lock зберігає й відновлює попередній PS через `xt_rsil/xt_wsr_ps`, тому unlock усередині ISR не вмикає переривання безумовно. Lock одного FIFO не рекурсивний. Розширений ring не збільшує довжину ISR-копії: максимум лишається 5/10 B, UART-копія foreground — до 64 B.

ESP8285 UART ISR й FIFO copy перевіряються в IRAM у кінцевому ELF. Штатний ESP32 Arduino/IDF UART ISR розміщено у flash, але `uart_driver_install(..., intr_alloc_flags=0)` реєструє його без `ESP_INTR_FLAG_IRAM`: SDK маскує цей interrupt на час вимкненого cache. Він не викликається з radio ISR. Це cache-safe відкладення, яке може збільшити ризик UART hardware overrun при довгому cache-off; безперервний UART-прийом під час flash erase цією роботою не гарантується. Радіо-ISR та новий handoff не залежать від цього UART ISR.

`canSend/sent` залишаються foreground API. MAC, що стартує TX безпосередньо з timer ISR, має забезпечити власну IRAM/DRAM generation/slot перевірку та передати повідомлення про старт TX foreground-власнику **до** обробки можливого RX ACK. Виклик transport-методів із ISR заборонений; це вимога інтеграції P4, не автоматична властивість mailbox.

Захист не розрізняє довільний replay/injection із правильним CRC й навмисно підробленим свіжим context. Wire DATA не має session tag. Гарантії потребують виконання MAC-контракту про generation, обмежений час життя та ізоляцію налаштованої пари; обмеження §7.3 протоколу залишаються чинними. Повторний IRQ/таймінговий audit потрібен при P4-інтеграції та зміні профілю.

## Перевірки

Native suite `test_eml_transport` моделює обидва фізичні розміри, втрати кожної handshake-фази, data/ACK, CRC, bidirectional binary streams, довгі UART-паузи, PAUSE, overflow, retransmit/дублікати, непереданий DATA, неправильний/пошкоджений ACK, зміну generation, старі descriptor, candidate timeout, reboot кожної сторони, quiet interval, 16/32-bit slot wrap та багато seq wrap. Перевіряється точний порядок та одноразовий вихід прийнятих байтів у межах сесії, відкидання неоднозначного хвоста при reboot, partial UART I/O і bounded mailbox.

`test_ota` має regression для 4096-байтового FIFO з 512 queued bytes (колишнє звуження до uint8) та некоректного count; `test_fifo` перевіряє відмови без втрати старих байтів. Стендове вимірювання latency/heap/ISR тривалості та RF-передача EML залишаються P4/P9.

Повторення перевірок із кореня репозиторію:

```bash
export PLATFORMIO_CORE_DIR="$PWD/.venv/platformio"
.venv/bin/pio test -d src -e native
export PLATFORMIO_BUILD_DIR="$PWD/.venv/build-p3"
.venv/bin/pio run -d src -e HappyModel_ES900TX_Bridge -j 4
.venv/bin/pio run -d src -e HappyModel_ES900RX_Bridge -j 4
```

Фактичні результати native/sanitizer, ELF/IRAM audit, обох збірок, embedded configuration та використання пам'яті записуються в [IMPLEMENTATION_PLAN.md](../IMPLEMENTATION_PLAN.md).
