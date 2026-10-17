Custom High-Speed UART Radio Bridge (ELRS 4.1 Fork)

## 1. Project Overview & Objective
This repository is a specialized fork of **ExpressLRS v4.1** engineered to operate as a **high-speed, interference-resilient, transparent bidirectional UART radio bridge** over short-to-medium distances (**up to 100+ meters**, including non-line-of-sight, foliage, and heavy EW/RF-noise environments).

- **Protocol-Agnostic Transport:** Completely abstract away from MAVLink, CRSF, or RC channel semantics. The firmware must treat both directions (`TX -> RX` and `RX -> TX`) as raw, low-latency, high-throughput byte streams.
- **Primary Use Case:** Wireless serial link between a ground control PC (connected via USB to the TX module) and a remote mast/relay station (connected via UART to the RX module) operating in high-RF-noise environments (including co-located high-power sub-GHz transmitters operating below 930 MHz).

---

## 2. Target Hardware Specifications & Constraints

### 2.1. Transmitter (Host / PC Side)
- **Hardware:** `HappyModel ExpressLRS ES900TX 915MHz` (`ES900-915MHz / HP037.0184-FCC / 2D20017`)
- **MCU:** `ESP32` (Dual-core Xtensa LX6 @ 240 MHz, 520 KB SRAM)
- **Radio Transceiver:** `Semtech SX1276` (SPI interface) + onboard high-power RF FEM/PA (up to `1000 mW` / `30 dBm`) + active cooling fan.
- **Host Interface:** Onboard `CP2102` USB-to-UART bridge routed directly to `ESP32 UART0` (`GPIO1` TX / `GPIO3` RX) when internal hardware DIP switches **`5` and `6` are `ON`** (`1–4` `OFF`).
- **Power Supply & High-Power RF Operation (`10 mW – 1000 mW`):**
  - The TX module is powered by a **dedicated external power supply (`5V–12V` / 2S Li-Ion connected to the JR bay `VCC` and `GND` pins)** alongside the USB data connection.
  - **Full TX Power Support:** Unlock and support the full hardware power range (**`10 mW`, `25 mW`, `50 mW`, `100 mW`, `250 mW`, `500 mW`, `1000 mW`**) as well as **Dynamic TX Power** based on link quality / RSSI / SNR feedback from the RX.
  - **Active Thermal Management:** Ensure the onboard cooling fan (`GPIO` fan control) is automatically enabled at `>= 250 mW` (or configurable threshold) to keep the RF PA cool when operating at `960–1020 MHz` up to `1000 mW`.

### 2.2. Receiver (Remote / Mast Side)
- **Hardware:** `HappyModel ExpressLRS ES900RX Nano 915MHz`
- **MCU:** `ESP8285` (Single-core Tensilica L106 @ 80/160 MHz, ~50 KB usable RAM, strict `IRAM` limit)
- **Radio Transceiver:** `Semtech SX1276` (SPI interface, up to `100 mW` / `20 dBm` `PA_BOOST` return link)
- **Host Interface:** Hardware `UART0` (`TX` / `RX` pads) at high baud rates (`115200`, `230400`, `420000`, `460800` bps).
- **ESP8285 Memory/ISR Constraint:**
  - Interrupt Service Routines (DIO0/DIO1 radio ISRs and UART FIFO handlers) MUST be marked `IRAM_ATTR`, kept minimal, and avoid flash-resident functions or dynamic heap allocations inside ISR context.

---

## 3. RF & PHY Layer Architecture (SX1276 Silicon Optimization)

### 3.1. Maximum Frequency Shift (`960 MHz – 1020 MHz`)
- **Silicon Ceiling:** The `SX1276` high-frequency PLL (`RFO_HF` / `PA_BOOST`, `Fstep = 32 MHz / 2^19 ≈ 61.035 Hz`) maintains VCO lock up to approximately **`1020–1024 MHz`**.
- **Custom Regulatory Domain (`FHSS`):**
  - Configure the FHSS hopping table in the upper frequency window: **`960.0 MHz` to `1020.0 MHz`** (60 MHz total hopping span).
  - Provide a compile-time and/or runtime configurable `FREQ_START_HZ` and `FREQ_END_HZ` (with a startup PLL lock verification routine checking `SX1276` PLL lock status so `FREQ_END_HZ` automatically backs off if a specific die unlocks near `1020 MHz`).
  - Channel spacing: **`500 kHz` to `1.0 MHz`** (yielding 60 to 120 non-overlapping hopping channels) to maximize jamming resistance and frequency separation from `<930 MHz` systems.

### 3.2. Maximum Channel Bandwidth & High-Speed Modulation
The `SX1276` contains two distinct hardware modems (LoRa and FSK/GFSK). Implement and optimize two selectable PHY profiles for maximum link resilience and throughput:

1. **Profile A — Wideband High-Speed GFSK / FSK (Primary Maximum-Throughput Mode):**
   - Leverage the `SX1276` FSK modem (building on ELRS 4.x `DK500` / `K1000` FSK driver support in `SX127xDriver`).
   - **Bitrate:** `250 kbps` – `500 kbps` (and experimental `1000 kbps` if stable on ESP8285 SPI DMA/FIFO).
   - **Channel Filter Bandwidth (`RxBw` & `AfcBw`):** Set to maximum `SX1276` FSK filter width (**`250 kHz` SSB / `500 kHz` DSB**).
   - **Frequency Deviation (`Fdev`):** Wide deviation (`125 kHz – 200 kHz`) to maximize FM capture effect and interference rejection.
2. **Profile B — Maximum-Bandwidth LoRa (`BW 500 kHz`, `SF6`, `CR 4/5`) (High-Immunity Fallback Mode):**
   - **Bandwidth:** `500 kHz` (`SX127X_BW_500_00_KHZ` — the hardware maximum of the `SX1276` LoRa state machine).
   - **Spreading Factor:** `SF6` (fastest symbol rate, Implicit Header mode) with `CR 4/5` (or `CR 4/6`).
   - **Errata Mitigation:** Ensure `SX1276` 500 kHz sensitivity optimization registers (`0x36 = 0x02`, `0x3A = 0x64` for high band) from Semtech Errata §2.1 are applied.

---

## 4. MAC & Transport Layer Optimization (Custom AirPort / Serial Tunnel)

### 4.1. Eliminating the Standard ELRS 8-Byte Frame Bottleneck
Standard ELRS AirPort encapsulates serial bytes inside `8-byte` (`OTA4`) or `16-byte` (`OTA8`) RC control packets, wasting over 60% of RF airtime on preambles, FHSS synth settling, and TX/RX switching.

- **Expanded OTA Payload Size:**
  - Increase the over-the-air packet size from `8/16 bytes` to **`32–64 bytes` per RF packet** (well within the `SX1276` 256-byte hardware FIFO and `64-byte` half-threshold).
  - Amortize preamble and FHSS hop overhead across larger byte chunks to boost net usable UART throughput by **2.5x–4x**.

### 4.2. Reliable Bidirectional TDD Framing & ARQ
Because `SX1276` is half-duplex in RF (TDD), true full-duplex UART behavior at the hardware pins must be sustained via ring buffers and fast TDD slot turnaround:

- **Packet Structure (`EML_Frame`):**
  - `Header (2 bytes)`:
    - `pkt_type / flags` (2 bits: Sync, Data, Keepalive)
    - `seq_num` (3 bits: rolling sequence number `0..7` for outbound payload)
    - `ack_num` (3 bits: piggybacked ACK for the last valid inbound `seq_num`)
    - `payload_len` (6 bits: `0..60` bytes of actual UART data in this frame)
    - `flow_ctrl` (2 bits: receiver UART TX buffer watermark / backpressure flag)
  - `Payload (0..60 bytes)`: Raw UART bytes dequeued from the local RX ring buffer.
  - `CRC16 (2 bytes)`: 16-bit CCITT/koopman CRC covering Header + Payload.
- **Fast Piggybacked ARQ (Zero-Loss Guarantee):**
  - Outbound UART bytes remain in the transmit sliding window until `ack_num` confirms receipt on the return TDD slot.
  - If a packet is corrupted by RF interference on hop $N$, it is automatically retransmitted on hop $N+1$ (at a new FHSS frequency) without dropping or reordering bytes in the UART stream.
- **Dynamic TDD Cadence:**
  - Support symmetric (`1:2` — alternating TX/RX slots) and adaptive TDD (if one side has an empty queue, send a minimal short sync/ACK header so airtime is yielded immediately to the side with pending data).

---

## 5. Key Areas in the ELRS 4.1 Codebase for Agents

When navigating and modifying the ExpressLRS 4.1 repository, focus on:

1. **Radio Driver (`src/lib/SX127xDriver/`):**
   - `SX127x.cpp`, `SX127xHal.cpp`, `SX127xRegs.h`
   - Unlock frequency ceiling up to `1024 MHz`, configure `500 kHz` LoRa / wideband FSK registers, and support variable/large FIFO packet lengths (`32–64 bytes`).
2. **Frequency Hopping (`src/lib/FHSS/`):**
   - `FHSS.cpp`, `FHSS.h`
   - Define the custom `FCC_HIGH_960_1020` regulatory domain with `500 kHz – 1 MHz` channel steps.
3. **Over-The-Air Framing (`src/lib/OTA/`):**
   - `OTA.cpp`, `OTA.h`
   - Replace RC channel packing/unpacking in AirPort mode with the expanded `EML_Frame` (`32–64 byte` payload + piggybacked `seq/ack` + `CRC16`).
4. **Serial / AirPort Queue Management (`src/tx_main.cpp`, `src/rx_main.cpp`, `src/lib/SerialUpdate/` or AirPort handlers):**
   - Enlarge UART RX/TX ring buffers (`4096 bytes` on ESP32 TX, `1024–2048 bytes` on ESP8285 RX).
   - Ensure zero-copy or batch reading/writing between hardware UART FIFO and the ARQ transmit/receive buffers.
5. **Target Configurations (`src/targets/` & `platformio.ini`):**
   - Pre-configure dedicated PlatformIO environments for:
     - `HappyModel_ES900TX_Bridge` (`ESP32` + `SX1276`, Airport enabled by default, full power range `10–1000 mW` with Dynamic Power & fan control, baud `460800` / `115200`).
     - `HappyModel_ES900RX_Bridge` (`ESP8285` + `SX1276`, Airport enabled by default, `100 mW` `PA_BOOST` downlink, baud `460800` / `115200`).

---

## 6. Agent Development Workflow & Verification Rules

1. **Always Build Both Targets After Any Change:**
   - Verify compilation and RAM/IRAM usage for both `ESP32` (TX) and `ESP8285` (RX) targets via PlatformIO (`pio run -e ...`).
   - Pay strict attention to `ESP8285` IRAM overflow during linking; move non-time-critical helper functions out of `IRAM_ATTR`.
2. **Timing & Airtime Budget Validation:**
   - Whenever modifying packet size, modulation, or FHSS interval (`ExpressLRS_currAirRate_Modparams`), calculate exact **Time-on-Air (ToA)** + turnaround margin (`~300–500 µs` for SX1276 TX/RX synth hop) to guarantee the packet + turnaround fits strictly inside the timer tick interval (`interval` in `µs`).
3. **Hardware & Thermal Management Invariants:**
   - Support full `POWER_OUTPUT_VALUES` (`10 mW` to `1000 mW`) on `ES900TX` with automatic fan activation (`>= 250 mW`) and `100 mW` `PA_BOOST` on `ES900RX`.
   - Maintain a fallback recovery method (Wi-Fi WebUI auto-start or UART bootloader entry via `GPIO0`) so neither module can be bricked by experimental RF register configurations.
