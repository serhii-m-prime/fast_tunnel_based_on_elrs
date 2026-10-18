# EML fast turnaround, QoS and runtime role (P9)

## Fast SX1276 turnaround

The EML ISR now uses this sequence for every scheduled hop:

1. enter Standby and write the three FRF registers;
2. enter `FSTX` or `FSRX` immediately;
3. overlap synthesizer lock with FIFO and RF-switch work;
4. finish with `TX` or `RXCONTINUOUS` without returning to Standby.

`SX127xDriver::SetMode()` retains the selected LoRa/FSK modem bit on every
transition. This is important because LongRangeMode is part of `RegOpMode`, not
a separate register.

The intended fast-path target is 150–200 microseconds, but that number is not a
measurement. `EML_TURNAROUND_US` remains 500 microseconds and the existing slot
intervals remain unchanged until DIO/RF measurements are made on ES900TX and
ES900RX. Current strict capacity budgets are:

| PHY / N | ToA (us) | Turnaround | RX done | Processing | Guard | Slot | Margin |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LoRa / 64 | 16032 | 500 | 200 | 600 | 300 | 17800 | 168 |
| LoRa / 32 | 8992 | 500 | 200 | 600 | 300 | 10600 | 8 |
| FSK250 / 64 | 2496 | 500 | 150 | 600 | 300 | 4100 | 54 |
| FSK250 / 32 | 1472 | 500 | 150 | 600 | 300 | 3100 | 78 |

No extra airtime is claimed before the hardware measurement. In particular,
the 8 microsecond LoRa/N32 calculated margin must not be reduced by assumption.

## Frame-boundary packing

`bridge-traffic-mode` selects one of three bounded ingress policies:

- `Raw transparent` (default): preserves the byte stream and its order exactly;
- `MAVLink2 framing + QoS`: recognizes frames beginning with `0xFD`, uses the
  payload length and incompatibility flags, and includes the optional 13-byte
  MAVLink2 signature in the boundary;
- `SBUS framing + QoS`: recognizes a 25-byte frame beginning with `0x0F` and a
  documented SBUS end marker.

A MAVLink2 wire frame is 12–280 bytes. An N=64 EML DATA payload holds at most
60 bytes (N=32 holds 28). A complete MAVLink frame that fits is admitted to the
transport immediately before one DATA allocation, so it is neither combined
with the following message nor split at a slot boundary. Larger messages cannot
physically fit and are transmitted losslessly as consecutive EML fragments.
Once fragmentation starts, that message owns the scheduler until its last
fragment, so a newly arriving high-priority frame can never be inserted into
the middle of an existing MAVLink wire frame.

The parser never discards an incomplete or malformed candidate. If no remaining
bytes arrive within 20 ms, the candidate is released as normal raw bytes. This
keeps arbitrary binary input live even when a protocol-aware mode was selected
incorrectly.

## Two-level QoS

The ingress has bounded normal and high-priority lanes. MAVLink2 message IDs 69
(`MANUAL_CONTROL`) and 76 (`COMMAND_LONG`) enter the high lane. Valid SBUS
frames do the same. Before each DATA allocation the high lane is inspected
first, so a critical frame uses the next slot for which the ARQ window can
allocate a new sequence. Existing in-flight DATA is never mutated and ARQ retry,
CRC, ACK and receiver backpressure behavior are unchanged.

QoS intentionally changes semantic message order. It is therefore opt-in.
Selecting MAVLink/SBUS mode means that a complete critical message may overtake
queued telemetry; raw mode remains the correct setting for arbitrary protocols
or applications requiring strict original byte order.

## Runtime Master/Slave role

`bridge-role` is persisted in `/options.json` and applied after reboot:

- `0`: Master/coordinator, owns the initial slot and advertises HELLO;
- `1`: Slave/responder, searches for the coordinator and phase-locks its timer;
- `2`: Auto, detects an active Master in the air at start-up (see below).

The MAC role is no longer a compile-time constant. The hardware timer follows
the runtime role as well: a coordinator registers no tick callback and runs the
tock-only slot-master timer, a responder runs the tick/tock timer that applies
phase shifts and fires its first tock immediately on resume. Before this, both
modes were selected by `TARGET_TX`/`TARGET_RX`, so an ES900RX image set to
Master called a null tick callback in the timer ISR, and an ES900TX image set to
Slave silently ignored phase shifts and started one slot late. Stock ELRS builds
keep the compile-time selection.

The standard image for each physical board contains both role paths, so one
ES900TX image can be used for either ES900TX role and one ES900RX image can be
used for either ES900RX role. ESP32 and ESP8285 still require different binaries
because they are different MCU architectures. The older role-named PlatformIO
environments remain as compatible build aliases/defaults.

### Auto role

An Auto node listens first as a responder whose band search is held on its first
(default) band, then advertises as a coordinator, and alternates until a session
starts. A session or handshake keeps the current role; after a link loss the
phase timer restarts in that role. Phase lengths derive from the PHY plan,
where `D` is the coordinator band dwell and `R = 4D + D` is the band cycle plus
one dwell:

| Phase | Length | Reason |
| --- | --- | --- |
| Listen | `R + D + k·D` | a coordinator cycling alone visits the listener's band for a full dwell (two HELLO periods) within any `R` window |
| Advertise | `2R + D + k·D` | a fixed responder dwells `R` per band, so any `2R` window holds one whole responder dwell |

`k` is drawn uniformly from 0..7 per phase, seeded by the hardware RNG and the
chip MAC, because both nodes of a pair share the UID. Two Auto nodes that draw
different `k` become coordinator at least one dwell apart, and a new coordinator
starts on the default band on which the other node is still listening. Equal
draws are resolved by later phases.

| PHY / N | Listen | Advertise | Auto vs fixed Master | Auto vs fixed Slave | Auto vs Auto (model, mean / worst of 2000) |
| --- | ---: | ---: | ---: | ---: | ---: |
| FSK250 / 64 | 6.1–13.2 s | 11.2–18.3 s | ≤ 5.1 s | ≤ 23.4 s | 15.6 s / 182 s |
| LoRa / 64 | 26.5–57.4 s | 48.6–79.5 s | ≤ 22.1 s | ≤ 101.5 s | 69.3 s / 790 s |

The Auto vs Auto figures come from the discrete rendezvous model in
`test_eml_role`, not from radios. Auto therefore trades link-up time for
convenience. For a deterministic start, set one side to a fixed role; Auto on
the other side then needs no configuration. `/bridge-info.json` reports the
configured `role` and the current `active-role`.

Peers must use opposite fixed roles, or Auto on at least one side, and the same
configured UID/binding identity, PHY, frame size and RF configuration. Role is
not included in configuration ID, because opposite values are required for one
link.

## Verification boundary

Native tests cover split UART batches, signed 280-byte MAVLink frames, priority
selection, long-frame reconstruction, parser timeout, SBUS recognition and raw
ordering. `test_eml_role` covers fixed roles, Auto phase bounds, hold/restart,
draw coverage and the rendezvous model for Auto vs Auto, Auto vs fixed Master
and Auto vs fixed Slave. All four Bridge environments build and pass the ELF/bin
audit. The software checks do not establish real synth-lock time, RF switching
time, GCS latency, WebUI role switching on flash, Auto election on air, or
same-image pair compatibility. Those measurements remain the P9.1/P9.4 hardware acceptance work in stage 10.
