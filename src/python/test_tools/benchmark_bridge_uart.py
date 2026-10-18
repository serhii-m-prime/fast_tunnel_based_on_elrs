#!/usr/bin/env python3
"""Measure an ES900 pair through two host serial ports, without firmware debug output.

The application records travel as ordinary UART bytes. A single host clock
measures end-to-end latency, including host/USB queues. --rate is the offered
test-stream rate (headers included), not the UART baud or net RF bitrate.
--self-test uses paired PTYs; its results are explicitly not hardware results.
"""
import argparse
import json
import os
from pathlib import Path
import secrets
import selectors
import struct
import threading
import time
import zlib

import serial

MAGIC = b"EMB7"
HEADER = struct.Struct(">4sIBIQH")  # magic, run nonce, source, sequence, monotonic ns, payload size
SCENARIOS = ("forward", "reverse", "symmetric", "asymmetric", "interactive", "host-read-pause")


def payload(sequence, source, length):
    return bytes((i * 73 + sequence * 19 + source) & 255 for i in range(length))


def packet(nonce, source, sequence, length):
    body = HEADER.pack(MAGIC, nonce, source, sequence, time.monotonic_ns(), length)
    body += payload(sequence, source, length)
    return body + struct.pack(">I", zlib.crc32(body))


def measure(ports, scenario, duration, rate, payload_size, drain_seconds):
    nonce = secrets.randbits(32)
    pending = [b"", b""]
    buffers = [bytearray(), bytearray()]
    sent, received, next_sequence = [0, 0], [0, 0], [0, 0]
    raw_tx, raw_rx = [0, 0], [0, 0]
    latency = [[], []]
    errors = {"crc": 0, "order": 0, "duplicate": 0, "payload": 0}
    credits = [0.0, 0.0]
    rates = [rate, 0.0]
    if scenario == "reverse": rates = [0.0, rate]
    if scenario == "symmetric": rates = [rate, rate]
    if scenario in ("asymmetric", "host-read-pause"): rates = [rate, rate / 8.0]
    if scenario == "interactive": rates = [rate, (HEADER.size + 8 + 4) * 10.0]
    sizes = [payload_size, 8 if scenario == "interactive" else payload_size]
    selector = selectors.DefaultSelector()
    for index, port in enumerate(ports):
        port.reset_input_buffer()
        os.set_blocking(port.fileno(), False)
        selector.register(port.fileno(), selectors.EVENT_READ, index)
    started = last = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            offering = now - started < duration
            for source in (0, 1):
                length = HEADER.size + sizes[source] + 4
                credits[source] = min(2 * length, credits[source] + (now - last) * rates[source])
                if offering and not pending[source] and rates[source] and credits[source] >= length:
                    pending[source] = packet(nonce, source, sent[source], sizes[source])
                    credits[source] -= length
                if pending[source]:
                    try:
                        written = os.write(ports[source].fileno(), pending[source])
                    except BlockingIOError:
                        written = 0
                    pending[source] = pending[source][written:]
                    raw_tx[source] += written
                    if not pending[source]: sent[source] += 1
            last = now
            for key, _ in selector.select(0.002):
                destination = key.data
                # Pausing host reads measures host queueing. With no CTS this
                # does not pretend to stop the receiver's hardware UART TX.
                if scenario == "host-read-pause" and destination == 1 and (now - started) % 1.0 < 0.15:
                    continue
                try:
                    chunk = os.read(key.fd, 65536)
                except BlockingIOError:
                    continue
                buffers[destination].extend(chunk)
                buffer = buffers[destination]
                while True:
                    at = buffer.find(MAGIC)
                    if at < 0:
                        if len(buffer) > 3: del buffer[:-3]
                        break
                    if at: del buffer[:at]
                    if len(buffer) < HEADER.size: break
                    _, run, source, sequence, sent_ns, count = HEADER.unpack_from(buffer)
                    if count > 4096 or source != (destination ^ 1):
                        del buffer[0]
                        errors["payload"] += 1
                        continue
                    length = HEADER.size + count + 4
                    if len(buffer) < length: break
                    body = bytes(buffer[:length - 4])
                    crc = struct.unpack_from(">I", buffer, length - 4)[0]
                    if zlib.crc32(body) != crc:
                        errors["crc"] += 1
                        del buffer[0]
                        continue
                    del buffer[:length]
                    if run != nonce: continue
                    if body[HEADER.size:] != payload(sequence, source, count): errors["payload"] += 1
                    if sequence < next_sequence[source]:
                        errors["duplicate"] += 1
                        continue
                    if sequence != next_sequence[source]: errors["order"] += 1
                    next_sequence[source] = sequence + 1
                    received[source] += 1
                    raw_rx[source] += length
                    latency[source].append((time.monotonic_ns() - sent_ns) / 1e6)
            if not offering and received == sent and not any(pending): break
            if now - started >= duration + drain_seconds: break
    finally:
        selector.close()
    elapsed = time.monotonic() - started
    directions = []
    for source in (0, 1):
        values = sorted(latency[source])
        percentiles = {f"latency_p{p}_ms": values[min(len(values) - 1, (len(values) * p + 99) // 100 - 1)] if values else None
                       for p in (50, 95, 99)}
        directions.append(dict(source="C" if source == 0 else "R", sent_records=sent[source],
                               received_records=received[source], missing_records=max(0, sent[source] - received[source]),
                               partial_tx_bytes=len(pending[source]), test_stream_tx_bytes=raw_tx[source],
                               test_stream_rx_bytes=raw_rx[source], test_stream_rx_bytes_per_second=raw_rx[source] / elapsed,
                               **percentiles))
    passed = not any(errors.values()) and not any(pending) and received == sent and sum(received) > 0
    return dict(scenario=scenario, status="PASS" if passed else "FAIL", elapsed_seconds=elapsed,
                offered_stream_bytes_per_second=rates, errors=errors, directions=directions)


def pty_bridge(masters, stop):
    import select
    pending = [bytearray(), bytearray()]
    for fd in masters: os.set_blocking(fd, False)
    while not stop.is_set():
        ready, _, _ = select.select(masters, [], [], 0.002)
        for index, fd in enumerate(masters):
            if fd in ready:
                try: pending[index ^ 1].extend(os.read(fd, 4096))
                except (BlockingIOError, OSError): pass
        for index, fd in enumerate(masters):
            if pending[index]:
                try:
                    n = os.write(fd, pending[index])
                    del pending[index][:n]
                except BlockingIOError: pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port-c")
    parser.add_argument("--port-r")
    parser.add_argument("--baud", type=int, choices=(115200, 230400, 420000, 460800), default=460800)
    parser.add_argument("--scenario", choices=SCENARIOS + ("all",), default="all")
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--rate", type=float, default=500, help="offered raw test-stream bytes/s per primary direction")
    parser.add_argument("--payload", type=int, default=64)
    parser.add_argument("--drain-seconds", type=float, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0 or args.drain_seconds <= 0 or not 1 <= args.payload <= 4096:
        parser.error("duration, rate and drain-seconds must be positive; payload must be 1..4096")
    if args.self_test and (args.port_c or args.port_r): parser.error("--self-test cannot use real serial ports")
    if not args.self_test and (not args.port_c or not args.port_r or args.port_c == args.port_r):
        parser.error("provide two distinct ports, or --self-test")
    masters, slaves = [], []
    if args.self_test:
        import pty
        for _ in range(2):
            master, slave = pty.openpty()
            masters.append(master); slaves.append(slave)
        names = [os.ttyname(fd) for fd in slaves]
    else:
        names = [args.port_c, args.port_r]
    ports = []
    stop = threading.Event()
    thread = None
    try:
        ports = [serial.Serial(name, args.baud, timeout=0, write_timeout=0) for name in names]
        if args.self_test:
            thread = threading.Thread(target=pty_bridge, args=(masters, stop), daemon=True)
            thread.start()
        rows = [measure(ports, scenario, args.duration, args.rate, args.payload, args.drain_seconds)
                for scenario in (SCENARIOS if args.scenario == "all" else (args.scenario,))]
    finally:
        stop.set()
        if thread: thread.join(1)
        for port in ports: port.close()
        for fd in masters + slaves: os.close(fd)
    report = dict(measurement="PTY tool self-test; no hardware" if args.self_test else "physical UART ports; record hardware/PHY settings separately",
                  baud=args.baud, ports=names, rows=rows,
                  note="RF retry counters require the firmware stats API/debugger; this raw UART tool cannot infer them.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    for row in rows: print(row["scenario"], row["status"])
    raise SystemExit(0 if all(row["status"] == "PASS" for row in rows) else 1)


if __name__ == "__main__":
    main()
