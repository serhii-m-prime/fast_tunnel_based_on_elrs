"""Audit a built ES900 EML image: removed RC stack, retained recovery and IRAM.

This checks build artifacts only. UART/RF timing and physical recovery still
require the P1.7/P4.7/P5.6 hardware tests.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from UnifiedConfiguration import findFirmwareEnd


def audit(build, role, mcu, core):
    elf, binary = build / "firmware.elf", build / "firmware.bin"
    toolchain, prefix, start, end, capacity = (
        ("toolchain-xtensa-esp32", "xtensa-esp32-elf", "_iram_start", "_iram_end", 131072)
        if mcu == "esp32" else
        ("toolchain-xtensa", "xtensa-lx106-elf", "_text_start", "_text_end", 49152)
    )
    nm = core / "packages" / toolchain / "bin" / (prefix + "-nm")
    symbols = subprocess.check_output([str(nm), "-C", "--defined-only", str(elf)], text=True)
    forbidden = (
        "TXLUA_device", "RXLUA_device", "BLE_device", "NimBLE", "Backpack_device",
        "VTX_device", "MSPVTx_device", "VTxSPI_device", "Handset_device",
        "CRSFRouter", "CRSFParser", "CRSFHandset", "TXModuleEndpoint", "RXEndpoint",
        "StubbornSender", "StubbornReceiver", "OtaPackChannelData", "OtaUnpackChannelData",
        "GenerateChannelData", "UnpackChannelData", "OtaUpdateSerializers", "ChannelData",
        "MSP::", "TcpMspConnector", "WifiJoystick", "SerialCRSF", "SerialSBUS",
        "MAVLink", "CROSSFIRE2MSP", "MSP2CROSSFIRE",
    )
    for name in forbidden:
        if name in symbols:
            raise ValueError("Removed component remains in ELF: " + name)
    required = ("emlRxDone(", "emlTxDone()", "emlTimerTock()", "WIFI_device", "LittleFS")
    if mcu == "esp32":
        required += ("Thermal_device",)
    for name in required:
        if name not in symbols:
            raise ValueError("Required component missing from ELF: " + name)

    if not (build / "src/bridge_main.cpp.o").exists():
        raise ValueError("Bridge entry point was not compiled")
    for name in ("tx_main.cpp.o", "rx_main.cpp.o", "dynpower.cpp.o"):
        if (build / "src" / name).exists():
            raise ValueError("Legacy main/dynamic-power source compiled: " + name)
    banned_libraries = {
        "tx-crsf", "rx-crsf", "CrsfProtocol", "CRSF2MSP", "Handset", "BLE", "Backpack",
        "MSP", "MAVLink", "VTX", "VTXSPI", "MSPVTX", "StubbornSender", "StubbornReceiver",
        "ServoOutput", "PWM", "NimBLE-Arduino", "ESP32-BLE-Gamepad", "SCREEN", "GSENSOR",
    }
    archives = {p.name[3:-2] for p in build.glob("lib*/lib*.a")}
    if archives & banned_libraries:
        raise ValueError("Excluded libraries compiled: " + str(sorted(archives & banned_libraries)))

    def address(name):
        match = re.search(r"^([0-9a-fA-F]+) \w " + re.escape(name) + r"$", symbols, re.M)
        if not match:
            raise ValueError("Missing IRAM boundary: " + name)
        return int(match[1], 16)

    used = address(end) - address(start)
    if not 0 < used < capacity:
        raise ValueError("IRAM budget exceeded")
    if mcu == "esp8285":
        for line in symbols.splitlines():
            if any(name in line for name in ("emlRxDone(", "emlTxDone()", "emlTimerTock()")):
                if not 0x40100000 <= int(line.split()[0], 16) < 0x4010C000:
                    raise ValueError("ESP8285 EML ISR outside IRAM: " + line)

    data = binary.read_bytes()
    for route in (b"/update\0", b"/options.json\0", b"/hardware.json\0", b"/reboot\0"):
        if route not in data:
            raise ValueError("Recovery route missing from image: " + repr(route))
    with binary.open("rb") as stream:
        stream.seek(findFirmwareEnd(stream) + 128 + 16)
        options = json.loads(stream.read(512).split(b"\0", 1)[0])
        hardware = json.loads(stream.read(2048).split(b"\0", 1)[0])
    if options.get("is-airport") is not True or options.get("wifi-on-interval") != 60:
        raise ValueError("AirPort/recovery defaults changed")
    if options.get("airport-uart-baud" if role == "tx" else "rcvr-uart-baud") != 460800:
        raise ValueError("UART default changed")
    if role == "tx" and mcu == "esp32" and options.get("unlock-higher-power") is not True:
        raise ValueError("TX higher-power option missing")
    return {
        "role": role, "mcu": mcu, "status": "PASS", "iram_used": used, "iram_free": capacity - used,
        "options": options, "hardware": hardware, "archives": sorted(archives),
        "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (elf, binary)},
        "hardware_tests_performed": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path, help="environment build directory with .elf/.bin")
    parser.add_argument("--role", choices=("tx", "rx"), required=True)
    parser.add_argument("--mcu", choices=("esp32", "esp8285"),
                        help="MCU family; defaults to esp32 for tx and esp8285 for rx for backward compatibility")
    parser.add_argument("--core", type=Path, required=True, help="PlatformIO Core directory")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    mcu = args.mcu or ("esp32" if args.role == "tx" else "esp8285")
    report = audit(args.build, args.role, mcu, args.core)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print("PASS:", args.role, "IRAM", report["iram_used"], "free", report["iram_free"])
