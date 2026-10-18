#!/usr/bin/env python3
"""Compile/run the real EML transport and TDD policy in a deterministic native model.

Reports admitted-byte -> simulated UART-drain latency. This is not an RF,
SPI/ISR, ESP8285 load, or electrical UART measurement. Run from any directory.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="JSON result path; CSV is written beside it")
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    src = Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory(prefix="eml-tdd-bench-") as temporary:
        binary = Path(temporary) / "benchmark"
        command = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
                   "-I" + str(src / "include"), "-I" + str(src / "lib/OTA"),
                   str(src / "python/test_tools/eml_tdd_benchmark.cpp"), "-o", str(binary)]
        if args.sanitize:
            command[2:2] = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        subprocess.run(command, check=True)
        result = json.loads(subprocess.check_output([str(binary)], text=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    rows = []
    for row in result["rows"]:
        shared = {key: value for key, value in row.items() if key != "directions"}
        for index, direction in enumerate(row["directions"]):
            rows.append(dict(shared, direction="C_to_R" if index == 0 else "R_to_C", **direction))
    with args.output.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"PASS: {len(result['rows'])} cases, exact ordered byte delivery; {args.output}")


if __name__ == "__main__":
    main()
