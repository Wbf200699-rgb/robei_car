#!/usr/bin/env python3
import argparse
import csv
import hashlib
from pathlib import Path
import shutil


SIGNALS = {
    "!": "stop_frames_remaining",
    '"': "stop_frames_complete",
    "#": "last_rr_pulse",
    "$": "last_rf_pulse",
    "%": "last_lr_pulse",
    "&": "last_lf_pulse",
    "'": "frame_count",
    "(": "frame_busy",
    ")": "c5_bus_tx",
    ".": "force_stop",
    "0": "lf_speed",
    "1": "lr_speed",
    "7": "rf_speed",
    "8": "rr_speed",
    "9": "rst_n",
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def decimal_value(value: str) -> str:
    if value and all(bit in "01" for bit in value):
        return str(int(value, 2))
    return ""


def extract(vcd_path: Path, output_path: Path) -> int:
    current_time = 0
    definitions_done = False
    count = 0
    with vcd_path.open("r", encoding="ascii", errors="replace", buffering=1024 * 1024) as source:
        with output_path.open("w", encoding="utf-8", newline="") as destination:
            writer = csv.writer(destination)
            writer.writerow(["time_ps", "time_ns", "signal", "value_bin", "value_dec"])
            for raw_line in source:
                line = raw_line.strip()
                if not definitions_done:
                    if line == "$enddefinitions $end":
                        definitions_done = True
                    continue
                if not line:
                    continue
                if line.startswith("#"):
                    current_time = int(line[1:])
                    continue
                if line[0] in "01xz":
                    identifier = line[1:]
                    value = line[0]
                elif line[0] in "bBrR":
                    parts = line.split()
                    if len(parts) != 2:
                        continue
                    value = parts[0][1:]
                    identifier = parts[1]
                else:
                    continue
                signal = SIGNALS.get(identifier)
                if signal is None:
                    continue
                writer.writerow(
                    [current_time, f"{current_time / 1000:.3f}", signal, value, decimal_value(value)]
                )
                count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract compact evidence from the C5 Robei VCD")
    parser.add_argument("vcd", type=Path)
    parser.add_argument("timing_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--prefix", default="c5_timing")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    transitions = args.output_dir / f"{args.prefix}_key_transitions.csv"
    event_count = extract(args.vcd, transitions)
    shutil.copy2(args.timing_csv, args.output_dir / f"{args.prefix}_robei_raw.csv")
    sha256 = file_sha256(args.vcd)
    metadata = args.output_dir / "source_vcd_metadata.txt"
    metadata.write_text(
        "source_file=" + str(args.vcd.resolve()) + "\n"
        + f"source_size_bytes={args.vcd.stat().st_size}\n"
        + f"source_sha256={sha256}\n"
        + f"extracted_transition_count={event_count}\n",
        encoding="utf-8",
        newline="\n",
    )
    print(f"extracted {event_count} transitions")
    print(f"sha256 {sha256}")


if __name__ == "__main__":
    main()
