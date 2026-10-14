#!/usr/bin/env python3
"""Run compact hardware-parameter simulations and enforce a VCD size cap."""

from __future__ import annotations

import csv
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

from generate_hardware_parameter_tests import ROOT, TESTS, main as generate


RESULT_DIR = ROOT / "little_car" / "test" / "results" / "hardware_parameters"
VCD_LIMIT_BYTES = 8 * 1024 * 1024


def find_tool(name: str) -> Path:
    found = shutil.which(name)
    if found:
        return Path(found)
    configured = os.environ.get("ROBEI_BIN")
    if configured:
        candidate = Path(configured) / (f"{name}.exe" if os.name == "nt" else name)
        if candidate.is_file():
            return candidate
    bundled = Path(r"D:\Apps\RobeiEDA\Robei\bin") / f"{name}.exe"
    if bundled.is_file():
        return bundled
    raise RuntimeError(f"cannot find {name}; install Icarus or RobeiEDA")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run() -> int:
    generate()
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    iverilog = find_tool("iverilog")
    vvp = find_tool("vvp")
    rows: list[dict[str, str | int]] = []
    failures: list[str] = []

    for spec in TESTS:
        name = spec["name"]
        source_name = spec["source"]
        source = ROOT / "little_car" / "test" / "verilog" / f"{name}.v"
        executable = RESULT_DIR / f"{name}.vvp"
        log_path = RESULT_DIR / f"{name}.log"
        vcd_path = RESULT_DIR / f"{source_name}.vcd"
        if vcd_path.exists():
            vcd_path.unlink()

        compile_result = subprocess.run(
            [str(iverilog), "-g2012", "-DPARAM_WAVE", "-o",
             str(executable), str(source)],
            cwd=ROOT, capture_output=True, text=True, timeout=30,
        )
        if compile_result.returncode:
            log_path.write_text(compile_result.stdout + compile_result.stderr,
                                encoding="utf-8")
            failures.append(f"{name}: compile failed")
            continue

        result = subprocess.run(
            [str(vvp), str(executable)], cwd=RESULT_DIR,
            capture_output=True, text=True, timeout=30,
        )
        output = result.stdout + result.stderr
        executable.unlink(missing_ok=True)
        log_path.write_text(output, encoding="utf-8", newline="\n")
        passed = result.returncode == 0 and "RESULT TEST_PASS" in output
        if not vcd_path.is_file():
            failures.append(f"{name}: no VCD generated")
            size = 0
            digest = ""
        else:
            size = vcd_path.stat().st_size
            digest = sha256(vcd_path)
            if size > VCD_LIMIT_BYTES:
                failures.append(f"{name}: VCD {size} exceeds {VCD_LIMIT_BYTES}")
        if not passed:
            failures.append(f"{name}: simulation did not pass")
        rows.append({
            "test": name,
            "status": "PASS" if passed else "FAIL",
            "log": log_path.name,
            "vcd": vcd_path.name,
            "vcd_bytes": size,
            "vcd_sha256": digest,
        })

    manifest = RESULT_DIR / "wave_manifest.csv"
    with manifest.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "test", "status", "log", "vcd", "vcd_bytes", "vcd_sha256"
        ])
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(f"{row['status']:4} {row['test']}: {row['vcd_bytes']} bytes")
    if failures:
        print("FAILED:", *failures, sep="\n  ", file=sys.stderr)
        return 1
    print(f"All {len(rows)} tests passed; manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
