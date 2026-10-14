#!/usr/bin/env python3
"""Check public-export hygiene, Robei references, and archived wave hashes."""

from __future__ import annotations

import csv
import hashlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SECRET_PATTERNS = [
    re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    re.compile(rb"\bgh[pousr]_[A-Za-z0-9]{30,}\b"),
    re.compile(rb"\bgithub_pat_[A-Za-z0-9_]{30,}\b"),
    re.compile(rb"\bAKIA[0-9A-Z]{16}\b"),
]


def fail(message: str) -> None:
    raise RuntimeError(message)


def main() -> None:
    waves = ROOT / "little_car/test/results/hardware_parameters"
    with (waves / "wave_manifest.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 9:
        fail(f"expected 9 wave records, found {len(rows)}")
    for row in rows:
        path = waves / row["vcd"]
        if path.parent.resolve() != waves.resolve():
            fail("wave path escapes evidence directory")
        data = path.read_bytes()
        if len(data) != int(row["vcd_bytes"]) or len(data) > 8 * 1024 * 1024:
            fail(f"wave size mismatch: {path.name}")
        if hashlib.sha256(data).hexdigest() != row["vcd_sha256"]:
            fail(f"wave hash mismatch: {path.name}")
        # Validate Git's selected bytes too: Windows autocrlf may have staged
        # LF data before the binary VCD attribute was introduced.
        relative = path.relative_to(ROOT).as_posix()
        staged = subprocess.run(["git", "show", f":{relative}"], cwd=ROOT,
                                capture_output=True)
        if staged.returncode == 0 and staged.stdout != data:
            # A newly regenerated wave is intentionally unstaged in CI.
            old_manifest = subprocess.run(
                ["git", "show", ":little_car/test/results/hardware_parameters/wave_manifest.csv"],
                cwd=ROOT, capture_output=True)
            manifest_path = waves / "wave_manifest.csv"
            selected_manifest = old_manifest.stdout.replace(b"\r\n", b"\n")
            live_manifest = manifest_path.read_bytes().replace(b"\r\n", b"\n")
            if old_manifest.returncode == 0 and selected_manifest == live_manifest:
                fail(f"staged wave differs from evidence: {path.name}; re-add with --renormalize")
        log = (waves / row["log"]).read_text(encoding="utf-8")
        if row["status"] != "PASS" or "RESULT TEST_PASS" not in log:
            fail(f"missing pass evidence: {row['test']}")
    print(f"PASS: {len(rows)} archived VCD hashes, sizes and logs")

    for project in [ROOT / "little_car/little_car.proj",
                    ROOT / "robei_model_project/robei_model_project.proj"]:
        root = ET.parse(project).getroot()
        for group in root:
            for node in group:
                name = node.get("name", "")
                if name and not (project.parent / group.tag / name).is_file():
                    fail(f"missing Robei source: {group.tag}/{name}")
    print("PASS: both Robei project file references")

    result = subprocess.run(["git", "ls-files", "--cached", "--others",
                             "--exclude-standard", "-z"], cwd=ROOT,
                            capture_output=True, check=True)
    paths = [ROOT / part.decode("utf-8") for part in result.stdout.split(b"\0") if part]
    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        if not path.is_file():
            continue
        if (path.name in {".env", "local.properties", "gateway_secrets.h"}
                or path.suffix.lower() in {".jks", ".keystore", ".p12", ".apk", ".bit"}
                or relative.startswith("little_car/verilog/")):
            fail(f"private/generated/vendor file selected: {relative}")
        if path.stat().st_size > 10 * 1024 * 1024:
            fail(f"unexpected large file: {relative}")
        content = path.read_bytes()
        if any(pattern.search(content) for pattern in SECRET_PATTERNS):
            # Print paths only; never echo a potential credential.
            fail(f"potential credential: {relative}")
    print(f"PASS: public-export checks for {len(paths)} selected files")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
