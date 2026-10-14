#!/usr/bin/env python3
"""Recreate portable FPGA build inputs from the maintained Robei models."""

from __future__ import annotations

import sys
from pathlib import Path

from robei_converter import const_to_xdc

ROOT = Path(__file__).resolve().parents[1]
REVIEW = ROOT / "little_car" / "source_review"


def main() -> None:
    # The review exporter restores graphical instances/wires as well as Code.
    # Redirect its output, so historical reading copies remain untouched.
    sys.path.insert(0, str(REVIEW))
    import export_sources

    output = ROOT / "little_car" / "build"
    output.mkdir(parents=True, exist_ok=True)
    export_sources.VERILOG_DIR = output
    export_sources.TESTBENCH_DIR = output / "testbenches"
    export_sources.MANIFEST_FILE = output / "module_manifest.csv"
    export_sources.FILELIST_FILE = output / "all_sources.vf"
    export_sources.main()
    files = sorted(output.glob("*.v"))
    if not files or not (output / "sys_route_gui_fsm_top.v").is_file():
        raise RuntimeError("no vehicle top exported")
    # Paths relative to filelist.vf remain valid after moving the checkout.
    (output / "filelist.vf").write_text(
        "".join(f"{path.name}\n" for path in files), encoding="utf-8"
    )
    constraints = ROOT / "little_car" / "const" / "top.const"
    (output / "top.xdc").write_text(
        const_to_xdc(constraints.read_text(encoding="utf-8")), encoding="utf-8"
    )
    print(f"Prepared {len(files)} RTL files and top.xdc in {output}")


if __name__ == "__main__":
    main()
