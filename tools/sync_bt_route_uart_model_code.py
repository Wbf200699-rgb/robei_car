#!/usr/bin/env python3
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
BUILD_V = ROOT / "little_car" / "build" / "bt_route_uart.v"
MIRROR_V = ROOT / "little_car" / "hdmi_rgb_input" / "bt_route_uart.v"
MODEL = ROOT / "little_car" / "model" / "bt_route_uart.model"
TOP_MODEL = ROOT / "little_car" / "model" / "sys_route_gui_fsm_top.model"


def encode_robei_code(code: str, newline_entity: str) -> str:
    code = code.replace("\r\n", "\n").replace("\r", "\n")
    code = code.replace("&", "&amp;")
    code = code.replace("<", "&lt;")
    code = code.replace(">", "&gt;")
    code = code.replace('"', "&quot;")
    return code.replace("\n", newline_entity)


def extract_module_body(verilog: str) -> str:
    match = re.search(
        r"\bmodule\s+bt_route_uart\s*\(.*?\);\s*(.*?)\s*endmodule\b",
        verilog,
        flags=re.S,
    )
    if not match:
        raise SystemExit("Cannot find bt_route_uart module body")
    body = match.group(1)
    body = re.sub(
        r"^\s*//----Code starts here: integrated by Robei-----\r?\n",
        "",
        body,
        count=1,
    )
    return body.strip("\r\n")


def replace_model(path: Path, instance_name: str, body: str) -> None:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        rf'(<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*\bCode=")(.*?)(" Color=)',
        re.S,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"Cannot find Code attribute for {instance_name} in {path}")
    old_code = match.group(2)
    newline_entity = "&#10;" if old_code.count("&#10;") > old_code.count("&#xa;") else "&#xa;"
    encoded = encode_robei_code(body, newline_entity)
    new_text = text[:match.start(2)] + encoded + text[match.end(2):]
    if new_text != text:
        path.write_text(new_text, encoding="utf-8", newline="\n")
    print(f"synced {path.relative_to(ROOT)}:{instance_name}")


def main() -> None:
    verilog = BUILD_V.read_text(encoding="utf-8")
    body = extract_module_body(verilog)
    replace_model(MODEL, "bt_route_uart", body)
    replace_model(TOP_MODEL, "bt_route_uart16", body)
    MIRROR_V.write_text(verilog, encoding="utf-8", newline="\n")
    print(f"synced {MIRROR_V.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
