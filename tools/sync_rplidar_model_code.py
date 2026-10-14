#!/usr/bin/env python3
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
BUILD_V = ROOT / "little_car" / "build" / "rplidar_c1_uart.v"
MODEL = ROOT / "little_car" / "model" / "rplidar_c1_uart.model"
TOP_MODEL = ROOT / "little_car" / "model" / "sys_route_gui_fsm_top.model"


def encode_robei_code(code: str) -> str:
    code = code.replace("&", "&amp;")
    code = code.replace("<", "&lt;")
    code = code.replace(">", "&gt;")
    code = code.replace('"', "&quot;")
    code = code.replace("\r\n", "\n").replace("\r", "\n")
    return code.replace("\n", "&#xa;")


def extract_module_body(verilog: str) -> str:
    match = re.search(
        r"\bmodule\s+rplidar_c1_uart\s*\(.*?\);\s*(.*?)\s*endmodule\b",
        verilog,
        flags=re.S,
    )
    if not match:
        raise SystemExit("Cannot find rplidar_c1_uart module body")
    body = match.group(1)
    body = re.sub(
        r"^\s*//----Code starts here: integrated by Robei-----\r?\n",
        "",
        body,
        count=1,
    )
    return body.strip("\r\n")


def main() -> None:
    body = extract_module_body(BUILD_V.read_text(encoding="utf-8"))
    encoded = encode_robei_code(body)

    def replace_model(path: Path, instance_name: str) -> None:
        text = path.read_text(encoding="utf-8")
        pattern = re.compile(
            rf'(<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*\bCode=")(.*?)(" Color=)',
            re.S,
        )
        newline_entity = "&#10;" if f'Name="{instance_name}"' in text and "&#10;" in text else "&#xa;"
        model_encoded = encoded if newline_entity == "&#xa;" else encoded.replace("&#xa;", "&#10;")
        new_text, count = pattern.subn(rf"\1{model_encoded}\3", text, count=1)
        if count != 1:
            raise SystemExit(f"Cannot replace Code attribute for {instance_name} in {path}")
        if new_text != text:
            path.write_text(new_text, encoding="utf-8", newline="\n")
        print(f"synced {path.relative_to(ROOT)}:{instance_name} from {BUILD_V.relative_to(ROOT)}")

    replace_model(MODEL, "rplidar_c1_uart")
    replace_model(TOP_MODEL, "rplidar_c1_uart19")


if __name__ == "__main__":
    main()
