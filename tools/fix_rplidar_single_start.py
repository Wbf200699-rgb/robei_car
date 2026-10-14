#!/usr/bin/env python3
from pathlib import Path
from html import unescape
import re


ROOT = Path(__file__).resolve().parents[1]
BUILD_V = ROOT / "little_car" / "build" / "rplidar_c1_uart.v"
STANDALONE_MODEL = ROOT / "little_car" / "model" / "rplidar_c1_uart.model"
TOP_MODEL = ROOT / "little_car" / "model" / "sys_route_gui_fsm_top.model"


def decode_code(value: str) -> str:
    return unescape(value).replace("\r\n", "\n").replace("\r", "\n")


def encode_code(code: str, newline_entity: str) -> str:
    code = code.replace("\r\n", "\n").replace("\r", "\n")
    code = code.replace("&", "&amp;")
    code = code.replace("<", "&lt;")
    code = code.replace(">", "&gt;")
    code = code.replace('"', "&quot;")
    return code.replace("\n", newline_entity)


def find_module_tag(text: str, instance_name: str) -> re.Match:
    pattern = re.compile(
        rf'<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>',
        flags=re.S,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"Cannot find module tag {instance_name}")
    return match


def extract_model_code(path: Path, instance_name: str) -> str:
    text = path.read_text(encoding="utf-8")
    tag = find_module_tag(text, instance_name).group(0)
    match = re.search(r'\bCode="([^"]*)"', tag, flags=re.S)
    if not match:
        raise SystemExit(f"Cannot find Code attribute in {instance_name}")
    return decode_code(match.group(1))


def replace_model_code(path: Path, instance_name: str, code: str) -> None:
    text = path.read_text(encoding="utf-8")
    match = find_module_tag(text, instance_name)
    tag = match.group(0)
    old_code = re.search(r'\bCode="([^"]*)"', tag, flags=re.S)
    if not old_code:
        raise SystemExit(f"Cannot find Code attribute in {instance_name}")
    newline_entity = "&#10;" if old_code.group(1).count("&#10;") > old_code.group(1).count("&#xa;") else "&#xa;"
    encoded = encode_code(code, newline_entity)
    new_tag = re.sub(r'\bCode="[^"]*"', f'Code="{encoded}"', tag, count=1, flags=re.S)
    new_text = text[:match.start()] + new_tag + text[match.end():]
    if new_text != text:
        path.write_text(new_text, encoding="utf-8", newline="\n")


def patch_body(body: str) -> str:
    body = body.replace("\r\n", "\n").replace("\r", "\n")

    body = re.sub(
        r"^\s*localparam\s+integer\s+START_RETRY_CYCLES\s*=.*?;\n",
        "",
        body,
        flags=re.M,
    )

    old_wait_desc = """                end else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                  byte_timeout_count <= 32'd0;
                  cmd_state <= CMD_RETRY_WAIT;
                end else if (byte_timeout_count >= START_RETRY_CYCLES - 1) begin
                  byte_timeout_count <= 32'd0;
                  desc_index <= 3'd0;
                  cmd_state <= CMD_SCAN_A5;
                end else byte_timeout_count <= byte_timeout_count + 32'd1;"""
    new_wait_desc = """                end else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                  byte_timeout_count <= byte_timeout_count;
                end else byte_timeout_count <= byte_timeout_count + 32'd1;"""
    if old_wait_desc not in body:
        raise SystemExit("Cannot find old WAIT_DESC retry block")
    body = body.replace(old_wait_desc, new_wait_desc, 1)

    body = body.replace(
        """                        scan_active <= 1'b1;
                        lidar_online <= 1'b1;
                        cmd_state <= CMD_SCAN;""",
        """                        scan_active <= 1'b1;
                        lidar_online <= 1'b0;
                        cmd_state <= CMD_SCAN;""",
        1,
    )

    old_scan_timeout = """                else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                  byte_timeout_count <= 32'd0;
                  scan_active <= 1'b0;
                  lidar_online <= 1'b0;
                  cmd_state <= CMD_RETRY_WAIT;
                end else byte_timeout_count <= byte_timeout_count + 32'd1;"""
    new_scan_timeout = """                else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                  byte_timeout_count <= byte_timeout_count;
                  lidar_online <= 1'b0;
                end else byte_timeout_count <= byte_timeout_count + 32'd1;
                if (sample_valid) lidar_online <= 1'b1;"""
    if old_scan_timeout not in body:
        raise SystemExit("Cannot find old SCAN timeout retry block")
    body = body.replace(old_scan_timeout, new_scan_timeout, 1)

    return body


def rewrite_build_verilog(body: str) -> None:
    text = BUILD_V.read_text(encoding="utf-8")
    match = re.search(
        r"(?s)\A(.*?//----Code starts here: integrated by Robei-----\n)(.*?)(\nendmodule\s*//rplidar_c1_uart\s*\Z|\nendmodule\s*//.*?\Z|\nendmodule\s*\Z)",
        text,
    )
    if not match:
        match = re.search(
            r"(?s)\A(.*?//----Code starts here: integrated by Robei-----\n)(.*?)(\nendmodule\s+//rplidar_c1_uart\s*\Z|\nendmodule\s+\S.*\Z|\nendmodule\s*\Z)",
            text,
        )
    if not match:
        raise SystemExit("Cannot split build rplidar_c1_uart.v")
    new_text = match.group(1) + body.rstrip() + match.group(3)
    BUILD_V.write_text(new_text, encoding="utf-8", newline="\n")


def main() -> None:
    base = extract_model_code(TOP_MODEL, "rplidar_c1_uart19")
    if "NO_VALID_RETRY_CYCLES" in base:
        raise SystemExit("Top model already contains the expanded retry implementation; aborting")
    body = patch_body(base)
    replace_model_code(TOP_MODEL, "rplidar_c1_uart19", body)
    replace_model_code(STANDALONE_MODEL, "rplidar_c1_uart", body)
    rewrite_build_verilog(body)
    print("RPLIDAR patched to single-start command behavior")


if __name__ == "__main__":
    main()
