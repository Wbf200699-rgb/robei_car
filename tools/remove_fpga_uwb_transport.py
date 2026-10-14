#!/usr/bin/env python3
"""Remove the retired FPGA LinkPG transport while preserving the BLE map path."""

from __future__ import annotations

from html import escape, unescape
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
BT_MODEL = MODEL_DIR / "bt_route_uart.model"
TOP_CONST = ROOT / "little_car" / "const" / "top.const"
PROJECT = ROOT / "little_car" / "little_car.proj"

TOP_NAME = "sys_route_gui_fsm_top"
TOP_INSTANCE = "sys_route_gui_fsm_top1"
BT_INSTANCE = "bt_route_uart16"
BRIDGE_INSTANCE = "linkpg_uwb_bridge25"
DEBUG_INSTANCE = "uwb_debug_uart27"
UWB_PORTS = ("uwb_uart_rx", "uwb_uart_tx", "uwb_debug_uart_tx")


def opening_tag(text: str) -> re.Match[str]:
    match = re.match(r"(?:<!DOCTYPE Robei>\s*\n)?(<Module\b[^>]*>)", text, flags=re.S)
    if not match:
        raise RuntimeError("cannot locate root Module opening tag")
    return match


def encode_code(code: str) -> str:
    normalized = code.replace("\r\n", "\n").replace("\r", "\n")
    return escape(normalized, quote=True).replace("\n", "&#xa;")


def remove_root_code_lines(text: str, port_names: tuple[str, ...]) -> str:
    match = opening_tag(text)
    tag = match.group(1)
    code_match = re.search(r'\bCode="([^"]*)"', tag, flags=re.S)
    if not code_match:
        raise RuntimeError("root Code attribute is missing")
    code = unescape(code_match.group(1))
    kept = [line for line in code.splitlines() if not any(name in line for name in port_names)]
    updated_tag = tag[: code_match.start(1)] + encode_code("\n".join(kept)) + tag[code_match.end(1) :]
    return text[: match.start(1)] + updated_tag + text[match.end(1) :]


def instance_from_leaf(model_path: Path, instance_name: str) -> str:
    text = model_path.read_text(encoding="utf-8")
    tag = opening_tag(text).group(1)
    replacements = {
        "Class": "bt_route_uart",
        "Name": instance_name,
        "Parent": TOP_NAME,
        "X": "2970",
        "Y": "545",
        "Width": "1600",
        "Height": "1080",
    }
    for attribute, value in replacements.items():
        tag, count = re.subn(
            rf'\b{re.escape(attribute)}="[^"]*"',
            f'{attribute}="{value}"',
            tag,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"bt_route_uart model is missing {attribute}")

    ports: list[str] = []
    for line in text.splitlines():
        if line.startswith(" <Port "):
            ports.append(" " + re.sub(r'Parent="[^"]+"', f'Parent="{instance_name}"', line))
    if len(ports) != 71:
        raise RuntimeError(f"expected 71 BLE UART ports after cleanup, got {len(ports)}")
    return " " + tag + "\n" + "\n".join(ports) + "\n </Module>"


def replace_instance(text: str, instance_name: str, block: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>'
        rf'.*?^[ \t]*</Module>',
        flags=re.M | re.S,
    )
    updated, count = pattern.subn(block, text, count=1)
    if count != 1:
        raise RuntimeError(f"cannot replace instance {instance_name}")
    return updated


def remove_instance(text: str, instance_name: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>'
        rf'.*?^[ \t]*</Module>[ \t]*\r?\n?',
        flags=re.M | re.S,
    )
    return pattern.sub("", text)


def remove_ports(text: str, parent: str, names: tuple[str, ...]) -> str:
    for name in names:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="{re.escape(parent)}")'
            rf'(?=[^>]*\bName="{re.escape(name)}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def remove_wires(text: str, predicates: tuple[str, ...]) -> str:
    lines = []
    for line in text.splitlines(keepends=True):
        if "<Wire " in line and any(predicate in line for predicate in predicates):
            continue
        lines.append(line)
    return "".join(lines)


def append_before_root_close(text: str, content: str) -> str:
    head, separator, tail = text.rpartition("</Module>")
    if not separator or tail.strip():
        raise RuntimeError("cannot locate root closing tag")
    return head.rstrip() + "\n" + content.rstrip() + "\n</Module>\n"


def clean_top_model() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = replace_instance(text, BT_INSTANCE, instance_from_leaf(BT_MODEL, BT_INSTANCE))
    text = remove_instance(text, BRIDGE_INSTANCE)
    text = remove_instance(text, DEBUG_INSTANCE)
    text = remove_ports(text, TOP_NAME, UWB_PORTS)
    text = remove_wires(
        text,
        (
            BRIDGE_INSTANCE,
            DEBUG_INSTANCE,
            "&gt;uwb_uart_rx",
            "&gt;uwb_uart_tx",
            "&gt;uwb_debug_uart_tx",
            f'{TOP_NAME}#{BT_INSTANCE}&gt;bt_uart_rx',
            f'{TOP_NAME}#{BT_INSTANCE}&gt;map_transport_ready',
        ),
    )
    direct_ble_wire = (
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;bt_uart_rx" '
        f'Name="{TOP_NAME}_bt_uart_rx" Parent="{TOP_NAME}" '
        f'To="{TOP_NAME}#{BT_INSTANCE}&gt;bt_uart_rx" />'
    )
    TOP_MODEL.write_text(
        append_before_root_close(text, direct_ble_wire), encoding="utf-8", newline="\n"
    )


def clean_constraints() -> None:
    text = TOP_CONST.read_text(encoding="utf-8")
    text = remove_root_code_lines(text, UWB_PORTS)
    text = remove_ports(text, TOP_INSTANCE, UWB_PORTS)
    text = remove_ports(text, "top", ("N14", "N13", "J1"))
    text = remove_wires(
        text,
        (
            "&gt;uwb_uart_rx",
            "&gt;uwb_uart_tx",
            "&gt;uwb_debug_uart_tx",
            "top&gt;N14",
            "top&gt;N13",
            "top&gt;J1",
        ),
    )
    TOP_CONST.write_text(text, encoding="utf-8", newline="\n")


def clean_project_registration() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    for model_name in ("linkpg_uwb_bridge", "uwb_debug_uart"):
        text = re.sub(
            rf'^[ \t]*<{model_name}\.model\b[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    PROJECT.write_text(text, encoding="utf-8", newline="\n")


def validate() -> None:
    top = ET.parse(TOP_MODEL).getroot()
    modules = {module.attrib.get("Name"): module for module in top.findall("Module")}
    if BRIDGE_INSTANCE in modules or DEBUG_INSTANCE in modules:
        raise RuntimeError("retired UWB instances remain in the top model")
    bt = modules.get(BT_INSTANCE)
    if bt is None or len(bt.findall("Port")) != 71:
        raise RuntimeError("BLE UART instance is missing or has the wrong port count")
    top_ports = {
        port.attrib.get("Name")
        for port in top.findall("Port")
        if port.attrib.get("Parent") == TOP_NAME
    }
    if any(name in top_ports for name in UWB_PORTS):
        raise RuntimeError("retired UWB top ports remain")
    destinations = [wire.attrib.get("To") for wire in top.findall("Wire")]
    if destinations.count(f"{TOP_NAME}#{BT_INSTANCE}>bt_uart_rx") != 1:
        raise RuntimeError("BLE RX is not connected exactly once")
    if any("map_transport_ready" in (destination or "") for destination in destinations):
        raise RuntimeError("retired map transport gate remains")

    const = ET.parse(TOP_CONST).getroot()
    physical = {
        port.attrib.get("Name")
        for port in const.findall("Port")
        if port.attrib.get("Parent") == "top"
    }
    if {"N13", "N14", "J1"} & physical:
        raise RuntimeError("retired FPGA UWB physical ports remain")

    project = ET.parse(PROJECT).getroot()
    model_group = project.find("model")
    registered = [] if model_group is None else [child.attrib.get("name") for child in model_group]
    if "linkpg_uwb_bridge.model" in registered or "uwb_debug_uart.model" in registered:
        raise RuntimeError("retired FPGA UWB models remain registered")


def main() -> None:
    clean_top_model()
    clean_constraints()
    clean_project_registration()
    validate()
    print("FPGA UWB transport removed; BLE RX and lidar map reporting remain active")


if __name__ == "__main__":
    main()
