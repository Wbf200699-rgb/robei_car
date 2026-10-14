#!/usr/bin/env python3
"""Idempotently add the LinkPG bridge to the Robei system and constraint graphs."""

from __future__ import annotations

from html import escape, unescape
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
BRIDGE_LEAF_MODEL = MODEL_DIR / "linkpg_uwb_bridge.model"
DEBUG_LEAF_MODEL = MODEL_DIR / "uwb_debug_uart.model"
TOP_CONST = ROOT / "little_car" / "const" / "top.const"
PROJECT = ROOT / "little_car" / "little_car.proj"

TOP_NAME = "sys_route_gui_fsm_top"
BRIDGE_INSTANCE = "linkpg_uwb_bridge25"
DEBUG_INSTANCE = "uwb_debug_uart27"


def opening_tag(text: str) -> re.Match[str]:
    match = re.match(r"(?:<!DOCTYPE Robei>\s*\n)?(<Module\b[^>]*>)", text, flags=re.S)
    if not match:
        raise RuntimeError("cannot locate root Module opening tag")
    return match


def encode_code(code: str) -> str:
    return escape(code, quote=True).replace("\r\n", "\n").replace("\r", "\n").replace("\n", "&#xa;")


def update_root_attribute(text: str, name: str, value: str) -> str:
    match = opening_tag(text)
    tag = match.group(1)
    if not re.search(rf'\b{re.escape(name)}="[^"]*"', tag):
        raise RuntimeError(f"root attribute {name} is missing")
    updated = re.sub(
        rf'\b{re.escape(name)}="[^"]*"',
        f'{name}="{value}"',
        tag,
        count=1,
    )
    return text[: match.start(1)] + updated + text[match.end(1) :]


def append_root_code(text: str, lines: list[str]) -> str:
    match = opening_tag(text)
    tag = match.group(1)
    code_match = re.search(r'\bCode="([^"]*)"', tag, flags=re.S)
    if not code_match:
        raise RuntimeError("root Code attribute is missing")
    code = unescape(code_match.group(1)).replace("\r\n", "\n").replace("\r", "\n")
    for line in lines:
        if line not in code.splitlines():
            code = code.rstrip() + "\n" + line
    updated = tag[: code_match.start(1)] + encode_code(code) + tag[code_match.end(1) :]
    return text[: match.start(1)] + updated + text[match.end(1) :]


def leaf_instance_block(
    model_path: Path,
    class_name: str,
    instance_name: str,
    x: str,
    y: str,
    expected_ports: int,
) -> str:
    text = model_path.read_text(encoding="utf-8")
    match = opening_tag(text)
    tag = match.group(1)
    replacements = {
        "Class": class_name,
        "Name": instance_name,
        "Parent": TOP_NAME,
        "X": x,
        "Y": y,
        "Width": "1600",
        "Height": "1080",
    }
    for name, value in replacements.items():
        tag, count = re.subn(
            rf'\b{re.escape(name)}="[^"]*"',
            f'{name}="{value}"',
            tag,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"leaf root attribute {name} is missing")

    ports: list[str] = []
    for line in text.splitlines():
        if line.startswith(" <Port "):
            line = re.sub(r'Parent="[^"]+"', f'Parent="{instance_name}"', line)
            ports.append(" " + line)
    if len(ports) != expected_ports:
        raise RuntimeError(
            f"expected {expected_ports} ports in {model_path.name}, got {len(ports)}"
        )
    return " " + tag + "\n" + "\n".join(ports) + "\n </Module>\n"


def remove_instance(text: str, instance_name: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>.*?^[ \t]*</Module>[ \t]*\r?\n?',
        flags=re.M | re.S,
    )
    return pattern.sub("", text)


def remove_named_root_ports(text: str, parent: str, names: list[str]) -> str:
    for name in names:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="{re.escape(parent)}")(?=[^>]*\bName="{re.escape(name)}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def remove_wires_to(text: str, destinations: list[str]) -> str:
    for destination in destinations:
        text = re.sub(
            rf'^[ \t]*<Wire\b(?=[^>]*\bTo="{re.escape(destination)}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def append_before_root_close(text: str, content: str) -> str:
    head, separator, tail = text.rpartition("</Module>")
    if not separator or tail.strip():
        raise RuntimeError("cannot locate unique root closing tag")
    return head + content + "</Module>\n"


def integrate_system_model() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = remove_instance(text, BRIDGE_INSTANCE)
    text = remove_instance(text, DEBUG_INSTANCE)
    text = update_root_attribute(text, "Width", "6600")
    text = update_root_attribute(text, "Height", "5600")
    text = remove_named_root_ports(
        text,
        TOP_NAME,
        ["uwb_uart_rx", "uwb_uart_tx", "uwb_debug_uart_tx"],
    )
    text = remove_wires_to(
        text,
        [
            f"{TOP_NAME}#bt_route_uart16&gt;bt_uart_rx",
            f"{TOP_NAME}#{BRIDGE_INSTANCE}&gt;clk",
            f"{TOP_NAME}#{BRIDGE_INSTANCE}&gt;rst_n",
            f"{TOP_NAME}#{BRIDGE_INSTANCE}&gt;uwb_uart_rx",
            f"{TOP_NAME}#{BRIDGE_INSTANCE}&gt;bt_uart_rx_in",
            f"{TOP_NAME}#{BRIDGE_INSTANCE}&gt;bt_uart_tx_tap",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;clk",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;rst_n",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;rx_packet_count",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;rx_crc_error_count",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;tx_packet_count",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;uwb_link_online",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_ack_valid",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_sequence",
            f"{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_result",
            f"{TOP_NAME}&gt;uwb_uart_tx",
            f"{TOP_NAME}&gt;uwb_debug_uart_tx",
        ],
    )

    new_ports = (
        f' <Port Color="#ffd700" Connect="" Datasize="1" Datatype="wire" Description="LinkPG TTL RX" Height="30" Inout="input" Name="uwb_uart_rx" Parent="{TOP_NAME}" Side="left" Width="30" X="-0.00605938" Y="0.853192" />\n'
        f' <Port Color="#40e0d0" Connect="" Datasize="1" Datatype="wire" Description="LinkPG TTL TX" Height="30" Inout="output" Name="uwb_uart_tx" Parent="{TOP_NAME}" Side="right" Width="30" X="0.993941" Y="0.853192" />\n'
        f' <Port Color="#00ff7f" Connect="" Datasize="1" Datatype="wire" Description="UWB diagnostic UART to onboard USB bridge" Height="30" Inout="output" Name="uwb_debug_uart_tx" Parent="{TOP_NAME}" Side="right" Width="30" X="0.993941" Y="0.883192" />\n'
    )
    bridge_block = leaf_instance_block(
        BRIDGE_LEAF_MODEL,
        "linkpg_uwb_bridge",
        BRIDGE_INSTANCE,
        "4750",
        "1900",
        16,
    )
    debug_block = leaf_instance_block(
        DEBUG_LEAF_MODEL,
        "uwb_debug_uart",
        DEBUG_INSTANCE,
        "4750",
        "4350",
        10,
    )
    wires = (
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;clk" Name="{TOP_NAME}_clk_{BRIDGE_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;clk" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#power_on_reset24&gt;system_rst_n" Name="{TOP_NAME}_rst_n" Parent="{TOP_NAME}" To="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;rst_n" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;uwb_uart_rx" Name="{TOP_NAME}_uwb_uart_rx" Parent="{TOP_NAME}" To="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;uwb_uart_rx" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;bt_uart_rx" Name="{TOP_NAME}_bt_uart_rx_bridge" Parent="{TOP_NAME}" To="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;bt_uart_rx_in" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#bt_route_uart16&gt;bt_uart_tx" Name="bt_route_uart16_bt_uart_tx_tap" Parent="{TOP_NAME}" To="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;bt_uart_tx_tap" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;bt_uart_rx_merged" Name="{BRIDGE_INSTANCE}_bt_uart_rx_merged" Parent="{TOP_NAME}" To="{TOP_NAME}#bt_route_uart16&gt;bt_uart_rx" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;uwb_uart_tx" Name="{BRIDGE_INSTANCE}_uwb_uart_tx" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;uwb_uart_tx" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;clk" Name="{TOP_NAME}_clk_{DEBUG_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;clk" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#power_on_reset24&gt;system_rst_n" Name="{TOP_NAME}_rst_n_{DEBUG_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;rst_n" />\n'
        f' <Wire Datasize="16" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;rx_packet_count" Name="{BRIDGE_INSTANCE}_rx_packet_count" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;rx_packet_count" />\n'
        f' <Wire Datasize="16" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;rx_crc_error_count" Name="{BRIDGE_INSTANCE}_rx_crc_error_count" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;rx_crc_error_count" />\n'
        f' <Wire Datasize="16" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;tx_packet_count" Name="{BRIDGE_INSTANCE}_tx_packet_count" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;tx_packet_count" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;uwb_link_online" Name="{BRIDGE_INSTANCE}_uwb_link_online" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;uwb_link_online" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;debug_last_ack_valid" Name="{BRIDGE_INSTANCE}_debug_last_ack_valid" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_ack_valid" />\n'
        f' <Wire Datasize="8" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;debug_last_sequence" Name="{BRIDGE_INSTANCE}_debug_last_sequence" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_sequence" />\n'
        f' <Wire Datasize="2" Datatype="wire" From="{TOP_NAME}#{BRIDGE_INSTANCE}&gt;debug_last_result" Name="{BRIDGE_INSTANCE}_debug_last_result" Parent="{TOP_NAME}" To="{TOP_NAME}#{DEBUG_INSTANCE}&gt;debug_last_result" />\n'
        f' <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{DEBUG_INSTANCE}&gt;uart_tx" Name="{DEBUG_INSTANCE}_uart_tx" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;uwb_debug_uart_tx" />\n'
    )
    text = append_before_root_close(text, bridge_block + debug_block + new_ports + wires)
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")


def integrate_constraints() -> None:
    text = TOP_CONST.read_text(encoding="utf-8")
    text = remove_named_root_ports(
        text,
        "sys_route_gui_fsm_top1",
        ["uwb_uart_rx", "uwb_uart_tx", "uwb_debug_uart_tx"],
    )
    text = remove_named_root_ports(text, "top", ["N14", "N13", "J1"])
    text = remove_wires_to(text, ["top&gt;N14", "top&gt;N13", "top&gt;J1"])
    text = append_root_code(
        text,
        [
            "set_property PACKAGE_PIN N14 [get_ports uwb_uart_rx]",
            "set_property IOSTANDARD LVCMOS33 [get_ports uwb_uart_rx]",
            "set_property PULLUP TRUE [get_ports uwb_uart_rx]",
            "set_property PACKAGE_PIN N13 [get_ports uwb_uart_tx]",
            "set_property IOSTANDARD LVCMOS33 [get_ports uwb_uart_tx]",
            "set_property DRIVE 8 [get_ports uwb_uart_tx]",
            "set_property SLEW SLOW [get_ports uwb_uart_tx]",
            "set_property PACKAGE_PIN J1 [get_ports uwb_debug_uart_tx]",
            "set_property IOSTANDARD LVCMOS33 [get_ports uwb_debug_uart_tx]",
            "set_property DRIVE 8 [get_ports uwb_debug_uart_tx]",
            "set_property SLEW SLOW [get_ports uwb_debug_uart_tx]",
        ],
    )

    child_ports = (
        '  <Port Color="#ffd700" Connect="" Datasize="1" Datatype="wire" Description="LinkPG TTL RX" Height="30" Inout="input" Name="uwb_uart_rx" Parent="sys_route_gui_fsm_top1" Side="left" Width="30" X="-0.0162514" Y="0.930792"/>\n'
        '  <Port Color="#40e0d0" Connect="" Datasize="1" Datatype="wire" Description="LinkPG TTL TX" Height="30" Inout="output" Name="uwb_uart_tx" Parent="sys_route_gui_fsm_top1" Side="right" Width="30" X="0.983749" Y="0.930792"/>\n'
        '  <Port Color="#00ff7f" Connect="" Datasize="1" Datatype="wire" Description="UWB diagnostics to onboard USB UART" Height="30" Inout="output" Name="uwb_debug_uart_tx" Parent="sys_route_gui_fsm_top1" Side="right" Width="30" X="0.983749" Y="0.960792"/>\n'
    )
    child_pattern = re.compile(
        r'(<Module\b(?=[^>]*\bName="sys_route_gui_fsm_top1")[^>]*>.*?)(\n </Module>)',
        flags=re.S,
    )
    text, count = child_pattern.subn(lambda match: match.group(1) + "\n" + child_ports.rstrip() + match.group(2), text, count=1)
    if count != 1:
        raise RuntimeError("cannot update sys_route_gui_fsm_top1 constraint instance")

    root_ports_and_wires = (
        ' <Port Color="#ffd700" Description="H4 pin 1, LinkPG TX to FPGA RX" Drive="" Height="30" Name="N14" Parent="top" Pull="up" Side="left" Voltage="LVCMOS33" Width="30" X="-0.0108342" Y="0.956"/>\n'
        ' <Port Color="#40e0d0" Description="H4 pin 2, FPGA TX to LinkPG RX" Drive="8" Height="30" Name="N13" Parent="top" Pull="none" Side="right" Voltage="LVCMOS33" Width="30" X="0.989166" Y="0.956"/>\n'
        ' <Port Color="#00ff7f" Description="Board FT4232H channel C RX, COM20" Drive="8" Height="30" Name="J1" Parent="top" Pull="none" Side="right" Voltage="LVCMOS33" Width="30" X="0.989166" Y="0.986"/>\n'
        ' <Wire Datasize="1" Datatype="wire" From="top#sys_route_gui_fsm_top1&gt;uwb_uart_rx" Name="0" Parent="top" To="top&gt;N14"/>\n'
        ' <Wire Datasize="1" Datatype="wire" From="top#sys_route_gui_fsm_top1&gt;uwb_uart_tx" Name="0" Parent="top" To="top&gt;N13"/>\n'
        ' <Wire Datasize="1" Datatype="wire" From="top#sys_route_gui_fsm_top1&gt;uwb_debug_uart_tx" Name="0" Parent="top" To="top&gt;J1"/>\n'
    )
    text = append_before_root_close(text, root_ports_and_wires)
    TOP_CONST.write_text(text, encoding="utf-8", newline="\n")


def register_model() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    entries = ["linkpg_uwb_bridge", "uwb_debug_uart"]
    for name in entries:
        text = re.sub(
            rf'^[ \t]*<{re.escape(name)}\.model[^>]+/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    marker = re.search(r'^[ \t]*</model>[ \t]*$', text, flags=re.M)
    if not marker:
        raise RuntimeError("project model folder closing tag is missing")
    lines = "".join(
        f'  <{name}.model name="{name}.model" parent="model" type="model"/>\n'
        for name in entries
    )
    text = text[: marker.start()] + lines + text[marker.start() :]
    PROJECT.write_text(text, encoding="utf-8", newline="\n")


def validate() -> None:
    top = ET.parse(TOP_MODEL).getroot()
    bridge = next(
        (item for item in top.findall("Module") if item.attrib.get("Name") == BRIDGE_INSTANCE),
        None,
    )
    debug = next(
        (item for item in top.findall("Module") if item.attrib.get("Name") == DEBUG_INSTANCE),
        None,
    )
    if bridge is None or len(bridge.findall("Port")) != 16:
        raise RuntimeError("LinkPG instance or its ports are missing from the system model")
    if debug is None or len(debug.findall("Port")) != 10:
        raise RuntimeError("UWB debug instance or its ports are missing from the system model")
    top_ports = {port.attrib.get("Name") for port in top.findall("Port") if port.attrib.get("Parent") == TOP_NAME}
    if not {"uwb_uart_rx", "uwb_uart_tx", "uwb_debug_uart_tx"}.issubset(top_ports):
        raise RuntimeError("system UWB top ports are missing")
    destinations = [wire.attrib.get("To") for wire in top.findall("Wire")]
    if destinations.count(f"{TOP_NAME}#bt_route_uart16>bt_uart_rx") != 1:
        raise RuntimeError("Bluetooth parser RX must have exactly one source")
    for port_name in (
        "rx_packet_count",
        "rx_crc_error_count",
        "tx_packet_count",
        "uwb_link_online",
        "debug_last_ack_valid",
        "debug_last_sequence",
        "debug_last_result",
    ):
        if destinations.count(f"{TOP_NAME}#{DEBUG_INSTANCE}>{port_name}") != 1:
            raise RuntimeError(f"UWB debug input {port_name} must have exactly one source")

    const = ET.parse(TOP_CONST).getroot()
    physical = {port.attrib.get("Name") for port in const.findall("Port") if port.attrib.get("Parent") == "top"}
    if not {"N14", "N13", "J1"}.issubset(physical):
        raise RuntimeError("N14/N13/J1 constraint ports are missing")


def main() -> None:
    integrate_system_model()
    integrate_constraints()
    register_model()
    validate()
    print("LinkPG UWB bridge and J1 diagnostic UART integrated into Robei graphs")


if __name__ == "__main__":
    main()
