#!/usr/bin/env python3
"""Idempotently integrate the BLE high-resolution lidar report path into Robei."""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
PROJECT = ROOT / "little_car" / "little_car.proj"

TOP_NAME = "sys_route_gui_fsm_top"
BT_INSTANCE = "bt_route_uart16"
LIDAR_INSTANCE = "rplidar_c1_uart19"
MAP_INSTANCE = "lidar_map_sampler26"


def root_opening_tag(text: str) -> str:
    match = re.match(r"(?:<!DOCTYPE Robei>\s*\n)?(<Module\b[^>]*>)", text, re.S)
    if not match:
        raise RuntimeError("cannot locate model root opening tag")
    return match.group(1)


def instance_from_leaf(
    leaf_name: str,
    instance_name: str,
    *,
    x: str,
    y: str,
    width: str,
    height: str,
) -> str:
    text = (MODEL_DIR / f"{leaf_name}.model").read_text(encoding="utf-8")
    tag = root_opening_tag(text)
    replacements = {
        "Class": leaf_name,
        "Name": instance_name,
        "Parent": TOP_NAME,
        "X": x,
        "Y": y,
        "Width": width,
        "Height": height,
    }
    for attribute, value in replacements.items():
        tag, count = re.subn(
            rf'\b{re.escape(attribute)}="[^"]*"',
            f'{attribute}="{value}"',
            tag,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"{leaf_name} is missing root attribute {attribute}")

    ports: list[str] = []
    for line in text.splitlines():
        if line.startswith(" <Port "):
            line = re.sub(r'Parent="[^"]+"', f'Parent="{instance_name}"', line)
            ports.append(" " + line)
    if not ports:
        raise RuntimeError(f"{leaf_name} contains no Robei ports")
    return " " + tag + "\n" + "\n".join(ports) + "\n </Module>"


def replace_instance(text: str, instance_name: str, block: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>'
        rf'.*?^[ \t]*</Module>',
        re.M | re.S,
    )
    updated, count = pattern.subn(block, text, count=1)
    if count != 1:
        raise RuntimeError(f"cannot replace top instance {instance_name}")
    return updated


def remove_instance(text: str, instance_name: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>'
        rf'.*?^[ \t]*</Module>[ \t]*\r?\n?',
        re.M | re.S,
    )
    return pattern.sub("", text)


def insert_before_first_root_port(text: str, block: str) -> str:
    pattern = re.compile(
        rf'^[ \t]*<Port\b(?=[^>]*\bParent="{re.escape(TOP_NAME)}")[^>]*/>',
        re.M,
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError("cannot locate first top-level Robei port")
    return text[: match.start()] + block + "\n" + text[match.start() :]


def remove_mapping_wires(text: str) -> str:
    patterns = [
        rf'(?=[^>]*(?:From|To)="[^"]*{re.escape(MAP_INSTANCE)}[^"]*")',
        rf'(?=[^>]*To="{re.escape(TOP_NAME)}#{re.escape(BT_INSTANCE)}&gt;map_transport_ready")',
    ]
    for condition in patterns:
        text = re.sub(
            rf'^[ \t]*<Wire\b{condition}[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def append_before_root_close(text: str, content: str) -> str:
    head, separator, tail = text.rpartition("</Module>")
    if not separator or tail.strip():
        raise RuntimeError("cannot locate unique root closing tag")
    return head.rstrip() + "\n" + content.rstrip() + "\n</Module>\n"


def integrate_top() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = replace_instance(
        text,
        BT_INSTANCE,
        instance_from_leaf(
            "bt_route_uart", BT_INSTANCE, x="2970", y="545", width="1600", height="1080"
        ),
    )
    text = remove_instance(text, MAP_INSTANCE)
    text = insert_before_first_root_port(
        text,
        instance_from_leaf(
            "lidar_map_sampler",
            MAP_INSTANCE,
            x="4750",
            y="3150",
            width="1600",
            height="1080",
        ),
    )
    text = remove_mapping_wires(text)

    wires = f"""
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;clk" Name="{TOP_NAME}_clk_{MAP_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;clk" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#power_on_reset24&gt;system_rst_n" Name="{TOP_NAME}_rst_n_{MAP_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;rst_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;scan_active" Name="{LIDAR_INSTANCE}_scan_active_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;scan_enable" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;sample_valid" Name="{LIDAR_INSTANCE}_sample_valid_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;sample_valid" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;new_scan" Name="{LIDAR_INSTANCE}_new_scan_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;new_scan" />
 <Wire Datasize="6" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;quality" Name="{LIDAR_INSTANCE}_quality_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;quality" />
 <Wire Datasize="15" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;angle_q6" Name="{LIDAR_INSTANCE}_angle_q6_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;angle_q6" />
 <Wire Datasize="16" Datatype="wire" From="{TOP_NAME}#{LIDAR_INSTANCE}&gt;distance_q2" Name="{LIDAR_INSTANCE}_distance_q2_map" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;distance_q2" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{BT_INSTANCE}&gt;map_chunk_accept" Name="{BT_INSTANCE}_map_chunk_accept" Parent="{TOP_NAME}" To="{TOP_NAME}#{MAP_INSTANCE}&gt;map_chunk_accept" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_chunk_valid" Name="{MAP_INSTANCE}_map_chunk_valid" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_chunk_valid" />
 <Wire Datasize="8" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_scan_seq" Name="{MAP_INSTANCE}_map_scan_seq" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_scan_seq" />
 <Wire Datasize="16" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_tick_10ms" Name="{MAP_INSTANCE}_map_tick_10ms" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_tick_10ms" />
 <Wire Datasize="3" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_part" Name="{MAP_INSTANCE}_map_part" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_part" />
 <Wire Datasize="144" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_chunk_data" Name="{MAP_INSTANCE}_map_chunk_data" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_chunk_data" />
 <Wire Datasize="8" Datatype="wire" From="{TOP_NAME}#{MAP_INSTANCE}&gt;map_chunk_crc" Name="{MAP_INSTANCE}_map_chunk_crc" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;map_chunk_crc" />
"""
    text = append_before_root_close(text, wires)
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")


def register_leaf() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    text = re.sub(
        r'^[ \t]*<lidar_map_sampler\.model\b[^>]*/>[ \t]*\r?\n?',
        "",
        text,
        flags=re.M,
    )
    marker = re.search(r'^[ \t]*</model>[ \t]*$', text, flags=re.M)
    if not marker:
        raise RuntimeError("project model folder closing tag is missing")
    entry = '  <lidar_map_sampler.model name="lidar_map_sampler.model" parent="model" type="model"/>\n'
    text = text[: marker.start()] + entry + text[marker.start() :]
    PROJECT.write_text(text, encoding="utf-8", newline="\n")


def validate() -> None:
    top = ET.parse(TOP_MODEL).getroot()
    modules = {module.attrib.get("Name"): module for module in top.findall("Module")}
    expected_ports = {
        BT_INSTANCE: 71,
        MAP_INSTANCE: 16,
    }
    for instance, count in expected_ports.items():
        module = modules.get(instance)
        if module is None or len(module.findall("Port")) != count:
            raise RuntimeError(f"{instance} is missing or has an unexpected port count")

    destinations = [wire.attrib.get("To") for wire in top.findall("Wire")]
    required_destinations = [
        f"{TOP_NAME}#{MAP_INSTANCE}>sample_valid",
        f"{TOP_NAME}#{MAP_INSTANCE}>map_chunk_accept",
        f"{TOP_NAME}#{BT_INSTANCE}>map_chunk_valid",
    ]
    for destination in required_destinations:
        if destinations.count(destination) != 1:
            raise RuntimeError(f"expected exactly one wire to {destination}")

    project = ET.parse(PROJECT).getroot()
    model_group = project.find("model")
    if model_group is None:
        raise RuntimeError("project model group is missing")
    registered = [child.attrib.get("name") for child in model_group]
    if registered.count("lidar_map_sampler.model") != 1:
        raise RuntimeError("lidar_map_sampler.model is not registered exactly once")


def main() -> None:
    integrate_top()
    register_leaf()
    validate()
    print("RPLIDAR 72-point mapping path integrated into Robei model graph")


if __name__ == "__main__":
    main()
