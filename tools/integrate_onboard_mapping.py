#!/usr/bin/env python3
"""Idempotently integrate board-resident mapping and configuration Flash."""

from __future__ import annotations

from pathlib import Path
import html
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
CAR_DIR = ROOT / "little_car"
MODEL_DIR = CAR_DIR / "model"
BUILD_DIR = CAR_DIR / "build"
MIRROR_DIR = CAR_DIR / "hdmi_rgb_input"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
TOP_CONST = CAR_DIR / "const" / "top.const"
PROJECT = CAR_DIR / "little_car.proj"
FILELIST = BUILD_DIR / "filelist.vf"
TOP_XDC = BUILD_DIR / "top.xdc"

TOP = "sys_route_gui_fsm_top"
BT = "bt_route_uart16"
LIDAR = "rplidar_c1_uart19"
MOTION = "motion_sensor_subsystem28"
STABILIZER = "c5_motion_stabilizer29"
MAP = "onboard_occupancy_map30"
FLASH = "map_flash_store31"


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
        "Parent": TOP,
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
        rf'^[ \t]*<Port\b(?=[^>]*\bParent="{re.escape(TOP)}")[^>]*/>', re.M
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError("cannot locate first top-level port")
    return text[: match.start()] + block + "\n" + text[match.start() :]


def remove_wire_if(text: str, condition: str) -> str:
    return re.sub(
        rf'^[ \t]*<Wire\b(?=[^>]*{condition})[^>]*/>[ \t]*\r?\n?',
        "",
        text,
        flags=re.M,
    )


def add_wire(
    text: str, name: str, source: str, destination: str, size: int = 1
) -> str:
    line = (
        f' <Wire Datasize="{size}" Datatype="wire" From="{source}" '
        f'Name="{name}" Parent="{TOP}" To="{destination}" />'
    )
    return text.rsplit("</Module>", 1)[0] + line + "\n</Module>\n"


def root_port_line(name: str, direction: str, side: str, x: str, y: str) -> str:
    color = "#6495ed" if direction == "input" else "#00ffff"
    return (
        f' <Port Color="{color}" Connect="" Datasize="1" Datatype="wire" '
        f'Description="" Height="30" Inout="{direction}" Name="{name}" '
        f'Parent="{TOP}" Side="{side}" Width="30" X="{x}" Y="{y}" />'
    )


def upsert_root_ports(text: str) -> str:
    names = (
        "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
        "flash_hold_n", "flash_miso"
    )
    for name in names:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="{TOP}")'
            rf'(?=[^>]*\bName="{name}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    first_wire = re.search(r"^[ \t]*<Wire\b", text, flags=re.M)
    if not first_wire:
        raise RuntimeError("top model has no wire insertion point")
    ports = "\n".join(
        (
            root_port_line("flash_cs_n", "output", "right", "0.993941", "0.833192"),
            root_port_line("flash_clk", "output", "bottom", "0.90", "0.994"),
            root_port_line("flash_mosi", "output", "right", "0.993941", "0.863192"),
            root_port_line("flash_wp_n", "output", "right", "0.993941", "0.893192"),
            root_port_line("flash_hold_n", "output", "right", "0.993941", "0.923192"),
            root_port_line("flash_miso", "input", "left", "-0.00605938", "0.853192"),
        )
    )
    return text[: first_wire.start()] + ports + "\n" + text[first_wire.start() :]


def integrate_top_model() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = replace_instance(
        text,
        BT,
        instance_from_leaf(
            "bt_route_uart", BT, x="2970", y="545", width="1900", height="1500"
        ),
    )
    for instance in (MAP, FLASH):
        text = remove_instance(text, instance)
    text = insert_before_first_root_port(
        text,
        instance_from_leaf(
            "onboard_occupancy_map",
            MAP,
            x="5050",
            y="4300",
            width="1750",
            height="1320",
        ),
    )
    text = insert_before_first_root_port(
        text,
        instance_from_leaf(
            "map_flash_store",
            FLASH,
            x="7000",
            y="4300",
            width="1450",
            height="1050",
        ),
    )
    text = upsert_root_ports(text)

    for instance in (MAP, FLASH):
        text = remove_wire_if(text, rf'(?:From|To)="[^"]*{instance}[^"]*"')
    for port in (
        "grid_chunk_valid", "grid_chunk_seq", "grid_chunk_base",
        "grid_chunk_data", "grid_chunk_crc", "onboard_map_active",
        "onboard_map_paused", "onboard_map_valid", "onboard_map_busy",
        "onboard_map_epoch", "onboard_map_scan_count",
        "onboard_map_occupied_count", "onboard_map_free_count",
        "onboard_map_dropped_count", "onboard_pose_x_mm",
        "onboard_pose_y_mm", "onboard_pose_heading", "onboard_pose_flags",
        "flash_ready", "flash_busy", "flash_operation", "flash_progress",
        "flash_error", "flash_jedec_id", "flash_saved_sequence",
        "onboard_map_start_pulse", "onboard_map_pause_pulse",
        "onboard_map_clear_pulse", "onboard_map_save_pulse",
        "onboard_map_load_pulse", "onboard_map_sync_pulse",
        "grid_chunk_accept",
    ):
        text = remove_wire_if(
            text, rf'To="{TOP}#{BT}&gt;{re.escape(port)}"'
        )
    for port in (
        "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
        "flash_hold_n", "flash_miso"
    ):
        text = remove_wire_if(text, rf'(?:From|To)="{TOP}&gt;{port}"')

    edges: list[tuple[str, str, str, int]] = []

    def edge(name: str, src_i: str, src_p: str, dst_i: str, dst_p: str, size: int = 1) -> None:
        edges.append(
            (name, f"{TOP}#{src_i}&gt;{src_p}", f"{TOP}#{dst_i}&gt;{dst_p}", size)
        )

    # Shared clocks and resets.
    edges.extend(
        [
            (f"{TOP}_clk_{MAP}", f"{TOP}&gt;clk", f"{TOP}#{MAP}&gt;clk", 1),
            (f"{TOP}_rst_n_{MAP}", f"{TOP}#power_on_reset24&gt;system_rst_n", f"{TOP}#{MAP}&gt;rst_n", 1),
            (f"{TOP}_clk_{FLASH}", f"{TOP}&gt;clk", f"{TOP}#{FLASH}&gt;clk", 1),
            (f"{TOP}_rst_n_{FLASH}", f"{TOP}#power_on_reset24&gt;system_rst_n", f"{TOP}#{FLASH}&gt;rst_n", 1),
        ]
    )

    for src, dst in (
        ("onboard_map_start_pulse", "map_start_pulse"),
        ("onboard_map_pause_pulse", "map_pause_pulse"),
        ("onboard_map_clear_pulse", "map_clear_pulse"),
        ("onboard_map_sync_pulse", "map_sync_pulse"),
    ):
        edge(f"{BT}_{src}", BT, src, MAP, dst)
    edge(f"{BT}_onboard_map_save_pulse", BT, "onboard_map_save_pulse", FLASH, "save_pulse")
    edge(f"{BT}_onboard_map_load_pulse", BT, "onboard_map_load_pulse", FLASH, "load_pulse")
    edge(f"{BT}_grid_chunk_accept", BT, "grid_chunk_accept", MAP, "grid_chunk_accept")

    lidar_edges = (
        ("sample_valid", "lidar_sample_valid", 1),
        ("new_scan", "lidar_new_scan", 1),
        ("quality", "lidar_quality", 6),
        ("angle_q6", "lidar_angle_q6", 15),
        ("distance_q2", "lidar_distance_q2", 16),
    )
    for src, dst, size in lidar_edges:
        edge(f"{LIDAR}_{src}_{MAP}", LIDAR, src, MAP, dst, size)

    motion_edges = (
        ("calibration_done", "sensor_calibration_done", 1),
        ("fused_sample_valid", "fused_sample_valid", 1),
        ("fused_gyro_z", "fused_gyro_z", 16),
        ("flow_sample_valid", "flow_sample_valid", 1),
        ("flow_squal", "flow_squal", 8),
        ("flow_delta_x", "flow_delta_x", 16),
        ("flow_delta_y", "flow_delta_y", 16),
    )
    for src, dst, size in motion_edges:
        edge(f"{MOTION}_{src}_{MAP}", MOTION, src, MAP, dst, size)

    for port, size in (
        ("gyro_polarity_valid", 1),
        ("gyro_positive_is_right", 1),
        ("lf_speed", 11), ("rf_speed", 11), ("lr_speed", 11), ("rr_speed", 11),
    ):
        edge(f"{STABILIZER}_{port}_{MAP}", STABILIZER, port, MAP, port, size)

    # Flash owns map RAM port A while busy.
    edge(f"{FLASH}_flash_busy_{MAP}", FLASH, "flash_busy", MAP, "storage_busy")
    for src, dst, size in (
        ("map_mem_req", "storage_mem_req", 1),
        ("map_mem_we", "storage_mem_we", 1),
        ("map_mem_addr", "storage_mem_addr", 14),
        ("map_mem_wdata", "storage_mem_wdata", 4),
        ("load_apply_pulse", "storage_load_apply", 1),
        ("loaded_map_epoch", "storage_loaded_epoch", 8),
        ("loaded_pose_x_mm", "storage_loaded_pose_x_mm", 16),
        ("loaded_pose_y_mm", "storage_loaded_pose_y_mm", 16),
        ("loaded_pose_heading", "storage_loaded_heading", 10),
        ("loaded_scan_count", "storage_loaded_scan_count", 16),
        ("loaded_occupied_count", "storage_loaded_occupied_count", 16),
        ("loaded_free_count", "storage_loaded_free_count", 16),
    ):
        edge(f"{FLASH}_{src}_{MAP}", FLASH, src, MAP, dst, size)
    edge(f"{MAP}_storage_mem_rdata", MAP, "storage_mem_rdata", FLASH, "map_mem_rdata", 4)
    edge(f"{MAP}_storage_mem_rvalid", MAP, "storage_mem_rvalid", FLASH, "map_mem_rvalid")

    map_metadata = (
        ("map_epoch", "map_epoch", 8),
        ("pose_x_mm", "pose_x_mm", 16),
        ("pose_y_mm", "pose_y_mm", 16),
        ("pose_heading", "pose_heading", 10),
        ("scan_count", "scan_count", 16),
        ("occupied_count", "occupied_count", 16),
        ("free_count", "free_count", 16),
    )
    for src, dst, size in map_metadata:
        edge(f"{MAP}_{src}_{FLASH}", MAP, src, FLASH, dst, size)

    map_to_bt = (
        ("grid_chunk_valid", "grid_chunk_valid", 1),
        ("grid_chunk_seq", "grid_chunk_seq", 8),
        ("grid_chunk_base", "grid_chunk_base", 14),
        ("grid_chunk_data", "grid_chunk_data", 256),
        ("grid_chunk_crc", "grid_chunk_crc", 8),
        ("map_active", "onboard_map_active", 1),
        ("map_paused", "onboard_map_paused", 1),
        ("map_valid", "onboard_map_valid", 1),
        ("map_busy", "onboard_map_busy", 1),
        ("map_epoch", "onboard_map_epoch", 8),
        ("scan_count", "onboard_map_scan_count", 16),
        ("occupied_count", "onboard_map_occupied_count", 16),
        ("free_count", "onboard_map_free_count", 16),
        ("dropped_sample_count", "onboard_map_dropped_count", 16),
        ("pose_x_mm", "onboard_pose_x_mm", 16),
        ("pose_y_mm", "onboard_pose_y_mm", 16),
        ("pose_heading", "onboard_pose_heading", 10),
        ("pose_flags", "onboard_pose_flags", 8),
    )
    for src, dst, size in map_to_bt:
        edge(f"{MAP}_{src}_{BT}", MAP, src, BT, dst, size)

    flash_to_bt = (
        ("flash_ready", "flash_ready", 1),
        ("flash_busy", "flash_busy", 1),
        ("flash_operation", "flash_operation", 2),
        ("flash_progress", "flash_progress", 8),
        ("flash_error", "flash_error", 8),
        ("flash_jedec_id", "flash_jedec_id", 24),
        ("saved_sequence", "flash_saved_sequence", 32),
    )
    for src, dst, size in flash_to_bt:
        edge(f"{FLASH}_{src}_{BT}", FLASH, src, BT, dst, size)

    edges.extend(
        [
            (f"{FLASH}_flash_cs_n", f"{TOP}#{FLASH}&gt;flash_cs_n", f"{TOP}&gt;flash_cs_n", 1),
            (f"{FLASH}_flash_clk", f"{TOP}#{FLASH}&gt;flash_clk", f"{TOP}&gt;flash_clk", 1),
            (f"{FLASH}_flash_mosi", f"{TOP}#{FLASH}&gt;flash_mosi", f"{TOP}&gt;flash_mosi", 1),
            (f"{FLASH}_flash_wp_n", f"{TOP}#{FLASH}&gt;flash_wp_n", f"{TOP}&gt;flash_wp_n", 1),
            (f"{FLASH}_flash_hold_n", f"{TOP}#{FLASH}&gt;flash_hold_n", f"{TOP}&gt;flash_hold_n", 1),
            (f"{TOP}_flash_miso", f"{TOP}&gt;flash_miso", f"{TOP}#{FLASH}&gt;flash_miso", 1),
        ]
    )

    for name, source, destination, size in edges:
        text = add_wire(text, name, source, destination, size)
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")


def register_models() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    for name in ("onboard_occupancy_map.model", "map_flash_store.model"):
        text = re.sub(
            rf'^[ \t]*<{re.escape(name)}\b[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    marker = re.search(r"^[ \t]*</model>[ \t]*$", text, flags=re.M)
    if not marker:
        raise RuntimeError("project model folder closing tag is missing")
    entries = "".join(
        f'  <{name} name="{name}" parent="model" type="model"/>\n'
        for name in ("onboard_occupancy_map.model", "map_flash_store.model")
    )
    text = text[: marker.start()] + entries + text[marker.start() :]
    PROJECT.write_text(text, encoding="utf-8", newline="\n")


def child_port_line(model_port: ET.Element, child_parent: str) -> str:
    attributes = dict(model_port.attrib)
    attributes["Parent"] = child_parent
    ordered = " ".join(f'{key}="{html.escape(value, quote=True)}"' for key, value in attributes.items())
    return f" <Port {ordered} />"


def update_top_const() -> None:
    text = TOP_CONST.read_text(encoding="utf-8")
    top_root = ET.parse(TOP_MODEL).getroot()
    model_ports = {port.attrib["Name"]: port for port in top_root.findall("Port")}
    child_name = "sys_route_gui_fsm_top1"

    for name in (
        "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
        "flash_hold_n", "flash_miso"
    ):
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="{child_name}")'
            rf'(?=[^>]*\bName="{name}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    child_close = re.search(
        rf'(<Module\b(?=[^>]*\bName="{child_name}")[^>]*>.*?)(^[ \t]*</Module>)',
        text,
        flags=re.M | re.S,
    )
    if not child_close:
        raise RuntimeError("top.const child model is missing")
    additions = "\n".join(
        child_port_line(model_ports[name], child_name)
        for name in (
            "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
            "flash_hold_n", "flash_miso"
        )
    )
    replacement = child_close.group(1).rstrip() + "\n" + additions + "\n " + child_close.group(2).lstrip()
    text = text[: child_close.start()] + replacement + text[child_close.end() :]

    physical = {
        "E18": ("#00ffff", "8", "none", "right", "0.92"),
        "D16": ("#00ffff", "8", "none", "right", "0.89"),
        "C18": ("#00ffff", "8", "none", "right", "0.95"),
        "D17": ("#00ffff", "4", "none", "right", "0.86"),
        "J16": ("#00ffff", "4", "none", "right", "0.83"),
        "C17": ("#6495ed", "", "up", "left", "0.95"),
    }
    for pin in physical:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="top")'
            rf'(?=[^>]*\bName="{pin}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    first_root_wire = re.search(r'^[ \t]*<Wire\b(?=[^>]*\bParent="top")', text, re.M)
    if not first_root_wire:
        raise RuntimeError("top.const root wire insertion point is missing")
    pin_lines = []
    for index, (pin, (color, drive, pull, side, y)) in enumerate(physical.items()):
        x = "0.989166" if side == "right" else "-0.0108342"
        pin_lines.append(
            f' <Port Color="{color}" Description="" Drive="{drive}" Height="30" '
            f'Name="{pin}" Parent="top" Pull="{pull}" Side="{side}" '
            f'Voltage="LVCMOS33" Width="30" X="{x}" Y="{y}"/>'
        )
    text = text[: first_root_wire.start()] + "\n".join(pin_lines) + "\n" + text[first_root_wire.start() :]

    for token in (
        "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
        "flash_hold_n", "flash_miso"
    ):
        text = re.sub(
            rf'^[ \t]*<Wire\b(?=[^>]*(?:From|To)="[^"]*{token}[^"]*")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    wires = "\n".join(
        (
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_cs_n" Name="flash_cs_n_E18" Parent="top" To="top&gt;E18" />',
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_clk" Name="flash_clk_D16" Parent="top" To="top&gt;D16" />',
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_mosi" Name="flash_mosi_C18" Parent="top" To="top&gt;C18" />',
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_wp_n" Name="flash_wp_n_D17" Parent="top" To="top&gt;D17" />',
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_hold_n" Name="flash_hold_n_J16" Parent="top" To="top&gt;J16" />',
            f' <Wire Datasize="1" Datatype="wire" From="top#{child_name}&gt;flash_miso" Name="flash_miso_C17" Parent="top" To="top&gt;C17" />',
        )
    )
    text = text.rsplit("</Module>", 1)[0] + wires + "\n</Module>\n"

    root_tag = root_opening_tag(text)
    code_match = re.search(r'\bCode="([^"]*)"', root_tag)
    if not code_match:
        raise RuntimeError("top.const root Code attribute is missing")
    code = html.unescape(code_match.group(1)).replace("&#xa;", "\n")
    code_lines = [
        line for line in code.splitlines()
        if not any(token in line for token in (
            "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
            "flash_hold_n", "flash_miso"
        ))
    ]
    code_lines.extend(
        (
            "set_property PACKAGE_PIN E18 [get_ports flash_cs_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_cs_n]",
            "set_property DRIVE 8 [get_ports flash_cs_n]",
            "set_property SLEW SLOW [get_ports flash_cs_n]",
            "set_property PACKAGE_PIN D16 [get_ports flash_clk]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_clk]",
            "set_property DRIVE 8 [get_ports flash_clk]",
            "set_property SLEW SLOW [get_ports flash_clk]",
            "set_property PACKAGE_PIN C18 [get_ports flash_mosi]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_mosi]",
            "set_property DRIVE 8 [get_ports flash_mosi]",
            "set_property SLEW SLOW [get_ports flash_mosi]",
            "set_property PACKAGE_PIN D17 [get_ports flash_wp_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_wp_n]",
            "set_property DRIVE 4 [get_ports flash_wp_n]",
            "set_property SLEW SLOW [get_ports flash_wp_n]",
            "set_property PACKAGE_PIN J16 [get_ports flash_hold_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_hold_n]",
            "set_property DRIVE 4 [get_ports flash_hold_n]",
            "set_property SLEW SLOW [get_ports flash_hold_n]",
            "set_property PACKAGE_PIN C17 [get_ports flash_miso]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_miso]",
            "set_property PULLUP TRUE [get_ports flash_miso]",
        )
    )
    encoded = html.escape("\n".join(code_lines), quote=True).replace("\n", "&#xa;")
    new_root_tag = re.sub(r'\bCode="[^"]*"', f'Code="{encoded}"', root_tag, count=1)
    text = text.replace(root_tag, new_root_tag, 1)
    TOP_CONST.write_text(text, encoding="utf-8", newline="\n")


def update_build_inputs() -> None:
    text = FILELIST.read_text(encoding="utf-8")
    paths = [
        str((BUILD_DIR / "onboard_occupancy_map.v").resolve()).replace("\\", "/"),
        str((BUILD_DIR / "map_flash_store.v").resolve()).replace("\\", "/"),
    ]
    lines = [line for line in text.splitlines() if line.strip() not in paths]
    top_path = str((BUILD_DIR / "sys_route_gui_fsm_top.v").resolve()).replace("\\", "/")
    try:
        top_index = lines.index(top_path)
    except ValueError as exc:
        raise RuntimeError("filelist has no generated top") from exc
    lines[top_index:top_index] = paths
    FILELIST.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")

    xdc = TOP_XDC.read_text(encoding="utf-8")
    xdc_lines = [
        line for line in xdc.splitlines()
        if not any(token in line for token in (
            "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
            "flash_hold_n", "flash_miso"
        ))
    ]
    xdc_lines.extend(
        (
            "set_property PACKAGE_PIN E18 [get_ports flash_cs_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_cs_n]",
            "set_property DRIVE 8 [get_ports flash_cs_n]",
            "set_property SLEW SLOW [get_ports flash_cs_n]",
            "set_property PACKAGE_PIN D16 [get_ports flash_clk]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_clk]",
            "set_property DRIVE 8 [get_ports flash_clk]",
            "set_property SLEW SLOW [get_ports flash_clk]",
            "set_property PACKAGE_PIN C18 [get_ports flash_mosi]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_mosi]",
            "set_property DRIVE 8 [get_ports flash_mosi]",
            "set_property SLEW SLOW [get_ports flash_mosi]",
            "set_property PACKAGE_PIN D17 [get_ports flash_wp_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_wp_n]",
            "set_property DRIVE 4 [get_ports flash_wp_n]",
            "set_property SLEW SLOW [get_ports flash_wp_n]",
            "set_property PACKAGE_PIN J16 [get_ports flash_hold_n]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_hold_n]",
            "set_property DRIVE 4 [get_ports flash_hold_n]",
            "set_property SLEW SLOW [get_ports flash_hold_n]",
            "set_property PACKAGE_PIN C17 [get_ports flash_miso]",
            "set_property IOSTANDARD LVCMOS33 [get_ports flash_miso]",
            "set_property PULLUP TRUE [get_ports flash_miso]",
        )
    )
    TOP_XDC.write_text("\n".join(xdc_lines) + "\n", encoding="utf-8", newline="\n")

    for name in ("bt_route_uart.v", "onboard_occupancy_map.v", "map_flash_store.v"):
        source = BUILD_DIR / name
        (MIRROR_DIR / name).write_text(
            source.read_text(encoding="utf-8"), encoding="utf-8", newline="\n"
        )


def validate() -> None:
    top = ET.parse(TOP_MODEL).getroot()
    modules = {module.attrib.get("Name"): module for module in top.findall("Module")}
    expected_ports = {BT: 115, MAP: 58, FLASH: 38}
    for instance, count in expected_ports.items():
        module = modules.get(instance)
        if module is None or len(module.findall("Port")) != count:
            raise RuntimeError(f"{instance} port count is not {count}")
    root_ports = {port.attrib.get("Name") for port in top.findall("Port")}
    for name in (
        "flash_cs_n", "flash_clk", "flash_mosi", "flash_wp_n",
        "flash_hold_n", "flash_miso"
    ):
        if name not in root_ports:
            raise RuntimeError(f"top port {name} is missing")

    destinations = [wire.attrib.get("To") for wire in top.findall("Wire")]
    for destination in (
        f"{TOP}#{MAP}>map_start_pulse",
        f"{TOP}#{MAP}>storage_mem_req",
        f"{TOP}#{FLASH}>map_mem_rdata",
        f"{TOP}#{BT}>grid_chunk_valid",
        f"{TOP}#{BT}>flash_jedec_id",
    ):
        if destinations.count(destination) != 1:
            raise RuntimeError(f"expected one wire to {destination}")

    project = ET.parse(PROJECT).getroot()
    model_group = project.find("model")
    names = [child.attrib.get("name") for child in model_group] if model_group is not None else []
    for name in ("onboard_occupancy_map.model", "map_flash_store.model"):
        if names.count(name) != 1:
            raise RuntimeError(f"project registration for {name} is invalid")

    const = ET.parse(TOP_CONST).getroot()
    const_ports = {port.attrib.get("Name") for port in const.findall("Port")}
    for pin in ("E18", "D16", "C18", "D17", "J16", "C17"):
        if pin not in const_ports:
            raise RuntimeError(f"top.const pin {pin} is missing")


def main() -> None:
    integrate_top_model()
    register_models()
    update_top_const()
    update_build_inputs()
    validate()
    print("board mapping, Flash storage and BLE map graph integrated")


if __name__ == "__main__":
    main()
