#!/usr/bin/env python3
"""Generate the graphical Robei hierarchy for the SD-backed map store."""

from __future__ import annotations

import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from robei_converter import verilog_to_model


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
RTL_DIR = ROOT / "little_car" / "hdmi_rgb_input"


def read_model(path: Path) -> ET.Element:
    text = re.sub(r"<!DOCTYPE[^>]*>", "", path.read_text(encoding="utf-8"))
    return ET.fromstring(text)


def write_model(path: Path, root: ET.Element) -> None:
    ET.indent(root, space=" ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    path.write_text(f"<!DOCTYPE Robei>\n{xml}\n", encoding="utf-8", newline="\n")


def make_instance(
    source: ET.Element,
    class_name: str,
    instance_name: str,
    parent_name: str,
    x: int,
    y: int,
    width: int,
    height: int,
) -> ET.Element:
    instance = copy.deepcopy(source)
    instance.set("Class", class_name)
    instance.set("Name", instance_name)
    instance.set("Parent", parent_name)
    instance.set("X", str(x))
    instance.set("Y", str(y))
    instance.set("Width", str(width))
    instance.set("Height", str(height))
    for port in instance.findall("Port"):
        port.set("Parent", instance_name)
    return instance


def endpoint(module_name: str, instance_name: str | None, port_name: str) -> str:
    if instance_name is None:
        return f"{module_name}>{port_name}"
    return f"{module_name}#{instance_name}>{port_name}"


def main() -> None:
    interface_xml = verilog_to_model(
        (RTL_DIR / "map_sd_store.v").read_text(encoding="utf-8")
    )
    interface = ET.fromstring(re.sub(r"<!DOCTYPE[^>]*>", "", interface_xml))
    controller = read_model(MODEL_DIR / "map_sd_store_controller.model")
    block_device = read_model(MODEL_DIR / "sd_spi_block_device.model")
    card_detect = read_model(MODEL_DIR / "sd_card_detect_adapter.model")

    module_name = "map_sd_store"
    root_attributes = copy.deepcopy(interface.attrib)
    root_attributes.update(
        {
            "Class": "module",
            "Code": "",
            "Name": module_name,
            "Parent": "0",
            "Width": "3600",
            "Height": "1900",
            "X": "0",
            "Y": "0",
        }
    )
    root = ET.Element("Module", root_attributes)
    for source_port in interface.findall("Port"):
        port = copy.deepcopy(source_port)
        port.set("Parent", module_name)
        if port.get("Inout") == "output":
            port.set("Datatype", "wire")
        root.append(port)

    controller_name = "map_sd_store_controller0"
    block_name = "sd_spi_block_device0"
    detect_name = "sd_card_detect_adapter0"
    root.append(
        make_instance(
            controller,
            "map_sd_store_controller",
            controller_name,
            module_name,
            700,
            180,
            1450,
            1450,
        )
    )
    root.append(
        make_instance(
            block_device,
            "sd_spi_block_device",
            block_name,
            module_name,
            2450,
            180,
            950,
            1250,
        )
    )
    root.append(
        make_instance(
            card_detect,
            "sd_card_detect_adapter",
            detect_name,
            module_name,
            2450,
            1500,
            950,
            260,
        )
    )

    top_inputs_to_controller = [
        ("clk", 1),
        ("rst_n", 1),
        ("save_pulse", 1),
        ("load_pulse", 1),
        ("map_epoch", 8),
        ("pose_x_mm", 16),
        ("pose_y_mm", 16),
        ("pose_heading", 10),
        ("scan_count", 16),
        ("occupied_count", 16),
        ("free_count", 16),
        ("map_mem_rdata", 4),
        ("map_mem_rvalid", 1),
    ]
    controller_outputs_to_top = [
        ("map_mem_req", 1),
        ("map_mem_we", 1),
        ("map_mem_addr", 14),
        ("map_mem_wdata", 4),
        ("load_apply_pulse", 1),
        ("loaded_map_epoch", 8),
        ("loaded_pose_x_mm", 16),
        ("loaded_pose_y_mm", 16),
        ("loaded_pose_heading", 10),
        ("loaded_scan_count", 16),
        ("loaded_occupied_count", 16),
        ("loaded_free_count", 16),
        ("flash_ready", 1),
        ("flash_busy", 1),
        ("flash_operation", 2),
        ("flash_progress", 8),
        ("flash_error", 8),
        ("flash_jedec_id", 24),
        ("saved_sequence", 32),
    ]
    connections: list[tuple[str, str, int, str]] = []
    for port_name, size in top_inputs_to_controller:
        connections.append(
            (
                endpoint(module_name, None, port_name),
                endpoint(module_name, controller_name, port_name),
                size,
                f"top_{port_name}_controller",
            )
        )
    for port_name, size in controller_outputs_to_top:
        connections.append(
            (
                endpoint(module_name, controller_name, port_name),
                endpoint(module_name, None, port_name),
                size,
                f"controller_{port_name}_top",
            )
        )

    for port_name in ("clk", "rst_n"):
        connections.append(
            (
                endpoint(module_name, None, port_name),
                endpoint(module_name, block_name, port_name),
                1,
                f"top_{port_name}_block",
            )
        )
    connections.extend(
        [
            (endpoint(module_name, None, "sd_miso"), endpoint(module_name, block_name, "sd_miso"), 1, "sd_miso"),
            (endpoint(module_name, block_name, "sd_cs_n"), endpoint(module_name, None, "sd_cs_n"), 1, "sd_cs_n"),
            (endpoint(module_name, block_name, "sd_sclk"), endpoint(module_name, None, "sd_sclk"), 1, "sd_sclk"),
            (endpoint(module_name, block_name, "sd_mosi"), endpoint(module_name, None, "sd_mosi"), 1, "sd_mosi"),
            (endpoint(module_name, controller_name, "sd_init_start"), endpoint(module_name, block_name, "init_start"), 1, "sd_init_start"),
            (endpoint(module_name, controller_name, "sd_block_start"), endpoint(module_name, block_name, "block_start"), 1, "sd_block_start"),
            (endpoint(module_name, controller_name, "sd_block_write"), endpoint(module_name, block_name, "block_write"), 1, "sd_block_write"),
            (endpoint(module_name, controller_name, "sd_block_lba"), endpoint(module_name, block_name, "block_lba"), 32, "sd_block_lba"),
            (endpoint(module_name, controller_name, "sd_buffer_read_data"), endpoint(module_name, block_name, "buffer_read_data"), 8, "sd_buffer_read_data"),
            (endpoint(module_name, block_name, "initialized"), endpoint(module_name, controller_name, "sd_initialized"), 1, "sd_initialized"),
            (endpoint(module_name, block_name, "init_done"), endpoint(module_name, controller_name, "sd_init_done"), 1, "sd_init_done"),
            (endpoint(module_name, block_name, "block_busy"), endpoint(module_name, controller_name, "sd_block_busy"), 1, "sd_block_busy"),
            (endpoint(module_name, block_name, "block_done"), endpoint(module_name, controller_name, "sd_block_done"), 1, "sd_block_done"),
            (endpoint(module_name, block_name, "error_code"), endpoint(module_name, controller_name, "sd_error_code"), 8, "sd_error_code"),
            (endpoint(module_name, block_name, "last_r1"), endpoint(module_name, controller_name, "sd_last_r1"), 8, "sd_last_r1"),
            (endpoint(module_name, block_name, "write_data_response"), endpoint(module_name, controller_name, "sd_write_data_response"), 8, "sd_write_data_response"),
            (endpoint(module_name, block_name, "high_capacity"), endpoint(module_name, controller_name, "sd_high_capacity"), 1, "sd_high_capacity"),
            (endpoint(module_name, block_name, "buffer_addr"), endpoint(module_name, controller_name, "sd_buffer_addr"), 9, "sd_buffer_addr"),
            (endpoint(module_name, block_name, "buffer_write_enable"), endpoint(module_name, controller_name, "sd_buffer_write_enable"), 1, "sd_buffer_write_enable"),
            (endpoint(module_name, block_name, "buffer_write_data"), endpoint(module_name, controller_name, "sd_buffer_write_data"), 8, "sd_buffer_write_data"),
        ]
    )

    for port_name in ("sd_cd", "sd_d1", "sd_d2"):
        connections.append(
            (
                endpoint(module_name, None, port_name),
                endpoint(module_name, detect_name, port_name),
                1,
                port_name,
            )
        )
    connections.append(
        (
            endpoint(module_name, detect_name, "sd_card_present"),
            endpoint(module_name, controller_name, "sd_card_present"),
            1,
            "sd_card_present",
        )
    )

    for source, destination, size, name in connections:
        ET.SubElement(
            root,
            "Wire",
            {
                "Datasize": str(size),
                "Datatype": "wire",
                "From": source,
                "Name": name,
                "Parent": module_name,
                "To": destination,
            },
        )

    write_model(MODEL_DIR / "map_sd_store.model", root)
    print(
        "generated map_sd_store.model: "
        f"{len(root.findall('Module'))} blocks, {len(root.findall('Wire'))} wires"
    )


if __name__ == "__main__":
    main()
