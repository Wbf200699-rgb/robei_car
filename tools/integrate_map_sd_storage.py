#!/usr/bin/env python3
"""Replace the legacy user-Flash map store with the onboard SD hierarchy."""

from __future__ import annotations

import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAR_DIR = ROOT / "little_car"
MODEL_PATH = CAR_DIR / "model" / "sys_route_gui_fsm_top.model"
SD_MODEL_PATH = CAR_DIR / "model" / "map_sd_store.model"
CONST_PATH = CAR_DIR / "const" / "top.const"
SD_CONST_PATH = CAR_DIR / "const" / "sd_readonly_probe.const"

OLD_INSTANCE = "map_flash_store31"
NEW_INSTANCE = "map_sd_store31"
OLD_PORTS = (
    "flash_cs_n",
    "flash_clk",
    "flash_mosi",
    "flash_wp_n",
    "flash_hold_n",
    "flash_miso",
)
SD_PORTS = (
    "sd_cd",
    "sd_d1",
    "sd_d2",
    "sd_miso",
    "sd_cs_n",
    "sd_sclk",
    "sd_mosi",
)
SD_PIN_MAP = {
    "sd_sclk": "E13",
    "sd_mosi": "E14",
    "sd_miso": "F14",
    "sd_cs_n": "F16",
    "sd_cd": "G13",
    "sd_d1": "F13",
    "sd_d2": "E17",
}
OLD_PIN_NAMES = ("E18", "D16", "C18", "D17", "J16", "C17")


def read_xml(path: Path) -> ET.Element:
    text = re.sub(r"<!DOCTYPE[^>]*>", "", path.read_text(encoding="utf-8"))
    return ET.fromstring(text)


def write_xml(path: Path, root: ET.Element) -> None:
    ET.indent(root, space=" ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    path.write_text(f"<!DOCTYPE Robei>\n{xml}\n", encoding="utf-8", newline="\n")


def direct_child_by_name(root: ET.Element, tag: str, name: str) -> ET.Element | None:
    return next((item for item in root.findall(tag) if item.get("Name") == name), None)


def insert_before_first_module(root: ET.Element, element: ET.Element) -> None:
    children = list(root)
    index = next(
        (idx for idx, child in enumerate(children) if child.tag == "Module"),
        len(children),
    )
    root.insert(index, element)


def renamed_sd_instance(source: ET.Element, old: ET.Element, parent: str) -> ET.Element:
    instance = copy.deepcopy(source)
    old_root_name = instance.get("Name", "map_sd_store")
    instance.set("Class", "map_sd_store")
    instance.set("Name", NEW_INSTANCE)
    instance.set("Parent", parent)
    for geometry in ("X", "Y", "Width", "Height", "Color"):
        if old.get(geometry) is not None:
            instance.set(geometry, old.get(geometry, ""))
    for port in instance.findall("Port"):
        port.set("Parent", NEW_INSTANCE)
    for child in instance.findall("Module"):
        child.set("Parent", NEW_INSTANCE)
    for wire in instance.findall("Wire"):
        wire.set("Parent", NEW_INSTANCE)
        for endpoint_name in ("From", "To"):
            value = wire.get(endpoint_name, "")
            if value.startswith(old_root_name + ">") or value.startswith(
                old_root_name + "#"
            ):
                wire.set(endpoint_name, NEW_INSTANCE + value[len(old_root_name) :])
    return instance


def replace_top_ports(top: ET.Element, old_ports: dict[str, ET.Element]) -> None:
    for port in list(top.findall("Port")):
        if port.get("Name") in OLD_PORTS or port.get("Name") in SD_PORTS:
            top.remove(port)

    geometry_source = {
        "sd_cs_n": ("flash_cs_n", "output", "right", "0.993941", "0.833192"),
        "sd_sclk": ("flash_clk", "output", "bottom", "0.90", "0.994"),
        "sd_mosi": ("flash_mosi", "output", "right", "0.993941", "0.873192"),
        "sd_miso": ("flash_miso", "input", "left", "-0.00605938", "0.823192"),
        "sd_cd": ("flash_miso", "input", "left", "-0.00605938", "0.853192"),
        "sd_d1": ("flash_miso", "input", "left", "-0.00605938", "0.883192"),
        "sd_d2": ("flash_miso", "input", "left", "-0.00605938", "0.913192"),
    }
    for name in SD_PORTS:
        template_name, direction, side, x_value, y_value = geometry_source[name]
        template = old_ports.get(template_name) or old_ports.get(name)
        if template is None:
            raise RuntimeError(f"no geometry template is available for {name}")
        port = copy.deepcopy(template)
        port.set("Name", name)
        port.set("Parent", top.get("Name", "sys_route_gui_fsm_top"))
        port.set("Inout", direction)
        port.set("Datatype", "wire")
        port.set("Side", side)
        port.set("X", x_value)
        port.set("Y", y_value)
        insert_before_first_module(top, port)


def integrate_model() -> None:
    top = read_xml(MODEL_PATH)
    sd_model = read_xml(SD_MODEL_PATH)
    old_ports = {port.get("Name", ""): copy.deepcopy(port) for port in top.findall("Port")}
    old_instance = direct_child_by_name(top, "Module", OLD_INSTANCE)
    if old_instance is None:
        old_instance = direct_child_by_name(top, "Module", NEW_INSTANCE)
    if old_instance is None:
        raise RuntimeError("map storage instance is missing from the vehicle top model")

    children = list(top)
    storage_index = children.index(old_instance)
    top.remove(old_instance)
    top.insert(
        storage_index,
        renamed_sd_instance(sd_model, old_instance, top.get("Name", "")),
    )
    replace_top_ports(top, old_ports)

    for wire in list(top.findall("Wire")):
        source = wire.get("From", "").replace(OLD_INSTANCE, NEW_INSTANCE)
        destination = wire.get("To", "").replace(OLD_INSTANCE, NEW_INSTANCE)
        wire.set("From", source)
        wire.set("To", destination)
        if any(
            source.endswith(">" + port_name) or destination.endswith(">" + port_name)
            for port_name in OLD_PORTS + SD_PORTS
        ):
            top.remove(wire)

    top_name = top.get("Name", "sys_route_gui_fsm_top")
    physical_wires = [
        (f"{top_name}>sd_cd", f"{top_name}#{NEW_INSTANCE}>sd_cd", 1),
        (f"{top_name}>sd_d1", f"{top_name}#{NEW_INSTANCE}>sd_d1", 1),
        (f"{top_name}>sd_d2", f"{top_name}#{NEW_INSTANCE}>sd_d2", 1),
        (f"{top_name}>sd_miso", f"{top_name}#{NEW_INSTANCE}>sd_miso", 1),
        (f"{top_name}#{NEW_INSTANCE}>sd_cs_n", f"{top_name}>sd_cs_n", 1),
        (f"{top_name}#{NEW_INSTANCE}>sd_sclk", f"{top_name}>sd_sclk", 1),
        (f"{top_name}#{NEW_INSTANCE}>sd_mosi", f"{top_name}>sd_mosi", 1),
    ]
    for source, destination, size in physical_wires:
        ET.SubElement(
            top,
            "Wire",
            {
                "Datasize": str(size),
                "Datatype": "wire",
                "From": source,
                "Name": source.split(">")[-1].replace("#", "_"),
                "Parent": top_name,
                "To": destination,
            },
        )

    destinations = [wire.get("To", "") for wire in top.findall("Wire")]
    duplicates = sorted({item for item in destinations if destinations.count(item) > 1})
    if duplicates:
        raise RuntimeError(f"multiple drivers after SD integration: {duplicates}")
    write_xml(MODEL_PATH, top)


def integrate_const() -> None:
    root = read_xml(CONST_PATH)
    sd_reference = read_xml(SD_CONST_PATH)
    top_model = read_xml(MODEL_PATH)
    child = root.find("Module")
    if child is None:
        raise RuntimeError("top.const has no constrained module instance")

    for port in list(child.findall("Port")):
        if port.get("Name") in OLD_PORTS or port.get("Name") in SD_PORTS:
            child.remove(port)
    top_sd_ports = {
        port.get("Name", ""): port
        for port in top_model.findall("Port")
        if port.get("Name") in SD_PORTS
    }
    for name in SD_PORTS:
        port = copy.deepcopy(top_sd_ports[name])
        port.set("Parent", child.get("Name", "sys_route_gui_fsm_top1"))
        child.append(port)

    for port in list(root.findall("Port")):
        if port.get("Name") in OLD_PIN_NAMES or port.get("Name") in SD_PIN_MAP.values():
            root.remove(port)
    reference_pins = {port.get("Name", ""): port for port in sd_reference.findall("Port")}
    for pin_name in SD_PIN_MAP.values():
        pin = copy.deepcopy(reference_pins[pin_name])
        pin.set("Parent", root.get("Name", "top"))
        root.insert(0, pin)

    for wire in list(root.findall("Wire")):
        endpoints = wire.get("From", "") + " " + wire.get("To", "")
        if any(port_name in endpoints for port_name in OLD_PORTS) or any(
            f">{pin_name}" in endpoints for pin_name in OLD_PIN_NAMES
        ):
            root.remove(wire)
        elif any(port_name in endpoints for port_name in SD_PORTS):
            root.remove(wire)

    root_name = root.get("Name", "top")
    child_name = child.get("Name", "sys_route_gui_fsm_top1")
    for signal_name, pin_name in SD_PIN_MAP.items():
        ET.SubElement(
            root,
            "Wire",
            {
                "Datasize": "1",
                "Datatype": "wire",
                "From": f"{root_name}#{child_name}>{signal_name}",
                "Name": f"{signal_name}_{pin_name}",
                "Parent": root_name,
                "To": f"{root_name}>{pin_name}",
            },
        )

    old_code_lines = root.get("Code", "").splitlines()
    root.set(
        "Code",
        "\n".join(
            line
            for line in old_code_lines
            if not any(f"get_ports {name}" in line for name in OLD_PORTS)
        ),
    )
    write_xml(CONST_PATH, root)


def main() -> None:
    integrate_model()
    integrate_const()
    print(
        "integrated map_sd_store31 and SD pins: "
        + ", ".join(f"{signal}={pin}" for signal, pin in SD_PIN_MAP.items())
    )


if __name__ == "__main__":
    main()
