#!/usr/bin/env python3
"""Wrap one Robei leaf model in a graphical one-block structural model."""

import argparse
import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def read_model(path: Path) -> ET.Element:
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"<!DOCTYPE[^>]*>", "", text)
    root = ET.fromstring(text)
    if root.findall("Module") or root.findall("Wire"):
        raise ValueError("input must be a leaf Robei model")
    return root


def write_model(path: Path, root: ET.Element) -> None:
    ET.indent(root, space=" ")
    path.parent.mkdir(parents=True, exist_ok=True)
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    path.write_text(f"<!DOCTYPE Robei>\n{xml}\n", encoding="utf-8")


def make_models(source: ET.Element, wrapper_name: str, implementation_name: str):
    implementation = copy.deepcopy(source)
    implementation.set("Class", "module")
    implementation.set("Name", implementation_name)
    implementation.set("Parent", "0")
    for port in implementation.findall("Port"):
        port.set("Parent", implementation_name)

    wrapper_attributes = copy.deepcopy(source.attrib)
    wrapper_attributes.update({
        "Class": "module",
        "Code": "",
        "Name": wrapper_name,
        "Parent": "0",
        "Width": "1800",
        "Height": "1080",
        "X": "0",
        "Y": "0",
    })
    wrapper = ET.Element("Module", wrapper_attributes)

    root_ports = []
    for source_port in source.findall("Port"):
        port = copy.deepcopy(source_port)
        port.set("Parent", wrapper_name)
        if port.get("Inout") == "output":
            port.set("Datatype", "wire")
        wrapper.append(port)
        root_ports.append(port)

    instance_name = f"{implementation_name}0"
    child = copy.deepcopy(implementation)
    child.set("Class", implementation_name)
    child.set("Name", instance_name)
    child.set("Parent", wrapper_name)
    child.set("Width", "900")
    child.set("Height", "700")
    child.set("X", "450")
    child.set("Y", "190")
    for port in child.findall("Port"):
        port.set("Parent", instance_name)
    wrapper.append(child)

    for port in root_ports:
        name = port.get("Name")
        direction = port.get("Inout", "input")
        root_endpoint = f"{wrapper_name}>{name}"
        child_endpoint = f"{wrapper_name}#{instance_name}>{name}"
        if direction == "output":
            source_endpoint, destination_endpoint = child_endpoint, root_endpoint
        else:
            source_endpoint, destination_endpoint = root_endpoint, child_endpoint
        ET.SubElement(wrapper, "Wire", {
            "Datasize": port.get("Datasize", "1"),
            "Datatype": "wire",
            "From": source_endpoint,
            "Name": f"{wrapper_name}_{name}",
            "Parent": wrapper_name,
            "To": destination_endpoint,
        })

    return implementation, wrapper


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--wrapper-name", required=True)
    parser.add_argument("--implementation-name", required=True)
    parser.add_argument("--wrapper-output", required=True, type=Path)
    parser.add_argument("--implementation-output", required=True, type=Path)
    args = parser.parse_args()

    source = read_model(args.input)
    implementation, wrapper = make_models(
        source, args.wrapper_name, args.implementation_name
    )
    write_model(args.implementation_output, implementation)
    write_model(args.wrapper_output, wrapper)


if __name__ == "__main__":
    main()
