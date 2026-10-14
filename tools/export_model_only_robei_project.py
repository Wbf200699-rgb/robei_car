#!/usr/bin/env python3
"""Export the current car design as a native RobeiEDA source project.

The exported root contains only RobeiEDA's four source folders: model, test,
const and verilog. Registered tests are preserved; RobeiEDA creates build
outputs itself when the project is compiled.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from robei_converter import model_to_verilog


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "little_car"
DEFAULT_OUTPUT = ROOT / "robei_model_project"


def project_group(root: ET.Element, name: str) -> ET.Element:
    group = root.find(name)
    if group is None:
        raise RuntimeError(f"source project has no {name!r} group")
    return group


def registered_names(group: ET.Element, suffix: str) -> list[str]:
    names: list[str] = []
    for item in group:
        name = item.get("name", "")
        if not name.endswith(suffix):
            raise RuntimeError(f"unexpected registered {group.tag} item: {name!r}")
        names.append(name)
    if len(names) != len(set(names)):
        raise RuntimeError(f"duplicate entries in source {group.tag} group")
    return names


def resolve_model_dependencies(source: Path,
                               initial_names: list[str]) -> list[str]:
    """Include child models referenced by graphical Module instances.

    Older projects can leave a referenced child model on disk without listing
    it in the source .proj. RobeiEDA still resolves that child by Module Class,
    so a standalone export must register the complete dependency closure.
    """
    model_dir = source / "model"
    available = {path.stem: path.name for path in model_dir.glob("*.model")}
    resolved = list(initial_names)
    seen = set(resolved)
    index = 0
    while index < len(resolved):
        name = resolved[index]
        index += 1
        text = (model_dir / name).read_text(encoding="utf-8")
        root = ET.fromstring(text.replace("<!DOCTYPE Robei>", "", 1))
        for instance in root.findall(".//Module"):
            dependency = available.get(instance.get("Class", ""))
            if dependency is not None and dependency not in seen:
                seen.add(dependency)
                resolved.append(dependency)
    return resolved


def resolve_verilog_support(source: Path, root_model: str) -> list[str]:
    """Find Code-level model instances invisible to Robei's graph walker."""
    model_dir = source / "model"
    available = {path.stem: path for path in model_dir.glob("*.model")}

    def parse(stem: str) -> ET.Element:
        text = available[stem].read_text(encoding="utf-8")
        return ET.fromstring(text.replace("<!DOCTYPE Robei>", "", 1))

    graphical = {root_model}
    pending = [root_model]
    while pending:
        stem = pending.pop()
        for instance in parse(stem).findall(".//Module"):
            dependency = instance.get("Class", "")
            if dependency in available and dependency not in graphical:
                graphical.add(dependency)
                pending.append(dependency)

    support: set[str] = set()
    pending = list(graphical)
    scanned: set[str] = set()
    while pending:
        stem = pending.pop()
        if stem in scanned:
            continue
        scanned.add(stem)
        code = parse(stem).get("Code", "")
        for dependency in available:
            pattern = (r"\b" + re.escape(dependency) +
                       r"\s+(?:#\s*\([^;]*?\)\s*)?"
                       r"[A-Za-z_$][\w$]*\s*\(")
            if (dependency != stem and re.search(pattern, code, re.S)):
                if dependency not in graphical:
                    support.add(dependency)
                    pending.append(dependency)
    return [f"{name}.v" for name in sorted(support)]


def write_project(path: Path, project_name: str,
                  model_names: list[str], test_names: list[str],
                  const_names: list[str],
                  verilog_names: list[str]) -> None:
    root = ET.Element(project_name, {
        "name": project_name,
        "parent": "0",
        "type": "proj",
    })
    model_group = ET.SubElement(root, "model", {
        "name": "model",
        "parent": project_name,
        "type": "folder",
    })
    for name in model_names:
        ET.SubElement(model_group, name, {
            "name": name,
            "parent": "model",
            "type": "model",
        })
    test_group = ET.SubElement(root, "test", {
        "name": "test",
        "parent": project_name,
        "type": "folder",
    })
    for name in test_names:
        ET.SubElement(test_group, name, {
            "name": name,
            "parent": "test",
            "type": "test",
        })
    const_group = ET.SubElement(root, "const", {
        "name": "const",
        "parent": project_name,
        "type": "folder",
    })
    for name in const_names:
        ET.SubElement(const_group, name, {
            "name": name,
            "parent": "const",
            "type": "const",
        })
    verilog_group = ET.SubElement(root, "verilog", {
        "name": "verilog",
        "parent": project_name,
        "type": "folder",
    })
    for name in verilog_names:
        ET.SubElement(verilog_group, name, {
            "name": name,
            "parent": "verilog",
            "type": "verilog",
        })

    ET.indent(root, space=" ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n",
                    encoding="utf-8", newline="\n")


def validate_export(output: Path, project_file: Path,
                    model_names: list[str], test_names: list[str],
                    const_names: list[str],
                    verilog_names: list[str]) -> None:

    exported_root = ET.parse(project_file).getroot()
    group_order = [child.tag for child in exported_root]
    if group_order != ["model", "test", "const", "verilog"]:
        raise RuntimeError(f"non-native project folder order: {group_order}")
    exported_models = registered_names(project_group(exported_root, "model"), ".model")
    exported_tests = registered_names(project_group(exported_root, "test"), ".test")
    exported_consts = registered_names(project_group(exported_root, "const"), ".const")
    exported_verilog = registered_names(
        project_group(exported_root, "verilog"), ".v")
    if (exported_models != model_names or exported_tests != test_names or
            exported_consts != const_names or
            exported_verilog != verilog_names):
        raise RuntimeError("exported project registration differs from source")

    disk_verilog = sorted(path.name for path in (output / "verilog").glob("*.v"))
    if disk_verilog != sorted(verilog_names):
        raise RuntimeError("exported Verilog support differs from registration")

    for name in model_names:
        path = output / "model" / name
        if not path.is_file():
            raise RuntimeError(f"missing exported model: {name}")
        text = path.read_text(encoding="utf-8")
        model_root = ET.fromstring(text.replace("<!DOCTYPE Robei>", "", 1))
        # The local converter intentionally supports only a subset of Robei's
        # graphical endpoint conventions. Validate behavioral leaves through
        # an in-memory conversion; RobeiEDA remains authoritative for graphical
        # structural models such as the HDMI serializer hierarchy.
        if not model_root.findall("Module"):
            generated = model_to_verilog(text)
            if not generated.lstrip().startswith("module "):
                raise RuntimeError(f"model conversion produced no module: {name}")

    for name in const_names:
        const_path = output / "const" / name
        if not const_path.is_file():
            raise RuntimeError(f"missing exported constraint: {name}")
        ET.fromstring(const_path.read_text(encoding="utf-8")
                     .replace("<!DOCTYPE Robei>", "", 1))

    for name in test_names:
        test_path = output / "test" / name
        if not test_path.is_file():
            raise RuntimeError(f"missing exported test: {name}")
        test_root = ET.fromstring(test_path.read_text(encoding="utf-8")
                                  .replace("<!DOCTYPE Robei>", "", 1))
        if test_root.get("Type") != "test":
            raise RuntimeError(f"exported test has wrong type: {name}")

    expected_dirs = {"model", "test", "const", "verilog"}
    actual_dirs = {path.name for path in output.iterdir() if path.is_dir()}
    if actual_dirs != expected_dirs:
        raise RuntimeError(f"non-native project directories: {sorted(actual_dirs)}")

    top_model = ET.parse(output / "model" / "sys_route_gui_fsm_top.model").getroot()
    top_const = ET.parse(output / "const" / "top.const").getroot()
    constrained_instance = top_const.find("Module")
    if constrained_instance is None:
        raise RuntimeError("top.const has no constrained model instance")
    if constrained_instance.get("Class") != top_model.get("Name"):
        raise RuntimeError("top.const does not instantiate the vehicle top model")
    model_ports = {port.get("Name") for port in top_model.findall("Port")}
    const_ports = {port.get("Name") for port in constrained_instance.findall("Port")}
    if model_ports != const_ports:
        raise RuntimeError("top.const ports do not mirror the vehicle top model")


def export(source: Path, output: Path) -> Path:
    source_project = source / "little_car.proj"
    source_root = ET.parse(source_project).getroot()
    model_names = resolve_model_dependencies(
        source,
        registered_names(project_group(source_root, "model"), ".model"),
    )
    const_names = registered_names(project_group(source_root, "const"), ".const")
    # The native handoff project carries the maintained compact parameter
    # suite, not the older interactive/abandoned benches still registered in
    # the development project for historical reference.
    test_names = [name for name in registered_names(
        project_group(source_root, "test"), ".test")
                  if name.endswith("_parameter.test")]
    verilog_names = resolve_verilog_support(source, "sys_route_gui_fsm_top")

    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    (output / "model").mkdir(parents=True)
    (output / "test").mkdir()
    (output / "const").mkdir()
    (output / "verilog").mkdir()

    for name in model_names:
        shutil.copy2(source / "model" / name, output / "model" / name)
    for name in const_names:
        source_const = source / "const" / name
        target_const = output / "const" / name
        shutil.copy2(source_const, target_const)
    for name in test_names:
        shutil.copy2(source / "test" / name, output / "test" / name)
    for name in verilog_names:
        model_name = f"{Path(name).stem}.model"
        model_text = (source / "model" / model_name).read_text(encoding="utf-8")
        generated = model_to_verilog(model_text)
        (output / "verilog" / name).write_text(
            generated, encoding="utf-8", newline="\n")

    project_name = output.name
    project_file = output / f"{project_name}.proj"
    write_project(project_file, project_name, model_names, test_names, const_names,
                  verilog_names)
    validate_export(output, project_file, model_names, test_names, const_names,
                    verilog_names)
    return project_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        project_file = export(args.source.resolve(), args.output.resolve())
    except Exception as exc:  # concise command-line failure
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    print(f"[OK] model-only Robei project: {project_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
