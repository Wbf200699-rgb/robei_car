#!/usr/bin/env python3
"""Export the Robei project into a readable, self-contained Verilog tree."""

from __future__ import annotations

import csv
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from source_annotations import (
    TESTBENCH_DOCS,
    annotate_body,
    module_doc,
    port_doc,
)


HERE = Path(__file__).resolve().parent
PROJECT_DIR = HERE.parent
PROJECT_FILE = PROJECT_DIR / "little_car.proj"
MODEL_DIR = PROJECT_DIR / "model"
TEST_VERILOG_DIR = PROJECT_DIR / "test" / "verilog"
VERILOG_DIR = HERE / "verilog"
TESTBENCH_DIR = HERE / "testbenches"
MANIFEST_FILE = HERE / "module_manifest.csv"
FILELIST_FILE = HERE / "all_sources.vf"


def clean_identifier(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_$]", "_", value)
    if not value or value[0].isdigit():
        value = "n_" + value
    return value


def load_xml(path: Path) -> ET.Element:
    text = path.read_text(encoding="utf-8-sig")
    text = re.sub(r"<!DOCTYPE[^>]*>", "", text)
    return ET.fromstring(text)


def project_model_paths() -> list[Path]:
    root = load_xml(PROJECT_FILE)
    model_folder = root.find("model")
    if model_folder is None:
        raise RuntimeError("little_car.proj has no model folder")
    return [MODEL_DIR / node.get("name", "") for node in model_folder]


def width_decl(size: int) -> str:
    return f"[{size - 1}:0] " if size > 1 else ""


def endpoint_parts(endpoint: str, top_name: str) -> tuple[str, str]:
    owner, port = endpoint.rsplit(">", 1)
    prefix = top_name + "#"
    if owner == top_name:
        return "", port
    if owner.startswith(prefix):
        return owner[len(prefix):], port
    raise ValueError(f"Unexpected endpoint {endpoint!r} in {top_name}")


def export_model(path: Path, source_scope: str) -> dict[str, object]:
    root = load_xml(path)
    module_name = root.get("Name") or path.stem
    root_ports = list(root.findall("Port"))
    instances = list(root.findall("Module"))
    instance_classes = {
        instance.get("Name", ""): instance.get("Class", "")
        for instance in instances
    }
    wires = list(root.findall("Wire"))
    root_code = (root.get("Code") or "").strip("\n")
    procedural_code = re.sub(r"(?m)^\s*assign\b[^;]*;", "", root_code)

    top_port_names = {p.get("Name", "") for p in root_ports}
    grouped_wires: dict[str, list[ET.Element]] = defaultdict(list)
    for wire in wires:
        grouped_wires[wire.get("From", "")].append(wire)

    endpoint_to_net: dict[str, str] = {}
    internal_nets: dict[str, tuple[int, str]] = {}
    internal_net_routes: dict[str, list[ET.Element]] = {}
    used_nets = set(top_port_names)

    for source, group in grouped_wires.items():
        top_candidates = []
        for wire in group:
            for endpoint in (wire.get("From", ""), wire.get("To", "")):
                owner, port = endpoint_parts(endpoint, module_name)
                if not owner:
                    top_candidates.append(port)

        if top_candidates:
            net_name = top_candidates[0]
        else:
            base = clean_identifier(group[0].get("Name", "net"))
            net_name = base
            suffix = 2
            while net_name in used_nets:
                net_name = f"{base}_{suffix}"
                suffix += 1
            size = int(group[0].get("Datasize", "1"))
            dtype = group[0].get("Datatype", "wire") or "wire"
            internal_nets[net_name] = (size, dtype)
            internal_net_routes[net_name] = group
            used_nets.add(net_name)

        for wire in group:
            endpoint_to_net[wire.get("From", "")] = net_name
            endpoint_to_net[wire.get("To", "")] = net_name

    summary, details = module_doc(module_name)
    lines = [
        "// =============================================================================",
        f"// 模块: {module_name}",
        f"// 作用: {summary}",
    ]
    lines.extend(f"// 说明: {detail}" for detail in details)
    lines.extend([
        "//",
        f"// Robei 源: little_car/model/{path.name}",
        f"// 导出范围: {'工程登记模块' if source_scope == 'project' else '顶层实际依赖模块'}",
        "// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。",
        "// 重生成: python little_car/source_review/export_sources.py",
        "// =============================================================================",
        "",
        f"module {module_name} (",
    ])

    declarations: list[tuple[str, str]] = []
    for port in root_ports:
        name = port.get("Name", "")
        direction = port.get("Inout", "input")
        dtype = port.get("Datatype", "wire") or "wire"
        size = int(port.get("Datasize", "1"))
        procedural_assignment = re.search(
            rf"\b{re.escape(name)}\s*(?:\[[^\]]+\]\s*)?(?:<=|=)",
            procedural_code,
        )
        if direction == "output" and dtype == "wire" and procedural_assignment:
            dtype = "reg"
        if direction == "output" and dtype == "reg":
            declaration = f"  output reg {width_decl(size)}{name}"
        else:
            declaration = f"  {direction} {dtype} {width_decl(size)}{name}"
        declarations.append((declaration, port_doc(module_name, name, direction, size)))
    for index, (declaration, comment) in enumerate(declarations):
        comma = "," if index != len(declarations) - 1 else ""
        lines.append(f"{declaration}{comma}  // {comment}")
    lines.append(");")

    if internal_nets:
        lines.extend([
            "",
            "  // ===========================================================================",
            "  // 层次内部网络",
            "  // 每条注释给出驱动端以及所有接收端，便于从顶层沿数据流追踪。",
            "  // ===========================================================================",
        ])
        for name, (size, dtype) in internal_nets.items():
            net_type = "wire" if dtype == "reg" else dtype
            route_group = internal_net_routes[name]
            source_owner, source_port = endpoint_parts(
                route_group[0].get("From", ""), module_name
            )
            source_label = f"{source_owner or 'TOP'}.{source_port}"
            target_labels = []
            for route in route_group:
                target_owner, target_port = endpoint_parts(
                    route.get("To", ""), module_name
                )
                target_labels.append(f"{target_owner or 'TOP'}.{target_port}")
            lines.append(f"  // {source_label} -> {', '.join(target_labels)}")
            source_module = instance_classes.get(source_owner, module_name)
            semantic = port_doc(source_module, source_port, "output", size)
            lines.append(
                f"  {net_type} {width_decl(size)}{name};  // {semantic} "
                "具体驱动端和全部接收端见上一行。"
            )

    if root_code.strip():
        lines.extend([
            "",
            "  // ===========================================================================",
            "  // 模块行为实现",
            "  // ===========================================================================",
            annotate_body(module_name, root_code),
        ])

    for instance in instances:
        class_name = instance.get("Class", "")
        instance_name = instance.get("Name", "")
        parameters = (instance.get("Parameters") or "").strip()
        child_ports = list(instance.findall("Port"))
        child_summary, child_details = module_doc(class_name)
        lines.extend([
            "",
            "  // ---------------------------------------------------------------------------",
            f"  // 子模块实例: {instance_name}",
            f"  // 类型: {class_name}",
            f"  // 作用: {child_summary}",
        ])
        lines.extend(f"  // 连接说明: {detail}" for detail in child_details[:2])
        lines.extend(["  // ---------------------------------------------------------------------------", f"  {class_name}"])
        if parameters:
            lines.append(f"  #({parameters})")
        lines.append(f"  {instance_name} (")
        connections = []
        for port in child_ports:
            port_name = port.get("Name", "")
            endpoint = f"{module_name}#{instance_name}>{port_name}"
            signal = endpoint_to_net.get(endpoint, "")
            connections.append(f"    .{port_name}({signal})")
        lines.append(",\n".join(connections))
        lines.append("  );")

    lines.extend(["", "endmodule", ""])
    output_path = VERILOG_DIR / f"{module_name}.v"
    output_path.write_text("\n".join(lines), encoding="utf-8", newline="\n")

    return {
        "module": module_name,
        "model_file": path.name,
        "source_scope": source_scope,
        "ports": len(root_ports),
        "instances": len(instances),
        "wires": len(wires),
        "code_lines": len(root_code.splitlines()) if root_code else 0,
        "output_file": output_path.name,
        "child_classes": " ".join(sorted({i.get("Class", "") for i in instances})),
    }


def main() -> None:
    VERILOG_DIR.mkdir(parents=True, exist_ok=True)
    TESTBENCH_DIR.mkdir(parents=True, exist_ok=True)

    for old_file in VERILOG_DIR.glob("*.v"):
        old_file.unlink()
    for old_file in TESTBENCH_DIR.glob("*.v"):
        old_file.unlink()

    records = []
    exported_modules = set()
    pending = [(path, "project") for path in project_model_paths()]
    while pending:
        path, source_scope = pending.pop(0)
        root = load_xml(path)
        module_name = root.get("Name") or path.stem
        if module_name in exported_modules:
            continue
        record = export_model(path, source_scope)
        records.append(record)
        exported_modules.add(module_name)
        for child_class in record["child_classes"].split():
            dependency_path = MODEL_DIR / f"{child_class}.model"
            if child_class not in exported_modules and dependency_path.exists():
                pending.append((dependency_path, "dependency"))
    with MANIFEST_FILE.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    FILELIST_FILE.write_text(
        "".join(f"verilog/{record['output_file']}\n" for record in records),
        encoding="ascii",
        newline="\n",
    )

    for testbench in sorted(TEST_VERILOG_DIR.glob("*.v")):
        destination = TESTBENCH_DIR / testbench.name
        description = TESTBENCH_DOCS.get(testbench.name, "Robei 独立验证 testbench。")
        header = (
            "// =============================================================================\n"
            f"// Testbench: {testbench.stem}\n"
            f"// 验证目标: {description}\n"
            f"// 原始文件: little_car/test/verilog/{testbench.name}\n"
            "// 本副本随 export_sources.py 重新生成，请在原测试目录修改测试逻辑。\n"
            "// =============================================================================\n\n"
        )
        destination.write_text(
            header + testbench.read_text(encoding="utf-8-sig"),
            encoding="utf-8",
            newline="\n",
        )

    print(f"Exported {len(records)} models to {VERILOG_DIR}")
    print(f"Copied {len(list(TESTBENCH_DIR.glob('*.v')))} testbenches")


if __name__ == "__main__":
    main()
