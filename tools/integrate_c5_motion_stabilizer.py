#!/usr/bin/env python3
"""Idempotently insert the local C5 motion stabilizer into the Robei graph."""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
PROJECT = ROOT / "little_car" / "little_car.proj"

TOP = "sys_route_gui_fsm_top"
ROUTE = "c5_route_controller9"
DRIVER = "c5_motor_frame_driver10"
MOTION = "motion_sensor_subsystem28"
STABILIZER = "c5_motion_stabilizer29"


def root_opening_tag(text: str) -> str:
    match = re.match(r"(?:<!DOCTYPE Robei>\s*\n)?(<Module\b[^>]*>)", text, re.S)
    if not match:
        raise RuntimeError("cannot locate model root opening tag")
    return match.group(1)


def instance_from_leaf() -> str:
    leaf = (MODEL_DIR / "c5_motion_stabilizer.model").read_text(encoding="utf-8")
    tag = root_opening_tag(leaf)
    replacements = {
        "Class": "c5_motion_stabilizer",
        "Name": STABILIZER,
        "Parent": TOP,
        "X": "3810",
        "Y": "1660",
        "Width": "900",
        "Height": "1180",
    }
    for attribute, value in replacements.items():
        tag, count = re.subn(
            rf'\b{attribute}="[^"]*"', f'{attribute}="{value}"', tag, count=1
        )
        if count != 1:
            raise RuntimeError(f"leaf model is missing {attribute}")

    ports: list[str] = []
    for line in leaf.splitlines():
        if line.startswith(" <Port "):
            ports.append(" " + re.sub(r'Parent="[^"]+"', f'Parent="{STABILIZER}"', line))
    if len(ports) < 19:
        raise RuntimeError(f"stabilizer leaf has too few ports: {len(ports)}")
    return " " + tag + "\n" + "\n".join(ports) + "\n </Module>"


def remove_instance(text: str) -> str:
    return re.sub(
        rf'^[ \t]*<Module\b(?=[^>]*\bName="{STABILIZER}")[^>]*>'
        rf'.*?^[ \t]*</Module>[ \t]*\r?\n?',
        "",
        text,
        flags=re.M | re.S,
    )


def insert_instance(text: str, block: str) -> str:
    first_port = re.search(
        rf'^[ \t]*<Port\b(?=[^>]*\bParent="{TOP}")[^>]*/>', text, flags=re.M
    )
    if not first_port:
        raise RuntimeError("cannot locate first top-level port")
    return text[: first_port.start()] + block + "\n" + text[first_port.start() :]


def remove_stabilizer_wires(text: str) -> str:
    text = re.sub(
        rf'^[ \t]*<Wire\b(?=[^>]*(?:From|To)="[^"]*{STABILIZER}[^"]*")[^>]*/>[ \t]*\r?\n?',
        "",
        text,
        flags=re.M,
    )
    # Replace any old speed/stop edges into the frame driver.  The source may
    # be the route controller, an earlier stabilizer instance, or bt_route_uart
    # depending on which integration script last touched the graph.
    for port in ("lf_speed", "rf_speed", "lr_speed", "rr_speed", "force_stop"):
        text = re.sub(
            rf'^[ \t]*<Wire\b(?=[^>]*To="{TOP}#{DRIVER}&gt;{port}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def append_before_root_close(text: str, content: str) -> str:
    head, separator, tail = text.rpartition("</Module>")
    if not separator or tail.strip():
        raise RuntimeError("cannot locate root closing tag")
    return head.rstrip() + "\n" + content.rstrip() + "\n</Module>\n"


def wire(name: str, source: str, destination: str, size: int = 1) -> str:
    return (
        f' <Wire Datasize="{size}" Datatype="wire" From="{source}" Name="{name}" '
        f'Parent="{TOP}" To="{destination}" />'
    )


def integrate_top() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = remove_instance(text)
    text = insert_instance(text, instance_from_leaf())
    text = remove_stabilizer_wires(text)

    wires = [
        wire(f"{TOP}_clk_{STABILIZER}", f"{TOP}&gt;clk", f"{TOP}#{STABILIZER}&gt;clk"),
        wire(f"{TOP}_rst_n_{STABILIZER}", f"{TOP}#power_on_reset24&gt;system_rst_n", f"{TOP}#{STABILIZER}&gt;rst_n"),
        wire(f"{MOTION}_calibration_done", f"{TOP}#{MOTION}&gt;calibration_done", f"{TOP}#{STABILIZER}&gt;calibration_done"),
        wire(f"{MOTION}_fused_sample_valid", f"{TOP}#{MOTION}&gt;fused_sample_valid", f"{TOP}#{STABILIZER}&gt;fused_sample_valid"),
        wire(f"{MOTION}_imu_degraded", f"{TOP}#{MOTION}&gt;imu_degraded", f"{TOP}#{STABILIZER}&gt;imu_degraded"),
        wire(f"{MOTION}_imu_disagreement", f"{TOP}#{MOTION}&gt;imu_disagreement", f"{TOP}#{STABILIZER}&gt;imu_disagreement"),
        wire(f"{MOTION}_fused_gyro_z", f"{TOP}#{MOTION}&gt;fused_gyro_z", f"{TOP}#{STABILIZER}&gt;fused_gyro_z", 16),
        wire("bt_route_uart16_local_calibration_start_pulse", f"{TOP}#bt_route_uart16&gt;local_calibration_start_pulse", f"{TOP}#{STABILIZER}&gt;local_calibration_start"),
        wire("bt_route_uart16_motor_stop_pulse", f"{TOP}#bt_route_uart16&gt;motor_stop_pulse", f"{TOP}#{STABILIZER}&gt;external_force_stop"),
        wire("obstacle_stop_merge20_obstacle_stop", f"{TOP}#obstacle_stop_merge20&gt;obstacle_stop", f"{TOP}#{STABILIZER}&gt;obstacle_stop"),
        wire("car_param_controller22_lf_default_speed", f"{TOP}#car_param_controller22&gt;lf_default_speed", f"{TOP}#{STABILIZER}&gt;cal_lf_default_speed", 10),
        wire("car_param_controller22_rf_default_speed", f"{TOP}#car_param_controller22&gt;rf_default_speed", f"{TOP}#{STABILIZER}&gt;cal_rf_default_speed", 10),
        wire("car_param_controller22_lr_default_speed", f"{TOP}#car_param_controller22&gt;lr_default_speed", f"{TOP}#{STABILIZER}&gt;cal_lr_default_speed", 10),
        wire("car_param_controller22_rr_default_speed", f"{TOP}#car_param_controller22&gt;rr_default_speed", f"{TOP}#{STABILIZER}&gt;cal_rr_default_speed", 10),
    ]
    for port in ("lf_speed", "rf_speed", "lr_speed", "rr_speed"):
        wires.append(
            wire(
                f"{ROUTE}_{port}",
                f"{TOP}#{ROUTE}&gt;{port}",
                f"{TOP}#{STABILIZER}&gt;base_{port}",
                11,
            )
        )
        wires.append(
            wire(
                f"{STABILIZER}_{port}",
                f"{TOP}#{STABILIZER}&gt;{port}",
                f"{TOP}#{DRIVER}&gt;{port}",
                11,
            )
        )

    wires.extend(
        [
            wire(f"{STABILIZER}_force_stop", f"{TOP}#{STABILIZER}&gt;force_stop", f"{TOP}#{DRIVER}&gt;force_stop"),
            wire(f"{STABILIZER}_local_calibration_state", f"{TOP}#{STABILIZER}&gt;local_calibration_state", f"{TOP}#bt_route_uart16&gt;local_calibration_state", 4),
            wire(f"{STABILIZER}_local_calibration_flags", f"{TOP}#{STABILIZER}&gt;local_calibration_flags", f"{TOP}#bt_route_uart16&gt;local_calibration_flags", 8),
            wire(f"{STABILIZER}_learned_yaw_trim", f"{TOP}#{STABILIZER}&gt;learned_yaw_trim", f"{TOP}#bt_route_uart16&gt;learned_yaw_trim", 11),
            wire(f"{STABILIZER}_yaw_correction", f"{TOP}#{STABILIZER}&gt;yaw_correction", f"{TOP}#bt_route_uart16&gt;yaw_correction", 11),
            wire(f"{STABILIZER}_calibration_gyro_peak", f"{TOP}#{STABILIZER}&gt;calibration_gyro_peak", f"{TOP}#bt_route_uart16&gt;calibration_gyro_peak", 16),
        ]
    )

    text = append_before_root_close(text, "\n".join(wires))
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")


def register_leaf() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    text = re.sub(
        r'^[ \t]*<c5_motion_stabilizer\.model\b[^>]*/>[ \t]*\r?\n?',
        "",
        text,
        flags=re.M,
    )
    marker = re.search(r'^[ \t]*</model>[ \t]*$', text, flags=re.M)
    if not marker:
        raise RuntimeError("project model folder closing tag is missing")
    entry = (
        '  <c5_motion_stabilizer.model name="c5_motion_stabilizer.model" '
        'parent="model" type="model"/>\n'
    )
    text = text[: marker.start()] + entry + text[marker.start() :]
    PROJECT.write_text(text, encoding="utf-8", newline="\n")


def validate() -> None:
    root = ET.parse(TOP_MODEL).getroot()
    instances = [module for module in root.findall("Module") if module.get("Name") == STABILIZER]
    leaf_port_count = len(ET.parse(MODEL_DIR / "c5_motion_stabilizer.model").getroot().findall("Port"))
    if len(instances) != 1 or len(instances[0].findall("Port")) != leaf_port_count:
        raise RuntimeError("stabilizer Robei instance is incomplete")

    destinations = [item.get("To") for item in root.findall("Wire")]
    required = [
        f"{TOP}#{STABILIZER}>fused_gyro_z",
        f"{TOP}#{STABILIZER}>base_lf_speed",
        f"{TOP}#{DRIVER}>lf_speed",
        f"{TOP}#{DRIVER}>rr_speed",
        f"{TOP}#{DRIVER}>force_stop",
        f"{TOP}#bt_route_uart16>local_calibration_flags",
    ]
    for destination in required:
        if destinations.count(destination) != 1:
            raise RuntimeError(f"expected one wire to {destination}")

    model_group = ET.parse(PROJECT).getroot().find("model")
    if model_group is None:
        raise RuntimeError("project model group is missing")
    names = [child.get("name") for child in model_group]
    if names.count("c5_motion_stabilizer.model") != 1:
        raise RuntimeError("stabilizer leaf is not registered exactly once")


def main() -> None:
    integrate_top()
    register_leaf()
    validate()
    print("C5 local motion stabilizer integrated into Robei model graph")


if __name__ == "__main__":
    main()
