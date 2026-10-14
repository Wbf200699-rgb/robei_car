#!/usr/bin/env python3
from pathlib import Path
import argparse
from collections import Counter
import html
import re
import xml.etree.ElementTree as ET

from robei_converter import parse_verilog_module


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
BUILD_DIR = ROOT / "little_car" / "build"
MIRROR_DIR = ROOT / "little_car" / "hdmi_rgb_input"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
PROJECT_FILE = ROOT / "little_car" / "little_car.proj"
TOP_CONST = ROOT / "little_car" / "const" / "top.const"
FILELIST = BUILD_DIR / "filelist.vf"
TOP_NAME = "sys_route_gui_fsm_top"
AUTO_EXPLORATION = "onboard_exploration_controller35"
FIELD_MAP_CONFIG = "field_map_config_uart36"


def sync_behavioral_model_code_from_build() -> list[str]:
    """Make every behavioral leaf model match the RTL used by Vivado.

    Graphical hierarchy models keep their Module/Wire structure. Only models
    with no graphical children or wires receive a new root Code attribute.
    This makes the Vivado file list the behavioral source of truth instead of
    merely checking that two copies of the same model have matching hashes.
    """
    synced: list[str] = []
    for raw_path in FILELIST.read_text(encoding="utf-8").splitlines():
        raw_path = raw_path.strip()
        if not raw_path:
            continue
        verilog_path = Path(raw_path)
        if verilog_path.stem == "vl53l1x_array5":
            # The final vehicle keeps only the protocol-compatible inactive
            # wrapper. Robei build/ is generated output and can still contain
            # the retired active driver after an interrupted compile.
            verilog_path = MIRROR_DIR / "vl53l1x_array5.v"
        model_path = MODEL_DIR / f"{verilog_path.stem}.model"
        if not model_path.is_file():
            continue

        model_text = model_path.read_text(encoding="utf-8")
        model_root = ET.fromstring(
            model_text.replace("<!DOCTYPE Robei>", "", 1)
        )
        if model_root.findall("Module") or model_root.findall("Wire"):
            continue

        body_code = parse_verilog_module(
            verilog_path.read_text(encoding="utf-8")
        )["body_code"]
        def logic_text(code: str) -> str:
            code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
            code = re.sub(r"//[^\r\n]*", "", code)
            return re.sub(r"\s+", "", code)

        current_code = model_root.get("Code", "")
        if logic_text(body_code) == logic_text(current_code):
            continue

        encoded = html.escape(body_code, quote=True).replace("\n", "&#xa;")
        pattern = re.compile(r'(<Module\b[^>]*\bCode=")[^"]*(")', re.S)
        updated, count = pattern.subn(
            lambda match: match.group(1) + encoded + match.group(2),
            model_text,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"cannot update root Code in {model_path.name}")
        model_path.write_text(updated, encoding="utf-8", newline="\n")
        synced.append(model_path.name)
    return synced


def encoded_code(model_name: str) -> str:
    code = ET.parse(MODEL_DIR / model_name).getroot().attrib["Code"]
    return html.escape(code, quote=True).replace("\n", "&#xa;")


def refresh_code(text: str, instance_name: str, model_name: str) -> str:
    encoded = encoded_code(model_name)
    pattern = re.compile(
        rf'(<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*\bCode=")[^"]*(")'
    )
    text, count = pattern.subn(lambda m: m.group(1) + encoded + m.group(2), text)
    if count != 1:
        raise RuntimeError(f"expected one {instance_name} Code attribute, got {count}")
    return text


def standalone_port_line(model_name: str, port_name: str, parent: str) -> str:
    text = (MODEL_DIR / model_name).read_text(encoding="utf-8")
    match = re.search(
        rf'^ <Port\b(?=[^>]*\bName="{re.escape(port_name)}")[^>]*/>$',
        text,
        flags=re.M,
    )
    if not match:
        raise RuntimeError(f"missing port {port_name} in {model_name}")
    line = match.group(0)
    line = re.sub(r'Parent="[^"]+"', f'Parent="{parent}"', line)
    return " " + line


def add_instance_ports(
    text: str, instance_name: str, model_name: str, port_names: list[str]
) -> str:
    for port_name in port_names:
        text = re.sub(
            rf'^\s*<Port\b(?=[^>]*\bParent="{re.escape(instance_name)}")(?=[^>]*\bName="{re.escape(port_name)}")[^>]*/>\s*\n?',
            "",
            text,
            flags=re.M,
        )
    pattern = re.compile(
        rf'(<Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>)(.*?)(\n\s*</Module>)',
        flags=re.S,
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"missing instance block {instance_name}")
    body = match.group(2)
    additions = [
        standalone_port_line(model_name, port_name, instance_name)
        for port_name in port_names
    ]
    replacement = match.group(1) + body + "\n" + "\n".join(additions) + match.group(3)
    return text[: match.start()] + replacement + text[match.end() :]


def replace_wire_to(text: str, destination: str, new_line: str) -> str:
    pattern = re.compile(
        rf'^ <Wire\b(?=[^>]*\bTo="{re.escape(destination)}")[^>]*/>$',
        flags=re.M,
    )
    text, count = pattern.subn(new_line, text)
    if count != 1:
        raise RuntimeError(f"expected one wire to {destination}, got {count}")
    return text


def append_wire(text: str, name: str, source: str, destination: str, size: int = 1) -> str:
    if f'To="{destination}"' in text:
        return text
    line = (
        f' <Wire Datasize="{size}" Datatype="wire" From="{source}" Name="{name}" '
        f'Parent="sys_route_gui_fsm_top" To="{destination}" />\n'
    )
    return text.rsplit("</Module>", 1)[0] + line + "</Module>\n"


def upsert_wire_to(text: str, name: str, source: str, destination: str, size: int = 1) -> str:
    line = (
        f' <Wire Datasize="{size}" Datatype="wire" From="{source}" Name="{name}" '
        f'Parent="sys_route_gui_fsm_top" To="{destination}" />'
    )
    pattern = re.compile(
        rf'^ <Wire\b(?=[^>]*\bTo="{re.escape(destination)}")[^>]*/>$',
        flags=re.M,
    )
    text, count = pattern.subn(line, text)
    if count > 1:
        raise RuntimeError(f"multiple wires drive {destination}")
    if count == 0:
        text = text.rsplit("</Module>", 1)[0] + line + "\n</Module>\n"
    return text


def normalize_fanout_wire_names(text: str) -> str:
    """Use one Robei net name for every branch from the same endpoint."""
    wire_pattern = re.compile(r"^ <Wire\b[^>]*/>$", flags=re.M)
    names_by_source: dict[str, list[str]] = {}
    for match in wire_pattern.finditer(text):
        line = match.group(0)
        source_match = re.search(r'\bFrom="([^"]+)"', line)
        name_match = re.search(r'\bName="([^"]+)"', line)
        if source_match and name_match and source_match.group(1):
            names_by_source.setdefault(source_match.group(1), []).append(
                name_match.group(1)
            )

    canonical: dict[str, str] = {}
    for source, names in names_by_source.items():
        counts = Counter(names)
        canonical[source] = min(
            counts,
            key=lambda name: (-counts[name], len(name), name),
        )

    def replace(match: re.Match[str]) -> str:
        line = match.group(0)
        source_match = re.search(r'\bFrom="([^"]+)"', line)
        if not source_match or source_match.group(1) not in canonical:
            return line
        name = canonical[source_match.group(1)]
        return re.sub(r'\bName="[^"]+"', f'Name="{name}"', line, count=1)

    return wire_pattern.sub(replace, text)


def expand_grouped_get_ports(text: str) -> str:
    """Expand Tcl port lists unsupported by Robei's XDC parser.

    Vivado accepts ``[get_ports {a b}]`` but the bundled Roplace parser treats
    the whitespace-separated list as malformed input. Keep the same property
    semantics using one native single-port command per line.
    """
    pattern = re.compile(
        r"set_property\s+(\S+)\s+(\S+)\s+\[get_ports \{([^}]+)\}\]"
    )

    def replace(match: re.Match[str]) -> str:
        prop, value, ports = match.groups()
        return "&#10;".join(
            f"set_property {prop} {value} [get_ports {port}]"
            for port in ports.split()
        )

    return pattern.sub(replace, text)


def model_instance_block(
    model_name: str, instance_name: str, x: int, y: int
) -> str:
    source_path = MODEL_DIR / model_name
    source_text = source_path.read_text(encoding="utf-8")
    source_text = re.sub(r"^<!DOCTYPE Robei>\s*", "", source_text, count=1)
    source_root = ET.parse(source_path).getroot()
    source_name = source_root.attrib["Name"]

    opening_match = re.match(r"<Module\b[^>]*>", source_text)
    if not opening_match:
        raise RuntimeError(f"missing root module in {model_name}")
    opening = opening_match.group(0)
    opening = re.sub(
        r'\bClass="[^"]+"', f'Class="{source_name}"', opening, count=1
    )
    opening = re.sub(r'\bName="[^"]+"', f'Name="{instance_name}"', opening, count=1)
    opening = re.sub(r'\bParent="[^"]+"', f'Parent="{TOP_NAME}"', opening, count=1)
    opening = re.sub(r'\bX="[^"]+"', f'X="{x}"', opening, count=1)
    opening = re.sub(r'\bY="[^"]+"', f'Y="{y}"', opening, count=1)
    source_text = opening + source_text[opening_match.end() :]
    source_text = source_text.replace(
        f'Parent="{source_name}"', f'Parent="{instance_name}"'
    )
    return " " + source_text.strip().replace("\n", "\n ")


def upsert_instance(
    text: str, model_name: str, instance_name: str, x: int, y: int
) -> str:
    pattern = re.compile(
        rf'^ <Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>.*?^ </Module>[ \t]*\r?\n?',
        flags=re.M | re.S,
    )
    text = pattern.sub("", text)
    block = model_instance_block(model_name, instance_name, x, y) + "\n"
    first_wire = re.search(r"^ <Wire\b", text, flags=re.M)
    if first_wire:
        return text[: first_wire.start()] + block + text[first_wire.start() :]
    return text.rsplit("</Module>", 1)[0] + block + "</Module>\n"


def remove_instance(text: str, instance_name: str) -> str:
    """Remove an obsolete unconnected graphical dependency anchor."""
    pattern = re.compile(
        rf'^ <Module\b(?=[^>]*\bName="{re.escape(instance_name)}")[^>]*>.*?^ </Module>[ \t]*\r?\n?',
        flags=re.M | re.S,
    )
    return pattern.sub("", text)


def remove_instance_wires(text: str, instance_name: str) -> str:
    """Remove wires whose source or destination belongs to an instance."""
    marker = re.escape(f"sys_route_gui_fsm_top#{instance_name}&gt;")
    pattern = re.compile(
        rf'^\s*<Wire\b[^\r\n]*{marker}[^\r\n]*/>\s*\r?\n?',
        flags=re.M,
    )
    return pattern.sub("", text)


def upsert_root_port(
    text: str, name: str, direction: str, side: str, x: str, y: str
) -> str:
    if re.search(
        rf'<Port\b(?=[^>]*\bName="{re.escape(name)}")(?=[^>]*\bParent="{TOP_NAME}")[^>]*/>',
        text,
    ):
        return text
    color = "#6495ed" if direction == "input" else "#00ffff"
    line = (
        f' <Port Color="{color}" Connect="" Datasize="1" Datatype="wire" '
        f'Description="" Height="30" Inout="{direction}" Name="{name}" '
        f'Parent="{TOP_NAME}" Side="{side}" Width="30" X="{x}" Y="{y}" />\n'
    )
    first_child = re.search(r"^ <Module\b", text, flags=re.M)
    if not first_child:
        raise RuntimeError("top model has no child module insertion point")
    return text[: first_child.start()] + line + text[first_child.start() :]


def register_models() -> None:
    text = PROJECT_FILE.read_text(encoding="utf-8")
    for retired_model in (
        "temp_run_status_source.model",
        "temp_ultrasonic_echo_source.model",
        "c5_motor_test_pattern.model",
        "vl53l1x_i2c_master.model",
        "vl53l1x_proximity_fusion.model",
    ):
        text = re.sub(
            rf'^\s*<{re.escape(retired_model)}\b[^>]*/>\s*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    additions = []
    for model_name in (
        "linkpg_tag_position_uart.model",
        "vehicle_debug_uart.model",
        "onboard_exploration_controller.model",
        "field_map_config_uart.model",
    ):
        if f'<{model_name} ' not in text:
            additions.append(
                f'  <{model_name} name="{model_name}" parent="model" type="model"/>\n'
            )
    if additions:
        marker = " </model>"
        if marker not in text:
            raise RuntimeError("project model-folder closing tag not found")
        text = text.replace(marker, "".join(additions) + marker, 1)
        PROJECT_FILE.write_text(text, encoding="utf-8", newline="\n")


def register_verilog_sources() -> None:
    text = FILELIST.read_text(encoding="utf-8")
    source = (BUILD_DIR / "field_map_config_uart.v").as_posix()
    if source not in text.splitlines():
        marker = (BUILD_DIR / "sys_route_gui_fsm_top.v").as_posix()
        line = source + "\n"
        text = text.replace(marker, line + marker, 1) if marker in text else text + line
        FILELIST.write_text(text, encoding="utf-8", newline="\n")


def sync_top_const() -> None:
    text = TOP_CONST.read_text(encoding="utf-8")
    constraints = [
        "set_property PACKAGE_PIN P5 [get_ports uwb_uart_rx]",
        "set_property IOSTANDARD LVCMOS33 [get_ports uwb_uart_rx]",
        "set_property PULLUP TRUE [get_ports uwb_uart_rx]",
        "set_property PACKAGE_PIN R2 [get_ports uwb_uart_tx]",
        "set_property IOSTANDARD LVCMOS33 [get_ports uwb_uart_tx]",
        "set_property DRIVE 8 [get_ports uwb_uart_tx]",
        "set_property SLEW SLOW [get_ports uwb_uart_tx]",
        "set_property PACKAGE_PIN J1 [get_ports debug_uart_tx]",
        "set_property IOSTANDARD LVCMOS33 [get_ports debug_uart_tx]",
        "set_property DRIVE 8 [get_ports debug_uart_tx]",
        "set_property SLEW SLOW [get_ports debug_uart_tx]",
    ]
    missing = [line for line in constraints if line not in text]
    if missing:
        encoded = "&#10;" + "&#10;".join(missing)
        text, count = re.subn(
            r'(<Module\b[^>]*\bCode=")([^"]*)(")',
            lambda match: match.group(1) + match.group(2) + encoded + match.group(3),
            text,
            count=1,
        )
        if count != 1:
            raise RuntimeError("top.const Code attribute not found")

    module_port_specs = (
        ("uwb_uart_rx", "input", "left", "-0.00605938", "0.943192"),
        ("uwb_uart_tx", "output", "right", "0.993941", "0.923192"),
        ("debug_uart_tx", "output", "right", "0.993941", "0.953192"),
    )
    module_pattern = re.compile(
        r'(<Module\b(?=[^>]*\bName="sys_route_gui_fsm_top1")[^>]*>)(.*?)(\n\s*</Module>)',
        flags=re.S,
    )
    match = module_pattern.search(text)
    if not match:
        raise RuntimeError("sys_route_gui_fsm_top1 block missing in top.const")
    body = match.group(2)
    for name, direction, side, x, y in module_port_specs:
        if re.search(
            rf'<Port\b(?=[^>]*\bName="{re.escape(name)}")(?=[^>]*\bParent="sys_route_gui_fsm_top1")[^>]*/>',
            body,
        ):
            continue
        color = "#6495ed" if direction == "input" else "#00ffff"
        body += (
            f'\n  <Port Color="{color}" Connect="" Datasize="1" Datatype="wire" '
            f'Description="" Height="30" Inout="{direction}" Name="{name}" '
            f'Parent="sys_route_gui_fsm_top1" Side="{side}" Width="30" '
            f'X="{x}" Y="{y}" />'
        )
    text = text[: match.start()] + match.group(1) + body + match.group(3) + text[match.end() :]

    physical_ports = (
        ("P5", "", "up", "right", "0.989166", "0.910000"),
        ("R2", "8", "none", "bottom", "0.700000", "0.978602"),
        ("J1", "8", "none", "bottom", "0.790000", "0.978602"),
    )
    first_wire = re.search(r"^ <Wire\b", text, flags=re.M)
    if not first_wire:
        raise RuntimeError("top.const wire insertion point missing")
    physical_lines = []
    for pin, drive, pull, side, x, y in physical_ports:
        if re.search(
            rf'<Port\b(?=[^>]*\bName="{re.escape(pin)}")(?=[^>]*\bParent="top")[^>]*/>',
            text,
        ):
            continue
        physical_lines.append(
            f' <Port Color="#6495ed" Description="" Drive="{drive}" Height="30" '
            f'Name="{pin}" Parent="top" Pull="{pull}" Slew="SLOW" Side="{side}" '
            f'Voltage="LVCMOS33" Width="30" X="{x}" Y="{y}" />\n'
        )
    if physical_lines:
        text = text[: first_wire.start()] + "".join(physical_lines) + text[first_wire.start() :]

    const_wires = (
        ("top#sys_route_gui_fsm_top1&gt;uwb_uart_rx", "top&gt;P5"),
        ("top#sys_route_gui_fsm_top1&gt;uwb_uart_tx", "top&gt;R2"),
        ("top#sys_route_gui_fsm_top1&gt;debug_uart_tx", "top&gt;J1"),
    )
    for source, destination in const_wires:
        if f'To="{destination}"' in text:
            continue
        line = (
            f' <Wire Datasize="1" Datatype="wire" From="{source}" Name="0" '
            f'Parent="top" To="{destination}" />\n'
        )
        text = text.rsplit("</Module>", 1)[0] + line + "</Module>\n"
    text = expand_grouped_get_ports(text)
    TOP_CONST.write_text(text, encoding="utf-8", newline="\n")


def sync_top_model() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    for retired_instance in (
        "temp_run_status_source4",
        "temp_ultrasonic_echo_source5",
        "c5_motor_test_pattern18",
    ):
        text = remove_instance(text, retired_instance)
        text = remove_instance_wires(text, retired_instance)
    text = re.sub(r'(<Module\b[^>]*\bName="sys_route_gui_fsm_top"[^>]*\bWidth=")[^"]+("[^>]*>)',
                  r'\g<1>9000\2', text, count=1)
    text = re.sub(r'(<Module\b[^>]*\bName="sys_route_gui_fsm_top"[^>]*\bHeight=")[^"]+("[^>]*>)',
                  r'\g<1>7000\2', text, count=1)
    text = upsert_root_port(
        text, "uwb_uart_rx", "input", "left", "-0.00605938", "0.943192"
    )
    text = upsert_root_port(
        text, "uwb_uart_tx", "output", "right", "0.993941", "0.923192"
    )
    text = upsert_root_port(
        text, "debug_uart_tx", "output", "right", "0.993941", "0.953192"
    )
    text = upsert_instance(
        text, "linkpg_tag_position_uart.model", "linkpg_tag_position_uart33", 5550, 2350
    )
    text = upsert_instance(
        text, "vehicle_debug_uart.model", "vehicle_debug_uart34", 7250, 1750
    )
    text = upsert_instance(
        text,
        "onboard_exploration_controller.model",
        AUTO_EXPLORATION,
        5100,
        3650,
    )
    text = upsert_instance(
        text, "field_map_config_uart.model", FIELD_MAP_CONFIG, 6500, 3650
    )
    # RobeiEDA follows graphical Module children when deciding which model
    # definitions to generate. The dual-IMU calibrator is instantiated from
    # behavioral Code, so an unconnected anchor keeps clean builds independent
    # from stale files in build/. Yosys removes this unused anchor.
    text = remove_instance(text, "vl53l1x_i2c_master37")
    text = upsert_instance(
        text,
        "dual_imu_static_calibrator.model",
        "dual_imu_static_calibrator38",
        8200,
        1200,
    )
    text = refresh_code(
        text, "motion_sensor_subsystem28", "motion_sensor_subsystem.model"
    )
    text = refresh_code(text, "vl53l1x_array527", "vl53l1x_array5.model")
    text = refresh_code(text, "bt_route_uart16", "bt_route_uart.model")
    text = refresh_code(text, "c5_route_controller9", "c5_route_controller.model")
    text = refresh_code(text, "c5_motor_frame_driver10", "c5_motor_frame_driver.model")
    text = refresh_code(text, "gui_route_renderer2", "gui_route_renderer.model")
    text = refresh_code(text, "c5_motion_stabilizer29", "c5_motion_stabilizer.model")
    text = refresh_code(text, "onboard_occupancy_map30", "onboard_occupancy_map.model")
    text = refresh_code(text, "rplidar_c1_uart19", "rplidar_c1_uart.model")
    text = refresh_code(
        text, "linkpg_tag_position_uart33", "linkpg_tag_position_uart.model"
    )
    text = refresh_code(text, "vehicle_debug_uart34", "vehicle_debug_uart.model")
    text = refresh_code(
        text, AUTO_EXPLORATION, "onboard_exploration_controller.model"
    )
    text = refresh_code(text, FIELD_MAP_CONFIG, "field_map_config_uart.model")
    # The SD map store is a graphical hierarchy. Refresh the implementation
    # blocks inside it without collapsing the visible controller/SPI wiring.
    text = refresh_code(
        text, "map_sd_store_controller0", "map_sd_store_controller.model"
    )
    text = refresh_code(text, "sd_spi_block_device0", "sd_spi_block_device.model")
    text = refresh_code(
        text, "sd_card_detect_adapter0", "sd_card_detect_adapter.model"
    )

    text = add_instance_ports(
        text,
        "motion_sensor_subsystem28",
        "motion_sensor_subsystem.model",
        ["vehicle_motion_expected"],
    )
    text = add_instance_ports(
        text,
        "bt_route_uart16",
        "bt_route_uart.model",
        [
            "ultrasonic_obstacle_stop",
            "merged_obstacle_stop",
            "motor_blocked",
            "ultrasonic_distance_cm",
            "ultrasonic_distance_valid",
            "ultrasonic_echo_timeout",
            "c5_frame_count",
            "c5_last_lf_pulse",
            "c5_last_rf_pulse",
            "c5_last_lr_pulse",
            "c5_last_rr_pulse",
            "c5_stop_frames_complete",
            "c5_stop_frames_remaining",
            "c5_frame_busy",
            "motor_stop_pulse",
            "local_calibration_state",
            "local_calibration_flags",
            "learned_yaw_trim",
            "yaw_correction",
            "calibration_gyro_peak",
            "local_calibration_start_pulse",
            "gyro_bias_learn_pulse",
        ],
    )
    text = add_instance_ports(
        text,
        "c5_motion_stabilizer29",
        "c5_motion_stabilizer.model",
        [
            "local_calibration_start",
            "external_force_stop",
            "obstacle_stop",
            "cal_lf_default_speed",
            "cal_rf_default_speed",
            "cal_lr_default_speed",
            "cal_rr_default_speed",
            "force_stop",
            "vehicle_motion_expected",
            "local_calibration_state",
            "local_calibration_flags",
            "learned_yaw_trim",
            "calibration_gyro_peak",
            "flow_sample_valid",
            "flow_forward_q8",
            "flow_lateral_q8",
            "flow_correction",
        ],
    )
    text = add_instance_ports(
        text,
        "onboard_occupancy_map30",
        "onboard_occupancy_map.model",
        [
            "field_config_valid",
            "field_pose_x_cell",
            "field_pose_y_cell",
            "field_heading_1024",
            "uwb_position_valid",
            "uwb_position_online",
            "uwb_position_x_cm",
            "uwb_position_y_cm",
            "flow_body_sample_valid",
            "flow_body_forward_q8",
            "flow_body_lateral_q8",
            "uwb_fusion_active",
            "uwb_reject_count",
            "gyro_bias_learn_pulse",
        ],
    )
    text = add_instance_ports(
        text,
        AUTO_EXPLORATION,
        "onboard_exploration_controller.model",
        [
            "field_config_valid",
            "field_low_x_cell",
            "field_high_x_cell",
            "field_low_y_cell",
            "field_high_y_cell",
        ],
    )
    text = add_instance_ports(
        text,
        "c5_route_controller9",
        "c5_route_controller.model",
        ["auto_mode", "auto_cmd", "motor_blocked"],
    )
    text = add_instance_ports(
        text,
        "c5_motor_frame_driver10",
        "c5_motor_frame_driver.model",
        [
            "force_stop",
            "frame_count",
            "last_lf_pulse",
            "last_rf_pulse",
            "last_lr_pulse",
            "last_rr_pulse",
            "stop_frames_complete",
            "stop_frames_remaining",
        ],
    )

    text = replace_wire_to(
        text,
        "sys_route_gui_fsm_top#bt_route_uart16&gt;lidar_obstacle_stop",
        ' <Wire Datasize="1" Datatype="wire" From="sys_route_gui_fsm_top#rplidar_c1_uart19&gt;obstacle_stop" Name="rplidar_c1_uart19_obstacle_stop_bt" Parent="sys_route_gui_fsm_top" To="sys_route_gui_fsm_top#bt_route_uart16&gt;lidar_obstacle_stop" />',
    )
    text = text.replace(
        'Name="obstacle_stop_merge20_obstacle_stop_manual"',
        'Name="obstacle_stop_merge20_obstacle_stop"',
    )

    text = append_wire(
        text,
        "ultrasonic_obstacle_guard6_obstacle_stop",
        "sys_route_gui_fsm_top#ultrasonic_obstacle_guard6&gt;obstacle_stop",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;ultrasonic_obstacle_stop",
    )
    text = append_wire(
        text,
        "obstacle_stop_merge20_obstacle_stop",
        "sys_route_gui_fsm_top#obstacle_stop_merge20&gt;obstacle_stop",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;merged_obstacle_stop",
    )
    text = append_wire(
        text,
        "c5_route_controller9_motor_blocked",
        "sys_route_gui_fsm_top#c5_route_controller9&gt;motor_blocked",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;motor_blocked",
    )
    text = text.replace(
        'Name="rplidar_c1_uart19_obstacle_stop_bt"',
        'Name="rplidar_c1_uart19_obstacle_stop"',
    )
    text = text.replace(
        'Name="ultrasonic_obstacle_guard6_obstacle_stop_bt"',
        'Name="ultrasonic_obstacle_guard6_obstacle_stop"',
    )
    text = text.replace(
        'Name="obstacle_stop_merge20_obstacle_stop_bt"',
        'Name="obstacle_stop_merge20_obstacle_stop"',
    )
    text = append_wire(
        text,
        "ultrasonic_obstacle_guard6_distance_cm",
        "sys_route_gui_fsm_top#ultrasonic_obstacle_guard6&gt;distance_cm",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;ultrasonic_distance_cm",
        16,
    )
    text = append_wire(
        text,
        "ultrasonic_obstacle_guard6_distance_valid",
        "sys_route_gui_fsm_top#ultrasonic_obstacle_guard6&gt;distance_valid",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;ultrasonic_distance_valid",
    )
    text = append_wire(
        text,
        "ultrasonic_obstacle_guard6_echo_timeout",
        "sys_route_gui_fsm_top#ultrasonic_obstacle_guard6&gt;echo_timeout",
        "sys_route_gui_fsm_top#bt_route_uart16&gt;ultrasonic_echo_timeout",
    )
    diagnostic_wires = [
        ("c5_motor_frame_driver10_frame_count", "frame_count", "c5_frame_count", 16),
        ("c5_motor_frame_driver10_last_lf_pulse", "last_lf_pulse", "c5_last_lf_pulse", 12),
        ("c5_motor_frame_driver10_last_rf_pulse", "last_rf_pulse", "c5_last_rf_pulse", 12),
        ("c5_motor_frame_driver10_last_lr_pulse", "last_lr_pulse", "c5_last_lr_pulse", 12),
        ("c5_motor_frame_driver10_last_rr_pulse", "last_rr_pulse", "c5_last_rr_pulse", 12),
        ("c5_motor_frame_driver10_stop_frames_complete", "stop_frames_complete", "c5_stop_frames_complete", 1),
        ("c5_motor_frame_driver10_stop_frames_remaining", "stop_frames_remaining", "c5_stop_frames_remaining", 2),
        ("c5_motor_frame_driver10_frame_busy", "frame_busy", "c5_frame_busy", 1),
    ]
    for name, source_port, destination_port, size in diagnostic_wires:
        text = append_wire(
            text,
            name,
            f"sys_route_gui_fsm_top#c5_motor_frame_driver10&gt;{source_port}",
            f"sys_route_gui_fsm_top#bt_route_uart16&gt;{destination_port}",
            size,
        )
    text = upsert_wire_to(
        text,
        "c5_motion_stabilizer29_force_stop",
        "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;force_stop",
        "sys_route_gui_fsm_top#c5_motor_frame_driver10&gt;force_stop",
    )
    calibration_wires = [
        ("bt_route_uart16_gyro_bias_learn_pulse", "bt_route_uart16", "gyro_bias_learn_pulse", "onboard_occupancy_map30", "gyro_bias_learn_pulse", 1),
        ("bt_route_uart16_local_calibration_start_pulse", "bt_route_uart16", "local_calibration_start_pulse", "c5_motion_stabilizer29", "local_calibration_start", 1),
        ("bt_route_uart16_motor_stop_pulse", "bt_route_uart16", "motor_stop_pulse", "c5_motion_stabilizer29", "external_force_stop", 1),
        ("obstacle_stop_merge20_obstacle_stop", "obstacle_stop_merge20", "obstacle_stop", "c5_motion_stabilizer29", "obstacle_stop", 1),
        ("car_param_controller22_lf_default_speed", "car_param_controller22", "lf_default_speed", "c5_motion_stabilizer29", "cal_lf_default_speed", 10),
        ("car_param_controller22_rf_default_speed", "car_param_controller22", "rf_default_speed", "c5_motion_stabilizer29", "cal_rf_default_speed", 10),
        ("car_param_controller22_lr_default_speed", "car_param_controller22", "lr_default_speed", "c5_motion_stabilizer29", "cal_lr_default_speed", 10),
        ("car_param_controller22_rr_default_speed", "car_param_controller22", "rr_default_speed", "c5_motion_stabilizer29", "cal_rr_default_speed", 10),
        ("c5_motion_stabilizer29_local_calibration_state", "c5_motion_stabilizer29", "local_calibration_state", "bt_route_uart16", "local_calibration_state", 4),
        ("c5_motion_stabilizer29_local_calibration_flags", "c5_motion_stabilizer29", "local_calibration_flags", "bt_route_uart16", "local_calibration_flags", 8),
        ("c5_motion_stabilizer29_learned_yaw_trim", "c5_motion_stabilizer29", "learned_yaw_trim", "bt_route_uart16", "learned_yaw_trim", 11),
        ("c5_motion_stabilizer29_yaw_correction", "c5_motion_stabilizer29", "yaw_correction", "bt_route_uart16", "yaw_correction", 11),
        ("c5_motion_stabilizer29_calibration_gyro_peak", "c5_motion_stabilizer29", "calibration_gyro_peak", "bt_route_uart16", "calibration_gyro_peak", 16),
        ("c5_motion_stabilizer29_vehicle_motion_expected", "c5_motion_stabilizer29", "vehicle_motion_expected", "motion_sensor_subsystem28", "vehicle_motion_expected", 1),
    ]
    for name, source_instance, source_port, destination_instance, destination_port, size in calibration_wires:
        text = upsert_wire_to(
            text,
            name,
            f"sys_route_gui_fsm_top#{source_instance}&gt;{source_port}",
            f"sys_route_gui_fsm_top#{destination_instance}&gt;{destination_port}",
            size,
        )

    positioning_wires = [
        ("sys_route_gui_fsm_top_clk_linkpg", "sys_route_gui_fsm_top&gt;clk", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;clk", 1),
        ("linkpg_tag_position_uart33_rst_n", "sys_route_gui_fsm_top#power_on_reset24&gt;system_rst_n", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;rst_n", 1),
        ("sys_route_gui_fsm_top_uwb_uart_rx", "sys_route_gui_fsm_top&gt;uwb_uart_rx", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;uwb_uart_rx", 1),
        ("sys_route_gui_fsm_top_uwb_uart_tx", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;uwb_uart_tx", "sys_route_gui_fsm_top&gt;uwb_uart_tx", 1),
        ("linkpg_position_valid_map", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_valid", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;uwb_position_valid", 1),
        ("linkpg_position_online_map", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_online", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;uwb_position_online", 1),
        ("linkpg_position_x_map", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_x_cm", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;uwb_position_x_cm", 16),
        ("linkpg_position_y_map", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_y_cm", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;uwb_position_y_cm", 16),
        ("map_flow_valid_stabilizer", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;flow_body_sample_valid", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;flow_sample_valid", 1),
        ("map_flow_forward_stabilizer", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;flow_body_forward_q8", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;flow_forward_q8", 16),
        ("map_flow_lateral_stabilizer", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;flow_body_lateral_q8", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;flow_lateral_q8", 16),
        ("sys_route_gui_fsm_top_clk_debug", "sys_route_gui_fsm_top&gt;clk", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;clk", 1),
        ("vehicle_debug_uart34_rst_n", "sys_route_gui_fsm_top#power_on_reset24&gt;system_rst_n", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;rst_n", 1),
        ("bt_connected_debug", "sys_route_gui_fsm_top#bt_route_uart16&gt;bt_connected", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;bt_connected", 1),
        ("lidar_enable_debug", "sys_route_gui_fsm_top#bt_route_uart16&gt;lidar_enable", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;lidar_enable", 1),
        ("lidar_scan_active_debug", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;scan_active", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;lidar_scan_active", 1),
        ("lidar_online_debug", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;lidar_online", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;lidar_online", 1),
        ("uwb_online_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_online", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_position_online", 1),
        ("uwb_good_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_good", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_position_good", 1),
        ("uwb_ack_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;start_ack_seen", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_start_ack_seen", 1),
        ("uwb_x_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_x_cm", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_position_x_cm", 16),
        ("uwb_y_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;position_y_cm", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_position_y_cm", 16),
        ("uwb_packets_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;rx_packet_count", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_packet_count", 16),
        ("uwb_crc_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;rx_crc_error_count", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_crc_error_count", 16),
        ("uwb_start_debug", "sys_route_gui_fsm_top#linkpg_tag_position_uart33&gt;start_request_count", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;uwb_start_request_count", 16),
        ("map_pose_x_debug", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_x_mm", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;map_pose_x_mm", 16),
        ("map_pose_y_debug", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_y_mm", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;map_pose_y_mm", 16),
        ("map_heading_debug", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_heading", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;map_pose_heading", 10),
        ("flow_correction_debug", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;flow_correction", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;flow_correction", 11),
        ("yaw_correction_debug", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;yaw_correction", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;yaw_correction", 11),
        ("motion_telemetry_debug", "sys_route_gui_fsm_top#motion_sensor_subsystem28&gt;telemetry_data", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;motion_telemetry_data", 320),
        ("sys_route_gui_fsm_top_debug_uart_tx", "sys_route_gui_fsm_top#vehicle_debug_uart34&gt;debug_uart_tx", "sys_route_gui_fsm_top&gt;debug_uart_tx", 1),
    ]
    for name, source, destination, size in positioning_wires:
        text = upsert_wire_to(text, name, source, destination, size)

    field_config_wires = [
        ("field_config_clk", "sys_route_gui_fsm_top&gt;clk", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;clk", 1),
        ("field_config_rst_n", "sys_route_gui_fsm_top#power_on_reset24&gt;system_rst_n", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;rst_n", 1),
        ("field_config_bt_rx", "sys_route_gui_fsm_top&gt;bt_uart_rx", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;bt_uart_rx", 1),
        ("field_config_valid_map", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;config_valid", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;field_config_valid", 1),
        ("field_config_pose_x_map", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;pose_x_cell", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;field_pose_x_cell", 8),
        ("field_config_pose_y_map", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;pose_y_cell", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;field_pose_y_cell", 8),
        ("field_config_heading_map", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;heading_1024", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;field_heading_1024", 10),
        ("field_config_valid_explore", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;config_valid", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;field_config_valid", 1),
        ("field_config_low_x_explore", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;low_x_cell", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;field_low_x_cell", 8),
        ("field_config_high_x_explore", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;high_x_cell", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;field_high_x_cell", 8),
        ("field_config_low_y_explore", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;low_y_cell", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;field_low_y_cell", 8),
        ("field_config_high_y_explore", f"sys_route_gui_fsm_top#{FIELD_MAP_CONFIG}&gt;high_y_cell", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;field_high_y_cell", 8),
    ]
    for name, source, destination, size in field_config_wires:
        text = upsert_wire_to(text, name, source, destination, size)

    exploration_wires = [
        ("auto_exploration_clk", "sys_route_gui_fsm_top&gt;clk", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;clk", 1),
        ("auto_exploration_rst_n", "sys_route_gui_fsm_top#power_on_reset24&gt;system_rst_n", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;rst_n", 1),
        ("auto_exploration_map_active", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;map_active", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;map_active", 1),
        ("auto_exploration_map_paused", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;map_paused", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;map_paused", 1),
        ("auto_exploration_manual_mode", "sys_route_gui_fsm_top#bt_route_uart16&gt;manual_mode", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;manual_mode", 1),
        ("auto_exploration_calibration_done", "sys_route_gui_fsm_top#motion_sensor_subsystem28&gt;calibration_done", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sensor_calibration_done", 1),
        ("auto_exploration_gyro_polarity", "sys_route_gui_fsm_top#c5_motion_stabilizer29&gt;gyro_polarity_valid", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;gyro_polarity_valid", 1),
        ("auto_exploration_lidar_online", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;lidar_online", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;lidar_online", 1),
        ("auto_exploration_scan_active", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;scan_active", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;scan_active", 1),
        ("auto_exploration_new_scan", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;new_scan", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;lidar_new_scan", 1),
        ("auto_exploration_obstacle", "sys_route_gui_fsm_top#obstacle_stop_merge20&gt;obstacle_stop", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;obstacle_stop", 1),
        ("auto_exploration_safe_mm", "sys_route_gui_fsm_top#car_param_controller22&gt;lidar_safe_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;safe_distance_mm", 16),
        ("auto_exploration_front", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_front_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_front_mm", 16),
        ("auto_exploration_front_right", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_front_right_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_front_right_mm", 16),
        ("auto_exploration_right", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_right_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_right_mm", 16),
        ("auto_exploration_rear_right", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_rear_right_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_rear_right_mm", 16),
        ("auto_exploration_rear", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_rear_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_rear_mm", 16),
        ("auto_exploration_rear_left", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_rear_left_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_rear_left_mm", 16),
        ("auto_exploration_left", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_left_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_left_mm", 16),
        ("auto_exploration_front_left", "sys_route_gui_fsm_top#rplidar_c1_uart19&gt;sector_front_left_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;sector_front_left_mm", 16),
        ("auto_exploration_pose_x", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_x_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;pose_x_mm", 16),
        ("auto_exploration_pose_y", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_y_mm", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;pose_y_mm", 16),
        ("auto_exploration_heading", "sys_route_gui_fsm_top#onboard_occupancy_map30&gt;pose_heading", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;pose_heading", 10),
        ("auto_exploration_mode_c5", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;auto_mode", "sys_route_gui_fsm_top#c5_route_controller9&gt;auto_mode", 1),
        ("auto_exploration_cmd_c5", f"sys_route_gui_fsm_top#{AUTO_EXPLORATION}&gt;auto_cmd", "sys_route_gui_fsm_top#c5_route_controller9&gt;auto_cmd", 3),
    ]
    for name, source, destination, size in exploration_wires:
        text = upsert_wire_to(text, name, source, destination, size)
    text = normalize_fanout_wire_names(text)
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")


def sync_verilog_mirrors() -> None:
    for name in (
        "bt_route_uart.v",
        "c5_route_controller.v",
        "c5_motor_frame_driver.v",
        "gui_route_renderer.v",
        "c5_motion_stabilizer.v",
        "onboard_occupancy_map.v",
        "sd_spi_block_device.v",
        "map_sd_store_controller.v",
        "sd_card_detect_adapter.v",
        "map_sd_store.v",
        "motion_sensor_subsystem.v",
        "rplidar_c1_uart.v",
        "linkpg_tag_position_uart.v",
        "vehicle_debug_uart.v",
        "onboard_exploration_controller.v",
        "field_map_config_uart.v",
    ):
        source = BUILD_DIR / name
        destination = MIRROR_DIR / name
        destination.write_text(source.read_text(encoding="utf-8"), encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--models-only",
        action="store_true",
        help="refresh embedded model code without copying generated build RTL",
    )
    args = parser.parse_args()
    synced_models = sync_behavioral_model_code_from_build()
    sync_top_model()
    register_models()
    register_verilog_sources()
    sync_top_const()
    if args.models_only:
        print(f"synced {len(synced_models)} behavioral models and vehicle top")
    else:
        sync_verilog_mirrors()
        print("synced vehicle runtime models and tracked Verilog mirrors")


if __name__ == "__main__":
    main()
