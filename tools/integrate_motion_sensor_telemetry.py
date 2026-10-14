#!/usr/bin/env python3
"""Integrate the validated PMW3901 + dual BMI088 subsystem into the car top."""

from __future__ import annotations

from pathlib import Path
import html
import re
import xml.etree.ElementTree as ET

from integrate_vl53_telemetry import (
    append_before_root_close,
    insert_before_first_root_port,
    insert_before_first_wire,
    instance_from_leaf,
    remove_instance,
    replace_instance,
    root_opening_tag,
)
from robei_converter import model_to_verilog, verilog_to_model


ROOT = Path(__file__).resolve().parents[1]
CAR_DIR = ROOT / "little_car"
MODEL_DIR = CAR_DIR / "model"
BUILD_DIR = CAR_DIR / "build"
SOURCE_DIR = CAR_DIR / "hdmi_rgb_input"
TOP_MODEL = MODEL_DIR / "sys_route_gui_fsm_top.model"
TOP_CONST = CAR_DIR / "const" / "top.const"
TOP_XDC = BUILD_DIR / "top.xdc"
PROJECT = CAR_DIR / "little_car.proj"
FILELIST = BUILD_DIR / "filelist.vf"

TOP_NAME = "sys_route_gui_fsm_top"
TOP_CONST_INSTANCE = "sys_route_gui_fsm_top1"
BT_INSTANCE = "bt_route_uart16"
RESET_INSTANCE = "power_on_reset24"
MOTION_INSTANCE = "motion_sensor_subsystem28"

TOP_SENSOR_PORTS = (
    "sensor_spi_miso",
    "sensor_spi_sclk",
    "sensor_spi_mosi",
    "pmw3901_cs_n",
    "bmi0_acc_cs_n",
    "bmi0_gyro_cs_n",
    "bmi1_acc_cs_n",
    "bmi1_gyro_cs_n",
)

# logical name, package pin, pull-up, drive strength (blank for input)
PIN_MAP = (
    ("sensor_spi_sclk", "A14", "up", "4"),
    ("sensor_spi_mosi", "B15", "none", "4"),
    ("sensor_spi_miso", "A13", "up", ""),
    ("pmw3901_cs_n", "B13", "up", "4"),
    ("bmi0_acc_cs_n", "F20", "up", "4"),
    ("bmi0_gyro_cs_n", "F19", "up", "4"),
    ("bmi1_acc_cs_n", "C20", "up", "4"),
    ("bmi1_gyro_cs_n", "E19", "up", "4"),
)


def instance_from_top_model() -> str:
    text = TOP_MODEL.read_text(encoding="utf-8")
    tag = root_opening_tag(text)
    replacements = {
        "Class": TOP_NAME,
        "Name": TOP_CONST_INSTANCE,
        "Parent": "top",
        "X": "330",
        "Y": "250",
        "Width": "2300",
        "Height": "1200",
    }
    for attribute, value in replacements.items():
        tag, count = re.subn(
            rf'\b{re.escape(attribute)}="[^"]*"',
            f'{attribute}="{value}"',
            tag,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"top model is missing root attribute {attribute}")

    ports = []
    for line in text.splitlines():
        if (line.lstrip().startswith("<Port ") and
                f'Parent="{TOP_NAME}"' in line):
            line = " " + line.lstrip()
            line = re.sub(
                r'Parent="[^"]+"', f'Parent="{TOP_CONST_INSTANCE}"', line
            )
            ports.append(line)
    return " " + tag + "\n" + "\n".join(ports) + "\n </Module>"


def prepare_leaf_models() -> None:
    sources = {
        "motion_sensor_subsystem": BUILD_DIR / "motion_sensor_subsystem.v",
        "bt_route_uart": BUILD_DIR / "bt_route_uart.v",
    }
    for name, source in sources.items():
        model_text = verilog_to_model(source.read_text(encoding="utf-8"))
        (MODEL_DIR / f"{name}.model").write_text(
            model_text, encoding="utf-8", newline="\n"
        )
        generated = model_to_verilog(model_text)
        (BUILD_DIR / f"{name}.v").write_text(
            generated, encoding="utf-8", newline="\n"
        )
        (SOURCE_DIR / f"{name}.v").write_text(
            generated, encoding="utf-8", newline="\n"
        )


def remove_top_sensor_ports(text: str) -> str:
    for port_name in TOP_SENSOR_PORTS:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="{TOP_NAME}")'
            rf'(?=[^>]*\bName="{port_name}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def remove_top_sensor_wires(text: str) -> str:
    names = "|".join(re.escape(name) for name in TOP_SENSOR_PORTS)
    conditions = (
        rf'(?=[^>]*(?:From|To)="[^"]*{MOTION_INSTANCE}[^"]*")',
        rf'(?=[^>]*(?:From|To)="{TOP_NAME}&gt;(?:{names})")',
        rf'(?=[^>]*(?:From|To)="{TOP_NAME}#{BT_INSTANCE}&gt;motion_telemetry_data")',
    )
    for condition in conditions:
        text = re.sub(
            rf'^[ \t]*<Wire\b{condition}[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
    return text


def integrate_top_model() -> None:
    text = TOP_MODEL.read_text(encoding="utf-8")
    text = replace_instance(
        text,
        BT_INSTANCE,
        instance_from_leaf(
            "bt_route_uart",
            BT_INSTANCE,
            parent=TOP_NAME,
            x="2970",
            y="545",
            width="1650",
            height="1110",
        ),
    )
    text = remove_instance(text, MOTION_INSTANCE)
    text = insert_before_first_root_port(
        text,
        instance_from_leaf(
            "motion_sensor_subsystem",
            MOTION_INSTANCE,
            parent=TOP_NAME,
            x="4750",
            y="5650",
            width="1500",
            height="1120",
        ),
        TOP_NAME,
    )
    text = remove_top_sensor_ports(text)
    text = remove_top_sensor_wires(text)

    root_ports = " " + """
 <Port Color="#6495ed" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="input" Name="sensor_spi_miso" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.10" Y="0.994" />
 <Port Color="#fff8dc" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="sensor_spi_sclk" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.18" Y="0.994" />
 <Port Color="#dc143c" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="sensor_spi_mosi" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.26" Y="0.994" />
 <Port Color="#00ffff" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="pmw3901_cs_n" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.34" Y="0.994" />
 <Port Color="#00008b" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="bmi0_acc_cs_n" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.42" Y="0.994" />
 <Port Color="#008b8b" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="bmi0_gyro_cs_n" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.50" Y="0.994" />
 <Port Color="#b8860b" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="bmi1_acc_cs_n" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.58" Y="0.994" />
 <Port Color="#ff1493" Connect="" Datasize="1" Datatype="wire" Description="" Height="30" Inout="output" Name="bmi1_gyro_cs_n" Parent="sys_route_gui_fsm_top" Side="bottom" Width="30" X="0.66" Y="0.994" />
""".strip()
    text = insert_before_first_wire(text, root_ports)

    wires = f"""
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;clk" Name="{TOP_NAME}_clk_{MOTION_INSTANCE}" Parent="{TOP_NAME}" To="{TOP_NAME}#{MOTION_INSTANCE}&gt;clk" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{RESET_INSTANCE}&gt;system_rst_n" Name="{MOTION_INSTANCE}_rst_n" Parent="{TOP_NAME}" To="{TOP_NAME}#{MOTION_INSTANCE}&gt;rst_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}&gt;sensor_spi_miso" Name="{MOTION_INSTANCE}_sensor_spi_miso" Parent="{TOP_NAME}" To="{TOP_NAME}#{MOTION_INSTANCE}&gt;sensor_spi_miso" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;sensor_spi_sclk" Name="{TOP_NAME}_sensor_spi_sclk" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;sensor_spi_sclk" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;sensor_spi_mosi" Name="{TOP_NAME}_sensor_spi_mosi" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;sensor_spi_mosi" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;pmw3901_cs_n" Name="{TOP_NAME}_pmw3901_cs_n" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;pmw3901_cs_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;bmi0_acc_cs_n" Name="{TOP_NAME}_bmi0_acc_cs_n" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;bmi0_acc_cs_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;bmi0_gyro_cs_n" Name="{TOP_NAME}_bmi0_gyro_cs_n" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;bmi0_gyro_cs_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;bmi1_acc_cs_n" Name="{TOP_NAME}_bmi1_acc_cs_n" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;bmi1_acc_cs_n" />
 <Wire Datasize="1" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;bmi1_gyro_cs_n" Name="{TOP_NAME}_bmi1_gyro_cs_n" Parent="{TOP_NAME}" To="{TOP_NAME}&gt;bmi1_gyro_cs_n" />
 <Wire Datasize="320" Datatype="wire" From="{TOP_NAME}#{MOTION_INSTANCE}&gt;telemetry_data" Name="{MOTION_INSTANCE}_telemetry_data" Parent="{TOP_NAME}" To="{TOP_NAME}#{BT_INSTANCE}&gt;motion_telemetry_data" />
"""
    text = append_before_root_close(text, wires)
    TOP_MODEL.write_text(text, encoding="utf-8", newline="\n")
    (BUILD_DIR / f"{TOP_NAME}.v").write_text(
        model_to_verilog(text), encoding="utf-8", newline="\n"
    )


def update_const_code(text: str) -> str:
    match = re.search(
        r'(<Module\b(?=[^>]*\bName="top")(?=[^>]*\bCode=")[^>]*\bCode=")'
        r'([^"]*)(")',
        text,
    )
    if not match:
        raise RuntimeError("cannot locate top.const Code attribute")
    code = html.unescape(match.group(2)).replace("\r\n", "\n")
    code_lines = [
        line for line in code.splitlines()
        if not any(name in line for name in TOP_SENSOR_PORTS)
    ]
    code_lines.extend((
        "set_property DRIVE 4 [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
        "set_property SLEW SLOW [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
        "set_property PULLUP TRUE [get_ports {sensor_spi_sclk sensor_spi_miso pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
    ))
    encoded = html.escape("\n".join(code_lines), quote=True).replace("\n", "&#xa;")
    return text[:match.start(2)] + encoded + text[match.end(2):]


def integrate_top_const() -> None:
    text = TOP_CONST.read_text(encoding="utf-8")
    text = replace_instance(text, TOP_CONST_INSTANCE, instance_from_top_model())
    for port_name, pin, _, _ in PIN_MAP:
        text = re.sub(
            rf'^[ \t]*<Port\b(?=[^>]*\bParent="top")'
            rf'(?=[^>]*\bName="{pin}")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )
        text = re.sub(
            rf'^[ \t]*<Wire\b(?=[^>]*(?:From|To)="top(?:#{TOP_CONST_INSTANCE})?&gt;(?:{re.escape(port_name)}|{pin})")[^>]*/>[ \t]*\r?\n?',
            "",
            text,
            flags=re.M,
        )

    physical_ports = []
    for index, (_, pin, pull, drive) in enumerate(PIN_MAP):
        physical_ports.append(
            f' <Port Color="#6495ed" Description="" Drive="{drive}" Height="30" '
            f'Name="{pin}" Parent="top" Pull="{pull}" Side="top" '
            f'Voltage="LVCMOS33" Width="30" X="{0.08 + index * 0.10:.2f}" Y="-0.02"/>'
        )
    text = insert_before_first_wire(text, "\n".join(physical_ports))

    wires = []
    for port_name, pin, _, _ in PIN_MAP:
        wires.append(
            f' <Wire Datasize="1" Datatype="wire" '
            f'From="top#{TOP_CONST_INSTANCE}&gt;{port_name}" Name="{port_name}_{pin}" '
            f'Parent="top" To="top&gt;{pin}" />'
        )
    text = append_before_root_close(text, "\n".join(wires))
    TOP_CONST.write_text(update_const_code(text), encoding="utf-8", newline="\n")


def update_xdc() -> None:
    text = TOP_XDC.read_text(encoding="utf-8")
    text = "\n".join(
        line for line in text.splitlines()
        if not any(name in line for name in TOP_SENSOR_PORTS)
    ).rstrip()
    lines = [""]
    for port_name, pin, _, _ in PIN_MAP:
        lines.append(f"set_property PACKAGE_PIN {pin} [get_ports {port_name}]")
    lines.extend((
        "set_property IOSTANDARD LVCMOS33 [get_ports {sensor_spi_sclk sensor_spi_mosi sensor_spi_miso pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
        "set_property DRIVE 4 [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
        "set_property SLEW SLOW [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
        "set_property PULLUP TRUE [get_ports {sensor_spi_sclk sensor_spi_miso pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]",
    ))
    TOP_XDC.write_text(text + "\n" + "\n".join(lines) + "\n", encoding="utf-8")


def update_filelist() -> None:
    entries = [line for line in FILELIST.read_text(encoding="utf-8").splitlines() if line]
    motion_path = str((BUILD_DIR / "motion_sensor_subsystem.v").resolve()).replace("\\", "/")
    entries = [line for line in entries if line != motion_path]
    top_path = str((BUILD_DIR / f"{TOP_NAME}.v").resolve()).replace("\\", "/")
    entries.insert(entries.index(top_path), motion_path)
    FILELIST.write_text("\n".join(entries) + "\n", encoding="utf-8", newline="\n")


def register_model() -> None:
    text = PROJECT.read_text(encoding="utf-8")
    if 'name="motion_sensor_subsystem.model"' not in text:
        marker = re.search(r"^[ \t]*</model>[ \t]*$", text, flags=re.M)
        if not marker:
            raise RuntimeError("project model folder closing tag is missing")
        entry = ('  <motion_sensor_subsystem.model '
                 'name="motion_sensor_subsystem.model" parent="model" type="model"/>\n')
        text = text[:marker.start()] + entry + text[marker.start():]
        PROJECT.write_text(text, encoding="utf-8", newline="\n")


def validate() -> None:
    top = ET.parse(TOP_MODEL).getroot()
    modules = {module.attrib.get("Name"): module for module in top.findall("Module")}
    if MOTION_INSTANCE not in modules:
        raise RuntimeError("motion sensor Robei instance is missing")
    motion_port_count = len(ET.parse(MODEL_DIR / "motion_sensor_subsystem.model").getroot().findall("Port"))
    bt_port_count = len(ET.parse(MODEL_DIR / "bt_route_uart.model").getroot().findall("Port"))
    if len(modules[MOTION_INSTANCE].findall("Port")) != motion_port_count:
        raise RuntimeError("motion sensor instance ports do not match its leaf model")
    if len(modules[BT_INSTANCE].findall("Port")) != bt_port_count:
        raise RuntimeError("Bluetooth instance ports do not match its leaf model")

    top_ports = {port.attrib.get("Name") for port in top.findall("Port")}
    if not set(TOP_SENSOR_PORTS).issubset(top_ports):
        raise RuntimeError("motion sensor top ports are incomplete")
    destinations = [wire.attrib.get("To") for wire in top.findall("Wire")]
    for destination in (
        f"{TOP_NAME}#{MOTION_INSTANCE}>sensor_spi_miso",
        f"{TOP_NAME}#{BT_INSTANCE}>motion_telemetry_data",
        f"{TOP_NAME}>pmw3901_cs_n",
    ):
        if destinations.count(destination) != 1:
            raise RuntimeError(f"expected exactly one wire to {destination}")

    const_root = ET.parse(TOP_CONST).getroot()
    const_instance = const_root.find("Module")
    if const_instance is None or len(const_instance.findall("Port")) != len(top_ports):
        raise RuntimeError("top.const instance does not mirror top model ports")
    generated_top = (BUILD_DIR / f"{TOP_NAME}.v").read_text(encoding="utf-8")
    for token in (
        "motion_sensor_subsystem motion_sensor_subsystem28",
        ".motion_telemetry_data(motion_sensor_subsystem28_telemetry_data)",
        "sensor_spi_miso",
    ):
        if token not in generated_top:
            raise RuntimeError(f"generated top is missing {token}")


def main() -> None:
    prepare_leaf_models()
    integrate_top_model()
    integrate_top_const()
    update_xdc()
    update_filelist()
    register_model()
    validate()
    print("PMW3901 + dual BMI088 telemetry integrated into Robei and the car build graph")


if __name__ == "__main__":
    main()
