#!/usr/bin/env python3
"""Generate compact Robei hardware-parameter tests and Icarus mirrors."""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from generate_robei_experiments import build_test  # noqa: E402


MODEL_DIR = ROOT / "little_car" / "model"
TB_DIR = ROOT / "little_car" / "tb"


def model_ports(model_name: str) -> tuple[list[str], list[str]]:
    root = ET.fromstring(
        (MODEL_DIR / model_name).read_text(encoding="utf-8")
        .replace("<!DOCTYPE Robei>", "", 1)
    )
    inputs = [p.get("Name", "") for p in root.findall("Port")
              if p.get("Inout") == "input"]
    outputs = [p.get("Name", "") for p in root.findall("Port")
               if p.get("Inout") == "output"]
    return inputs, outputs


def extract_body(tb_name: str, model_name: str, defaults: dict[str, str],
                 wave_signals: list[str]) -> str:
    """Turn a conventional single-DUT TB into Robei root test Code.

    Robei owns root port declarations and embeds the DUT model graphically, so
    declarations and the textual DUT instance are removed. Aliases used by a
    conventional Verilog instance are rewritten to the model's real port names.
    """
    path = TB_DIR / f"{tb_name}.v"
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"(?m)^`timescale.*\n", "", text)
    text = re.sub(r"(?m)^`define.*\n", "", text)
    text = re.sub(rf"(?s)^\s*module\s+{re.escape(tb_name)}\s*;\s*", "", text)
    text = re.sub(r"(?s)\s*endmodule\s*$", "", text)

    model_class = Path(model_name).stem
    match = re.search(
        rf"(?s)\b{re.escape(model_class)}\s+dut\s*\((.*?)\)\s*;", text
    )
    if not match:
        raise RuntimeError(f"cannot find {model_class} dut in {path}")
    connections = match.group(1)
    aliases: dict[str, str] = {}
    for port, expression in re.findall(
        r"\.([A-Za-z_$][\w$]*)\s*\(\s*([^()]*)\s*\)", connections
    ):
        expression = expression.strip()
        if re.fullmatch(r"[A-Za-z_$][\w$]*", expression) and expression != port:
            aliases[expression] = port
    text = text[:match.start()] + text[match.end():]
    for alias in sorted(aliases, key=len, reverse=True):
        text = re.sub(rf"\b{re.escape(alias)}\b", aliases[alias], text)
    text = re.sub(r"\bdut\.", f"{model_class}0.", text)

    inputs, outputs = model_ports(model_name)
    all_ports = set(inputs + outputs)
    # Root reg/wire ports are generated from the model. Remove declarations
    # that now name those ports, while preserving local arrays and counters.
    declaration = re.compile(r"(?ms)^\s*(?:reg|wire)\b.*?;\s*\n")
    kept: list[str] = []
    cursor = 0
    for item in declaration.finditer(text):
        kept.append(text[cursor:item.start()])
        declared = item.group(0)
        if not any(re.search(rf"\b{re.escape(port)}\b", declared)
                   for port in all_ports):
            kept.append(declared)
        cursor = item.end()
    kept.append(text[cursor:])
    text = "".join(kept)
    text = re.sub(r"(?m)^\s*\$dump(?:file|vars|on|off)\s*\(.*?\);\s*$", "", text)

    missing_defaults = set(inputs) - set(defaults)
    if missing_defaults:
        raise RuntimeError(f"missing defaults for {tb_name}: {sorted(missing_defaults)}")
    init_lines = "\n".join(f"  {name} = {defaults[name]};" for name in inputs)
    wave_args = ", ".join(wave_signals)
    prefix = f'''\n`ifdef PARAM_WAVE
initial begin
  $dumpfile("{tb_name}.vcd");
  $dumpvars(0, {wave_args});
end
`endif

initial begin
{init_lines}
end
'''
    return prefix + "\n" + text.strip() + "\n"


TESTS = [
    {
        "name": "power_on_reset_parameter",
        "source": "tb_power_on_reset_parameter",
        "model": "power_on_reset.model",
        "defaults": {"clk": "1'b0", "ext_rst_n": "1'b0"},
        "waves": ["ext_rst_n", "system_rst_n", "power_on_reset0.por_count"],
    },
    {
        "name": "hdmi_timing_parameter",
        "source": "tb_hdmi_timing_parameter",
        "model": "hdmi_timing_gen.model",
        "defaults": {"clk_pix": "1'b0", "rst_n": "1'b0"},
        "waves": ["rst_n", "hsync", "vsync", "v_de", "pixel_x", "pixel_y",
                  "hdmi_timing_gen0.h_cnt", "hdmi_timing_gen0.v_cnt"],
    },
    {
        "name": "ultrasonic_parameter",
        "source": "tb_ultrasonic_parameter",
        "model": "ultrasonic_obstacle_guard.model",
        "defaults": {"clk": "1'b0", "rst_n": "1'b0", "echo": "1'b0",
                     "step_length": "16'd20", "enable": "1'b1"},
        # Deliberately omit clk: 174k clock cycles otherwise dominate the VCD.
        "waves": ["rst_n", "echo", "trig", "distance_cm", "distance_valid",
                  "obstacle_stop", "echo_timeout",
                  "ultrasonic_obstacle_guard0.state"],
    },
    {
        "name": "c5_motion_stabilizer_parameter",
        "source": "tb_c5_motion_stabilizer",
        "model": "c5_motion_stabilizer.model",
        "prefix": "`define C5_CAL_SIM_FAST",
        "defaults": {
            "clk": "1'b0", "rst_n": "1'b0", "calibration_done": "1'b0",
            "fused_sample_valid": "1'b0", "imu_degraded": "1'b0",
            "imu_disagreement": "1'b0", "fused_gyro_z": "16'd0",
            "flow_sample_valid": "1'b0", "flow_forward_q8": "16'd0",
            "flow_lateral_q8": "16'd0", "local_calibration_start": "1'b0",
            "external_force_stop": "1'b0", "obstacle_stop": "1'b0",
            "cal_lf_default_speed": "10'd700", "cal_rf_default_speed": "10'd700",
            "cal_lr_default_speed": "10'd700", "cal_rr_default_speed": "10'd700",
            "base_lf_speed": "11'd0", "base_rf_speed": "11'd0",
            "base_lr_speed": "11'd0", "base_rr_speed": "11'd0",
        },
        "waves": ["rst_n", "fused_sample_valid", "fused_gyro_z",
                  "base_lf_speed", "base_rf_speed", "lf_speed", "rf_speed",
                  "correction_active", "yaw_correction", "force_stop"],
    },
    {
        "name": "dual_imu_calibration_parameter",
        "source": "tb_dual_imu_static_calibrator",
        "model": "dual_imu_static_calibrator.model",
        "prefix": "`define DUAL_IMU_CAL_SIM_FAST",
        "defaults": {
            "clk": "1'b0", "rst_n": "1'b0", "start": "1'b0",
            "sample_valid": "1'b0", "imu0_online": "1'b1", "imu1_online": "1'b1",
            "imu0_accel_x": "16'sd30", "imu0_accel_y": "-16'sd160",
            "imu0_accel_z": "-16'sd5504", "imu0_gyro_x": "16'sd2",
            "imu0_gyro_y": "-16'sd3", "imu0_gyro_z": "16'sd4",
            "imu1_accel_x": "16'sd1176", "imu1_accel_y": "-16'sd5350",
            "imu1_accel_z": "16'sd165", "imu1_gyro_x": "16'sd30",
            "imu1_gyro_y": "-16'sd1", "imu1_gyro_z": "-16'sd11",
        },
        "waves": ["rst_n", "start", "sample_valid", "imu0_online", "imu1_online",
                  "busy", "done", "pass", "result_flags", "imu0_gyro_peak",
                  "imu1_gyro_peak"],
    },
    {
        "name": "rplidar_protocol_parameter",
        "source": "tb_rplidar_c1_uart_restart",
        "model": "rplidar_c1_uart.model",
        "defaults": {"clk": "1'b0", "rst_n": "1'b0", "lidar_uart_rx": "1'b1",
                     "safe_distance_mm": "16'd450", "scan_enable": "1'b1"},
        "waves": ["rst_n", "scan_enable", "lidar_uart_tx", "lidar_online",
                  "scan_active", "sample_valid", "new_scan", "quality", "angle_q6",
                  "distance_q2", "front_distance_mm", "obstacle_stop",
                  "rplidar_c1_uart0.cmd_state"],
    },
    {
        "name": "lidar_map_sampler_parameter",
        "source": "tb_lidar_map_sampler",
        "model": "lidar_map_sampler.model",
        "defaults": {"clk": "1'b0", "rst_n": "1'b0", "scan_enable": "1'b0",
                     "sample_valid": "1'b0", "new_scan": "1'b0", "quality": "6'd0",
                     "angle_q6": "15'd0", "distance_q2": "16'd0",
                     "map_chunk_accept": "1'b0"},
        "waves": ["rst_n", "scan_enable", "sample_valid", "new_scan", "quality",
                  "angle_q6", "distance_q2", "map_chunk_valid", "map_scan_seq",
                  "map_part", "map_chunk_crc", "map_drop_count"],
    },
    {
        "name": "onboard_exploration_parameter",
        "source": "tb_onboard_exploration_controller",
        "model": "onboard_exploration_controller.model",
        "prefix": "`define ONBOARD_EXPLORATION_SIM_FAST",
        "defaults": {
            "clk": "1'b0", "rst_n": "1'b0", "map_active": "1'b0",
            "map_paused": "1'b0", "manual_mode": "1'b0",
            "sensor_calibration_done": "1'b1", "gyro_polarity_valid": "1'b1",
            "lidar_online": "1'b1", "scan_active": "1'b1",
            "lidar_new_scan": "1'b0", "obstacle_stop": "1'b0",
            "safe_distance_mm": "16'd450", "sector_front_mm": "16'd2000",
            "sector_front_right_mm": "16'd2000", "sector_right_mm": "16'd2000",
            "sector_rear_right_mm": "16'd2000", "sector_rear_mm": "16'd2000",
            "sector_rear_left_mm": "16'd2000", "sector_left_mm": "16'd2000",
            "sector_front_left_mm": "16'd2000", "pose_x_mm": "16'd3200",
            "pose_y_mm": "16'd3200", "pose_heading": "10'd0",
            "field_config_valid": "1'b0", "field_low_x_cell": "8'd10",
            "field_high_x_cell": "8'd40", "field_low_y_cell": "8'd10",
            "field_high_y_cell": "8'd44",
        },
        "waves": ["rst_n", "map_active", "map_paused", "manual_mode",
                  "lidar_new_scan", "obstacle_stop", "sector_front_mm",
                  "pose_x_mm", "pose_heading", "auto_mode", "auto_cmd",
                  "exploration_state", "stop_reason"],
    },
    {
        "name": "linkpg_uart_parameter",
        "source": "tb_linkpg_tag_position_uart",
        "model": "linkpg_tag_position_uart.model",
        "prefix": "`define LINKPG_POSITION_SIM_FAST",
        "defaults": {"clk": "1'b0", "rst_n": "1'b0", "uwb_uart_rx": "1'b1"},
        "waves": ["rst_n", "uwb_uart_rx", "uwb_uart_tx", "position_valid",
                  "position_online", "position_good", "position_x_cm",
                  "position_y_cm", "position_z_cm", "rx_packet_count",
                  "rx_crc_error_count", "start_request_count"],
    },
]


def main() -> None:
    standard_test_dir = ROOT / "robei_model_project" / "test"
    for spec in TESTS:
        code = extract_body(spec["source"], spec["model"], spec["defaults"],
                            spec["waves"])
        build_test(spec["name"], spec["model"], code,
                   model_prefix=spec.get("prefix", ""))
        if standard_test_dir.is_dir():
            shutil.copy2(ROOT / "little_car" / "test" /
                         f"{spec['name']}.test", standard_test_dir)
        print(f"generated {spec['name']}")


if __name__ == "__main__":
    main()
