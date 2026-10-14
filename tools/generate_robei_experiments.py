#!/usr/bin/env python3
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
TEST_DIR = ROOT / "little_car" / "test"
VERILOG_DIR = TEST_DIR / "verilog"


def width(port: ET.Element) -> str:
    size = int(port.attrib.get("Datasize", "1"))
    return "" if size == 1 else f"[{size - 1}:0] "


def render_verilog(
    name: str, dut: ET.Element, dut_class: str, dut_name: str, code: str
) -> str:
    ports = list(dut.findall("Port"))
    dut_header = ",\n".join(
        f"  {p.attrib['Inout']} {p.attrib.get('Datatype', 'wire')} "
        f"{width(p)}{p.attrib['Name']}"
        for p in ports
    )
    declarations = "\n".join(
        f"  {'reg' if p.attrib['Inout'] == 'input' else 'wire'} "
        f"{width(p)}{p.attrib['Name']};"
        for p in ports
    )
    connections = ",\n".join(
        f"    .{p.attrib['Name']}({p.attrib['Name']})" for p in ports
    )
    return (
        "`timescale 1ns/1ps\n"
        f"module {dut_class}(\n{dut_header}\n);\n"
        f"{dut.attrib.get('Code', '')}\nendmodule\n\n"
        f"module {name};\n{declarations}\n\n{code}\n\n"
        f"  {dut_class} {dut_name}(\n{connections}\n  );\nendmodule\n"
    )


def build_test(
    name: str,
    model_name: str,
    code: str,
    *,
    model_prefix: str = "",
) -> None:
    dut = ET.parse(MODEL_DIR / model_name).getroot()
    dut_class = dut.attrib.get("Name", Path(model_name).stem)
    if model_prefix:
        dut.attrib["Code"] = model_prefix.rstrip() + "\n" + dut.attrib.get("Code", "")
    dut_name = f"{dut_class}0"
    dut.attrib.update(
        Class=dut_class,
        Name=dut_name,
        Parent=name,
        Type="model",
        X="650",
        Y="300",
    )

    root = ET.Element(
        "Module",
        {
            "Category": "Current",
            "Class": "module",
            "Code": code,
            "Color": "#d3d3d3",
            "Comment": "Generated parameter experiment; keep console and CSV output.",
            "Height": "1080",
            "Name": name,
            "Parent": "0",
            "Timescale": "1ns/1ps",
            "Type": "test",
            "Width": "1600",
            "X": "0",
            "Y": "0",
        },
    )

    dut_ports = list(dut.findall("Port"))
    for port in dut_ports:
        port.attrib["Parent"] = dut_name
    root.append(deepcopy(dut))

    inputs = [p for p in dut_ports if p.attrib["Inout"] == "input"]
    outputs = [p for p in dut_ports if p.attrib["Inout"] == "output"]
    ordered = inputs + outputs
    for index, source_port in enumerate(ordered):
        port = deepcopy(source_port)
        is_input = source_port.attrib["Inout"] == "input"
        port.attrib.update(
            Parent=name,
            Datatype="reg" if is_input else "wire",
            Side="left" if is_input else "right",
            X="-0.01875" if is_input else "0.98125",
            Y=f"{(index + 1) / (len(ordered) + 1):.6f}",
        )
        root.append(port)

        port_name = source_port.attrib["Name"]
        size = source_port.attrib.get("Datasize", "1")
        root.append(
            ET.Element(
                "Wire",
                {
                    "Datasize": size,
                    "Datatype": "wire",
                    "From": f"{name}>{port_name}" if is_input else f"{name}#{dut_name}>{port_name}",
                    "Name": f"{name}_{port_name}",
                    "Parent": name,
                    "To": f"{name}#{dut_name}>{port_name}" if is_input else f"{name}>{port_name}",
                },
            )
        )

    ET.indent(root, space=" ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    (TEST_DIR / f"{name}.test").write_text(
        "<!DOCTYPE Robei>\n" + xml + "\n", encoding="utf-8", newline="\n"
    )
    VERILOG_DIR.mkdir(parents=True, exist_ok=True)
    (VERILOG_DIR / f"{name}.v").write_text(
        render_verilog(name, dut, dut_class, dut_name, code),
        encoding="utf-8",
        newline="\n",
    )


C5_TIMING = r'''
localparam integer BIT_NS = 8680;
integer fd;
integer errors;
integer bytes_seen;
integer count_before_idle;
reg [7:0] rx_byte;
reg [15:0] last_count;
time reset_release_ns;
time first_frame_start_ns;
time first_frame_done_ns;
time movement_start_ns;
time movement_done_ns;
time refresh_start_ns;

initial begin
  clk = 1'b0;
end
always #10 clk = ~clk;

task uart_receive;
  integer bit_index;
  integer stop_byte_index;
  reg [7:0] expected_stop_byte;
  begin
    @(negedge c5_bus_tx);
    #(BIT_NS + BIT_NS/2);
    for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
      rx_byte[bit_index] = c5_bus_tx;
      #(BIT_NS);
    end
    bytes_seen = bytes_seen + 1;
    $fwrite(fd, "UART,%0t,%0d,%02x,%c\n", $time, bytes_seen, rx_byte, rx_byte);
    if ((bytes_seen <= 27) || (bytes_seen >= 152)) begin
      if (bytes_seen <= 27)
        stop_byte_index = bytes_seen - 1;
      else
        stop_byte_index = bytes_seen - 152;
      case (stop_byte_index % 9)
        0: expected_stop_byte = "#";
        1: expected_stop_byte = "2";
        2,3: expected_stop_byte = "5";
        4: expected_stop_byte = "P";
        5: expected_stop_byte = "D";
        6: expected_stop_byte = "S";
        7: expected_stop_byte = "T";
        default: expected_stop_byte = "!";
      endcase
      if (rx_byte != expected_stop_byte) errors = errors + 1;
    end
    if ((bytes_seen == 28 || bytes_seen == 90) && rx_byte != "{")
      errors = errors + 1;
    if ((bytes_seen == 29 || bytes_seen == 91) && rx_byte != "#")
      errors = errors + 1;
    if ((bytes_seen == 89 || bytes_seen == 151) && rx_byte != "}")
      errors = errors + 1;
  end
endtask

always uart_receive();

always @(posedge frame_busy) begin
  if (frame_count == 0 && first_frame_start_ns == 0)
    first_frame_start_ns = $time;
  $fwrite(fd, "FRAME_START,%0t,%0d,%0d\n", $time, frame_count, stop_frames_remaining);
end

always @(negedge frame_busy) begin
  if (frame_count == 1 && first_frame_done_ns == 0)
    first_frame_done_ns = $time;
  $fwrite(fd, "FRAME_DONE,%0t,%0d,%0d,%0d,%0d,%0d,%0d\n", $time,
          frame_count, last_lf_pulse, last_rf_pulse, last_lr_pulse,
          last_rr_pulse, stop_frames_remaining);
end

initial begin
  fd = $fopen("c5_timing_results.csv", "w");
  $fwrite(fd, "event,time_ns,value1,value2,value3,value4,value5\n");
  errors = 0;
  bytes_seen = 0;
  last_count = 0;
  reset_release_ns = 0;
  first_frame_start_ns = 0;
  first_frame_done_ns = 0;
  movement_start_ns = 0;
  movement_done_ns = 0;
  refresh_start_ns = 0;
  rst_n = 1'b0;
  motor_enable = 1'b1;
  force_stop = 1'b0;
  lf_speed = 11'd0;
  rf_speed = 11'd0;
  lr_speed = 11'd0;
  rr_speed = 11'd0;
  #200;
  rst_n = 1'b1;
  reset_release_ns = $time;

  wait (stop_frames_complete && frame_count == 3);
  #1000;
  $display("RESULT startup_wait_ns=%0d", first_frame_start_ns-reset_release_ns);
  $display("RESULT startup_stop_complete_ns=%0d", $time-reset_release_ns);
  if (first_frame_start_ns-reset_release_ns < 499999000 ||
      first_frame_start_ns-reset_release_ns > 500001000) errors = errors + 1;

  lf_speed = 11'd700;
  rf_speed = 11'd700;
  lr_speed = 11'd700;
  rr_speed = 11'd700;
  @(posedge frame_busy);
  movement_start_ns = $time;
  wait (frame_count == 4);
  movement_done_ns = $time;
  #1000;
  $display("RESULT forward_pulses=%0d,%0d,%0d,%0d", last_lf_pulse,
           last_rf_pulse, last_lr_pulse, last_rr_pulse);
  if (last_lf_pulse != 2200 || last_rf_pulse != 800 ||
      last_lr_pulse != 2200 || last_rr_pulse != 800) errors = errors + 1;

  @(posedge frame_busy);
  refresh_start_ns = $time;
  wait (frame_count == 5);
  #1000;
  $display("RESULT movement_refresh_start_interval_ns=%0d", refresh_start_ns-movement_start_ns);

  @(negedge clk);
  lf_speed = 11'd0;
  rf_speed = 11'd0;
  lr_speed = 11'd0;
  rr_speed = 11'd0;
  force_stop = 1'b1;
  @(negedge clk);
  force_stop = 1'b0;
  wait (stop_frames_complete && stop_frames_remaining == 0);
  count_before_idle = frame_count;
  #250000000;
  $display("RESULT idle_frame_delta_250ms=%0d", frame_count-count_before_idle);
  if (frame_count != count_before_idle) errors = errors + 1;
  if (bytes_seen != 178) errors = errors + 1;

  if (errors == 0) $display("RESULT TEST_PASS c5_timing");
  else $display("RESULT TEST_FAIL c5_timing errors=%0d", errors);
  $fwrite(fd, "SUMMARY,%0t,%0d,%0d,%0d,%0d,%0d\n", $time, errors,
          frame_count, bytes_seen, first_frame_start_ns-reset_release_ns,
          refresh_start_ns-movement_start_ns);
  $fclose(fd);
  $finish;
end
'''


C5_MAPPING = r'''
integer fd;
integer errors;
integer base_count;

initial begin
  clk = 1'b0;
end
always #10 clk = ~clk;

task apply_speeds;
  input [10:0] lf;
  input [10:0] rf;
  input [10:0] lr;
  input [10:0] rr;
  input [8*20-1:0] label_text;
  begin
    base_count = frame_count;
    lf_speed = lf;
    rf_speed = rf;
    lr_speed = lr;
    rr_speed = rr;
    wait (frame_count != base_count);
    #1000;
    $display("RESULT %0s pulses=%0d,%0d,%0d,%0d", label_text,
             last_lf_pulse,last_rf_pulse,last_lr_pulse,last_rr_pulse);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d\n",label_text,last_lf_pulse,
            last_rf_pulse,last_lr_pulse,last_rr_pulse);
  end
endtask

initial begin
  fd = $fopen("c5_mapping_results.csv", "w");
  $fwrite(fd,"case,lf_pulse,rf_pulse,lr_pulse,rr_pulse\n");
  errors = 0;
  rst_n = 1'b0;
  motor_enable = 1'b1;
  force_stop = 1'b0;
  lf_speed = 0; rf_speed = 0; lr_speed = 0; rr_speed = 0;
  #200; rst_n = 1'b1;
  #20;
  c5_motor_frame_driver0.startup_count = 24999999;
  wait (stop_frames_complete);
  apply_speeds(11'd700,11'd700,11'd700,11'd700,"forward_700");
  if (last_lf_pulse!=2200 || last_rf_pulse!=800 || last_lr_pulse!=2200 || last_rr_pulse!=800) errors=errors+1;
  apply_speeds(-11'sd700,-11'sd700,-11'sd700,-11'sd700,"reverse_700");
  if (last_lf_pulse!=800 || last_rf_pulse!=2200 || last_lr_pulse!=800 || last_rr_pulse!=2200) errors=errors+1;
  apply_speeds(-11'sd650,11'd650,-11'sd650,11'd650,"spin_left_650");
  if (last_lf_pulse!=850 || last_rf_pulse!=850 || last_lr_pulse!=850 || last_rr_pulse!=850) errors=errors+1;
  apply_speeds(11'd650,-11'sd650,11'd650,-11'sd650,"spin_right_650");
  if (last_lf_pulse!=2150 || last_rf_pulse!=2150 || last_lr_pulse!=2150 || last_rr_pulse!=2150) errors=errors+1;
  apply_speeds(11'd650,11'd750,11'd800,11'd625,"wheel_trim_sample");
  if (errors==0) $display("RESULT TEST_PASS c5_mapping");
  else $display("RESULT TEST_FAIL c5_mapping errors=%0d",errors);
  $fclose(fd);
  $finish;
end
'''


PARAMETERS = r'''
integer fd;
integer errors;
integer i;

initial clk=0;
always #10 clk=~clk;

task pulse_inc; begin bt_cfg_inc_pulse=1; #20; bt_cfg_inc_pulse=0; #20; end endtask
task pulse_dec; begin bt_cfg_dec_pulse=1; #20; bt_cfg_dec_pulse=0; #20; end endtask
task select_param; input [2:0] p; begin bt_cfg_select=p; bt_cfg_select_valid=1; #20; bt_cfg_select_valid=0; #20; end endtask
task log_values;
  input [8*20-1:0] label_text;
  begin
    $display("RESULT %0s overall=%0d step=%0d lf=%0d rf=%0d lr=%0d rr=%0d lidar=%0d",
      label_text,overall_speed,step_length_set,lf_default_speed,rf_default_speed,
      lr_default_speed,rr_default_speed,lidar_safe_mm);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",label_text,overall_speed,
      step_length_set,lf_default_speed,rf_default_speed,lr_default_speed,
      rr_default_speed,lidar_safe_mm);
  end
endtask

initial begin
  fd=$fopen("car_parameter_results.csv","w");
  $fwrite(fd,"case,overall,step,lf,rf,lr,rr,lidar_mm\n");
  errors=0; rst_n=0;
  key_up_n=1; key_down_n=1; key_left_n=1; key_right_n=1; key_mid_n=1;
  bt_cfg_mode_pulse=0; bt_cfg_next_pulse=0; bt_cfg_prev_pulse=0;
  bt_cfg_inc_pulse=0; bt_cfg_dec_pulse=0; bt_cfg_select=0; bt_cfg_select_valid=0;
  #200; rst_n=1; #100; log_values("defaults");
  if(overall_speed!=700 || step_length_set!=20 || lf_default_speed!=700 ||
     rf_default_speed!=700 || lr_default_speed!=700 || rr_default_speed!=700 ||
     lidar_safe_mm!=450) errors=errors+1;
  select_param(0); pulse_inc(); log_values("overall_plus_25");
  select_param(1); pulse_inc(); log_values("step_plus_5");
  select_param(2); pulse_dec(); log_values("lf_minus_25");
  select_param(3); pulse_inc(); log_values("rf_plus_25");
  select_param(4); pulse_inc(); pulse_inc(); log_values("lr_plus_50");
  select_param(5); pulse_dec(); pulse_dec(); log_values("rr_minus_50");
  select_param(6); pulse_inc(); log_values("lidar_plus_50");
  select_param(0); for(i=0;i<40;i=i+1) pulse_inc(); log_values("speed_max");
  for(i=0;i<40;i=i+1) pulse_dec(); log_values("speed_min");
  select_param(6); for(i=0;i<30;i=i+1) pulse_inc(); log_values("lidar_max");
  for(i=0;i<40;i=i+1) pulse_dec(); log_values("lidar_min");
  if(overall_speed!=100 || lidar_safe_mm!=150) errors=errors+1;
  if(errors==0) $display("RESULT TEST_PASS car_parameters");
  else $display("RESULT TEST_FAIL car_parameters errors=%0d",errors);
  $fclose(fd); $finish;
end
'''


ROUTE = r'''
integer fd;
integer errors;

initial clk=0;
always #10 clk=~clk;

task log_case;
  input [8*24-1:0] label_text;
  begin
    #100;
    $display("RESULT %0s speeds=%0d,%0d,%0d,%0d blocked=%0d",label_text,
      $signed(lf_speed),$signed(rf_speed),$signed(lr_speed),$signed(rr_speed),motor_blocked);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d,%0d\n",label_text,$signed(lf_speed),
      $signed(rf_speed),$signed(lr_speed),$signed(rr_speed),motor_blocked);
  end
endtask

initial begin
  fd=$fopen("c5_route_results.csv","w");
  $fwrite(fd,"case,lf_speed,rf_speed,lr_speed,rr_speed,motor_blocked\n");
  errors=0; rst_n=0; motor_enable=1; state_info=0; route_info=0;
  active_route_steps=0; forward_ms=1000; turn_ms=500; forward_speed=700;
  turn_speed=700; step_length=20; overall_speed=700;
  lf_default_speed=650; rf_default_speed=750; lr_default_speed=800; rr_default_speed=625;
  manual_mode=1; manual_cmd=0; obstacle_stop=0;
  #200; rst_n=1;
  manual_cmd=1; log_case("manual_forward_trimmed");
  if($signed(lf_speed)!=650 || $signed(rf_speed)!=750 || $signed(lr_speed)!=800 || $signed(rr_speed)!=625) errors=errors+1;
  obstacle_stop=1; log_case("forward_blocked");
  if(lf_speed!=0 || rf_speed!=0 || lr_speed!=0 || rr_speed!=0 || !motor_blocked) errors=errors+1;
  manual_cmd=2; log_case("reverse_escape");
  if($signed(lf_speed)!=-650 || $signed(rf_speed)!=-750 || $signed(lr_speed)!=-800 || $signed(rr_speed)!=-625 || motor_blocked) errors=errors+1;
  manual_cmd=3; log_case("left_escape");
  manual_cmd=4; log_case("right_escape");
  obstacle_stop=0; manual_cmd=0; log_case("manual_stop");
  if(lf_speed!=0 || rf_speed!=0 || lr_speed!=0 || rr_speed!=0) errors=errors+1;
  manual_mode=0; state_info=5'b00011; active_route_steps=1; route_info[1:0]=2'b11;
  log_case("automatic_forward");
  if(errors==0) $display("RESULT TEST_PASS c5_route");
  else $display("RESULT TEST_FAIL c5_route errors=%0d",errors);
  $fclose(fd); $finish;
end
'''


def main() -> None:
    TEST_DIR.mkdir(parents=True, exist_ok=True)
    build_test("c5_motor_frame_driver_robei_test", "c5_motor_frame_driver.model", C5_TIMING)
    build_test("c5_motor_mapping_robei_test", "c5_motor_frame_driver.model", C5_MAPPING)
    build_test("car_param_controller_robei_test", "car_param_controller.model", PARAMETERS)
    build_test("c5_route_controller_robei_test", "c5_route_controller.model", ROUTE)
    print("generated four Robei experiment files")


if __name__ == "__main__":
    main()
