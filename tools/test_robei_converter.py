import unittest
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.robei_converter import const_to_xdc, model_to_verilog, verilog_to_model


class ModelToVerilogTests(unittest.TestCase):
    def test_const_to_xdc_accepts_logical_to_physical_wire_direction(self):
        const_xml = """<!DOCTYPE Robei>
<Module Name="top" Code="">
 <Module Name="design0">
  <Port Name="debug_uart_tx" />
 </Module>
 <Port Name="J1" Parent="top" Voltage="LVCMOS33" Drive="8" Pull="none" />
 <Wire From="top#design0&gt;debug_uart_tx" To="top&gt;J1" />
</Module>
"""

        converted = const_to_xdc(const_xml)

        self.assertIn('PACKAGE_PIN J1 [get_ports debug_uart_tx]', converted)
        self.assertIn('IOSTANDARD LVCMOS33 [get_ports debug_uart_tx]', converted)
        self.assertNotIn('[get_ports J1]', converted)
        self.assertNotIn('[get_ports {', converted)

    def test_const_to_xdc_expands_extra_code_port_lists_for_roplace(self):
        const_xml = """<!DOCTYPE Robei>
<Module Name="top" Code="set_property PULLUP TRUE [get_ports {scl sda}]&#10;set_property DRIVE 4 [get_ports {scl sda}]">
 <Module Name="design0">
  <Port Name="scl" />
  <Port Name="sda" />
 </Module>
</Module>
"""

        converted = const_to_xdc(const_xml)

        self.assertIn('PULLUP TRUE [get_ports scl]', converted)
        self.assertIn('PULLUP TRUE [get_ports sda]', converted)
        self.assertIn('DRIVE 4 [get_ports scl]', converted)
        self.assertIn('DRIVE 4 [get_ports sda]', converted)
        self.assertNotIn('[get_ports {', converted)

    def test_leaf_model_keeps_behavior_code(self):
        source = """module passthrough (
  input wire value_in,
  output wire value_out
);
assign value_out = value_in;
endmodule
"""

        converted = model_to_verilog(verilog_to_model(source))

        self.assertIn("module passthrough", converted)
        self.assertIn("input wire value_in", converted)
        self.assertIn("output wire value_out", converted)
        self.assertIn("assign value_out = value_in;", converted)

    def test_structural_model_exports_instances_and_fanout(self):
        model = """<!DOCTYPE Robei>
<Module Name="top" Code="  assign fixed_high = 1'b1;">
 <Module Class="producer" Name="producer0" Parameters="">
  <Port Name="clk" Inout="input" Datatype="wire" Datasize="1" />
  <Port Name="serial_out" Inout="output" Datatype="wire" Datasize="1" />
 </Module>
 <Module Class="consumer" Name="consumer0" Parameters="">
  <Port Name="serial_in" Inout="input" Datatype="wire" Datasize="1" />
 </Module>
 <Port Name="clk" Inout="input" Datatype="wire" Datasize="1" />
 <Port Name="serial_tx" Inout="output" Datatype="wire" Datasize="1" />
 <Port Name="fixed_high" Inout="output" Datatype="wire" Datasize="1" />
 <Wire Name="top_clk" From="top&gt;clk" To="top#producer0&gt;clk" Datasize="1" Datatype="wire" />
 <Wire Name="producer0_serial_out" From="top#producer0&gt;serial_out" To="top&gt;serial_tx" Datasize="1" Datatype="wire" />
 <Wire Name="producer0_serial_tap" From="top#producer0&gt;serial_out" To="top#consumer0&gt;serial_in" Datasize="1" Datatype="wire" />
</Module>
"""

        converted = model_to_verilog(model)

        self.assertIn("producer producer0", converted)
        self.assertIn("consumer consumer0", converted)
        self.assertIn(".clk(clk)", converted)
        self.assertIn(".serial_out(serial_tx)", converted)
        self.assertIn(".serial_in(serial_tx)", converted)
        self.assertIn("assign fixed_high = 1'b1;", converted)

    def test_structural_model_rejects_multiple_drivers(self):
        model = """<Module Name="top" Code="">
 <Module Class="sink" Name="sink0" Parameters="">
  <Port Name="value" Inout="input" Datatype="wire" Datasize="1" />
 </Module>
 <Port Name="a" Inout="input" Datatype="wire" Datasize="1" />
 <Port Name="b" Inout="input" Datatype="wire" Datasize="1" />
 <Wire Name="a_value" From="top&gt;a" To="top#sink0&gt;value" Datasize="1" Datatype="wire" />
 <Wire Name="b_value" From="top&gt;b" To="top#sink0&gt;value" Datasize="1" Datatype="wire" />
</Module>
"""

        with self.assertRaisesRegex(ValueError, "多个 Wire 驱动"):
            model_to_verilog(model)

    def test_current_vehicle_top_exports_every_robei_instance(self):
        repo_root = Path(__file__).resolve().parents[1]
        model_path = repo_root / 'little_car' / 'model' / 'sys_route_gui_fsm_top.model'
        model_text = model_path.read_text(encoding='utf-8')
        xml_text = re.sub(r'<!DOCTYPE[^>]*>', '', model_text)
        root = ET.fromstring(xml_text)

        converted = model_to_verilog(model_text)

        children = root.findall('Module')
        # Instance count grows as physical sensors and local control stages are
        # added. The loop below is the important invariant: every Robei child
        # must be exported, regardless of the current graph size.
        self.assertGreaterEqual(len(children), 26)
        for child in children:
            declaration = f'{child.get("Class")} {child.get("Name")} ('
            self.assertIn(declaration, converted)
        child_classes = {child.get('Class') for child in children}
        self.assertIn('bt_route_uart', child_classes)
        self.assertIn('vl53l1x_array5', child_classes)
        self.assertIn('c5_motion_stabilizer', child_classes)
        self.assertIn('map_sd_store', child_classes)
        self.assertIn('linkpg_tag_position_uart', child_classes)
        self.assertIn('vehicle_debug_uart', child_classes)
        self.assertNotIn('map_flash_store', child_classes)
        self.assertNotIn('linkpg_uwb_bridge', child_classes)
        self.assertNotIn('uwb_debug_uart', child_classes)
        self.assertIn('.bt_uart_rx(bt_uart_rx)', converted)
        self.assertIn('.sample_valid(rplidar_c1_uart19_sample_valid_map)', converted)
        self.assertIn('.map_chunk_data(lidar_map_sampler26_map_chunk_data)', converted)
        self.assertIn('.vl53_distance_mm(vl53l1x_array527_distance_mm)', converted)
        self.assertIn('.base_lf_speed(c5_route_controller9_lf_speed)', converted)
        self.assertIn('.lf_speed(c5_motion_stabilizer29_lf_speed)', converted)
        self.assertIn('.i2c_scl(tof_i2c_scl)', converted)
        self.assertIn('.sd_sclk(sd_sclk)', converted)
        self.assertIn('.sd_mosi(sd_mosi)', converted)
        self.assertIn('.sd_miso(sd_miso)', converted)
        self.assertNotIn('map_transport_ready', converted)
        self.assertIn('.uwb_uart_rx(uwb_uart_rx)', converted)
        self.assertIn('.uwb_uart_tx(uwb_uart_tx)', converted)
        self.assertIn('.debug_uart_tx(debug_uart_tx)', converted)
        self.assertGreater(len(converted), 20000)


if __name__ == '__main__':
    unittest.main()
