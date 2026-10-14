`timescale 1ns / 1ps

module tb_route_gui_test_top;

  reg clk;
  reg rst_n;

  wire hdmi_clk_p;
  wire hdmi_clk_n;
  wire hdmi_d0_p;
  wire hdmi_d0_n;
  wire hdmi_d1_p;
  wire hdmi_d1_n;
  wire hdmi_d2_p;
  wire hdmi_d2_n;

  route_gui_test_top dut(
    .clk(clk),
    .rst_n(rst_n),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_d2_n(hdmi_d2_n)
  );

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    #200;
    rst_n = 1'b1;
  end

  initial begin
    $display("tb_route_gui_test_top started");

    wait (rst_n == 1'b1);
    wait (dut.pixel_de == 1'b1);
    $display("First active pixel: x=%0d y=%0d rgb=%h time=%0t",
             dut.pixel_x, dut.pixel_y, dut.rgb_data, $time);

    wait (dut.pixel_y == 12'd180 && dut.pixel_de == 1'b1);
    $display("Menu area: x=%0d y=%0d rgb=%h time=%0t",
             dut.pixel_x, dut.pixel_y, dut.rgb_data, $time);

    wait (dut.pixel_y == 12'd300 && dut.pixel_x >= 12'd660 && dut.pixel_de == 1'b1);
    $display("Route map area: x=%0d y=%0d rgb=%h time=%0t",
             dut.pixel_x, dut.pixel_y, dut.rgb_data, $time);

    wait (dut.pixel_y == 12'd500 && dut.pixel_de == 1'b1);
    $display("End area: x=%0d y=%0d rgb=%h time=%0t",
             dut.pixel_x, dut.pixel_y, dut.rgb_data, $time);

    #2000;
    $display("tb_route_gui_test_top finished");
    $finish;
  end

endmodule
