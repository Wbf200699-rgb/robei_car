`timescale 1ns / 1ps

module tb_gui_route_renderer_optimized;
  reg clk_pix = 1'b0;
  reg rst_n = 1'b0;
  reg [11:0] pixel_x = 12'd0;
  reg [11:0] pixel_y = 12'd0;
  reg pixel_de = 1'b0;
  reg [1:0] menu_info = 2'b00;
  reg [63:0] route_info = 64'd0;
  reg [4:0] state_info = 5'b00000;
  reg [63:0] old_route_info = 64'd0;
  reg [5:0] active_route_steps = 6'd1;
  reg [5:0] completed_route_steps = 6'd0;
  reg [15:0] step_length = 16'd20;
  reg [15:0] speed = 16'd25;
  reg [15:0] total_route_length = 16'd12345;
  reg [15:0] complete_route_length = 16'd0;
  reg [15:0] remaining_route_length = 16'd20;
  reg [7:0] now_x = 8'd128;
  reg [7:0] now_y = 8'd128;
  reg [7:0] target_x = 8'd129;
  reg [7:0] target_y = 8'd128;
  reg bt_connected = 1'b1;
  wire [23:0] rgb_data;

  gui_route_renderer dut (
    .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_de(pixel_de),
    .menu_info(menu_info), .route_info(route_info), .clk_pix(clk_pix),
    .rst_n(rst_n), .state_info(state_info), .old_route_info(old_route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps), .step_length(step_length),
    .speed(speed), .total_route_length(total_route_length),
    .complete_route_length(complete_route_length),
    .remaining_route_length(remaining_route_length), .now_x(now_x),
    .now_y(now_y), .target_x(target_x), .target_y(target_y),
    .bt_connected(bt_connected), .rgb_data(rgb_data)
  );

  always #5 clk_pix = ~clk_pix;

  task sample_pixel;
    input [11:0] x;
    input [11:0] y;
    input [23:0] expected;
    begin
      pixel_x = x;
      pixel_y = y;
      @(posedge clk_pix);
      @(posedge clk_pix);
      @(posedge clk_pix);
      if (rgb_data !== expected)
        $fatal(1, "pixel (%0d,%0d): expected %h, got %h", x, y, expected, rgb_data);
    end
  endtask

  initial begin
    repeat (3) @(posedge clk_pix);
    rst_n = 1'b1;
    repeat (260) @(posedge clk_pix);

    sample_pixel(12'd10, 12'd10, 24'h000000);
    pixel_de = 1'b1;
    sample_pixel(12'd100, 12'd250, 24'h1D4ED8);
    sample_pixel(12'd20, 12'd20, 24'h334155);
    sample_pixel(12'd733, 12'd370, 24'h0891B2);

    // TOTAL=12345: x=894,y=18 is a set pixel in the leading "1".
    sample_pixel(12'd894, 12'd18, 24'h0F172A);
    // BT status remains green and uses no text-heavy rendering path.
    sample_pixel(12'd60, 12'd90, 24'hDCFCE7);

    $display("RESULT TEST_PASS gui_route_renderer_optimized");
    $finish;
  end
endmodule
