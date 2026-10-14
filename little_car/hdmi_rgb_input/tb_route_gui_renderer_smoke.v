`timescale 1ns / 1ps

module tb_route_gui_renderer_smoke;

  localparam [1:0] MENU_FOCUS_CHANGE = 2'b00;
  localparam [4:0] STATE_MENU = 5'b00000;

  reg clk_pix;
  reg rst_n;
  reg [11:0] pixel_x;
  reg [11:0] pixel_y;
  reg pixel_de;
  reg [1:0] menu_info;
  reg [4:0] state_info;
  reg [63:0] route_info;
  reg [63:0] old_route_info;
  reg [5:0] active_route_steps;
  reg [5:0] completed_route_steps;
  reg [15:0] step_length;
  reg [15:0] speed;
  reg [15:0] total_route_length;
  reg [15:0] complete_route_length;
  reg [15:0] remaining_route_length;
  reg [7:0] now_x;
  reg [7:0] now_y;
  reg [7:0] target_x;
  reg [7:0] target_y;

  wire [23:0] rgb_data;

  route_gui_renderer dut (
    .clk_pix(clk_pix),
    .rst_n(rst_n),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y),
    .pixel_de(pixel_de),
    .menu_info(menu_info),
    .state_info(state_info),
    .route_info(route_info),
    .old_route_info(old_route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps),
    .step_length(step_length),
    .speed(speed),
    .total_route_length(total_route_length),
    .complete_route_length(complete_route_length),
    .remaining_route_length(remaining_route_length),
    .now_x(now_x),
    .now_y(now_y),
    .target_x(target_x),
    .target_y(target_y),
    .rgb_data(rgb_data)
  );

  initial begin
    clk_pix = 1'b0;
    forever #5 clk_pix = ~clk_pix;
  end

  task sample_pixel;
    input [11:0] x;
    input [11:0] y;
    begin
      pixel_x = x;
      pixel_y = y;
      @(posedge clk_pix);
      @(posedge clk_pix);
    end
  endtask

  initial begin
    rst_n = 1'b0;
    pixel_x = 12'd0;
    pixel_y = 12'd0;
    pixel_de = 1'b0;
    menu_info = MENU_FOCUS_CHANGE;
    state_info = STATE_MENU;
    route_info = 64'd0;
    old_route_info = 64'd0;
    active_route_steps = 6'd1;
    completed_route_steps = 6'd0;
    step_length = 16'd20;
    speed = 16'd25;
    total_route_length = 16'd20;
    complete_route_length = 16'd0;
    remaining_route_length = 16'd20;
    now_x = 8'd128;
    now_y = 8'd128;
    target_x = 8'd129;
    target_y = 8'd128;

    repeat (3) @(posedge clk_pix);
    rst_n = 1'b1;

    sample_pixel(12'd10, 12'd10);
    if (rgb_data !== 24'h000000)
      $fatal(1, "renderer tb: expected blank pixel when pixel_de=0, got %h", rgb_data);

    pixel_de = 1'b1;

    sample_pixel(12'd100, 12'd250);
    if (rgb_data !== 24'h1D4ED8)
      $fatal(1, "renderer tb: expected selected change_box blue, got %h", rgb_data);

    sample_pixel(12'd20, 12'd20);
    if (rgb_data !== 24'h334155)
      $fatal(1, "renderer tb: expected border color, got %h", rgb_data);

    sample_pixel(12'd733, 12'd370);
    if (rgb_data !== 24'h0891B2)
      $fatal(1, "renderer tb: expected target marker color, got %h", rgb_data);

    $display("[%0t] PASS tb_route_gui_renderer_smoke", $time);
    $finish;
  end

endmodule
