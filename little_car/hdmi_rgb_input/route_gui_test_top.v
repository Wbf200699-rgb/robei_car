module route_gui_test_top(
input wire clk,
input wire rst_n,
output wire hdmi_clk_p,
output wire hdmi_clk_n,
output wire hdmi_d0_p,
output wire hdmi_d0_n,
output wire hdmi_d1_p,
output wire hdmi_d1_n,
output wire hdmi_d2_p,
output wire hdmi_d2_n
);

  wire [23:0] rgb_data;
  wire [11:0] pixel_x;
  wire [11:0] pixel_y;
  wire pixel_de;
  wire pixel_clk;
  wire [1:0] menu_info;
  wire [4:0] state_info;
  wire [63:0] route_info;
  wire [63:0] old_route_info;
  wire [5:0] active_route_steps;
  wire [5:0] completed_route_steps;
  wire [15:0] step_length;
  wire [15:0] speed;
  wire [15:0] total_route_length;
  wire [15:0] complete_route_length;
  wire [15:0] remaining_route_length;
  wire [7:0] now_x;
  wire [7:0] now_y;
  wire [7:0] target_x;
  wire [7:0] target_y;

  route_demo_info_source route_demo_info_source_inst(
    .system_info(menu_info),
    .route_info(route_info)
  );

  menu_to_state_info menu_to_state_info_inst(
    .menu_info(menu_info),
    .state_info(state_info)
  );

  temp_gui_status_source temp_gui_status_source_inst(
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
    .target_y(target_y)
  );

  route_gui_renderer route_gui_renderer_inst(
    .clk_pix(pixel_clk),
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

  hdmi_top hdmi_top_inst(
    .clk(clk),
    .rst_n(rst_n),
    .rgb_data(rgb_data),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y),
    .pixel_de(pixel_de),
    .pixel_clk(pixel_clk),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_d2_n(hdmi_d2_n)
  );

endmodule    //route_gui_test_top
