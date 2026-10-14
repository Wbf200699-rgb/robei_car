module map_sd_store_controller (
  input wire clk,
  input wire rst_n,
  input wire save_pulse,
  input wire load_pulse,
  input wire [7:0] map_epoch,
  input wire [15:0] pose_x_mm,
  input wire [15:0] pose_y_mm,
  input wire [9:0] pose_heading,
  input wire [15:0] scan_count,
  input wire [15:0] occupied_count,
  input wire [15:0] free_count,
  input wire [3:0] map_mem_rdata,
  input wire map_mem_rvalid,
  input wire sd_card_present,
  input wire sd_initialized,
  input wire sd_init_done,
  input wire sd_block_busy,
  input wire sd_block_done,
  input wire [7:0] sd_error_code,
  input wire [7:0] sd_last_r1,
  input wire [7:0] sd_write_data_response,
  input wire sd_high_capacity,
  input wire [8:0] sd_buffer_addr,
  input wire sd_buffer_write_enable,
  input wire [7:0] sd_buffer_write_data,
  output wire map_mem_req,
  output wire map_mem_we,
  output wire [13:0] map_mem_addr,
  output wire [3:0] map_mem_wdata,
  output wire load_apply_pulse,
  output wire [7:0] loaded_map_epoch,
  output wire [15:0] loaded_pose_x_mm,
  output wire [15:0] loaded_pose_y_mm,
  output wire [9:0] loaded_pose_heading,
  output wire [15:0] loaded_scan_count,
  output wire [15:0] loaded_occupied_count,
  output wire [15:0] loaded_free_count,
  output wire sd_init_start,
  output wire sd_block_start,
  output wire sd_block_write,
  output wire [31:0] sd_block_lba,
  output wire [7:0] sd_buffer_read_data,
  output wire flash_ready,
  output wire flash_busy,
  output wire [1:0] flash_operation,
  output wire [7:0] flash_progress,
  output wire [7:0] flash_error,
  output wire [23:0] flash_jedec_id,
  output wire [31:0] saved_sequence
);

  parameter integer CLK_HZ = 50000000;
  parameter integer BOOT_WAIT_CYCLES = CLK_HZ / 1000;
  parameter integer AUTO_LOAD_ON_BOOT = 0;

  // Competition profile: the Android host owns map persistence.  Keep this
  // Robei-visible adapter so the existing graphical hierarchy and SD pinout do
  // not change, but do not spend FPGA resources duplicating the phone's map.
  assign map_mem_req = 1'b0;
  assign map_mem_we = 1'b0;
  assign map_mem_addr = 14'd0;
  assign map_mem_wdata = 4'd0;
  assign load_apply_pulse = 1'b0;
  assign loaded_map_epoch = 8'd0;
  assign loaded_pose_x_mm = 16'd0;
  assign loaded_pose_y_mm = 16'd0;
  assign loaded_pose_heading = 10'd0;
  assign loaded_scan_count = 16'd0;
  assign loaded_occupied_count = 16'd0;
  assign loaded_free_count = 16'd0;
  assign sd_init_start = 1'b0;
  assign sd_block_start = 1'b0;
  assign sd_block_write = 1'b0;
  assign sd_block_lba = 32'd0;
  assign sd_buffer_read_data = 8'd0;
  assign flash_ready = 1'b0;
  assign flash_busy = 1'b0;
  assign flash_operation = 2'd0;
  assign flash_progress = 8'd0;
  assign flash_error = 8'd0;
  assign flash_jedec_id = 24'd0;
  assign saved_sequence = 32'd0;

endmodule
