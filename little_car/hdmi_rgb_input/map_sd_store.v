module map_sd_store (
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
  input wire sd_cd,
  input wire sd_d1,
  input wire sd_d2,
  input wire sd_miso,
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
  output wire sd_cs_n,
  output wire sd_sclk,
  output wire sd_mosi,
  output wire flash_ready,
  output wire flash_busy,
  output wire [1:0] flash_operation,
  output wire [7:0] flash_progress,
  output wire [7:0] flash_error,
  output wire [23:0] flash_jedec_id,
  output wire [31:0] saved_sequence
);

  wire sd_init_start;
  wire sd_block_start;
  wire sd_block_write;
  wire [31:0] sd_block_lba;
  wire [7:0] sd_buffer_read_data;
  wire sd_initialized;
  wire sd_init_done;
  wire sd_block_busy;
  wire sd_block_done;
  wire [7:0] sd_error_code;
  wire [7:0] sd_last_r1;
  wire [7:0] sd_write_data_response;
  wire sd_high_capacity;
  wire [8:0] sd_buffer_addr;
  wire sd_buffer_write_enable;
  wire [7:0] sd_buffer_write_data;
  wire sd_card_present;

  map_sd_store_controller map_sd_store_controller0 (
    .clk(clk),
    .rst_n(rst_n),
    .save_pulse(save_pulse),
    .load_pulse(load_pulse),
    .map_epoch(map_epoch),
    .pose_x_mm(pose_x_mm),
    .pose_y_mm(pose_y_mm),
    .pose_heading(pose_heading),
    .scan_count(scan_count),
    .occupied_count(occupied_count),
    .free_count(free_count),
    .map_mem_rdata(map_mem_rdata),
    .map_mem_rvalid(map_mem_rvalid),
    .sd_card_present(sd_card_present),
    .sd_initialized(sd_initialized),
    .sd_init_done(sd_init_done),
    .sd_block_busy(sd_block_busy),
    .sd_block_done(sd_block_done),
    .sd_error_code(sd_error_code),
    .sd_last_r1(sd_last_r1),
    .sd_write_data_response(sd_write_data_response),
    .sd_high_capacity(sd_high_capacity),
    .sd_buffer_addr(sd_buffer_addr),
    .sd_buffer_write_enable(sd_buffer_write_enable),
    .sd_buffer_write_data(sd_buffer_write_data),
    .map_mem_req(map_mem_req),
    .map_mem_we(map_mem_we),
    .map_mem_addr(map_mem_addr),
    .map_mem_wdata(map_mem_wdata),
    .load_apply_pulse(load_apply_pulse),
    .loaded_map_epoch(loaded_map_epoch),
    .loaded_pose_x_mm(loaded_pose_x_mm),
    .loaded_pose_y_mm(loaded_pose_y_mm),
    .loaded_pose_heading(loaded_pose_heading),
    .loaded_scan_count(loaded_scan_count),
    .loaded_occupied_count(loaded_occupied_count),
    .loaded_free_count(loaded_free_count),
    .sd_init_start(sd_init_start),
    .sd_block_start(sd_block_start),
    .sd_block_write(sd_block_write),
    .sd_block_lba(sd_block_lba),
    .sd_buffer_read_data(sd_buffer_read_data),
    .flash_ready(flash_ready),
    .flash_busy(flash_busy),
    .flash_operation(flash_operation),
    .flash_progress(flash_progress),
    .flash_error(flash_error),
    .flash_jedec_id(flash_jedec_id),
    .saved_sequence(saved_sequence)
  );

  sd_spi_block_device sd_spi_block_device0 (
    .clk(clk),
    .rst_n(rst_n),
    .init_start(sd_init_start),
    .block_start(sd_block_start),
    .block_write(sd_block_write),
    .block_lba(sd_block_lba),
    .buffer_read_data(sd_buffer_read_data),
    .sd_miso(sd_miso),
    .buffer_addr(sd_buffer_addr),
    .buffer_write_enable(sd_buffer_write_enable),
    .buffer_write_data(sd_buffer_write_data),
    .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi),
    .initialized(sd_initialized),
    .init_busy(),
    .init_done(sd_init_done),
    .block_busy(sd_block_busy),
    .block_done(sd_block_done),
    .error_code(sd_error_code),
    .last_r1(sd_last_r1),
    .write_data_response(sd_write_data_response),
    .acmd41_attempts(),
    .ocr(),
    .high_capacity(sd_high_capacity)
  );

  sd_card_detect_adapter sd_card_detect_adapter0 (
    .sd_cd(sd_cd),
    .sd_d1(sd_d1),
    .sd_d2(sd_d2),
    .sd_card_present(sd_card_present)
  );

endmodule
