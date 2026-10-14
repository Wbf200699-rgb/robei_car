`timescale 1ns/1ps

module tb_map_sd_store_controller;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg save_pulse = 1'b0;
  reg load_pulse = 1'b0;
  reg [7:0] map_epoch = 8'd7;
  reg [15:0] pose_x_mm = 16'd3200;
  reg [15:0] pose_y_mm = 16'd3150;
  reg [9:0] pose_heading = 10'd137;
  reg [15:0] scan_count = 16'd1234;
  reg [15:0] occupied_count = 16'd321;
  reg [15:0] free_count = 16'd4567;
  wire map_mem_req;
  wire map_mem_we;
  wire [13:0] map_mem_addr;
  wire signed [3:0] map_mem_wdata;
  reg signed [3:0] map_mem_rdata = 4'sd0;
  reg map_mem_rvalid = 1'b0;
  wire load_apply_pulse;
  wire [7:0] loaded_map_epoch;
  wire [15:0] loaded_pose_x_mm;
  wire [15:0] loaded_pose_y_mm;
  wire [9:0] loaded_pose_heading;
  wire [15:0] loaded_scan_count;
  wire [15:0] loaded_occupied_count;
  wire [15:0] loaded_free_count;
  wire sd_init_start;
  wire sd_initialized;
  wire sd_init_done;
  wire sd_block_start;
  wire sd_block_write;
  wire [31:0] sd_block_lba;
  wire sd_block_busy;
  wire sd_block_done;
  wire [7:0] sd_error_code;
  wire [7:0] sd_last_r1;
  wire [7:0] sd_write_data_response;
  wire sd_high_capacity;
  wire [8:0] sd_buffer_addr;
  wire [7:0] sd_buffer_read_data;
  wire sd_buffer_write_enable;
  wire [7:0] sd_buffer_write_data;
  wire flash_ready;
  wire flash_busy;
  wire [1:0] flash_operation;
  wire [7:0] flash_progress;
  wire [7:0] flash_error;
  wire [23:0] flash_jedec_id;
  wire [31:0] saved_sequence;

  reg signed [3:0] map_cells [0:16383];
  reg signed [3:0] original_cells [0:16383];
  reg map_read_pending = 1'b0;
  reg [13:0] map_read_addr = 14'd0;
  reg [7:0] fail_write_ordinal = 8'd0;
  integer index;
  integer failures = 0;
  integer load_apply_count = 0;

  always #10 clk = ~clk;

  always @(posedge clk) begin
    map_mem_rvalid <= map_read_pending;
    if (map_read_pending)
      map_mem_rdata <= map_cells[map_read_addr];
    map_read_pending <= 1'b0;
    if (map_mem_req) begin
      if (map_mem_we)
        map_cells[map_mem_addr] <= map_mem_wdata;
      else begin
        map_read_pending <= 1'b1;
        map_read_addr <= map_mem_addr;
      end
    end
    if (load_apply_pulse)
      load_apply_count <= load_apply_count + 1;
  end

  map_sd_store_controller #(
    .BOOT_WAIT_CYCLES(4),
    .AUTO_LOAD_ON_BOOT(1)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .save_pulse(save_pulse), .load_pulse(load_pulse),
    .map_epoch(map_epoch), .pose_x_mm(pose_x_mm),
    .pose_y_mm(pose_y_mm), .pose_heading(pose_heading),
    .scan_count(scan_count), .occupied_count(occupied_count),
    .free_count(free_count),
    .map_mem_req(map_mem_req), .map_mem_we(map_mem_we),
    .map_mem_addr(map_mem_addr), .map_mem_wdata(map_mem_wdata),
    .map_mem_rdata(map_mem_rdata), .map_mem_rvalid(map_mem_rvalid),
    .load_apply_pulse(load_apply_pulse),
    .loaded_map_epoch(loaded_map_epoch),
    .loaded_pose_x_mm(loaded_pose_x_mm),
    .loaded_pose_y_mm(loaded_pose_y_mm),
    .loaded_pose_heading(loaded_pose_heading),
    .loaded_scan_count(loaded_scan_count),
    .loaded_occupied_count(loaded_occupied_count),
    .loaded_free_count(loaded_free_count),
    .sd_card_present(1'b1),
    .sd_init_start(sd_init_start),
    .sd_initialized(sd_initialized),
    .sd_init_done(sd_init_done),
    .sd_block_start(sd_block_start),
    .sd_block_write(sd_block_write),
    .sd_block_lba(sd_block_lba),
    .sd_block_busy(sd_block_busy),
    .sd_block_done(sd_block_done),
    .sd_error_code(sd_error_code),
    .sd_last_r1(sd_last_r1),
    .sd_write_data_response(sd_write_data_response),
    .sd_high_capacity(sd_high_capacity),
    .sd_buffer_addr(sd_buffer_addr),
    .sd_buffer_read_data(sd_buffer_read_data),
    .sd_buffer_write_enable(sd_buffer_write_enable),
    .sd_buffer_write_data(sd_buffer_write_data),
    .flash_ready(flash_ready), .flash_busy(flash_busy),
    .flash_operation(flash_operation), .flash_progress(flash_progress),
    .flash_error(flash_error), .flash_jedec_id(flash_jedec_id),
    .saved_sequence(saved_sequence)
  );

  fake_sd_block_device block_model (
    .clk(clk), .rst_n(rst_n),
    .init_start(sd_init_start),
    .initialized(sd_initialized), .init_done(sd_init_done),
    .block_start(sd_block_start), .block_write(sd_block_write),
    .block_lba(sd_block_lba), .block_busy(sd_block_busy),
    .block_done(sd_block_done), .error_code(sd_error_code),
    .last_r1(sd_last_r1),
    .write_data_response(sd_write_data_response),
    .high_capacity(sd_high_capacity),
    .buffer_addr(sd_buffer_addr),
    .buffer_read_data(sd_buffer_read_data),
    .buffer_write_enable(sd_buffer_write_enable),
    .buffer_write_data(sd_buffer_write_data),
    .fail_write_ordinal(fail_write_ordinal)
  );

  task pulse_save;
    begin
      @(negedge clk); save_pulse = 1'b1;
      @(negedge clk); save_pulse = 1'b0;
      wait (flash_busy);
      wait (!flash_busy);
      @(posedge clk);
    end
  endtask

  task pulse_load;
    begin
      @(negedge clk); load_pulse = 1'b1;
      @(negedge clk); load_pulse = 1'b0;
      wait (flash_busy);
      wait (!flash_busy);
      @(posedge clk);
    end
  endtask

  task compare_original_map;
    begin
      for (index = 0; index < 16384; index = index + 1) begin
        if (map_cells[index] !== original_cells[index]) begin
          $display("ERROR map mismatch index=%0d got=%0d expected=%0d",
                   index, map_cells[index], original_cells[index]);
          failures = failures + 1;
          index = 16384;
        end
      end
    end
  endtask

  initial begin
    $dumpfile("temp/tb_map_sd_store_controller.vcd");
    $dumpvars(0, tb_map_sd_store_controller);
    for (index = 0; index < 16384; index = index + 1) begin
      map_cells[index] = ((index * 5) & 4'hf);
      original_cells[index] = map_cells[index];
    end

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (flash_ready && !flash_busy);
    if (saved_sequence != 0 || flash_error != 0 ||
        flash_jedec_id != 24'h534448) begin
      $display("ERROR empty boot sequence=%0d error=%02x id=%06x",
               saved_sequence, flash_error, flash_jedec_id);
      failures = failures + 1;
    end

    pulse_save();
    $display("DEBUG commit magic=%c%c%c%c%c%c%c%c marker=%02x slot=%0d seq=%08x/%08x hcrc=%08x/%08x pcrc=%08x/%08x sector_crc=%08x/%08x",
             dut.sector_buffer[0], dut.sector_buffer[1],
             dut.sector_buffer[2], dut.sector_buffer[3],
             dut.sector_buffer[4], dut.sector_buffer[5],
             dut.sector_buffer[6], dut.sector_buffer[7],
             dut.sector_buffer[21], dut.sector_buffer[20][0],
             {dut.sector_buffer[11], dut.sector_buffer[10],
              dut.sector_buffer[9], dut.sector_buffer[8]}, dut.pending_sequence,
             {dut.sector_buffer[15], dut.sector_buffer[14],
              dut.sector_buffer[13], dut.sector_buffer[12]}, dut.pending_header_crc,
             {dut.sector_buffer[19], dut.sector_buffer[18],
              dut.sector_buffer[17], dut.sector_buffer[16]}, dut.pending_payload_crc,
             {dut.sector_buffer[511], dut.sector_buffer[510],
              dut.sector_buffer[509], dut.sector_buffer[508]}, dut.crc_result);
    if (flash_error != 0 || saved_sequence != 1 ||
        !dut.slot_a_valid || dut.slot_b_valid) begin
      $display("ERROR first save error=%02x sequence=%0d valid=%0d,%0d",
               flash_error, saved_sequence,
               dut.slot_a_valid, dut.slot_b_valid);
      failures = failures + 1;
    end

    for (index = 0; index < 16384; index = index + 1)
      map_cells[index] = 4'sd0;
    pulse_load();
    compare_original_map();
    if ((loaded_map_epoch != map_epoch) ||
        (loaded_pose_x_mm != pose_x_mm) ||
        (loaded_pose_y_mm != pose_y_mm) ||
        (loaded_pose_heading != pose_heading) ||
        (loaded_scan_count != scan_count) ||
        (loaded_occupied_count != occupied_count) ||
        (loaded_free_count != free_count)) begin
      $display("ERROR metadata epoch=%0d pose=%0d,%0d heading=%0d scan=%0d",
               loaded_map_epoch, loaded_pose_x_mm, loaded_pose_y_mm,
               loaded_pose_heading, loaded_scan_count);
      failures = failures + 1;
    end

    // A second save targets slot B. Fail during its payload, after the invalid
    // commit has reached the card. Slot A must remain the bootable record.
    for (index = 0; index < 16384; index = index + 1)
      map_cells[index] = -original_cells[index];
    map_epoch = 8'd8;
    pose_x_mm = 16'd3500;
    fail_write_ordinal = block_model.write_count + 8'd5;
    pulse_save();
    if (flash_error[7] != 1'b1 || saved_sequence != 1) begin
      $display("ERROR interrupted save error=%02x sequence=%0d writes=%0d",
               flash_error, saved_sequence, block_model.write_count);
      failures = failures + 1;
    end

    fail_write_ordinal = 8'd0;
    for (index = 0; index < 16384; index = index + 1)
      map_cells[index] = 4'sd0;
    load_apply_count = 0;
    @(negedge clk); rst_n = 1'b0;
    repeat (5) @(posedge clk);
    @(negedge clk); rst_n = 1'b1;
    wait (flash_ready && !flash_busy);
    repeat (5) @(posedge clk);
    compare_original_map();
    if ((saved_sequence != 1) || (load_apply_count != 1) ||
        (loaded_map_epoch != 8'd7) || (loaded_pose_x_mm != 16'd3200)) begin
      $display("ERROR reboot recovery sequence=%0d loads=%0d epoch=%0d x=%0d",
               saved_sequence, load_apply_count,
               loaded_map_epoch, loaded_pose_x_mm);
      failures = failures + 1;
    end

    $display("RESULT first_sequence=1 recovered_sequence=%0d writes=%0d reads=%0d",
             saved_sequence, block_model.write_count, block_model.read_count);
    $display("RESULT metadata epoch=%0d pose=%0d,%0d heading=%0d scan=%0d occupied=%0d free=%0d",
             loaded_map_epoch, loaded_pose_x_mm, loaded_pose_y_mm,
             loaded_pose_heading, loaded_scan_count,
             loaded_occupied_count, loaded_free_count);
    $display("RESULT power_cut_error_preserved_slot_a=%0d slot_b_commit_marker=%02x",
             dut.slot_a_valid, block_model.slot_b[(17 * 512) + 21]);
    if (failures == 0)
      $display("RESULT TEST_PASS map_sd_store_controller");
    else
      $display("RESULT TEST_FAIL map_sd_store_controller failures=%0d", failures);
    $finish;
  end

  initial begin
    repeat (10000000) @(posedge clk);
    $display("RESULT TEST_FAIL map_sd_store_timeout state=%0d op=%0d progress=%0d error=%02x",
             dut.state, flash_operation, flash_progress, flash_error);
    $finish;
  end
endmodule

module fake_sd_block_device (
  input wire clk,
  input wire rst_n,
  input wire init_start,
  output reg initialized,
  output reg init_done,
  input wire block_start,
  input wire block_write,
  input wire [31:0] block_lba,
  output reg block_busy,
  output reg block_done,
  output reg [7:0] error_code,
  output reg [7:0] last_r1,
  output reg [7:0] write_data_response,
  output reg high_capacity,
  output reg [8:0] buffer_addr,
  input wire [7:0] buffer_read_data,
  output reg buffer_write_enable,
  output reg [7:0] buffer_write_data,
  input wire [7:0] fail_write_ordinal
);
  reg [7:0] slot_a [0:9215];
  reg [7:0] slot_b [0:9215];
  reg [2:0] init_count;
  reg active_write;
  reg [31:0] active_lba;
  reg [9:0] byte_index;
  reg pending_failure;
  reg read_phase;
  reg [7:0] write_count;
  reg [15:0] read_count;
  integer index;

  function [7:0] storage_read;
    input [31:0] lba;
    input [8:0] address;
    integer offset;
    begin
      if ((lba >= 32'd4096) && (lba <= 32'd4113)) begin
        offset = ((lba - 32'd4096) * 512) + address;
        storage_read = slot_a[offset];
      end else if ((lba >= 32'd8192) && (lba <= 32'd8209)) begin
        offset = ((lba - 32'd8192) * 512) + address;
        storage_read = slot_b[offset];
      end else begin
        storage_read = 8'hff;
      end
    end
  endfunction

  task storage_write;
    input [31:0] lba;
    input [8:0] address;
    input [7:0] value;
    integer offset;
    begin
      if ((lba >= 32'd4096) && (lba <= 32'd4113)) begin
        offset = ((lba - 32'd4096) * 512) + address;
        slot_a[offset] = value;
      end else if ((lba >= 32'd8192) && (lba <= 32'd8209)) begin
        offset = ((lba - 32'd8192) * 512) + address;
        slot_b[offset] = value;
      end
    end
  endtask

  initial begin
    initialized = 1'b0;
    init_done = 1'b0;
    block_busy = 1'b0;
    block_done = 1'b0;
    error_code = 8'h00;
    last_r1 = 8'hff;
    write_data_response = 8'hff;
    high_capacity = 1'b1;
    buffer_addr = 9'd0;
    buffer_write_enable = 1'b0;
    buffer_write_data = 8'hff;
    init_count = 3'd0;
    active_write = 1'b0;
    active_lba = 32'd0;
    byte_index = 10'd0;
    pending_failure = 1'b0;
    read_phase = 1'b0;
    write_count = 8'd0;
    read_count = 16'd0;
    for (index = 0; index < 9216; index = index + 1) begin
      slot_a[index] = 8'hff;
      slot_b[index] = 8'hff;
    end
  end

  always @(posedge clk) begin
    init_done <= 1'b0;
    block_done <= 1'b0;
    buffer_write_enable <= 1'b0;
    if (!rst_n) begin
      initialized <= 1'b0;
      block_busy <= 1'b0;
      init_count <= 3'd0;
      error_code <= 8'h00;
    end else begin
      if (init_start && !initialized && (init_count == 0))
        init_count <= 3'd1;
      if (init_count != 0) begin
        if (init_count == 3'd4) begin
          initialized <= 1'b1;
          init_done <= 1'b1;
          init_count <= 3'd0;
          error_code <= 8'h00;
        end else begin
          init_count <= init_count + 3'd1;
        end
      end

      if (block_start && initialized && !block_busy) begin
        block_busy <= 1'b1;
        active_write <= block_write;
        active_lba <= block_lba;
        byte_index <= 10'd0;
        buffer_addr <= 9'd0;
        error_code <= 8'h00;
        last_r1 <= 8'h00;
        pending_failure <= block_write &&
            (fail_write_ordinal != 0) &&
            ((write_count + 8'd1) == fail_write_ordinal);
        read_phase <= 1'b0;
        if (block_write)
          write_count <= write_count + 8'd1;
        else
          read_count <= read_count + 16'd1;
      end else if (block_busy) begin
        if (active_write) begin
          storage_write(active_lba, byte_index[8:0], buffer_read_data);
          write_data_response <= 8'h05;
          if (byte_index == 10'd511) begin
            block_busy <= 1'b0;
            block_done <= 1'b1;
            if (pending_failure)
              error_code <= 8'h15;
            else
              error_code <= 8'h00;
          end else begin
            byte_index <= byte_index + 10'd1;
            buffer_addr <= byte_index[8:0] + 9'd1;
          end
        end else if (!read_phase) begin
          buffer_addr <= byte_index[8:0];
          buffer_write_data <= storage_read(active_lba, byte_index[8:0]);
          buffer_write_enable <= 1'b1;
          read_phase <= 1'b1;
        end else begin
          read_phase <= 1'b0;
          if (byte_index == 10'd511) begin
            block_busy <= 1'b0;
            block_done <= 1'b1;
            error_code <= 8'h00;
          end else begin
            byte_index <= byte_index + 10'd1;
          end
        end
      end
    end
  end
endmodule
