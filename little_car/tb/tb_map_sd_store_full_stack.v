`timescale 1ns/1ps

module tb_map_sd_store_full_stack;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg save_pulse = 1'b0;
  reg load_pulse = 1'b0;
  reg [7:0] map_epoch = 8'd12;
  reg [15:0] pose_x_mm = 16'd3000;
  reg [15:0] pose_y_mm = 16'd3250;
  reg [9:0] pose_heading = 10'd768;
  reg [15:0] scan_count = 16'd2048;
  reg [15:0] occupied_count = 16'd512;
  reg [15:0] free_count = 16'd4096;
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
  wire sd_cs_n;
  wire sd_sclk;
  wire sd_mosi;
  wire sd_miso;
  wire flash_ready;
  wire flash_busy;
  wire [1:0] flash_operation;
  wire [7:0] flash_progress;
  wire [7:0] flash_error;
  wire [23:0] flash_jedec_id;
  wire [31:0] saved_sequence;

  reg signed [3:0] map_cells [0:16383];
  reg signed [3:0] expected_cells [0:16383];
  reg map_read_pending = 1'b0;
  reg [13:0] map_read_addr = 14'd0;
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

  map_sd_store #(
    .BOOT_WAIT_CYCLES(4),
    .SD_STARTUP_WAIT_CYCLES(20),
    .SD_ACMD_RETRY_WAIT_CYCLES(8),
    .SD_INIT_SPI_HALF_CYCLES(2),
    .SD_DATA_SPI_HALF_CYCLES(1),
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
    .sd_cd(1'b0), .sd_d1(1'b1), .sd_d2(1'b1),
    .sd_miso(sd_miso), .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk), .sd_mosi(sd_mosi),
    .flash_ready(flash_ready), .flash_busy(flash_busy),
    .flash_operation(flash_operation), .flash_progress(flash_progress),
    .flash_error(flash_error), .flash_jedec_id(flash_jedec_id),
    .saved_sequence(saved_sequence)
  );

  sd_spi_map_card_model card (
    .sd_cs_n(sd_cs_n), .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi), .sd_miso(sd_miso)
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

  initial begin
    $dumpfile("temp/tb_map_sd_store_full_stack.vcd");
    $dumpvars(1, tb_map_sd_store_full_stack);
    for (index = 0; index < 16384; index = index + 1) begin
      map_cells[index] = ((index * 3) + (index >> 8)) & 4'hf;
      expected_cells[index] = map_cells[index];
    end

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (flash_ready && !flash_busy);
    pulse_save();
    if ((flash_error != 0) || (saved_sequence != 1)) begin
      $display("ERROR full save error=%02x sequence=%0d",
               flash_error, saved_sequence);
      failures = failures + 1;
    end

    for (index = 0; index < 16384; index = index + 1)
      map_cells[index] = 4'sd0;
    pulse_load();
    repeat (5) @(posedge clk);
    for (index = 0; index < 16384; index = index + 1) begin
      if (map_cells[index] !== expected_cells[index]) begin
        $display("ERROR full readback index=%0d got=%0d expected=%0d",
                 index, map_cells[index], expected_cells[index]);
        failures = failures + 1;
        index = 16384;
      end
    end
    if ((loaded_map_epoch != map_epoch) ||
        (loaded_pose_x_mm != pose_x_mm) ||
        (loaded_pose_y_mm != pose_y_mm) ||
        (loaded_pose_heading != pose_heading) ||
        (loaded_scan_count != scan_count) ||
        (loaded_occupied_count != occupied_count) ||
        (loaded_free_count != free_count) || (load_apply_count != 1)) begin
      $display("ERROR full metadata epoch=%0d pose=%0d,%0d heading=%0d loads=%0d",
               loaded_map_epoch, loaded_pose_x_mm, loaded_pose_y_mm,
               loaded_pose_heading, load_apply_count);
      failures = failures + 1;
    end

    $display("RESULT full_stack sequence=%0d writes=%0d reads=%0d response=%02x",
             saved_sequence, card.cmd24_count, card.cmd17_count,
             dut.sd_spi_block_device0.write_data_response);
    $display("RESULT full_stack metadata epoch=%0d pose=%0d,%0d heading=%0d scan=%0d",
             loaded_map_epoch, loaded_pose_x_mm, loaded_pose_y_mm,
             loaded_pose_heading, loaded_scan_count);
    if ((card.cmd24_count != 19) || (card.cmd17_count != 54)) begin
      $display("ERROR full command counts write=%0d read=%0d",
               card.cmd24_count, card.cmd17_count);
      failures = failures + 1;
    end
    if (failures == 0)
      $display("RESULT TEST_PASS map_sd_store_full_stack");
    else
      $display("RESULT TEST_FAIL map_sd_store_full_stack failures=%0d", failures);
    $finish;
  end

  initial begin
    repeat (12000000) @(posedge clk);
    $display("RESULT TEST_FAIL map_sd_full_timeout state=%0d host=%0d error=%02x",
             dut.map_sd_store_controller0.state,
             dut.sd_spi_block_device0.state, flash_error);
    $finish;
  end
endmodule

module sd_spi_map_card_model (
  input wire sd_cs_n,
  input wire sd_sclk,
  input wire sd_mosi,
  output reg sd_miso
);
  reg [7:0] slot_a [0:9215];
  reg [7:0] slot_b [0:9215];
  reg [7:0] tx_queue [0:2047];
  integer queue_head;
  integer queue_tail;
  reg [7:0] rx_shift;
  integer rx_bit_count;
  reg [7:0] tx_shift;
  integer tx_bit_index;
  reg load_next_byte;
  reg collecting_command;
  integer command_index;
  reg [7:0] command_packet [0:5];
  integer acmd_attempt;
  reg card_ready;
  reg write_wait_token;
  reg write_collecting;
  integer write_byte_index;
  integer write_crc_index;
  reg [31:0] active_write_lba;
  integer index;
  reg [7:0] completed_byte;
  reg [5:0] decoded_command;
  reg [31:0] decoded_argument;
  integer cmd24_count;
  integer cmd17_count;

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

  task enqueue;
    input [7:0] value;
    begin
      tx_queue[queue_tail] = value;
      queue_tail = queue_tail + 1;
    end
  endtask

  task process_command;
    begin
      decoded_command = command_packet[0][5:0];
      decoded_argument = {
        command_packet[1], command_packet[2],
        command_packet[3], command_packet[4]
      };
      case (decoded_command)
        6'd0: enqueue(8'h01);
        6'd8: begin
          enqueue(8'h01); enqueue(8'h00); enqueue(8'h00);
          enqueue(8'h01); enqueue(8'haa);
        end
        6'd55: enqueue(card_ready ? 8'h00 : 8'h01);
        6'd41: begin
          acmd_attempt = acmd_attempt + 1;
          if (acmd_attempt >= 3) begin
            card_ready = 1'b1;
            enqueue(8'h00);
          end else enqueue(8'h01);
        end
        6'd58: begin
          enqueue(8'h00); enqueue(8'hc0); enqueue(8'hff);
          enqueue(8'h80); enqueue(8'h00);
        end
        6'd17: begin
          cmd17_count = cmd17_count + 1;
          enqueue(8'h00); enqueue(8'hff); enqueue(8'hfe);
          for (index = 0; index < 512; index = index + 1)
            enqueue(storage_read(decoded_argument, index[8:0]));
          enqueue(8'h12); enqueue(8'h34);
        end
        6'd24: begin
          cmd24_count = cmd24_count + 1;
          active_write_lba = decoded_argument;
          write_wait_token = 1'b1;
          enqueue(8'h00);
        end
        default: enqueue(8'h04);
      endcase
    end
  endtask

  initial begin
    sd_miso = 1'b1;
    queue_head = 0; queue_tail = 0;
    rx_shift = 8'd0; rx_bit_count = 0;
    tx_shift = 8'hff; tx_bit_index = 6;
    load_next_byte = 1'b0;
    collecting_command = 1'b0; command_index = 0;
    acmd_attempt = 0; card_ready = 1'b0;
    write_wait_token = 1'b0; write_collecting = 1'b0;
    write_byte_index = 0; write_crc_index = 0;
    active_write_lba = 32'd0;
    cmd24_count = 0; cmd17_count = 0;
    for (index = 0; index < 9216; index = index + 1) begin
      slot_a[index] = 8'hff;
      slot_b[index] = 8'hff;
    end
  end

  always @(negedge sd_cs_n) begin
    queue_head = 0; queue_tail = 0;
    rx_shift = 8'd0; rx_bit_count = 0;
    tx_shift = 8'hff; tx_bit_index = 6;
    load_next_byte = 1'b0;
    collecting_command = 1'b0; command_index = 0;
    sd_miso = 1'b1;
  end

  always @(posedge sd_cs_n) begin
    sd_miso = 1'b1;
    collecting_command = 1'b0;
    write_wait_token = 1'b0;
    write_collecting = 1'b0;
  end

  always @(posedge sd_sclk) begin
    if (!sd_cs_n) begin
      rx_shift = {rx_shift[6:0], sd_mosi};
      if (rx_bit_count == 7) begin
        completed_byte = rx_shift;
        rx_shift = 8'd0;
        rx_bit_count = 0;
        load_next_byte = 1'b1;
        if (write_collecting) begin
          if (write_byte_index < 512) begin
            storage_write(active_write_lba, write_byte_index[8:0], completed_byte);
            write_byte_index = write_byte_index + 1;
          end else begin
            write_crc_index = write_crc_index + 1;
            if (write_crc_index == 2) begin
              write_collecting = 1'b0;
              write_crc_index = 0;
              enqueue(8'h05);
              enqueue(8'h00); enqueue(8'h00); enqueue(8'hff);
            end
          end
        end else if (write_wait_token) begin
          if (completed_byte == 8'hfe) begin
            write_wait_token = 1'b0;
            write_collecting = 1'b1;
            write_byte_index = 0;
            write_crc_index = 0;
          end
        end else if (!collecting_command) begin
          if (completed_byte[7:6] == 2'b01) begin
            collecting_command = 1'b1;
            command_index = 1;
            command_packet[0] = completed_byte;
          end
        end else begin
          command_packet[command_index] = completed_byte;
          if (command_index == 5) begin
            collecting_command = 1'b0;
            command_index = 0;
            process_command();
          end else command_index = command_index + 1;
        end
      end else rx_bit_count = rx_bit_count + 1;
    end
  end

  always @(negedge sd_sclk) begin
    if (!sd_cs_n) begin
      if (load_next_byte) begin
        if (queue_head < queue_tail) begin
          tx_shift = tx_queue[queue_head];
          queue_head = queue_head + 1;
        end else tx_shift = 8'hff;
        sd_miso = tx_shift[7];
        tx_bit_index = 6;
        load_next_byte = 1'b0;
      end else begin
        sd_miso = tx_shift[tx_bit_index];
        if (tx_bit_index > 0) tx_bit_index = tx_bit_index - 1;
      end
    end
  end
endmodule
