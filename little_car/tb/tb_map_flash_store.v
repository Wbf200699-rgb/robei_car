`timescale 1ns/1ps

module mt25ql128_top_window_model (
  input wire cs_n,
  input wire sclk,
  input wire mosi,
  output reg miso
);
  reg [7:0] memory [0:131071];
  reg [7:0] input_shift;
  reg [2:0] input_bit;
  reg [7:0] output_byte;
  reg [2:0] output_bit;
  reg [7:0] command;
  reg [1:0] address_count;
  reg [23:0] address;
  reg write_enable;
  reg [1:0] jedec_index;
  integer index;
  integer erase_index;
  integer selected_clock_count;
  integer xip_sequence_index;
  reg xip_mode;
  reg reset_enable_seen;
  reg reset_memory_seen;
  reg [7:0] received_byte;

  function integer expected_xip_clocks;
    input integer sequence_index;
    begin
      case (sequence_index)
        0: expected_xip_clocks = 7;
        1: expected_xip_clocks = 9;
        2: expected_xip_clocks = 13;
        3: expected_xip_clocks = 17;
        4: expected_xip_clocks = 25;
        default: expected_xip_clocks = 33;
      endcase
    end
  endfunction

  function integer window_index;
    input [23:0] flash_address;
    begin
      if (flash_address >= 24'hFE0000)
        window_index = flash_address - 24'hFE0000;
      else
        window_index = 0;
    end
  endfunction

  initial begin
    miso = 1'b0;
    input_shift = 8'd0;
    input_bit = 3'd0;
    output_byte = 8'h00;
    output_bit = 3'd7;
    command = 8'd0;
    address_count = 2'd0;
    address = 24'd0;
    write_enable = 1'b0;
    jedec_index = 2'd0;
    selected_clock_count = 0;
    xip_sequence_index = 0;
    xip_mode = 1'b1;
    reset_enable_seen = 1'b0;
    reset_memory_seen = 1'b0;
    for (index = 0; index < 131072; index = index + 1)
      memory[index] = 8'hFF;
  end

  always @(negedge cs_n) begin
    input_shift = 8'd0;
    input_bit = 3'd0;
    output_bit = 3'd7;
    output_byte = 8'h00;
    command = 8'd0;
    address_count = 2'd0;
    address = 24'd0;
    jedec_index = 2'd0;
    selected_clock_count = 0;
    miso = 1'b0;
  end

  always @(posedge sclk) begin
    if (!cs_n && xip_mode) begin
      selected_clock_count = selected_clock_count + 1;
    end else if (!cs_n) begin
      received_byte = {input_shift[6:0], mosi};
      input_shift = received_byte;
      if (input_bit == 3'd7) begin
        input_bit = 3'd0;
        if (command == 8'd0) begin
          command = received_byte;
          if (received_byte == 8'h66) begin
            reset_enable_seen = 1'b1;
          end else if ((received_byte == 8'h99) && reset_enable_seen) begin
            reset_memory_seen = 1'b1;
            reset_enable_seen = 1'b0;
          end else if ((received_byte == 8'h9F) && reset_memory_seen) begin
            output_byte = 8'h20;
            output_bit = 3'd7;
            jedec_index = 2'd0;
          end else if (received_byte == 8'h05) begin
            output_byte = {6'd0, write_enable, 1'b0};
            output_bit = 3'd7;
          end else if (received_byte == 8'h06) begin
            write_enable = 1'b1;
          end
        end else if ((command == 8'h03) || (command == 8'h02) ||
                     (command == 8'hD8)) begin
          if (address_count < 2'd3) begin
            address = {address[15:0], received_byte};
            address_count = address_count + 2'd1;
            if ((address_count == 2'd3) && (command == 8'h03)) begin
              output_byte = memory[window_index(address)];
              output_bit = 3'd7;
            end
          end else if (command == 8'h03) begin
            output_byte = memory[window_index(address + 24'd1)];
            address = address + 24'd1;
            output_bit = 3'd7;
          end else if ((command == 8'h02) && write_enable) begin
            memory[window_index(address)] =
                memory[window_index(address)] & received_byte;
            // MT25QL page-program wraps within the current 256-byte page.
            // Modelling that behavior makes the test fail if the controller
            // forgets to terminate and restart at a page boundary.
            address = {address[23:8], address[7:0] + 8'd1};
          end
        end else if ((command == 8'h9F) && reset_memory_seen) begin
          if (jedec_index == 2'd0) output_byte = 8'hBA;
          else if (jedec_index == 2'd1) output_byte = 8'h18;
          else output_byte = 8'hFF;
          jedec_index = jedec_index + 2'd1;
          output_bit = 3'd7;
        end else if (command == 8'h05) begin
          output_byte = {6'd0, write_enable, 1'b0};
          output_bit = 3'd7;
        end
      end else begin
        input_bit = input_bit + 3'd1;
      end
    end
  end

  always @(negedge sclk) begin
    if (!cs_n) begin
      miso = output_byte[output_bit];
      if (output_bit == 3'd0) output_bit = 3'd7;
      else output_bit = output_bit - 3'd1;
    end
  end

  always @(posedge cs_n) begin
    if (xip_mode) begin
      if (selected_clock_count == expected_xip_clocks(xip_sequence_index)) begin
        if (xip_sequence_index == 5)
          xip_mode = 1'b0;
        else
          xip_sequence_index = xip_sequence_index + 1;
      end else begin
        xip_sequence_index = 0;
      end
    end else if ((command == 8'hD8) && write_enable &&
        (address >= 24'hFE0000)) begin
      for (erase_index = 0; erase_index < 65536; erase_index = erase_index + 1)
        memory[window_index({address[23:16], 16'd0}) + erase_index] = 8'hFF;
      write_enable = 1'b0;
    end else if ((command == 8'h02) && write_enable) begin
      write_enable = 1'b0;
    end
  end
endmodule

module tb_map_flash_store;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #10 clk = ~clk;

  reg save_pulse = 1'b0;
  reg load_pulse = 1'b0;
  reg [7:0] map_epoch = 8'd3;
  reg [15:0] pose_x_mm = 16'd3200;
  reg [15:0] pose_y_mm = 16'd3150;
  reg [9:0] pose_heading = 10'd17;
  reg [15:0] scan_count = 16'd42;
  reg [15:0] occupied_count = 16'd11;
  reg [15:0] free_count = 16'd123;

  wire map_mem_req;
  wire map_mem_we;
  wire [13:0] map_mem_addr;
  wire signed [3:0] map_mem_wdata;
  reg signed [3:0] map_mem_rdata;
  reg map_mem_rvalid;
  reg signed [3:0] map_cells [0:16383];
  reg map_read_pending;
  reg [13:0] map_read_addr;

  wire load_apply_pulse;
  wire [7:0] loaded_map_epoch;
  wire [15:0] loaded_pose_x_mm;
  wire [15:0] loaded_pose_y_mm;
  wire [9:0] loaded_pose_heading;
  wire [15:0] loaded_scan_count;
  wire [15:0] loaded_occupied_count;
  wire [15:0] loaded_free_count;
  wire flash_cs_n;
  wire flash_clk;
  wire flash_mosi;
  wire flash_miso;
  wire flash_ready;
  wire flash_busy;
  wire [1:0] flash_operation;
  wire [7:0] flash_progress;
  wire [7:0] flash_error;
  wire [23:0] flash_jedec_id;
  wire [31:0] saved_sequence;

  integer cell_index;
  integer failures = 0;
  integer crc_index;
  integer flash_clk_edges = 0;
  reg [15:0] physical_crc;
  reg load_seen = 1'b0;

  function [15:0] crc16_next_tb;
    input [15:0] crc_in;
    input [7:0] data_in;
    integer bit_index;
    reg [15:0] value;
    begin
      value = crc_in ^ ({8'd0, data_in} << 8);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[15]) value = (value << 1) ^ 16'h1021;
        else value = value << 1;
      end
      crc16_next_tb = value;
    end
  endfunction

  always @(posedge clk) begin
    map_mem_rvalid <= map_read_pending;
    if (map_read_pending)
      map_mem_rdata <= map_cells[map_read_addr];
    map_read_pending <= map_mem_req && !map_mem_we;
    if (map_mem_req && !map_mem_we)
      map_read_addr <= map_mem_addr;
    if (map_mem_req && map_mem_we)
      map_cells[map_mem_addr] <= map_mem_wdata;
    if (load_apply_pulse)
      load_seen <= 1'b1;
  end

  // Count the externally visible GPIO clock, not an internal hierarchy-only
  // signal. This catches a regression where SPI transactions complete in RTL
  // while the D16 top-level output is accidentally omitted again.
  always @(posedge flash_clk)
    flash_clk_edges = flash_clk_edges + 1;

  map_flash_store #(
    .CLK_HZ(100000),
    .SPI_DIV(1),
    .PAYLOAD_BYTE_COUNT(300)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .save_pulse(save_pulse), .load_pulse(load_pulse),
    .map_epoch(map_epoch), .pose_x_mm(pose_x_mm), .pose_y_mm(pose_y_mm),
    .pose_heading(pose_heading), .scan_count(scan_count),
    .occupied_count(occupied_count), .free_count(free_count),
    .map_mem_req(map_mem_req), .map_mem_we(map_mem_we),
    .map_mem_addr(map_mem_addr), .map_mem_wdata(map_mem_wdata),
    .map_mem_rdata(map_mem_rdata), .map_mem_rvalid(map_mem_rvalid),
    .load_apply_pulse(load_apply_pulse),
    .loaded_map_epoch(loaded_map_epoch),
    .loaded_pose_x_mm(loaded_pose_x_mm), .loaded_pose_y_mm(loaded_pose_y_mm),
    .loaded_pose_heading(loaded_pose_heading),
    .loaded_scan_count(loaded_scan_count),
    .loaded_occupied_count(loaded_occupied_count),
    .loaded_free_count(loaded_free_count),
    .flash_cs_n(flash_cs_n), .flash_clk(flash_clk),
    .flash_mosi(flash_mosi), .flash_miso(flash_miso),
    .flash_ready(flash_ready), .flash_busy(flash_busy),
    .flash_operation(flash_operation), .flash_progress(flash_progress),
    .flash_error(flash_error), .flash_jedec_id(flash_jedec_id),
    .saved_sequence(saved_sequence)
  );

  mt25ql128_top_window_model flash_model (
    .cs_n(flash_cs_n), .sclk(flash_clk),
    .mosi(flash_mosi), .miso(flash_miso)
  );

  task pulse_save;
    begin
      @(negedge clk); save_pulse = 1'b1;
      @(negedge clk); save_pulse = 1'b0;
    end
  endtask

  task pulse_load;
    begin
      @(negedge clk); load_pulse = 1'b1;
      @(negedge clk); load_pulse = 1'b0;
    end
  endtask

  initial begin
    map_mem_rdata = 4'sd0;
    map_mem_rvalid = 1'b0;
    map_read_pending = 1'b0;
    map_read_addr = 14'd0;
    for (cell_index = 0; cell_index < 16384; cell_index = cell_index + 1)
      map_cells[cell_index] = 4'sd0;
    map_cells[10] = 4'sd5;
    map_cells[11] = -4'sd3;
    map_cells[100] = 4'sd7;
    map_cells[520] = -4'sd6;
    map_cells[521] = 4'sd6;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (flash_ready && !flash_busy);
    if (flash_model.xip_mode || !flash_model.reset_memory_seen) begin
      $display("ERROR recovery xip=%0d reset=%0d sequence=%0d",
               flash_model.xip_mode, flash_model.reset_memory_seen,
               flash_model.xip_sequence_index);
      failures = failures + 1;
    end
    if (flash_jedec_id != 24'h20BA18) begin
      $display("ERROR JEDEC=%06X", flash_jedec_id);
      failures = failures + 1;
    end
    if (flash_clk_edges == 0) begin
      $display("ERROR flash_clk D16 output never toggled");
      failures = failures + 1;
    end

    pulse_save();
    wait (flash_busy);
    wait (!flash_busy);
    $display("DEBUG headerA=%02X%02X%02X%02X ver=%02X size=%02X%02X len=%02X%02X crc=%02X%02X valid=%02X",
             flash_model.memory[0], flash_model.memory[1],
             flash_model.memory[2], flash_model.memory[3],
             flash_model.memory[4], flash_model.memory[5], flash_model.memory[6],
             flash_model.memory[24], flash_model.memory[25],
             flash_model.memory[26], flash_model.memory[27],
             flash_model.memory[31]);
    $display("DEBUG headerB=%02X%02X%02X%02X valid=%02X state=%0d crc=%04X",
             flash_model.memory[65536], flash_model.memory[65537],
             flash_model.memory[65538], flash_model.memory[65539],
             flash_model.memory[65567], dut.state, dut.save_crc);
    $display("DEBUG dutHeaderA=%02X%02X%02X%02X ver=%02X size=%02X%02X len=%02X%02X valid=%02X slotValid=%0d",
             dut.header_a[0], dut.header_a[1], dut.header_a[2], dut.header_a[3],
             dut.header_a[4], dut.header_a[5], dut.header_a[6],
             dut.header_a[24], dut.header_a[25], dut.header_a[31],
             dut.slot_a_valid);
    physical_crc = 16'hFFFF;
    for (crc_index = 0; crc_index < 300; crc_index = crc_index + 1)
      physical_crc = crc16_next_tb(physical_crc, flash_model.memory[256 + crc_index]);
    $display("DEBUG payload=%02X %02X %02X %02X physical_crc=%04X",
             flash_model.memory[256], flash_model.memory[257],
             flash_model.memory[258], flash_model.memory[259], physical_crc);
    if (flash_error != 0 || saved_sequence != 1) begin
      $display("ERROR save error=%02X sequence=%0d", flash_error, saved_sequence);
      failures = failures + 1;
    end

    map_cells[10] = 4'sd0;
    map_cells[11] = 4'sd0;
    map_cells[100] = 4'sd0;
    map_cells[520] = 4'sd0;
    map_cells[521] = 4'sd0;
    pulse_load();
    wait (flash_busy);
    wait (!flash_busy);
    repeat (5) @(posedge clk);
    if (!load_seen || flash_error != 0) begin
      $display("ERROR load seen=%0d error=%02X load_crc=%04X expected=%04X",
               load_seen, flash_error, dut.load_crc, dut.expected_load_crc);
      failures = failures + 1;
    end
    if ((map_cells[10] != 4'sd5) || (map_cells[11] != -4'sd3) ||
        (map_cells[100] != 4'sd7) || (map_cells[520] != -4'sd6) ||
        (map_cells[521] != 4'sd6)) begin
      $display("ERROR restored cells=%0d,%0d,%0d,%0d,%0d",
               map_cells[10], map_cells[11], map_cells[100],
               map_cells[520], map_cells[521]);
      failures = failures + 1;
    end
    if ((loaded_pose_x_mm != pose_x_mm) || (loaded_pose_y_mm != pose_y_mm) ||
        (loaded_pose_heading != pose_heading) ||
        (loaded_scan_count != scan_count) ||
        (loaded_occupied_count != occupied_count) ||
        (loaded_free_count != free_count) ||
        (loaded_map_epoch != map_epoch)) begin
      $display("ERROR restored metadata pose=%0d,%0d heading=%0d scan=%0d",
               loaded_pose_x_mm, loaded_pose_y_mm,
               loaded_pose_heading, loaded_scan_count);
      $display("DEBUG heading bytes A=%02X,%02X B=%02X,%02X load_slot=%0d",
               dut.header_a[16], dut.header_a[17],
               dut.header_b[16], dut.header_b[17],
               dut.load_slot);
      failures = failures + 1;
    end

    $display("RESULT flash_jedec=%06X sequence=%0d flash_clk_edges=%0d",
             flash_jedec_id, saved_sequence, flash_clk_edges);
    $display("RESULT restored_cells=%0d,%0d,%0d,%0d,%0d",
             map_cells[10], map_cells[11], map_cells[100],
             map_cells[520], map_cells[521]);
    if (failures == 0)
      $display("RESULT TEST_PASS map_flash_store");
    else
      $display("RESULT TEST_FAIL map_flash_store failures=%0d", failures);
    $finish;
  end

  initial begin
    repeat (2000000) @(posedge clk);
    $display("ERROR watchdog state=%0d operation=%0d progress=%0d error=%02X",
             dut.state, flash_operation, flash_progress, flash_error);
    $finish;
  end
endmodule
