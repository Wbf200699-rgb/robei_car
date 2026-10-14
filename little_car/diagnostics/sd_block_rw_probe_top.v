module sd_block_rw_probe_top (
  input wire clk,
  input wire rst_n,

  input wire sd_cd,
  input wire sd_d1,
  input wire sd_d2,
  input wire sd_miso,
  output wire sd_cs_n,
  output wire sd_sclk,
  output wire sd_mosi,

  output reg debug_uart_tx,
  output wire probe_done_led,
  output wire probe_ok_led
);

  parameter integer CLK_HZ = 50000000;
  parameter [31:0] SCRATCH_LBA = 32'd3072;
  localparam integer UART_CLKS_PER_BIT = CLK_HZ / 115200;
  localparam integer REPORT_GAP_CYCLES = CLK_HZ;

  localparam [3:0] ST_INIT_START  = 4'd0;
  localparam [3:0] ST_INIT_WAIT   = 4'd1;
  localparam [3:0] ST_FILL        = 4'd2;
  localparam [3:0] ST_WRITE_START = 4'd3;
  localparam [3:0] ST_WRITE_WAIT  = 4'd4;
  localparam [3:0] ST_CLEAR       = 4'd5;
  localparam [3:0] ST_READ_START  = 4'd6;
  localparam [3:0] ST_READ_WAIT   = 4'd7;
  localparam [3:0] ST_COMPARE     = 4'd8;
  localparam [3:0] ST_DONE        = 4'd9;

  localparam [1:0] UART_IDLE  = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA  = 2'd2;
  localparam [1:0] UART_STOP  = 2'd3;

  function [7:0] pattern_byte;
    input [8:0] address;
    begin
      // Both halves of the sector use different salts. The folded address
      // bits make adjacent bytes and the two 256-byte halves distinguishable.
      pattern_byte = address[7:0] ^ {address[4:0], address[8:6]} ^
          (address[8] ? 8'h3c : 8'hc3);
    end
  endfunction

  function [31:0] crc32_byte;
    input [31:0] current;
    input [7:0] data;
    integer bit_index;
    reg [31:0] value;
    begin
      value = current ^ data;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[0])
          value = (value >> 1) ^ 32'hedb88320;
        else
          value = value >> 1;
      end
      crc32_byte = value;
    end
  endfunction

  reg [7:0] sector_buffer [0:511];
  wire [8:0] sd_buffer_addr;
  wire [7:0] sd_buffer_read_data = sector_buffer[sd_buffer_addr];
  wire sd_buffer_write_enable;
  wire [7:0] sd_buffer_write_data;

  reg [3:0] state;
  reg [8:0] local_index;
  reg init_start;
  reg block_start;
  reg block_write;
  reg [31:0] block_lba;
  reg probe_done;
  reg probe_ok;
  reg [15:0] mismatch_count;
  reg [7:0] first_read_byte;
  reg [7:0] last_read_byte;
  reg [31:0] fill_crc_state;
  reg [31:0] read_crc_state;
  reg [31:0] expected_crc;
  reg [31:0] actual_crc;

  wire initialized;
  wire init_busy;
  wire init_done;
  wire block_busy;
  wire block_done;
  wire [7:0] error_code;
  wire [7:0] last_r1;
  wire [7:0] write_data_response;
  wire [15:0] acmd41_attempts;
  wire [31:0] ocr;
  wire high_capacity;

  assign probe_done_led = probe_done;
  assign probe_ok_led = probe_ok;

  // A reset-free, single-write-port memory process lets both Vivado and Yosys
  // infer distributed RAM instead of expanding the 512-byte buffer into FFs.
  always @(posedge clk) begin
    if (state == ST_FILL)
      sector_buffer[local_index] <= pattern_byte(local_index);
    else if (state == ST_CLEAR)
      sector_buffer[local_index] <= 8'h00;
    else if (sd_buffer_write_enable)
      sector_buffer[sd_buffer_addr] <= sd_buffer_write_data;
  end

  sd_spi_block_device sd_spi_block_device0 (
    .clk(clk),
    .rst_n(rst_n),
    .init_start(init_start),
    .block_start(block_start),
    .block_write(block_write),
    .block_lba(block_lba),
    .buffer_addr(sd_buffer_addr),
    .buffer_read_data(sd_buffer_read_data),
    .buffer_write_enable(sd_buffer_write_enable),
    .buffer_write_data(sd_buffer_write_data),
    .sd_miso(sd_miso),
    .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi),
    .initialized(initialized),
    .init_busy(init_busy),
    .init_done(init_done),
    .block_busy(block_busy),
    .block_done(block_done),
    .error_code(error_code),
    .last_r1(last_r1),
    .write_data_response(write_data_response),
    .acmd41_attempts(acmd41_attempts),
    .ocr(ocr),
    .high_capacity(high_capacity)
  );

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_INIT_START;
      local_index <= 9'd0;
      init_start <= 1'b0;
      block_start <= 1'b0;
      block_write <= 1'b0;
      block_lba <= SCRATCH_LBA;
      probe_done <= 1'b0;
      probe_ok <= 1'b0;
      mismatch_count <= 16'd0;
      first_read_byte <= 8'd0;
      last_read_byte <= 8'd0;
      fill_crc_state <= 32'hffffffff;
      read_crc_state <= 32'hffffffff;
      expected_crc <= 32'd0;
      actual_crc <= 32'd0;
    end else begin
      init_start <= 1'b0;
      block_start <= 1'b0;

      case (state)
        ST_INIT_START: begin
          init_start <= 1'b1;
          state <= ST_INIT_WAIT;
        end

        ST_INIT_WAIT: begin
          if (init_done) begin
            if (initialized && (error_code == 8'h00)) begin
              local_index <= 9'd0;
              fill_crc_state <= 32'hffffffff;
              state <= ST_FILL;
            end else begin
              probe_done <= 1'b1;
              probe_ok <= 1'b0;
              state <= ST_DONE;
            end
          end
        end

        ST_FILL: begin
          fill_crc_state <= crc32_byte(
              fill_crc_state, pattern_byte(local_index));
          if (local_index == 9'd511) begin
            expected_crc <= ~crc32_byte(
                fill_crc_state, pattern_byte(local_index));
            local_index <= 9'd0;
            state <= ST_WRITE_START;
          end else begin
            local_index <= local_index + 9'd1;
          end
        end

        ST_WRITE_START: begin
          block_lba <= SCRATCH_LBA;
          block_write <= 1'b1;
          block_start <= 1'b1;
          state <= ST_WRITE_WAIT;
        end

        ST_WRITE_WAIT: begin
          if (block_done) begin
            if ((error_code == 8'h00) &&
                ((write_data_response & 8'h1f) == 8'h05)) begin
              local_index <= 9'd0;
              state <= ST_CLEAR;
            end else begin
              probe_done <= 1'b1;
              probe_ok <= 1'b0;
              state <= ST_DONE;
            end
          end
        end

        ST_CLEAR: begin
          if (local_index == 9'd511) begin
            local_index <= 9'd0;
            state <= ST_READ_START;
          end else begin
            local_index <= local_index + 9'd1;
          end
        end

        ST_READ_START: begin
          block_lba <= SCRATCH_LBA;
          block_write <= 1'b0;
          block_start <= 1'b1;
          state <= ST_READ_WAIT;
        end

        ST_READ_WAIT: begin
          if (block_done) begin
            if (error_code == 8'h00) begin
              local_index <= 9'd0;
              mismatch_count <= 16'd0;
              read_crc_state <= 32'hffffffff;
              state <= ST_COMPARE;
            end else begin
              probe_done <= 1'b1;
              probe_ok <= 1'b0;
              state <= ST_DONE;
            end
          end
        end

        ST_COMPARE: begin
          read_crc_state <= crc32_byte(
              read_crc_state, sector_buffer[local_index]);
          if (sector_buffer[local_index] != pattern_byte(local_index))
            mismatch_count <= mismatch_count + 16'd1;
          if (local_index == 9'd0)
            first_read_byte <= sector_buffer[local_index];

          if (local_index == 9'd511) begin
            last_read_byte <= sector_buffer[local_index];
            actual_crc <= ~crc32_byte(
                read_crc_state, sector_buffer[local_index]);
            probe_done <= 1'b1;
            probe_ok <=
                (mismatch_count == 16'd0) &&
                (sector_buffer[local_index] == pattern_byte(local_index)) &&
                (expected_crc == ~crc32_byte(
                    read_crc_state, sector_buffer[local_index]));
            state <= ST_DONE;
          end else begin
            local_index <= local_index + 9'd1;
          end
        end

        default: begin
          state <= ST_DONE;
        end
      endcase
    end
  end

  function [7:0] hex_char;
    input [3:0] value;
    begin
      hex_char = value < 10 ? ("0" + value) : ("A" + value - 10);
    end
  endfunction

  function [7:0] bit_char;
    input value;
    begin
      bit_char = value ? "1" : "0";
    end
  endfunction

  // Report format:
  // @SDW,CDD1D2,initialized,done,pass,error,r1,response,ccs,attempts,
  //      lba,mismatches,first,last,expected_crc,actual_crc\n
  function [7:0] report_char;
    input [6:0] index;
    begin
      report_char = "?";
      if (index == 0) report_char = "@";
      else if (index == 1) report_char = "S";
      else if (index == 2) report_char = "D";
      else if (index == 3) report_char = "W";
      else if (index == 4) report_char = ",";
      else if (index == 5) report_char = bit_char(sd_cd);
      else if (index == 6) report_char = bit_char(sd_d1);
      else if (index == 7) report_char = bit_char(sd_d2);
      else if (index == 8) report_char = ",";
      else if (index == 9) report_char = bit_char(initialized);
      else if (index == 10) report_char = ",";
      else if (index == 11) report_char = bit_char(probe_done);
      else if (index == 12) report_char = ",";
      else if (index == 13) report_char = bit_char(probe_ok);
      else if (index == 14) report_char = ",";
      else if (index <= 16)
        report_char = hex_char(error_code[(16-index)*4 +: 4]);
      else if (index == 17) report_char = ",";
      else if (index <= 19)
        report_char = hex_char(last_r1[(19-index)*4 +: 4]);
      else if (index == 20) report_char = ",";
      else if (index <= 22)
        report_char = hex_char(write_data_response[(22-index)*4 +: 4]);
      else if (index == 23) report_char = ",";
      else if (index == 24) report_char = bit_char(high_capacity);
      else if (index == 25) report_char = ",";
      else if (index <= 29)
        report_char = hex_char(acmd41_attempts[(29-index)*4 +: 4]);
      else if (index == 30) report_char = ",";
      else if (index <= 38)
        report_char = hex_char(SCRATCH_LBA[(38-index)*4 +: 4]);
      else if (index == 39) report_char = ",";
      else if (index <= 43)
        report_char = hex_char(mismatch_count[(43-index)*4 +: 4]);
      else if (index == 44) report_char = ",";
      else if (index <= 46)
        report_char = hex_char(first_read_byte[(46-index)*4 +: 4]);
      else if (index == 47) report_char = ",";
      else if (index <= 49)
        report_char = hex_char(last_read_byte[(49-index)*4 +: 4]);
      else if (index == 50) report_char = ",";
      else if (index <= 58)
        report_char = hex_char(expected_crc[(58-index)*4 +: 4]);
      else if (index == 59) report_char = ",";
      else if (index <= 67)
        report_char = hex_char(actual_crc[(67-index)*4 +: 4]);
      else report_char = 8'h0a;
    end
  endfunction

  reg uart_start;
  reg [7:0] uart_tx_data;
  reg uart_busy;
  reg uart_done;
  reg [1:0] uart_state;
  reg [15:0] uart_clock_count;
  reg [2:0] uart_bit_index;
  reg [7:0] uart_shift;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      debug_uart_tx <= 1'b1;
      uart_busy <= 1'b0;
      uart_done <= 1'b0;
      uart_state <= UART_IDLE;
      uart_clock_count <= 16'd0;
      uart_bit_index <= 3'd0;
      uart_shift <= 8'd0;
    end else begin
      uart_done <= 1'b0;
      case (uart_state)
        UART_IDLE: begin
          debug_uart_tx <= 1'b1;
          uart_busy <= 1'b0;
          if (uart_start) begin
            uart_shift <= uart_tx_data;
            debug_uart_tx <= 1'b0;
            uart_busy <= 1'b1;
            uart_clock_count <= 16'd0;
            uart_bit_index <= 3'd0;
            uart_state <= UART_START;
          end
        end
        UART_START: begin
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            debug_uart_tx <= uart_shift[0];
            uart_state <= UART_DATA;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        UART_DATA: begin
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            if (uart_bit_index == 3'd7) begin
              debug_uart_tx <= 1'b1;
              uart_state <= UART_STOP;
            end else begin
              uart_bit_index <= uart_bit_index + 3'd1;
              debug_uart_tx <= uart_shift[uart_bit_index + 3'd1];
            end
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        default: begin
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            uart_busy <= 1'b0;
            uart_done <= 1'b1;
            uart_state <= UART_IDLE;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
      endcase
    end
  end

  reg report_active;
  reg [6:0] report_index;
  reg [31:0] report_gap_count;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uart_start <= 1'b0;
      uart_tx_data <= 8'd0;
      report_active <= 1'b0;
      report_index <= 7'd0;
      report_gap_count <= 32'd0;
    end else begin
      uart_start <= 1'b0;
      if (!probe_done) begin
        report_gap_count <= 32'd0;
      end else if (!report_active) begin
        if (report_gap_count >= REPORT_GAP_CYCLES - 1) begin
          report_gap_count <= 32'd0;
          report_active <= 1'b1;
          report_index <= 7'd0;
          uart_tx_data <= report_char(7'd0);
          uart_start <= 1'b1;
        end else report_gap_count <= report_gap_count + 32'd1;
      end else if (uart_done) begin
        if (report_index == 7'd68) begin
          report_active <= 1'b0;
          report_gap_count <= 32'd0;
        end else begin
          report_index <= report_index + 7'd1;
          uart_tx_data <= report_char(report_index + 7'd1);
          uart_start <= 1'b1;
        end
      end
    end
  end

endmodule
