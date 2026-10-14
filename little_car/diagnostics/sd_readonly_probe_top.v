module sd_readonly_probe_top (
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
  localparam integer UART_CLKS_PER_BIT = CLK_HZ / 115200;
  localparam integer REPORT_GAP_CYCLES = CLK_HZ;

  localparam [1:0] UART_IDLE  = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA  = 2'd2;
  localparam [1:0] UART_STOP  = 2'd3;

  reg probe_start;
  reg probe_started;
  wire probe_busy;
  wire probe_done;
  wire probe_ok;
  wire [7:0] error_code;
  wire [7:0] cmd0_r1;
  wire [7:0] cmd8_r1;
  wire [31:0] cmd8_echo;
  wire [7:0] acmd41_r1;
  wire [15:0] acmd41_attempts;
  wire [7:0] cmd58_r1;
  wire [31:0] ocr;
  wire high_capacity;
  wire [7:0] primary_read_r1;
  wire [7:0] primary_token;
  wire [63:0] primary_magic;
  wire [31:0] primary_version;
  wire [31:0] primary_stored_crc;
  wire [31:0] primary_calculated_crc;
  wire [7:0] backup_read_r1;
  wire [7:0] backup_token;
  wire [63:0] backup_magic;
  wire [31:0] backup_version;
  wire [31:0] backup_stored_crc;
  wire [31:0] backup_calculated_crc;

  assign probe_done_led = probe_done;
  assign probe_ok_led = probe_ok;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      probe_start <= 1'b0;
      probe_started <= 1'b0;
    end else begin
      probe_start <= 1'b0;
      if (!probe_started) begin
        probe_start <= 1'b1;
        probe_started <= 1'b1;
      end
    end
  end

  sd_spi_readonly sd_spi_readonly0 (
    .clk(clk),
    .rst_n(rst_n),
    .start(probe_start),
    .sd_miso(sd_miso),
    .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi),
    .probe_busy(probe_busy),
    .probe_done(probe_done),
    .probe_ok(probe_ok),
    .error_code(error_code),
    .cmd0_r1(cmd0_r1),
    .cmd8_r1(cmd8_r1),
    .cmd8_echo(cmd8_echo),
    .acmd41_r1(acmd41_r1),
    .acmd41_attempts(acmd41_attempts),
    .cmd58_r1(cmd58_r1),
    .ocr(ocr),
    .high_capacity(high_capacity),
    .primary_read_r1(primary_read_r1),
    .primary_token(primary_token),
    .primary_magic(primary_magic),
    .primary_version(primary_version),
    .primary_stored_crc(primary_stored_crc),
    .primary_calculated_crc(primary_calculated_crc),
    .backup_read_r1(backup_read_r1),
    .backup_token(backup_token),
    .backup_magic(backup_magic),
    .backup_version(backup_version),
    .backup_stored_crc(backup_stored_crc),
    .backup_calculated_crc(backup_calculated_crc)
  );

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

  function [6:0] report_last_index;
    input [1:0] line;
    begin
      case (line)
        2'd0: report_last_index = 7'd33;
        2'd1: report_last_index = 7'd72;
        default: report_last_index = 7'd54;
      endcase
    end
  endfunction

  function [7:0] report_char;
    input [1:0] line;
    input [6:0] index;
    begin
      report_char = "?";
      case (line)
        2'd0: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "S";
          else if (index == 2) report_char = "D";
          else if (index == 3) report_char = "0";
          else if (index == 4) report_char = ",";
          else if (index == 5) report_char = bit_char(sd_cd);
          else if (index == 6) report_char = bit_char(sd_d1);
          else if (index == 7) report_char = bit_char(sd_d2);
          else if (index == 8) report_char = ",";
          else if (index == 9) report_char = bit_char(probe_done);
          else if (index == 10) report_char = bit_char(probe_ok);
          else if (index == 11) report_char = ",";
          else if (index <= 13)
            report_char = hex_char(error_code[(13-index)*4 +: 4]);
          else if (index == 14) report_char = ",";
          else if (index <= 16)
            report_char = hex_char(cmd0_r1[(16-index)*4 +: 4]);
          else if (index == 17) report_char = ",";
          else if (index <= 19)
            report_char = hex_char(cmd8_r1[(19-index)*4 +: 4]);
          else if (index == 20) report_char = ",";
          else if (index <= 22)
            report_char = hex_char(acmd41_r1[(22-index)*4 +: 4]);
          else if (index == 23) report_char = ",";
          else if (index <= 27)
            report_char = hex_char(acmd41_attempts[(27-index)*4 +: 4]);
          else if (index == 28) report_char = ",";
          else if (index <= 30)
            report_char = hex_char(cmd58_r1[(30-index)*4 +: 4]);
          else if (index == 31) report_char = ",";
          else if (index == 32) report_char = bit_char(high_capacity);
          else report_char = 8'h0a;
        end

        2'd1: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "S";
          else if (index == 2) report_char = "D";
          else if (index == 3) report_char = "1";
          else if (index == 4) report_char = ",";
          else if (index <= 12)
            report_char = hex_char(cmd8_echo[(12-index)*4 +: 4]);
          else if (index == 13) report_char = ",";
          else if (index <= 21)
            report_char = hex_char(ocr[(21-index)*4 +: 4]);
          else if (index == 22) report_char = ",";
          else if (index <= 24)
            report_char = hex_char(primary_read_r1[(24-index)*4 +: 4]);
          else if (index == 25) report_char = ",";
          else if (index <= 27)
            report_char = hex_char(primary_token[(27-index)*4 +: 4]);
          else if (index == 28) report_char = ",";
          else if (index <= 44)
            report_char = hex_char(primary_magic[(44-index)*4 +: 4]);
          else if (index == 45) report_char = ",";
          else if (index <= 53)
            report_char = hex_char(primary_version[(53-index)*4 +: 4]);
          else if (index == 54) report_char = ",";
          else if (index <= 62)
            report_char = hex_char(primary_stored_crc[(62-index)*4 +: 4]);
          else if (index == 63) report_char = ",";
          else if (index <= 71)
            report_char = hex_char(primary_calculated_crc[(71-index)*4 +: 4]);
          else report_char = 8'h0a;
        end

        default: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "S";
          else if (index == 2) report_char = "D";
          else if (index == 3) report_char = "2";
          else if (index == 4) report_char = ",";
          else if (index <= 6)
            report_char = hex_char(backup_read_r1[(6-index)*4 +: 4]);
          else if (index == 7) report_char = ",";
          else if (index <= 9)
            report_char = hex_char(backup_token[(9-index)*4 +: 4]);
          else if (index == 10) report_char = ",";
          else if (index <= 26)
            report_char = hex_char(backup_magic[(26-index)*4 +: 4]);
          else if (index == 27) report_char = ",";
          else if (index <= 35)
            report_char = hex_char(backup_version[(35-index)*4 +: 4]);
          else if (index == 36) report_char = ",";
          else if (index <= 44)
            report_char = hex_char(backup_stored_crc[(44-index)*4 +: 4]);
          else if (index == 45) report_char = ",";
          else if (index <= 53)
            report_char = hex_char(backup_calculated_crc[(53-index)*4 +: 4]);
          else report_char = 8'h0a;
        end
      endcase
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
  reg [1:0] report_line;
  reg [6:0] report_index;
  reg [31:0] report_gap_count;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uart_start <= 1'b0;
      uart_tx_data <= 8'd0;
      report_active <= 1'b0;
      report_line <= 2'd0;
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
          report_line <= 2'd0;
          report_index <= 7'd0;
          uart_tx_data <= report_char(2'd0, 7'd0);
          uart_start <= 1'b1;
        end else report_gap_count <= report_gap_count + 32'd1;
      end else if (uart_done) begin
        if (report_index == report_last_index(report_line)) begin
          if (report_line == 2'd2) begin
            report_active <= 1'b0;
            report_gap_count <= 32'd0;
          end else begin
            report_line <= report_line + 2'd1;
            report_index <= 7'd0;
            uart_tx_data <= report_char(report_line + 2'd1, 7'd0);
            uart_start <= 1'b1;
          end
        end else begin
          report_index <= report_index + 7'd1;
          uart_tx_data <= report_char(report_line, report_index + 7'd1);
          uart_start <= 1'b1;
        end
      end
    end
  end

endmodule
