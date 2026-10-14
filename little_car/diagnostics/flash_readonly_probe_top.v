module flash_readonly_probe_top (
  input wire clk,
  input wire rst_n,
  output reg flash_cs_n,
  output reg flash_clk,
  output reg flash_mosi,
  output wire flash_wp_n,
  output wire flash_hold_n,
  input wire flash_miso,
  output reg debug_uart_tx
);

  parameter integer CLK_HZ = 50000000;
  localparam integer UART_CLKS_PER_BIT = CLK_HZ / 115200;
  localparam integer STARTUP_WAIT_CYCLES = CLK_HZ / 100;
  localparam integer TEST_GAP_CYCLES = CLK_HZ / 10000;
  localparam integer RESET_GAP_CYCLES = CLK_HZ / 1000;
  localparam integer REPORT_GAP_CYCLES = CLK_HZ;

  localparam [2:0] PROBE_WAIT = 3'd0;
  localparam [2:0] PROBE_TEST_START = 3'd1;
  localparam [2:0] PROBE_BYTE_START = 3'd2;
  localparam [2:0] PROBE_BYTE_WAIT = 3'd3;
  localparam [2:0] PROBE_TEST_GAP = 3'd4;
  localparam [2:0] PROBE_DONE = 3'd5;

  localparam [1:0] UART_IDLE = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA = 2'd2;
  localparam [1:0] UART_STOP = 2'd3;

  assign flash_wp_n = 1'b1;
  assign flash_hold_n = 1'b1;

  function [7:0] hex_char;
    input [3:0] value;
    begin
      hex_char = value < 10 ? ("0" + value) : ("A" + value - 10);
    end
  endfunction

  function [4:0] test_byte_count;
    input [3:0] test_index;
    begin
      case (test_index)
        4'd0, 4'd2, 4'd5, 4'd10, 4'd11: test_byte_count = 5'd4;
        4'd1: test_byte_count = 5'd5;
        4'd3, 4'd4: test_byte_count = 5'd1;
        4'd6, 4'd7, 4'd8: test_byte_count = 5'd2;
        default: test_byte_count = 5'd13;
      endcase
    end
  endfunction

  function [7:0] test_tx_byte;
    input [3:0] test_index;
    input [4:0] byte_index;
    begin
      test_tx_byte = 8'hFF;
      case (test_index)
        4'd0, 4'd2, 4'd5, 4'd10, 4'd11:
          if (byte_index == 0) test_tx_byte = 8'h9F;
        4'd1:
          if (byte_index == 0) test_tx_byte = 8'hAB;
        4'd3: test_tx_byte = 8'h66;
        4'd4: test_tx_byte = 8'h99;
        4'd6:
          if (byte_index == 0) test_tx_byte = 8'h05;
        4'd7:
          if (byte_index == 0) test_tx_byte = 8'h35;
        4'd8:
          if (byte_index == 0) test_tx_byte = 8'h15;
        default: begin
          if (byte_index == 0) test_tx_byte = 8'h5A;
          else if (byte_index <= 4) test_tx_byte = 8'h00;
        end
      endcase
    end
  endfunction

  reg spi_start;
  reg spi_cpol;
  reg spi_fast;
  reg [7:0] spi_tx_data;
  reg [7:0] spi_rx_data;
  reg spi_busy;
  reg spi_done;
  reg [15:0] spi_div_count;
  reg [2:0] spi_bit_index;
  reg spi_last_sampled;
  reg [15:0] miso_low_samples;
  wire [15:0] spi_half_divider = spi_fast ? 16'd4 : 16'd49;

  // Read-only, byte-wide SPI master. W25Q parts accept mode 0 and mode 3;
  // both sample on a rising edge, while the idle clock level differs.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      flash_clk <= 1'b0;
      flash_mosi <= 1'b0;
      spi_rx_data <= 8'd0;
      spi_busy <= 1'b0;
      spi_done <= 1'b0;
      spi_div_count <= 16'd0;
      spi_bit_index <= 3'd7;
      spi_last_sampled <= 1'b0;
      miso_low_samples <= 16'd0;
    end else begin
      spi_done <= 1'b0;
      if (!spi_busy) begin
        flash_clk <= spi_cpol;
        spi_div_count <= 16'd0;
        if (spi_start) begin
          spi_busy <= 1'b1;
          spi_bit_index <= 3'd7;
          spi_last_sampled <= 1'b0;
          spi_rx_data <= 8'd0;
          flash_mosi <= spi_tx_data[7];
        end
      end else if (spi_div_count >= spi_half_divider) begin
        spi_div_count <= 16'd0;
        if (!flash_clk) begin
          flash_clk <= 1'b1;
          spi_rx_data[spi_bit_index] <= flash_miso;
          if (!flash_miso) miso_low_samples <= miso_low_samples + 16'd1;
          if (spi_bit_index == 0) begin
            spi_last_sampled <= 1'b1;
          end else begin
            spi_bit_index <= spi_bit_index - 3'd1;
          end
        end else if (spi_last_sampled) begin
          flash_clk <= spi_cpol;
          flash_mosi <= 1'b0;
          spi_busy <= 1'b0;
          spi_done <= 1'b1;
          spi_last_sampled <= 1'b0;
        end else begin
          flash_clk <= 1'b0;
          flash_mosi <= spi_tx_data[spi_bit_index];
        end
      end else begin
        spi_div_count <= spi_div_count + 16'd1;
      end
    end
  end

  reg [2:0] probe_state;
  reg [3:0] probe_test_index;
  reg [4:0] probe_byte_index;
  reg [31:0] probe_wait_count;
  reg probe_complete;
  reg [23:0] id_mode0_initial;
  reg [7:0] release_device_id;
  reg [23:0] id_after_release;
  reg [23:0] id_after_reset;
  reg [7:0] status_reg1;
  reg [7:0] status_reg2;
  reg [7:0] status_reg3;
  reg [63:0] sfdp_signature;
  reg [23:0] id_mode3_slow;
  reg [23:0] id_mode0_fast;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      flash_cs_n <= 1'b1;
      spi_start <= 1'b0;
      spi_cpol <= 1'b0;
      spi_fast <= 1'b0;
      spi_tx_data <= 8'd0;
      probe_state <= PROBE_WAIT;
      probe_test_index <= 4'd0;
      probe_byte_index <= 5'd0;
      probe_wait_count <= 32'd0;
      probe_complete <= 1'b0;
      id_mode0_initial <= 24'd0;
      release_device_id <= 8'd0;
      id_after_release <= 24'd0;
      id_after_reset <= 24'd0;
      status_reg1 <= 8'd0;
      status_reg2 <= 8'd0;
      status_reg3 <= 8'd0;
      sfdp_signature <= 64'd0;
      id_mode3_slow <= 24'd0;
      id_mode0_fast <= 24'd0;
    end else begin
      spi_start <= 1'b0;
      case (probe_state)
        PROBE_WAIT: begin
          flash_cs_n <= 1'b1;
          if (probe_wait_count >= STARTUP_WAIT_CYCLES - 1) begin
            probe_wait_count <= 32'd0;
            probe_state <= PROBE_TEST_START;
          end else probe_wait_count <= probe_wait_count + 32'd1;
        end

        PROBE_TEST_START: begin
          spi_cpol <= (probe_test_index == 4'd10);
          spi_fast <= (probe_test_index == 4'd11);
          probe_byte_index <= 5'd0;
          flash_cs_n <= 1'b0;
          probe_state <= PROBE_BYTE_START;
        end

        PROBE_BYTE_START: begin
          if (!spi_busy) begin
            spi_tx_data <= test_tx_byte(probe_test_index, probe_byte_index);
            spi_start <= 1'b1;
            probe_state <= PROBE_BYTE_WAIT;
          end
        end

        PROBE_BYTE_WAIT: begin
          if (spi_done) begin
            case (probe_test_index)
              4'd0: begin
                if (probe_byte_index == 1) id_mode0_initial[23:16] <= spi_rx_data;
                if (probe_byte_index == 2) id_mode0_initial[15:8] <= spi_rx_data;
                if (probe_byte_index == 3) id_mode0_initial[7:0] <= spi_rx_data;
              end
              4'd1: if (probe_byte_index == 4) release_device_id <= spi_rx_data;
              4'd2: begin
                if (probe_byte_index == 1) id_after_release[23:16] <= spi_rx_data;
                if (probe_byte_index == 2) id_after_release[15:8] <= spi_rx_data;
                if (probe_byte_index == 3) id_after_release[7:0] <= spi_rx_data;
              end
              4'd5: begin
                if (probe_byte_index == 1) id_after_reset[23:16] <= spi_rx_data;
                if (probe_byte_index == 2) id_after_reset[15:8] <= spi_rx_data;
                if (probe_byte_index == 3) id_after_reset[7:0] <= spi_rx_data;
              end
              4'd6: if (probe_byte_index == 1) status_reg1 <= spi_rx_data;
              4'd7: if (probe_byte_index == 1) status_reg2 <= spi_rx_data;
              4'd8: if (probe_byte_index == 1) status_reg3 <= spi_rx_data;
              4'd9: if (probe_byte_index >= 5)
                sfdp_signature <= {sfdp_signature[55:0], spi_rx_data};
              4'd10: begin
                if (probe_byte_index == 1) id_mode3_slow[23:16] <= spi_rx_data;
                if (probe_byte_index == 2) id_mode3_slow[15:8] <= spi_rx_data;
                if (probe_byte_index == 3) id_mode3_slow[7:0] <= spi_rx_data;
              end
              4'd11: begin
                if (probe_byte_index == 1) id_mode0_fast[23:16] <= spi_rx_data;
                if (probe_byte_index == 2) id_mode0_fast[15:8] <= spi_rx_data;
                if (probe_byte_index == 3) id_mode0_fast[7:0] <= spi_rx_data;
              end
              default: begin end
            endcase

            if (probe_byte_index + 1 >= test_byte_count(probe_test_index)) begin
              flash_cs_n <= 1'b1;
              probe_wait_count <= 32'd0;
              probe_state <= PROBE_TEST_GAP;
            end else begin
              probe_byte_index <= probe_byte_index + 5'd1;
              probe_state <= PROBE_BYTE_START;
            end
          end
        end

        PROBE_TEST_GAP: begin
          flash_cs_n <= 1'b1;
          if (probe_wait_count >=
              ((probe_test_index == 4'd4) ? RESET_GAP_CYCLES : TEST_GAP_CYCLES) - 1) begin
            probe_wait_count <= 32'd0;
            if (probe_test_index == 4'd11) begin
              probe_complete <= 1'b1;
              probe_state <= PROBE_DONE;
            end else begin
              probe_test_index <= probe_test_index + 4'd1;
              probe_state <= PROBE_TEST_START;
            end
          end else probe_wait_count <= probe_wait_count + 32'd1;
        end

        default: begin
          flash_cs_n <= 1'b1;
          probe_complete <= 1'b1;
          probe_state <= PROBE_DONE;
        end
      endcase
    end
  end

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

  function [4:0] report_last_index;
    input [2:0] line;
    begin
      case (line)
        0: report_last_index = 5'd10;
        1: report_last_index = 5'd13;
        2: report_last_index = 5'd19;
        3: report_last_index = 5'd20;
        default: report_last_index = 5'd22;
      endcase
    end
  endfunction

  function [7:0] report_char;
    input [2:0] line;
    input [4:0] index;
    reg [23:0] selected_id;
    reg [7:0] selected_status;
    begin
      report_char = "?";
      case (line)
        0: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "F";
          else if (index == 2) report_char = "0";
          else if (index == 3) report_char = ",";
          else if (index <= 9)
            report_char = hex_char(id_mode0_initial[(9-index)*4 +: 4]);
          else report_char = 8'h0A;
        end
        1: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "F";
          else if (index == 2) report_char = "A";
          else if (index == 3) report_char = ",";
          else if (index <= 5)
            report_char = hex_char(release_device_id[(5-index)*4 +: 4]);
          else if (index == 6) report_char = ",";
          else if (index <= 12)
            report_char = hex_char(id_after_release[(12-index)*4 +: 4]);
          else report_char = 8'h0A;
        end
        2: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "F";
          else if (index == 2) report_char = "R";
          else if (index == 3) report_char = ",";
          else if (index <= 9)
            report_char = hex_char(id_after_reset[(9-index)*4 +: 4]);
          else if ((index == 10) || (index == 13) || (index == 16))
            report_char = ",";
          else if (index <= 12) begin
            selected_status = status_reg1;
            report_char = hex_char(selected_status[(12-index)*4 +: 4]);
          end else if (index <= 15) begin
            selected_status = status_reg2;
            report_char = hex_char(selected_status[(15-index)*4 +: 4]);
          end else if (index <= 18) begin
            selected_status = status_reg3;
            report_char = hex_char(selected_status[(18-index)*4 +: 4]);
          end else report_char = 8'h0A;
        end
        3: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "F";
          else if (index == 2) report_char = "S";
          else if (index == 3) report_char = ",";
          else if (index <= 19)
            report_char = hex_char(sfdp_signature[(19-index)*4 +: 4]);
          else report_char = 8'h0A;
        end
        default: begin
          if (index == 0) report_char = "@";
          else if (index == 1) report_char = "F";
          else if (index == 2) report_char = "M";
          else if (index == 3) report_char = ",";
          else if (index <= 9) begin
            selected_id = id_mode3_slow;
            report_char = hex_char(selected_id[(9-index)*4 +: 4]);
          end else if (index == 10) report_char = ",";
          else if (index <= 16) begin
            selected_id = id_mode0_fast;
            report_char = hex_char(selected_id[(16-index)*4 +: 4]);
          end else if (index == 17) report_char = ",";
          else if (index <= 21)
            report_char = hex_char(miso_low_samples[(21-index)*4 +: 4]);
          else report_char = 8'h0A;
        end
      endcase
    end
  endfunction

  reg report_active;
  reg [2:0] report_line;
  reg [4:0] report_index;
  reg [31:0] report_gap_count;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uart_start <= 1'b0;
      uart_tx_data <= 8'd0;
      report_active <= 1'b0;
      report_line <= 3'd0;
      report_index <= 5'd0;
      report_gap_count <= 32'd0;
    end else begin
      uart_start <= 1'b0;
      if (!probe_complete) begin
        report_gap_count <= 32'd0;
      end else if (!report_active) begin
        if (report_gap_count >= REPORT_GAP_CYCLES - 1) begin
          report_gap_count <= 32'd0;
          report_active <= 1'b1;
          report_line <= 3'd0;
          report_index <= 5'd0;
          uart_tx_data <= report_char(3'd0, 5'd0);
          uart_start <= 1'b1;
        end else report_gap_count <= report_gap_count + 32'd1;
      end else if (uart_done) begin
        if (report_index == report_last_index(report_line)) begin
          if (report_line == 3'd4) begin
            report_active <= 1'b0;
            report_gap_count <= 32'd0;
          end else begin
            report_line <= report_line + 3'd1;
            report_index <= 5'd0;
            uart_tx_data <= report_char(report_line + 3'd1, 5'd0);
            uart_start <= 1'b1;
          end
        end else begin
          report_index <= report_index + 5'd1;
          uart_tx_data <= report_char(report_line, report_index + 5'd1);
          uart_start <= 1'b1;
        end
      end
    end
  end

endmodule
