module map_flash_store (
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

  output reg map_mem_req,
  output reg map_mem_we,
  output reg [13:0] map_mem_addr,
  output reg signed [3:0] map_mem_wdata,
  input wire signed [3:0] map_mem_rdata,
  input wire map_mem_rvalid,

  output reg load_apply_pulse,
  output reg [7:0] loaded_map_epoch,
  output reg [15:0] loaded_pose_x_mm,
  output reg [15:0] loaded_pose_y_mm,
  output reg [9:0] loaded_pose_heading,
  output reg [15:0] loaded_scan_count,
  output reg [15:0] loaded_occupied_count,
  output reg [15:0] loaded_free_count,

  output reg flash_cs_n,
  output wire flash_clk,
  output reg flash_mosi,
  output wire flash_wp_n,
  output wire flash_hold_n,
  input wire flash_miso,

  output reg flash_ready,
  output reg flash_busy,
  output reg [1:0] flash_operation,
  output reg [7:0] flash_progress,
  output reg [7:0] flash_error,
  output reg [23:0] flash_jedec_id,
  output reg [31:0] saved_sequence
);

  parameter integer CLK_HZ = 50000000;
  parameter integer SPI_DIV = 5;
  parameter integer PAYLOAD_BYTE_COUNT = 8192;

  localparam [23:0] SLOT_A_BASE = 24'hFE0000;
  localparam [23:0] SLOT_B_BASE = 24'hFF0000;
  localparam [23:0] PAYLOAD_OFFSET = 24'h000100;
  localparam [13:0] PAYLOAD_BYTES = PAYLOAD_BYTE_COUNT;
  localparam [7:0] HEADER_BYTES = 8'd32;
  localparam [7:0] VALID_MARKER = 8'hA5;

  localparam [7:0] ST_BOOT_WAIT = 8'd0;
  localparam [7:0] ST_JEDEC = 8'd1;
  localparam [7:0] ST_JEDEC_EVAL = 8'd2;
  localparam [7:0] ST_HEADER_CMD = 8'd3;
  localparam [7:0] ST_HEADER_DATA = 8'd4;
  localparam [7:0] ST_HEADER_EVAL = 8'd5;
  localparam [7:0] ST_IDLE = 8'd6;
  localparam [7:0] ST_WREN = 8'd7;
  localparam [7:0] ST_ERASE_CMD = 8'd8;
  localparam [7:0] ST_POLL_CMD = 8'd9;
  localparam [7:0] ST_POLL_DATA = 8'd10;
  localparam [7:0] ST_POLL_DELAY = 8'd11;
  localparam [7:0] ST_SAVE_PAGE_CMD = 8'd12;
  localparam [7:0] ST_SAVE_READ_LOW = 8'd13;
  localparam [7:0] ST_SAVE_READ_HIGH = 8'd14;
  localparam [7:0] ST_SAVE_DATA = 8'd15;
  localparam [7:0] ST_SAVE_HEADER_CMD = 8'd16;
  localparam [7:0] ST_SAVE_HEADER_DATA = 8'd17;
  localparam [7:0] ST_SAVE_COMMIT_CMD = 8'd18;
  localparam [7:0] ST_SAVE_COMMIT_DATA = 8'd19;
  localparam [7:0] ST_SAVE_DONE = 8'd20;
  localparam [7:0] ST_LOAD_VERIFY_CMD = 8'd21;
  localparam [7:0] ST_LOAD_VERIFY_DATA = 8'd22;
  localparam [7:0] ST_LOAD_VERIFY_EVAL = 8'd23;
  localparam [7:0] ST_LOAD_READ_CMD = 8'd24;
  localparam [7:0] ST_LOAD_READ_DATA = 8'd25;
  localparam [7:0] ST_LOAD_WRITE_LOW = 8'd26;
  localparam [7:0] ST_LOAD_WRITE_HIGH = 8'd27;
  localparam [7:0] ST_LOAD_DONE = 8'd28;
  localparam [7:0] ST_FAIL = 8'd29;
  localparam [7:0] ST_BUS_WARMUP = 8'd30;
  localparam [7:0] ST_XIP_START = 8'd31;
  localparam [7:0] ST_XIP_CLOCK = 8'd32;
  localparam [7:0] ST_XIP_GAP = 8'd33;
  localparam [7:0] ST_RESET_ENABLE = 8'd34;
  localparam [7:0] ST_RESET_ENABLE_GAP = 8'd35;
  localparam [7:0] ST_RESET_MEMORY = 8'd36;
  localparam [7:0] ST_RESET_WAIT = 8'd37;

  localparam integer RECOVERY_GAP_CYCLES =
      ((CLK_HZ / 1000000) < 2) ? 2 : (CLK_HZ / 1000000);
  localparam integer RESET_WAIT_CYCLES =
      ((CLK_HZ / 10000) < 2) ? 2 : (CLK_HZ / 10000);

  localparam [7:0] ERR_NONE = 8'h00;
  localparam [7:0] ERR_JEDEC = 8'h01;
  localparam [7:0] ERR_NOT_READY = 8'h02;
  localparam [7:0] ERR_ADDRESS_GUARD = 8'h03;
  localparam [7:0] ERR_NO_VALID_MAP = 8'h04;
  localparam [7:0] ERR_MAP_CRC = 8'h05;
  localparam [7:0] ERR_FLASH_TIMEOUT = 8'h06;

  reg [7:0] state;
  reg [7:0] return_state;
  reg [7:0] command_index;
  reg byte_started;
  reg [31:0] boot_wait_count;
  reg [31:0] poll_delay_count;
  reg [15:0] poll_attempt_count;

  reg spi_start;
  reg spi_raw_start;
  reg [5:0] spi_raw_cycles;
  reg [7:0] spi_tx_data;
  wire [7:0] spi_rx_data;
  wire spi_busy;
  wire spi_done;
  wire spi_clock;

  reg header_slot;
  reg [7:0] header_index;
  reg [7:0] header_a [0:31];
  reg [7:0] header_b [0:31];

  reg target_slot;
  reg [23:0] target_base;
  reg [31:0] pending_sequence;
  reg [15:0] save_crc;
  reg [13:0] payload_index;
  reg [7:0] page_byte_index;
  reg signed [3:0] saved_low_nibble;
  reg [7:0] packed_map_byte;
  reg [7:0] save_header_index;

  reg load_slot;
  reg [23:0] load_base;
  reg [15:0] load_crc;
  reg [15:0] expected_load_crc;
  reg [7:0] load_byte;
  reg [2:0] xip_sequence_index;

  wire slot_a_valid =
      (header_a[0] == "L") && (header_a[1] == "C") &&
      (header_a[2] == "M") && (header_a[3] == "1") &&
      (header_a[4] == 8'd1) && (header_a[5] == 8'd128) &&
      (header_a[6] == 8'd128) && (header_a[7] == 8'd50) &&
      ({header_a[24], header_a[25]} == PAYLOAD_BYTE_COUNT) &&
      (header_a[31] == VALID_MARKER);
  wire slot_b_valid =
      (header_b[0] == "L") && (header_b[1] == "C") &&
      (header_b[2] == "M") && (header_b[3] == "1") &&
      (header_b[4] == 8'd1) && (header_b[5] == 8'd128) &&
      (header_b[6] == 8'd128) && (header_b[7] == 8'd50) &&
      ({header_b[24], header_b[25]} == PAYLOAD_BYTE_COUNT) &&
      (header_b[31] == VALID_MARKER);
  wire [31:0] slot_a_sequence =
      {header_a[8], header_a[9], header_a[10], header_a[11]};
  wire [31:0] slot_b_sequence =
      {header_b[8], header_b[9], header_b[10], header_b[11]};

  wire jedec_supported =
      (flash_jedec_id == 24'h20BA18) ||
      (flash_jedec_id == 24'hEF4018);
  wire target_base_safe =
      (target_base == SLOT_A_BASE) || (target_base == SLOT_B_BASE);
  wire load_base_safe =
      (load_base == SLOT_A_BASE) || (load_base == SLOT_B_BASE);
  wire [23:0] save_page_address =
      target_base + PAYLOAD_OFFSET + {10'd0, payload_index};
  wire [23:0] load_payload_address = load_base + PAYLOAD_OFFSET;

  function [15:0] crc16_next;
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
      crc16_next = value;
    end
  endfunction

  function [7:0] selected_header_byte;
    input slot;
    input [7:0] index;
    begin
      if (slot) selected_header_byte = header_b[index];
      else selected_header_byte = header_a[index];
    end
  endfunction

  function [7:0] save_header_byte;
    input [7:0] index;
    begin
      case (index)
        8'd0: save_header_byte = "L";
        8'd1: save_header_byte = "C";
        8'd2: save_header_byte = "M";
        8'd3: save_header_byte = "1";
        8'd4: save_header_byte = 8'd1;
        8'd5: save_header_byte = 8'd128;
        8'd6: save_header_byte = 8'd128;
        8'd7: save_header_byte = 8'd50;
        8'd8: save_header_byte = pending_sequence[31:24];
        8'd9: save_header_byte = pending_sequence[23:16];
        8'd10: save_header_byte = pending_sequence[15:8];
        8'd11: save_header_byte = pending_sequence[7:0];
        8'd12: save_header_byte = pose_x_mm[15:8];
        8'd13: save_header_byte = pose_x_mm[7:0];
        8'd14: save_header_byte = pose_y_mm[15:8];
        8'd15: save_header_byte = pose_y_mm[7:0];
        8'd16: save_header_byte = {6'd0, pose_heading[9:8]};
        8'd17: save_header_byte = pose_heading[7:0];
        8'd18: save_header_byte = scan_count[15:8];
        8'd19: save_header_byte = scan_count[7:0];
        8'd20: save_header_byte = occupied_count[15:8];
        8'd21: save_header_byte = occupied_count[7:0];
        8'd22: save_header_byte = free_count[15:8];
        8'd23: save_header_byte = free_count[7:0];
        8'd24: save_header_byte = PAYLOAD_BYTE_COUNT[15:8];
        8'd25: save_header_byte = PAYLOAD_BYTE_COUNT[7:0];
        8'd26: save_header_byte = save_crc[15:8];
        8'd27: save_header_byte = save_crc[7:0];
        8'd28: save_header_byte = map_epoch;
        8'd29: save_header_byte = 8'h00;
        default: save_header_byte = 8'hFF;
      endcase
    end
  endfunction

  function [5:0] xip_recovery_cycles;
    input [2:0] index;
    begin
      case (index)
        3'd0: xip_recovery_cycles = 6'd7;
        3'd1: xip_recovery_cycles = 6'd9;
        3'd2: xip_recovery_cycles = 6'd13;
        3'd3: xip_recovery_cycles = 6'd17;
        3'd4: xip_recovery_cycles = 6'd25;
        3'd5: xip_recovery_cycles = 6'd33;
        // An all-ones byte also exits a possible four-line QIO command mode.
        default: xip_recovery_cycles = 6'd8;
      endcase
    end
  endfunction

  // Byte-wide mode-0 SPI engine. It deliberately runs at 5 MHz by default,
  // far below the supported 128-Mbit Flash limits and with ample setup margin
  // for the ordinary D16 GPIO clock route.
  reg [15:0] spi_div_count;
  reg [2:0] spi_bit_index;
  reg [7:0] spi_rx_shift;
  reg spi_engine_busy;
  reg spi_engine_done;
  reg spi_clock_reg;
  reg spi_engine_raw;
  reg [5:0] spi_raw_remaining;

  assign spi_busy = spi_engine_busy;
  assign spi_done = spi_engine_done;
  assign spi_rx_data = spi_rx_shift;
  assign spi_clock = spi_clock_reg;
  // The A7-50T resource table routes the user-accessible W25Q128 clock to
  // package pin D16. D16 is an ordinary Bank 16 user I/O, not the dedicated
  // configuration CCLK pin, so the SPI clock must leave through a normal top
  // level output. Driving STARTUPE2 here would toggle CCLK_0 (package L12) and
  // leave the actual W25Q128 clock input static.
  assign flash_clk = spi_clock_reg;
  // DQ2 and DQ3 are active-low WP#/HOLD# in single-SPI mode. They are
  // configuration pins, so explicitly keep them inactive after startup rather
  // than relying on board pull-ups or the unused-pin bitstream policy.
  assign flash_wp_n = 1'b1;
  assign flash_hold_n = 1'b1;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      spi_div_count <= 16'd0;
      spi_bit_index <= 3'd7;
      spi_rx_shift <= 8'd0;
      spi_engine_busy <= 1'b0;
      spi_engine_done <= 1'b0;
      spi_clock_reg <= 1'b0;
      spi_engine_raw <= 1'b0;
      spi_raw_remaining <= 6'd0;
      flash_mosi <= 1'b0;
    end else begin
      spi_engine_done <= 1'b0;
      if (!spi_engine_busy) begin
        spi_clock_reg <= 1'b0;
        spi_div_count <= 16'd0;
        if (spi_raw_start) begin
          spi_engine_busy <= 1'b1;
          spi_engine_raw <= 1'b1;
          spi_raw_remaining <= spi_raw_cycles;
          flash_mosi <= 1'b1;
        end else if (spi_start) begin
          spi_engine_busy <= 1'b1;
          spi_engine_raw <= 1'b0;
          spi_bit_index <= 3'd7;
          spi_rx_shift <= 8'd0;
          flash_mosi <= spi_tx_data[7];
        end
      end else if (spi_div_count >= SPI_DIV - 1) begin
        spi_div_count <= 16'd0;
        if (!spi_clock_reg) begin
          spi_clock_reg <= 1'b1;
          if (!spi_engine_raw)
            spi_rx_shift[spi_bit_index] <= flash_miso;
        end else begin
          spi_clock_reg <= 1'b0;
          if (spi_engine_raw) begin
            if (spi_raw_remaining <= 6'd1) begin
              spi_engine_busy <= 1'b0;
              spi_engine_done <= 1'b1;
              spi_engine_raw <= 1'b0;
              spi_raw_remaining <= 6'd0;
              flash_mosi <= 1'b0;
            end else begin
              spi_raw_remaining <= spi_raw_remaining - 6'd1;
            end
          end else if (spi_bit_index == 3'd0) begin
            spi_engine_busy <= 1'b0;
            spi_engine_done <= 1'b1;
            flash_mosi <= 1'b0;
          end else begin
            spi_bit_index <= spi_bit_index - 3'd1;
            flash_mosi <= spi_tx_data[spi_bit_index - 3'd1];
          end
        end
      end else begin
        spi_div_count <= spi_div_count + 16'd1;
      end
    end
  end

  integer init_index;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_BOOT_WAIT;
      return_state <= ST_IDLE;
      command_index <= 8'd0;
      byte_started <= 1'b0;
      boot_wait_count <= 32'd0;
      poll_delay_count <= 32'd0;
      poll_attempt_count <= 16'd0;
      spi_start <= 1'b0;
      spi_raw_start <= 1'b0;
      spi_raw_cycles <= 6'd0;
      spi_tx_data <= 8'd0;
      flash_cs_n <= 1'b1;
      header_slot <= 1'b0;
      header_index <= 8'd0;
      target_slot <= 1'b0;
      target_base <= SLOT_A_BASE;
      pending_sequence <= 32'd1;
      save_crc <= 16'hFFFF;
      payload_index <= 14'd0;
      page_byte_index <= 8'd0;
      saved_low_nibble <= 4'sd0;
      packed_map_byte <= 8'd0;
      save_header_index <= 8'd0;
      load_slot <= 1'b0;
      load_base <= SLOT_A_BASE;
      load_crc <= 16'hFFFF;
      expected_load_crc <= 16'd0;
      load_byte <= 8'd0;
      xip_sequence_index <= 3'd0;
      map_mem_req <= 1'b0;
      map_mem_we <= 1'b0;
      map_mem_addr <= 14'd0;
      map_mem_wdata <= 4'sd0;
      load_apply_pulse <= 1'b0;
      loaded_map_epoch <= 8'd0;
      loaded_pose_x_mm <= 16'd0;
      loaded_pose_y_mm <= 16'd0;
      loaded_pose_heading <= 10'd0;
      loaded_scan_count <= 16'd0;
      loaded_occupied_count <= 16'd0;
      loaded_free_count <= 16'd0;
      flash_ready <= 1'b0;
      flash_busy <= 1'b1;
      flash_operation <= 2'd1;
      flash_progress <= 8'd0;
      flash_error <= ERR_NONE;
      flash_jedec_id <= 24'd0;
      saved_sequence <= 32'd0;
      for (init_index = 0; init_index < 32; init_index = init_index + 1) begin
        header_a[init_index] <= 8'hFF;
        header_b[init_index] <= 8'hFF;
      end
    end else begin
      spi_start <= 1'b0;
      spi_raw_start <= 1'b0;
      map_mem_req <= 1'b0;
      map_mem_we <= 1'b0;
      load_apply_pulse <= 1'b0;

      case (state)
        ST_BOOT_WAIT: begin
          flash_cs_n <= 1'b1;
          if (boot_wait_count >= CLK_HZ / 1000) begin
            boot_wait_count <= 32'd0;
            // Toggle one byte with CS high before issuing recovery commands.
            // This establishes the mode-0 idle level on the ordinary D16 GPIO
            // and gives the external Flash bus a deterministic settling period
            // after the FPGA reset has been released.
            spi_tx_data <= 8'h00;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
            state <= ST_BUS_WARMUP;
          end else begin
            boot_wait_count <= boot_wait_count + 32'd1;
          end
        end

        ST_BUS_WARMUP: begin
          flash_cs_n <= 1'b1;
          if (spi_done) begin
            byte_started <= 1'b0;
            xip_sequence_index <= 3'd0;
            state <= ST_XIP_START;
          end
        end

        // Micron documents this exact sequence for recovering when the
        // controller resets while the memory remains in an unknown XIP bit
        // alignment. MOSI stays high so the XIP confirmation bit is one.
        ST_XIP_START: begin
          if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            spi_raw_cycles <= xip_recovery_cycles(xip_sequence_index);
            spi_raw_start <= 1'b1;
            byte_started <= 1'b1;
            state <= ST_XIP_CLOCK;
          end
        end

        ST_XIP_CLOCK: begin
          if (spi_done) begin
            flash_cs_n <= 1'b1;
            byte_started <= 1'b0;
            boot_wait_count <= 32'd0;
            state <= ST_XIP_GAP;
          end
        end

        ST_XIP_GAP: begin
          flash_cs_n <= 1'b1;
          if (boot_wait_count >= RECOVERY_GAP_CYCLES - 1) begin
            boot_wait_count <= 32'd0;
            if (xip_sequence_index == 3'd6) begin
              state <= ST_RESET_ENABLE;
            end else begin
              xip_sequence_index <= xip_sequence_index + 3'd1;
              state <= ST_XIP_START;
            end
          end else begin
            boot_wait_count <= boot_wait_count + 32'd1;
          end
        end

        ST_RESET_ENABLE: begin
          if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            spi_tx_data <= 8'h66;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            flash_cs_n <= 1'b1;
            byte_started <= 1'b0;
            boot_wait_count <= 32'd0;
            state <= ST_RESET_ENABLE_GAP;
          end
        end

        ST_RESET_ENABLE_GAP: begin
          flash_cs_n <= 1'b1;
          if (boot_wait_count >= RECOVERY_GAP_CYCLES - 1) begin
            boot_wait_count <= 32'd0;
            state <= ST_RESET_MEMORY;
          end else begin
            boot_wait_count <= boot_wait_count + 32'd1;
          end
        end

        ST_RESET_MEMORY: begin
          if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            spi_tx_data <= 8'h99;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            flash_cs_n <= 1'b1;
            byte_started <= 1'b0;
            boot_wait_count <= 32'd0;
            state <= ST_RESET_WAIT;
          end
        end

        ST_RESET_WAIT: begin
          flash_cs_n <= 1'b1;
          if (boot_wait_count >= RESET_WAIT_CYCLES - 1) begin
            boot_wait_count <= 32'd0;
            command_index <= 8'd0;
            flash_cs_n <= 1'b0;
            state <= ST_JEDEC;
          end else begin
            boot_wait_count <= boot_wait_count + 32'd1;
          end
        end

        ST_JEDEC: begin
          if (!byte_started && !spi_busy) begin
            case (command_index)
              8'd0: spi_tx_data <= 8'h9F;
              default: spi_tx_data <= 8'hFF;
            endcase
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (command_index == 8'd1) flash_jedec_id[23:16] <= spi_rx_data;
            if (command_index == 8'd2) flash_jedec_id[15:8] <= spi_rx_data;
            if (command_index == 8'd3) flash_jedec_id[7:0] <= spi_rx_data;
            if (command_index == 8'd3) begin
              flash_cs_n <= 1'b1;
              state <= ST_JEDEC_EVAL;
            end else begin
              command_index <= command_index + 8'd1;
            end
          end
        end

        ST_JEDEC_EVAL: begin
          if (!jedec_supported) begin
            flash_error <= ERR_JEDEC;
            flash_ready <= 1'b0;
            flash_busy <= 1'b0;
            flash_operation <= 2'd0;
            state <= ST_IDLE;
          end else begin
            header_slot <= 1'b0;
            header_index <= 8'd0;
            command_index <= 8'd0;
            flash_cs_n <= 1'b0;
            state <= ST_HEADER_CMD;
          end
        end

        ST_HEADER_CMD: begin
          if (!byte_started && !spi_busy) begin
            case (command_index)
              8'd0: spi_tx_data <= 8'h03;
              8'd1: spi_tx_data <= header_slot ? SLOT_B_BASE[23:16] : SLOT_A_BASE[23:16];
              8'd2: spi_tx_data <= 8'h00;
              default: spi_tx_data <= 8'h00;
            endcase
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (command_index == 8'd3) begin
              header_index <= 8'd0;
              state <= ST_HEADER_DATA;
            end else begin
              command_index <= command_index + 8'd1;
            end
          end
        end

        ST_HEADER_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= 8'hFF;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (header_slot) header_b[header_index] <= spi_rx_data;
            else header_a[header_index] <= spi_rx_data;
            if (header_index == HEADER_BYTES - 1) begin
              flash_cs_n <= 1'b1;
              if (!header_slot) begin
                header_slot <= 1'b1;
                command_index <= 8'd0;
                flash_cs_n <= 1'b0;
                state <= ST_HEADER_CMD;
              end else begin
                state <= ST_HEADER_EVAL;
              end
            end else begin
              header_index <= header_index + 8'd1;
            end
          end
        end

        ST_HEADER_EVAL: begin
          if (slot_a_valid && (!slot_b_valid || (slot_a_sequence >= slot_b_sequence)))
            saved_sequence <= slot_a_sequence;
          else if (slot_b_valid)
            saved_sequence <= slot_b_sequence;
          else
            saved_sequence <= 32'd0;
          flash_ready <= 1'b1;
          flash_busy <= 1'b0;
          flash_operation <= 2'd0;
          flash_progress <= 8'd0;
          flash_error <= ERR_NONE;
          state <= ST_IDLE;
        end

        ST_IDLE: begin
          flash_cs_n <= 1'b1;
          byte_started <= 1'b0;
          flash_busy <= 1'b0;
          flash_operation <= 2'd0;
          flash_progress <= 8'd0;
          if (save_pulse) begin
            if (!flash_ready) begin
              flash_error <= ERR_NOT_READY;
            end else begin
              flash_busy <= 1'b1;
              flash_operation <= 2'd2;
              flash_error <= ERR_NONE;
              save_crc <= 16'hFFFF;
              payload_index <= 14'd0;
              page_byte_index <= 8'd0;
              pending_sequence <= saved_sequence + 32'd1;
              if (!slot_a_valid ||
                  (slot_b_valid && (slot_a_sequence <= slot_b_sequence))) begin
                target_slot <= 1'b0;
                target_base <= SLOT_A_BASE;
              end else begin
                target_slot <= 1'b1;
                target_base <= SLOT_B_BASE;
              end
              return_state <= ST_ERASE_CMD;
              state <= ST_WREN;
            end
          end else if (load_pulse) begin
            if (!flash_ready) begin
              flash_error <= ERR_NOT_READY;
            end else if (!slot_a_valid && !slot_b_valid) begin
              flash_error <= ERR_NO_VALID_MAP;
            end else begin
              flash_busy <= 1'b1;
              flash_operation <= 2'd3;
              flash_error <= ERR_NONE;
              payload_index <= 14'd0;
              load_crc <= 16'hFFFF;
              command_index <= 8'd0;
              if (slot_a_valid && (!slot_b_valid || (slot_a_sequence >= slot_b_sequence))) begin
                load_slot <= 1'b0;
                load_base <= SLOT_A_BASE;
                expected_load_crc <= {header_a[26], header_a[27]};
              end else begin
                load_slot <= 1'b1;
                load_base <= SLOT_B_BASE;
                expected_load_crc <= {header_b[26], header_b[27]};
              end
              flash_cs_n <= 1'b0;
              state <= ST_LOAD_VERIFY_CMD;
            end
          end
        end

        ST_WREN: begin
          if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            spi_tx_data <= 8'h06;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            flash_cs_n <= 1'b1;
            command_index <= 8'd0;
            state <= return_state;
          end
        end

        ST_ERASE_CMD: begin
          if (!target_base_safe) begin
            flash_error <= ERR_ADDRESS_GUARD;
            state <= ST_FAIL;
          end else begin
            if (!byte_started && !spi_busy) begin
              flash_cs_n <= 1'b0;
              case (command_index)
                8'd0: spi_tx_data <= 8'hD8;
                8'd1: spi_tx_data <= target_base[23:16];
                8'd2: spi_tx_data <= target_base[15:8];
                default: spi_tx_data <= target_base[7:0];
              endcase
              spi_start <= 1'b1;
              byte_started <= 1'b1;
            end
            if (spi_done) begin
              byte_started <= 1'b0;
              if (command_index == 8'd3) begin
                flash_cs_n <= 1'b1;
                return_state <= ST_SAVE_PAGE_CMD;
                poll_attempt_count <= 16'd0;
                poll_delay_count <= 32'd0;
                state <= ST_POLL_DELAY;
              end else begin
                command_index <= command_index + 8'd1;
              end
            end
          end
        end

        ST_POLL_DELAY: begin
          flash_cs_n <= 1'b1;
          if (poll_delay_count >= CLK_HZ / 20000) begin
            poll_delay_count <= 32'd0;
            command_index <= 8'd0;
            flash_cs_n <= 1'b0;
            state <= ST_POLL_CMD;
          end else begin
            poll_delay_count <= poll_delay_count + 32'd1;
          end
        end

        ST_POLL_CMD: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= 8'h05;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            state <= ST_POLL_DATA;
          end
        end

        ST_POLL_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= 8'hFF;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            flash_cs_n <= 1'b1;
            if (spi_rx_data[0]) begin
              if (poll_attempt_count == 16'hFFFF) begin
                flash_error <= ERR_FLASH_TIMEOUT;
                state <= ST_FAIL;
              end else begin
                poll_attempt_count <= poll_attempt_count + 16'd1;
                state <= ST_POLL_DELAY;
              end
            end else begin
              command_index <= 8'd0;
              if ((return_state == ST_SAVE_PAGE_CMD) ||
                  (return_state == ST_SAVE_HEADER_CMD)) begin
                state <= ST_WREN;
              end else begin
                state <= return_state;
              end
            end
          end
        end

        ST_SAVE_PAGE_CMD: begin
          if (!target_base_safe ||
              (save_page_address[23:16] != target_base[23:16])) begin
            flash_error <= ERR_ADDRESS_GUARD;
            state <= ST_FAIL;
          end else begin
            if (!byte_started && !spi_busy) begin
              flash_cs_n <= 1'b0;
              case (command_index)
                8'd0: spi_tx_data <= 8'h02;
                8'd1: spi_tx_data <= save_page_address[23:16];
                8'd2: spi_tx_data <= save_page_address[15:8];
                default: spi_tx_data <= save_page_address[7:0];
              endcase
              spi_start <= 1'b1;
              byte_started <= 1'b1;
            end
            if (spi_done) begin
              byte_started <= 1'b0;
              if (command_index == 8'd3) begin
                map_mem_addr <= {payload_index[12:0], 1'b0};
                map_mem_req <= 1'b1;
                map_mem_we <= 1'b0;
                state <= ST_SAVE_READ_LOW;
              end else begin
                command_index <= command_index + 8'd1;
              end
            end
          end
        end

        ST_SAVE_READ_LOW: begin
          if (map_mem_rvalid) begin
            saved_low_nibble <= map_mem_rdata;
            map_mem_addr <= {payload_index[12:0], 1'b1};
            map_mem_req <= 1'b1;
            map_mem_we <= 1'b0;
            state <= ST_SAVE_READ_HIGH;
          end
        end

        ST_SAVE_READ_HIGH: begin
          if (map_mem_rvalid) begin
            packed_map_byte <= {map_mem_rdata, saved_low_nibble};
            state <= ST_SAVE_DATA;
          end
        end

        ST_SAVE_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= packed_map_byte;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            save_crc <= crc16_next(save_crc, packed_map_byte);
            flash_progress <= payload_index[12:5];
            if ((page_byte_index == 8'hFF) ||
                (payload_index == PAYLOAD_BYTES - 1)) begin
              flash_cs_n <= 1'b1;
              page_byte_index <= 8'd0;
              if (payload_index == PAYLOAD_BYTES - 1) begin
                return_state <= ST_SAVE_HEADER_CMD;
              end else begin
                payload_index <= payload_index + 14'd1;
                return_state <= ST_SAVE_PAGE_CMD;
              end
              poll_attempt_count <= 16'd0;
              poll_delay_count <= 32'd0;
              state <= ST_POLL_DELAY;
            end else begin
              payload_index <= payload_index + 14'd1;
              page_byte_index <= page_byte_index + 8'd1;
              map_mem_addr <= (payload_index + 14'd1) << 1;
              map_mem_req <= 1'b1;
              map_mem_we <= 1'b0;
              state <= ST_SAVE_READ_LOW;
            end
          end
        end

        ST_SAVE_HEADER_CMD: begin
          if (!target_base_safe) begin
            flash_error <= ERR_ADDRESS_GUARD;
            state <= ST_FAIL;
          end else if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            case (command_index)
              8'd0: spi_tx_data <= 8'h02;
              8'd1: spi_tx_data <= target_base[23:16];
              8'd2: spi_tx_data <= target_base[15:8];
              default: spi_tx_data <= target_base[7:0];
            endcase
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (command_index == 8'd3) begin
              save_header_index <= 8'd0;
              state <= ST_SAVE_HEADER_DATA;
            end else begin
              command_index <= command_index + 8'd1;
            end
          end
        end

        ST_SAVE_HEADER_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= save_header_byte(save_header_index);
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (save_header_index == HEADER_BYTES - 1) begin
              flash_cs_n <= 1'b1;
              return_state <= ST_SAVE_COMMIT_CMD;
              poll_attempt_count <= 16'd0;
              poll_delay_count <= 32'd0;
              state <= ST_POLL_DELAY;
            end else begin
              save_header_index <= save_header_index + 8'd1;
            end
          end
        end

        ST_SAVE_COMMIT_CMD: begin
          // Poll returned here after the uncommitted header page. WREN is
          // required again before programming the final validity byte.
          return_state <= ST_SAVE_COMMIT_DATA;
          state <= ST_WREN;
        end

        ST_SAVE_COMMIT_DATA: begin
          if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            case (command_index)
              8'd0: spi_tx_data <= 8'h02;
              8'd1: spi_tx_data <= target_base[23:16];
              8'd2: spi_tx_data <= target_base[15:8];
              8'd3: spi_tx_data <= 8'h1F;
              default: spi_tx_data <= VALID_MARKER;
            endcase
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (command_index == 8'd4) begin
              flash_cs_n <= 1'b1;
              return_state <= ST_SAVE_DONE;
              poll_attempt_count <= 16'd0;
              poll_delay_count <= 32'd0;
              state <= ST_POLL_DELAY;
            end else begin
              command_index <= command_index + 8'd1;
            end
          end
        end

        ST_SAVE_DONE: begin
          saved_sequence <= pending_sequence;
          // Refresh both headers so the next save/load decision is based on
          // bytes actually stored in Flash, not speculative runtime state.
          header_slot <= 1'b0;
          header_index <= 8'd0;
          command_index <= 8'd0;
          flash_cs_n <= 1'b0;
          state <= ST_HEADER_CMD;
        end

        ST_LOAD_VERIFY_CMD, ST_LOAD_READ_CMD: begin
          if (!load_base_safe) begin
            flash_error <= ERR_ADDRESS_GUARD;
            state <= ST_FAIL;
          end else if (!byte_started && !spi_busy) begin
            flash_cs_n <= 1'b0;
            case (command_index)
              8'd0: spi_tx_data <= 8'h03;
              8'd1: spi_tx_data <= load_payload_address[23:16];
              8'd2: spi_tx_data <= load_payload_address[15:8];
              default: spi_tx_data <= load_payload_address[7:0];
            endcase
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            if (command_index == 8'd3) begin
              payload_index <= 14'd0;
              if (state == ST_LOAD_VERIFY_CMD) begin
                load_crc <= 16'hFFFF;
                state <= ST_LOAD_VERIFY_DATA;
              end else begin
                state <= ST_LOAD_READ_DATA;
              end
            end else begin
              command_index <= command_index + 8'd1;
            end
          end
        end

        ST_LOAD_VERIFY_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= 8'hFF;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            load_crc <= crc16_next(load_crc, spi_rx_data);
            flash_progress <= payload_index[12:5];
            if (payload_index == PAYLOAD_BYTES - 1) begin
              flash_cs_n <= 1'b1;
              state <= ST_LOAD_VERIFY_EVAL;
            end else begin
              payload_index <= payload_index + 14'd1;
            end
          end
        end

        ST_LOAD_VERIFY_EVAL: begin
          if (load_crc != expected_load_crc) begin
            flash_error <= ERR_MAP_CRC;
            state <= ST_FAIL;
          end else begin
            command_index <= 8'd0;
            flash_cs_n <= 1'b0;
            state <= ST_LOAD_READ_CMD;
          end
        end

        ST_LOAD_READ_DATA: begin
          if (!byte_started && !spi_busy) begin
            spi_tx_data <= 8'hFF;
            spi_start <= 1'b1;
            byte_started <= 1'b1;
          end
          if (spi_done) begin
            byte_started <= 1'b0;
            load_byte <= spi_rx_data;
            map_mem_addr <= payload_index << 1;
            map_mem_wdata <= spi_rx_data[3:0];
            map_mem_req <= 1'b1;
            map_mem_we <= 1'b1;
            state <= ST_LOAD_WRITE_LOW;
          end
        end

        ST_LOAD_WRITE_LOW: begin
          map_mem_addr <= (payload_index << 1) + 14'd1;
          map_mem_wdata <= load_byte[7:4];
          map_mem_req <= 1'b1;
          map_mem_we <= 1'b1;
          state <= ST_LOAD_WRITE_HIGH;
        end

        ST_LOAD_WRITE_HIGH: begin
          flash_progress <= payload_index[12:5];
          if (payload_index == PAYLOAD_BYTES - 1) begin
            flash_cs_n <= 1'b1;
            state <= ST_LOAD_DONE;
          end else begin
            payload_index <= payload_index + 14'd1;
            state <= ST_LOAD_READ_DATA;
          end
        end

        ST_LOAD_DONE: begin
          loaded_pose_x_mm <= {selected_header_byte(load_slot, 8'd12),
                               selected_header_byte(load_slot, 8'd13)};
          loaded_pose_y_mm <= {selected_header_byte(load_slot, 8'd14),
                               selected_header_byte(load_slot, 8'd15)};
          loaded_pose_heading <= {selected_header_byte(load_slot, 8'd16),
                                  selected_header_byte(load_slot, 8'd17)};
          loaded_scan_count <= {selected_header_byte(load_slot, 8'd18),
                                selected_header_byte(load_slot, 8'd19)};
          loaded_occupied_count <= {selected_header_byte(load_slot, 8'd20),
                                    selected_header_byte(load_slot, 8'd21)};
          loaded_free_count <= {selected_header_byte(load_slot, 8'd22),
                                selected_header_byte(load_slot, 8'd23)};
          loaded_map_epoch <= selected_header_byte(load_slot, 8'd28);
          load_apply_pulse <= 1'b1;
          flash_busy <= 1'b0;
          flash_operation <= 2'd0;
          flash_progress <= 8'hFF;
          flash_error <= ERR_NONE;
          state <= ST_IDLE;
        end

        ST_FAIL: begin
          flash_cs_n <= 1'b1;
          flash_busy <= 1'b0;
          flash_operation <= 2'd0;
          byte_started <= 1'b0;
          state <= ST_IDLE;
        end

        default: begin
          flash_error <= 8'hFF;
          state <= ST_FAIL;
        end
      endcase
    end
  end

endmodule
