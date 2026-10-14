module sd_spi_readonly (
  input wire clk,
  input wire rst_n,
  input wire start,

  input wire sd_miso,
  output reg sd_cs_n,
  output reg sd_sclk,
  output reg sd_mosi,

  output reg probe_busy,
  output reg probe_done,
  output reg probe_ok,
  output reg [7:0] error_code,

  output reg [7:0] cmd0_r1,
  output reg [7:0] cmd8_r1,
  output reg [31:0] cmd8_echo,
  output reg [7:0] acmd41_r1,
  output reg [15:0] acmd41_attempts,
  output reg [7:0] cmd58_r1,
  output reg [31:0] ocr,
  output reg high_capacity,

  output reg [7:0] primary_read_r1,
  output reg [7:0] primary_token,
  output reg [63:0] primary_magic,
  output reg [31:0] primary_version,
  output reg [31:0] primary_stored_crc,
  output reg [31:0] primary_calculated_crc,

  output reg [7:0] backup_read_r1,
  output reg [7:0] backup_token,
  output reg [63:0] backup_magic,
  output reg [31:0] backup_version,
  output reg [31:0] backup_stored_crc,
  output reg [31:0] backup_calculated_crc
);

  parameter integer CLK_HZ = 50000000;
  parameter integer STARTUP_WAIT_CYCLES = CLK_HZ / 100;
  parameter integer ACMD_RETRY_WAIT_CYCLES = CLK_HZ / 1000;
  parameter integer INIT_SPI_HALF_CYCLES = 125;
  parameter integer DATA_SPI_HALF_CYCLES = 5;

  localparam [6:0] ST_IDLE                 = 7'd0;
  localparam [6:0] ST_STARTUP_WAIT         = 7'd1;
  localparam [6:0] ST_POWER_CLOCKS         = 7'd2;
  localparam [6:0] ST_ISSUE_CMD0           = 7'd3;
  localparam [6:0] ST_EVAL_CMD0            = 7'd4;
  localparam [6:0] ST_ISSUE_CMD8           = 7'd5;
  localparam [6:0] ST_EVAL_CMD8            = 7'd6;
  localparam [6:0] ST_ISSUE_CMD55          = 7'd7;
  localparam [6:0] ST_EVAL_CMD55           = 7'd8;
  localparam [6:0] ST_ISSUE_ACMD41         = 7'd9;
  localparam [6:0] ST_EVAL_ACMD41          = 7'd10;
  localparam [6:0] ST_ACMD_RETRY_WAIT      = 7'd11;
  localparam [6:0] ST_ISSUE_CMD58          = 7'd12;
  localparam [6:0] ST_EVAL_CMD58           = 7'd13;
  localparam [6:0] ST_PREPARE_PRIMARY      = 7'd14;
  localparam [6:0] ST_PREPARE_BACKUP       = 7'd15;
  localparam [6:0] ST_ISSUE_READ           = 7'd16;
  localparam [6:0] ST_EVAL_READ            = 7'd17;
  localparam [6:0] ST_READ_TOKEN           = 7'd18;
  localparam [6:0] ST_READ_DATA            = 7'd19;
  localparam [6:0] ST_READ_CRC1            = 7'd20;
  localparam [6:0] ST_READ_CRC2            = 7'd21;
  localparam [6:0] ST_READ_RELEASE         = 7'd22;
  localparam [6:0] ST_READ_RELEASE_WAIT    = 7'd23;
  localparam [6:0] ST_READ_EVAL            = 7'd24;
  localparam [6:0] ST_COMPLETE             = 7'd25;
  localparam [6:0] ST_FAILED               = 7'd26;

  localparam [6:0] ST_CMD_ASSERT           = 7'd64;
  localparam [6:0] ST_CMD_SEND             = 7'd65;
  localparam [6:0] ST_CMD_RESPONSE         = 7'd66;
  localparam [6:0] ST_CMD_EXTRA            = 7'd67;
  localparam [6:0] ST_CMD_RELEASE          = 7'd68;
  localparam [6:0] ST_CMD_RELEASE_WAIT     = 7'd69;

  localparam [7:0] ERR_NONE                = 8'h00;
  localparam [7:0] ERR_CMD0                = 8'h01;
  localparam [7:0] ERR_CMD8                = 8'h02;
  localparam [7:0] ERR_CMD55               = 8'h03;
  localparam [7:0] ERR_ACMD41              = 8'h04;
  localparam [7:0] ERR_CMD58               = 8'h05;
  localparam [7:0] ERR_PRIMARY_CMD17       = 8'h06;
  localparam [7:0] ERR_PRIMARY_TOKEN       = 8'h07;
  localparam [7:0] ERR_PRIMARY_SUPERBLOCK  = 8'h08;
  localparam [7:0] ERR_BACKUP_CMD17        = 8'h09;
  localparam [7:0] ERR_BACKUP_TOKEN        = 8'h0a;
  localparam [7:0] ERR_BACKUP_SUPERBLOCK   = 8'h0b;
  localparam [7:0] ERR_SUPERBLOCK_MISMATCH = 8'h0c;

  localparam [31:0] PRIMARY_SUPERBLOCK_LBA = 32'd2048;
  localparam [31:0] BACKUP_SUPERBLOCK_LBA  = 32'd2049;
  localparam [63:0] SUPERBLOCK_MAGIC       = 64'h524f424549534431;

  function [7:0] command_byte;
    input [5:0] command;
    input [31:0] argument;
    input [7:0] command_crc;
    input [2:0] index;
    begin
      case (index)
        3'd0: command_byte = {2'b01, command};
        3'd1: command_byte = argument[31:24];
        3'd2: command_byte = argument[23:16];
        3'd3: command_byte = argument[15:8];
        3'd4: command_byte = argument[7:0];
        default: command_byte = command_crc;
      endcase
    end
  endfunction

  function [31:0] crc32_update;
    input [31:0] old_crc;
    input [7:0] data_byte;
    integer bit_index;
    reg [31:0] next_crc;
    begin
      next_crc = old_crc ^ data_byte;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (next_crc[0])
          next_crc = (next_crc >> 1) ^ 32'hedb88320;
        else
          next_crc = next_crc >> 1;
      end
      crc32_update = next_crc;
    end
  endfunction

  reg spi_start;
  reg spi_fast;
  reg [7:0] spi_tx_data;
  reg [7:0] spi_rx_data;
  reg spi_busy;
  reg spi_done;
  reg [15:0] spi_div_count;
  reg [2:0] spi_bit_index;
  reg spi_last_sampled;
  wire [15:0] spi_half_cycles = spi_fast ?
      DATA_SPI_HALF_CYCLES[15:0] : INIT_SPI_HALF_CYCLES[15:0];

  // SD SPI mode 0: MOSI changes while SCLK is low and MISO is sampled on the
  // rising edge. The byte engine is deliberately independent of command and
  // block framing so the same timing can be reused by the writable host later.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sd_sclk <= 1'b0;
      sd_mosi <= 1'b1;
      spi_rx_data <= 8'hff;
      spi_busy <= 1'b0;
      spi_done <= 1'b0;
      spi_div_count <= 16'd0;
      spi_bit_index <= 3'd7;
      spi_last_sampled <= 1'b0;
    end else begin
      spi_done <= 1'b0;
      if (!spi_busy) begin
        sd_sclk <= 1'b0;
        spi_div_count <= 16'd0;
        if (spi_start) begin
          spi_busy <= 1'b1;
          spi_bit_index <= 3'd7;
          spi_last_sampled <= 1'b0;
          spi_rx_data <= 8'd0;
          sd_mosi <= spi_tx_data[7];
        end else begin
          sd_mosi <= 1'b1;
        end
      end else if (spi_div_count >= spi_half_cycles - 1'b1) begin
        spi_div_count <= 16'd0;
        if (!sd_sclk) begin
          sd_sclk <= 1'b1;
          spi_rx_data[spi_bit_index] <= sd_miso;
          if (spi_bit_index == 3'd0)
            spi_last_sampled <= 1'b1;
          else
            spi_bit_index <= spi_bit_index - 3'd1;
        end else if (spi_last_sampled) begin
          sd_sclk <= 1'b0;
          sd_mosi <= 1'b1;
          spi_busy <= 1'b0;
          spi_done <= 1'b1;
          spi_last_sampled <= 1'b0;
        end else begin
          sd_sclk <= 1'b0;
          sd_mosi <= spi_tx_data[spi_bit_index];
        end
      end else begin
        spi_div_count <= spi_div_count + 16'd1;
      end
    end
  end

  reg [6:0] state;
  reg [6:0] command_return_state;
  reg byte_pending;
  reg [31:0] wait_count;
  reg [3:0] power_clock_count;

  reg [5:0] command_number;
  reg [31:0] command_argument;
  reg [7:0] command_crc;
  reg command_has_extra;
  reg command_keep_selected;
  reg [2:0] command_index;
  reg [4:0] response_wait_count;
  reg [1:0] extra_index;
  reg [7:0] command_r1;
  reg [31:0] command_extra;

  reg card_v2;
  reg read_is_backup;
  reg [31:0] read_lba;
  reg [15:0] token_wait_count;
  reg [9:0] read_byte_index;
  reg [63:0] read_magic;
  reg [31:0] read_version;
  reg [31:0] read_stored_crc;
  reg [31:0] read_crc_work;
  reg primary_valid;

  wire [31:0] read_calculated_crc = read_crc_work ^ 32'hffffffff;
  wire read_superblock_valid =
      (read_magic == SUPERBLOCK_MAGIC) &&
      (read_version == 32'd1) &&
      (read_stored_crc == read_calculated_crc);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sd_cs_n <= 1'b1;
      spi_start <= 1'b0;
      spi_fast <= 1'b0;
      spi_tx_data <= 8'hff;
      state <= ST_IDLE;
      command_return_state <= ST_IDLE;
      byte_pending <= 1'b0;
      wait_count <= 32'd0;
      power_clock_count <= 4'd0;
      command_number <= 6'd0;
      command_argument <= 32'd0;
      command_crc <= 8'h01;
      command_has_extra <= 1'b0;
      command_keep_selected <= 1'b0;
      command_index <= 3'd0;
      response_wait_count <= 5'd0;
      extra_index <= 2'd0;
      command_r1 <= 8'hff;
      command_extra <= 32'd0;
      card_v2 <= 1'b0;
      read_is_backup <= 1'b0;
      read_lba <= 32'd0;
      token_wait_count <= 16'd0;
      read_byte_index <= 10'd0;
      read_magic <= 64'd0;
      read_version <= 32'd0;
      read_stored_crc <= 32'd0;
      read_crc_work <= 32'hffffffff;
      primary_valid <= 1'b0;
      probe_busy <= 1'b0;
      probe_done <= 1'b0;
      probe_ok <= 1'b0;
      error_code <= ERR_NONE;
      cmd0_r1 <= 8'hff;
      cmd8_r1 <= 8'hff;
      cmd8_echo <= 32'd0;
      acmd41_r1 <= 8'hff;
      acmd41_attempts <= 16'd0;
      cmd58_r1 <= 8'hff;
      ocr <= 32'd0;
      high_capacity <= 1'b0;
      primary_read_r1 <= 8'hff;
      primary_token <= 8'hff;
      primary_magic <= 64'd0;
      primary_version <= 32'd0;
      primary_stored_crc <= 32'd0;
      primary_calculated_crc <= 32'd0;
      backup_read_r1 <= 8'hff;
      backup_token <= 8'hff;
      backup_magic <= 64'd0;
      backup_version <= 32'd0;
      backup_stored_crc <= 32'd0;
      backup_calculated_crc <= 32'd0;
    end else begin
      spi_start <= 1'b0;

      case (state)
        ST_IDLE: begin
          sd_cs_n <= 1'b1;
          probe_busy <= 1'b0;
          if (start) begin
            probe_busy <= 1'b1;
            probe_done <= 1'b0;
            probe_ok <= 1'b0;
            error_code <= ERR_NONE;
            spi_fast <= 1'b0;
            wait_count <= 32'd0;
            power_clock_count <= 4'd0;
            acmd41_attempts <= 16'd0;
            primary_valid <= 1'b0;
            state <= ST_STARTUP_WAIT;
          end
        end

        ST_STARTUP_WAIT: begin
          sd_cs_n <= 1'b1;
          if (wait_count >= STARTUP_WAIT_CYCLES - 1) begin
            wait_count <= 32'd0;
            byte_pending <= 1'b0;
            power_clock_count <= 4'd0;
            state <= ST_POWER_CLOCKS;
          end else begin
            wait_count <= wait_count + 32'd1;
          end
        end

        ST_POWER_CLOCKS: begin
          sd_cs_n <= 1'b1;
          if (spi_done) begin
            byte_pending <= 1'b0;
            if (power_clock_count == 4'd9) begin
              power_clock_count <= 4'd0;
              state <= ST_ISSUE_CMD0;
            end else begin
              power_clock_count <= power_clock_count + 4'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_ISSUE_CMD0: begin
          command_number <= 6'd0;
          command_argument <= 32'd0;
          command_crc <= 8'h95;
          command_has_extra <= 1'b0;
          command_keep_selected <= 1'b0;
          command_return_state <= ST_EVAL_CMD0;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_CMD0: begin
          cmd0_r1 <= command_r1;
          if (command_r1 == 8'h01)
            state <= ST_ISSUE_CMD8;
          else begin
            error_code <= ERR_CMD0;
            state <= ST_FAILED;
          end
        end

        ST_ISSUE_CMD8: begin
          command_number <= 6'd8;
          command_argument <= 32'h000001aa;
          command_crc <= 8'h87;
          command_has_extra <= 1'b1;
          command_keep_selected <= 1'b0;
          command_return_state <= ST_EVAL_CMD8;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_CMD8: begin
          cmd8_r1 <= command_r1;
          cmd8_echo <= command_extra;
          if ((command_r1 == 8'h01) &&
              (command_extra[11:0] == 12'h1aa)) begin
            card_v2 <= 1'b1;
            state <= ST_ISSUE_CMD55;
          end else if (command_r1[2]) begin
            card_v2 <= 1'b0;
            state <= ST_ISSUE_CMD55;
          end else begin
            error_code <= ERR_CMD8;
            state <= ST_FAILED;
          end
        end

        ST_ISSUE_CMD55: begin
          command_number <= 6'd55;
          command_argument <= 32'd0;
          command_crc <= 8'h01;
          command_has_extra <= 1'b0;
          command_keep_selected <= 1'b0;
          command_return_state <= ST_EVAL_CMD55;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_CMD55: begin
          if ((command_r1 == 8'h01) || (command_r1 == 8'h00))
            state <= ST_ISSUE_ACMD41;
          else begin
            error_code <= ERR_CMD55;
            state <= ST_FAILED;
          end
        end

        ST_ISSUE_ACMD41: begin
          command_number <= 6'd41;
          command_argument <= card_v2 ? 32'h40000000 : 32'd0;
          command_crc <= 8'h01;
          command_has_extra <= 1'b0;
          command_keep_selected <= 1'b0;
          command_return_state <= ST_EVAL_ACMD41;
          acmd41_attempts <= acmd41_attempts + 16'd1;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_ACMD41: begin
          acmd41_r1 <= command_r1;
          if (command_r1 == 8'h00) begin
            state <= ST_ISSUE_CMD58;
          end else if ((command_r1 == 8'h01) &&
                       (acmd41_attempts < 16'd1024)) begin
            wait_count <= 32'd0;
            state <= ST_ACMD_RETRY_WAIT;
          end else begin
            error_code <= ERR_ACMD41;
            state <= ST_FAILED;
          end
        end

        ST_ACMD_RETRY_WAIT: begin
          if (wait_count >= ACMD_RETRY_WAIT_CYCLES - 1) begin
            wait_count <= 32'd0;
            state <= ST_ISSUE_CMD55;
          end else begin
            wait_count <= wait_count + 32'd1;
          end
        end

        ST_ISSUE_CMD58: begin
          command_number <= 6'd58;
          command_argument <= 32'd0;
          command_crc <= 8'h01;
          command_has_extra <= 1'b1;
          command_keep_selected <= 1'b0;
          command_return_state <= ST_EVAL_CMD58;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_CMD58: begin
          cmd58_r1 <= command_r1;
          ocr <= command_extra;
          if (command_r1 == 8'h00) begin
            high_capacity <= command_extra[30];
            spi_fast <= 1'b1;
            state <= ST_PREPARE_PRIMARY;
          end else begin
            error_code <= ERR_CMD58;
            state <= ST_FAILED;
          end
        end

        ST_PREPARE_PRIMARY: begin
          read_is_backup <= 1'b0;
          read_lba <= PRIMARY_SUPERBLOCK_LBA;
          read_magic <= 64'd0;
          read_version <= 32'd0;
          read_stored_crc <= 32'd0;
          read_crc_work <= 32'hffffffff;
          read_byte_index <= 10'd0;
          token_wait_count <= 16'd0;
          state <= ST_ISSUE_READ;
        end

        ST_PREPARE_BACKUP: begin
          read_is_backup <= 1'b1;
          read_lba <= BACKUP_SUPERBLOCK_LBA;
          read_magic <= 64'd0;
          read_version <= 32'd0;
          read_stored_crc <= 32'd0;
          read_crc_work <= 32'hffffffff;
          read_byte_index <= 10'd0;
          token_wait_count <= 16'd0;
          state <= ST_ISSUE_READ;
        end

        ST_ISSUE_READ: begin
          command_number <= 6'd17;
          command_argument <= high_capacity ? read_lba : (read_lba << 9);
          command_crc <= 8'h01;
          command_has_extra <= 1'b0;
          command_keep_selected <= 1'b1;
          command_return_state <= ST_EVAL_READ;
          state <= ST_CMD_ASSERT;
        end

        ST_EVAL_READ: begin
          if (read_is_backup)
            backup_read_r1 <= command_r1;
          else
            primary_read_r1 <= command_r1;

          if (command_r1 == 8'h00) begin
            byte_pending <= 1'b0;
            token_wait_count <= 16'd0;
            state <= ST_READ_TOKEN;
          end else begin
            sd_cs_n <= 1'b1;
            error_code <= read_is_backup ?
                ERR_BACKUP_CMD17 : ERR_PRIMARY_CMD17;
            state <= ST_FAILED;
          end
        end

        ST_READ_TOKEN: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            if (spi_rx_data == 8'hfe) begin
              if (read_is_backup)
                backup_token <= spi_rx_data;
              else
                primary_token <= spi_rx_data;
              read_byte_index <= 10'd0;
              state <= ST_READ_DATA;
            end else if (token_wait_count == 16'hffff) begin
              if (read_is_backup)
                backup_token <= spi_rx_data;
              else
                primary_token <= spi_rx_data;
              sd_cs_n <= 1'b1;
              error_code <= read_is_backup ?
                  ERR_BACKUP_TOKEN : ERR_PRIMARY_TOKEN;
              state <= ST_FAILED;
            end else begin
              token_wait_count <= token_wait_count + 16'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_READ_DATA: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            if (read_byte_index < 10'd508)
              read_crc_work <= crc32_update(read_crc_work, spi_rx_data);
            if (read_byte_index < 10'd8)
              read_magic <= {read_magic[55:0], spi_rx_data};
            case (read_byte_index)
              10'd8: read_version[7:0] <= spi_rx_data;
              10'd9: read_version[15:8] <= spi_rx_data;
              10'd10: read_version[23:16] <= spi_rx_data;
              10'd11: read_version[31:24] <= spi_rx_data;
              10'd508: read_stored_crc[7:0] <= spi_rx_data;
              10'd509: read_stored_crc[15:8] <= spi_rx_data;
              10'd510: read_stored_crc[23:16] <= spi_rx_data;
              10'd511: read_stored_crc[31:24] <= spi_rx_data;
              default: begin end
            endcase
            if (read_byte_index == 10'd511) begin
              state <= ST_READ_CRC1;
            end else begin
              read_byte_index <= read_byte_index + 10'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_READ_CRC1: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            state <= ST_READ_CRC2;
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_READ_CRC2: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            sd_cs_n <= 1'b1;
            state <= ST_READ_RELEASE;
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_READ_RELEASE: begin
          sd_cs_n <= 1'b1;
          if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
            state <= ST_READ_RELEASE_WAIT;
          end
        end

        ST_READ_RELEASE_WAIT: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            state <= ST_READ_EVAL;
          end
        end

        ST_READ_EVAL: begin
          if (read_is_backup) begin
            backup_magic <= read_magic;
            backup_version <= read_version;
            backup_stored_crc <= read_stored_crc;
            backup_calculated_crc <= read_calculated_crc;
            if (!read_superblock_valid) begin
              error_code <= ERR_BACKUP_SUPERBLOCK;
              state <= ST_COMPLETE;
            end else if (!primary_valid ||
                         (primary_stored_crc != read_stored_crc)) begin
              error_code <= ERR_SUPERBLOCK_MISMATCH;
              state <= ST_COMPLETE;
            end else begin
              state <= ST_COMPLETE;
            end
          end else begin
            primary_magic <= read_magic;
            primary_version <= read_version;
            primary_stored_crc <= read_stored_crc;
            primary_calculated_crc <= read_calculated_crc;
            primary_valid <= read_superblock_valid;
            if (!read_superblock_valid)
              error_code <= ERR_PRIMARY_SUPERBLOCK;
            state <= ST_PREPARE_BACKUP;
          end
        end

        ST_COMPLETE: begin
          sd_cs_n <= 1'b1;
          probe_busy <= 1'b0;
          probe_done <= 1'b1;
          probe_ok <= (error_code == ERR_NONE);
          state <= ST_COMPLETE;
        end

        ST_FAILED: begin
          sd_cs_n <= 1'b1;
          probe_busy <= 1'b0;
          probe_done <= 1'b1;
          probe_ok <= 1'b0;
          state <= ST_FAILED;
        end

        ST_CMD_ASSERT: begin
          sd_cs_n <= 1'b0;
          command_index <= 3'd0;
          response_wait_count <= 5'd0;
          extra_index <= 2'd0;
          command_r1 <= 8'hff;
          command_extra <= 32'd0;
          byte_pending <= 1'b0;
          state <= ST_CMD_SEND;
        end

        ST_CMD_SEND: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            if (command_index == 3'd5) begin
              response_wait_count <= 5'd0;
              state <= ST_CMD_RESPONSE;
            end else begin
              command_index <= command_index + 3'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= command_byte(
                command_number, command_argument, command_crc, command_index);
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_CMD_RESPONSE: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            if (!spi_rx_data[7]) begin
              command_r1 <= spi_rx_data;
              if (command_has_extra) begin
                extra_index <= 2'd0;
                state <= ST_CMD_EXTRA;
              end else if (command_keep_selected) begin
                state <= command_return_state;
              end else begin
                sd_cs_n <= 1'b1;
                state <= ST_CMD_RELEASE;
              end
            end else if (response_wait_count == 5'd15) begin
              command_r1 <= 8'hff;
              if (command_keep_selected) begin
                state <= command_return_state;
              end else begin
                sd_cs_n <= 1'b1;
                state <= ST_CMD_RELEASE;
              end
            end else begin
              response_wait_count <= response_wait_count + 5'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_CMD_EXTRA: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            command_extra <= {command_extra[23:0], spi_rx_data};
            if (extra_index == 2'd3) begin
              if (command_keep_selected) begin
                state <= command_return_state;
              end else begin
                sd_cs_n <= 1'b1;
                state <= ST_CMD_RELEASE;
              end
            end else begin
              extra_index <= extra_index + 2'd1;
            end
          end else if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
          end
        end

        ST_CMD_RELEASE: begin
          sd_cs_n <= 1'b1;
          if (!byte_pending && !spi_busy) begin
            spi_tx_data <= 8'hff;
            spi_start <= 1'b1;
            byte_pending <= 1'b1;
            state <= ST_CMD_RELEASE_WAIT;
          end
        end

        ST_CMD_RELEASE_WAIT: begin
          if (spi_done) begin
            byte_pending <= 1'b0;
            state <= command_return_state;
          end
        end

        default: begin
          sd_cs_n <= 1'b1;
          error_code <= 8'hff;
          state <= ST_FAILED;
        end
      endcase
    end
  end

endmodule
