`timescale 1ns/1ps

module tb_sd_spi_readonly;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg start = 1'b0;
  wire sd_cs_n;
  wire sd_sclk;
  wire sd_mosi;
  wire sd_miso;
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

  always #10 clk = ~clk;

  sd_spi_readonly #(
    .STARTUP_WAIT_CYCLES(20),
    .ACMD_RETRY_WAIT_CYCLES(8),
    .INIT_SPI_HALF_CYCLES(2),
    .DATA_SPI_HALF_CYCLES(1)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
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

  sd_spi_card_model card (
    .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi),
    .sd_miso(sd_miso)
  );

  initial begin
    $dumpfile("temp/tb_sd_spi_readonly.vcd");
    $dumpvars(0, tb_sd_spi_readonly);
    repeat (5) @(posedge clk);
    rst_n <= 1'b1;
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    fork
      begin
        wait (probe_done);
      end
      begin
        repeat (2000000) @(posedge clk);
        $display("RESULT TEST_FAIL sd_timeout");
        $finish;
      end
    join_any
    disable fork;

    $display("RESULT init cmd0=%02x cmd8=%02x echo=%08x acmd41=%02x attempts=%0d cmd58=%02x ocr=%08x hc=%0d",
             cmd0_r1, cmd8_r1, cmd8_echo, acmd41_r1, acmd41_attempts,
             cmd58_r1, ocr, high_capacity);
    $display("RESULT primary r1=%02x token=%02x magic=%016x version=%08x stored=%08x calculated=%08x",
             primary_read_r1, primary_token, primary_magic, primary_version,
             primary_stored_crc, primary_calculated_crc);
    $display("RESULT backup r1=%02x token=%02x magic=%016x version=%08x stored=%08x calculated=%08x",
             backup_read_r1, backup_token, backup_magic, backup_version,
             backup_stored_crc, backup_calculated_crc);

    if (!probe_ok || error_code != 8'h00 ||
        cmd0_r1 != 8'h01 || cmd8_r1 != 8'h01 ||
        cmd8_echo != 32'h000001aa || acmd41_r1 != 8'h00 ||
        cmd58_r1 != 8'h00 || !high_capacity ||
        primary_read_r1 != 8'h00 || primary_token != 8'hfe ||
        primary_magic != 64'h524f424549534431 ||
        primary_version != 32'd1 ||
        primary_stored_crc != primary_calculated_crc ||
        backup_read_r1 != 8'h00 || backup_token != 8'hfe ||
        backup_magic != primary_magic ||
        backup_stored_crc != backup_calculated_crc ||
        backup_stored_crc != primary_stored_crc) begin
      $display("RESULT TEST_FAIL sd_readonly error=%02x", error_code);
      $finish;
    end

    $display("RESULT TEST_PASS sd_readonly");
    $finish;
  end
endmodule

module sd_spi_card_model (
  input wire sd_cs_n,
  input wire sd_sclk,
  input wire sd_mosi,
  output reg sd_miso
);
  reg [7:0] sector [0:511];
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
  integer index;
  reg [31:0] sector_crc;
  reg [7:0] completed_byte;
  reg [5:0] decoded_command;
  reg [31:0] decoded_argument;

  function [31:0] crc32_update;
    input [31:0] old_crc;
    input [7:0] data_byte;
    integer bit_number;
    reg [31:0] next_crc;
    begin
      next_crc = old_crc ^ data_byte;
      for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
        if (next_crc[0])
          next_crc = (next_crc >> 1) ^ 32'hedb88320;
        else
          next_crc = next_crc >> 1;
      end
      crc32_update = next_crc;
    end
  endfunction

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
          enqueue(8'h01);
          enqueue(8'h00);
          enqueue(8'h00);
          enqueue(8'h01);
          enqueue(8'haa);
        end
        6'd55: enqueue(card_ready ? 8'h00 : 8'h01);
        6'd41: begin
          acmd_attempt = acmd_attempt + 1;
          if (acmd_attempt >= 3) begin
            card_ready = 1'b1;
            enqueue(8'h00);
          end else begin
            enqueue(8'h01);
          end
        end
        6'd58: begin
          enqueue(8'h00);
          enqueue(8'hc0);
          enqueue(8'hff);
          enqueue(8'h80);
          enqueue(8'h00);
        end
        6'd17: begin
          enqueue(8'h00);
          enqueue(8'hff);
          enqueue(8'hff);
          enqueue(8'hfe);
          for (index = 0; index < 512; index = index + 1)
            enqueue(sector[index]);
          enqueue(8'h12);
          enqueue(8'h34);
        end
        default: enqueue(8'h04);
      endcase
    end
  endtask

  initial begin
    sd_miso = 1'b1;
    queue_head = 0;
    queue_tail = 0;
    rx_shift = 8'd0;
    rx_bit_count = 0;
    tx_shift = 8'hff;
    tx_bit_index = 6;
    load_next_byte = 1'b0;
    collecting_command = 1'b0;
    command_index = 0;
    acmd_attempt = 0;
    card_ready = 1'b0;
    for (index = 0; index < 512; index = index + 1)
      sector[index] = 8'h00;
    sector[0] = "R";
    sector[1] = "O";
    sector[2] = "B";
    sector[3] = "E";
    sector[4] = "I";
    sector[5] = "S";
    sector[6] = "D";
    sector[7] = "1";
    sector[8] = 8'h01;
    sector[12] = 8'h00;
    sector[13] = 8'h02;
    sector[16] = 8'h00;
    sector[17] = 8'h08;
    sector[24] = 8'h00;
    sector[25] = 8'h00;
    sector[26] = 8'h01;
    sector_crc = 32'hffffffff;
    for (index = 0; index < 508; index = index + 1)
      sector_crc = crc32_update(sector_crc, sector[index]);
    sector_crc = sector_crc ^ 32'hffffffff;
    sector[508] = sector_crc[7:0];
    sector[509] = sector_crc[15:8];
    sector[510] = sector_crc[23:16];
    sector[511] = sector_crc[31:24];
  end

  always @(negedge sd_cs_n) begin
    queue_head = 0;
    queue_tail = 0;
    rx_shift = 8'd0;
    rx_bit_count = 0;
    tx_shift = 8'hff;
    tx_bit_index = 6;
    load_next_byte = 1'b0;
    collecting_command = 1'b0;
    command_index = 0;
    sd_miso = 1'b1;
  end

  always @(posedge sd_cs_n) begin
    sd_miso = 1'b1;
    collecting_command = 1'b0;
  end

  always @(posedge sd_sclk) begin
    if (!sd_cs_n) begin
      rx_shift = {rx_shift[6:0], sd_mosi};
      if (rx_bit_count == 7) begin
        completed_byte = rx_shift;
        rx_shift = 8'd0;
        rx_bit_count = 0;
        load_next_byte = 1'b1;
        if (!collecting_command) begin
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
          end else begin
            command_index = command_index + 1;
          end
        end
      end else begin
        rx_bit_count = rx_bit_count + 1;
      end
    end
  end

  always @(negedge sd_sclk) begin
    if (!sd_cs_n) begin
      if (load_next_byte) begin
        if (queue_head < queue_tail) begin
          tx_shift = tx_queue[queue_head];
          queue_head = queue_head + 1;
        end else begin
          tx_shift = 8'hff;
        end
        sd_miso = tx_shift[7];
        tx_bit_index = 6;
        load_next_byte = 1'b0;
      end else begin
        sd_miso = tx_shift[tx_bit_index];
        if (tx_bit_index > 0)
          tx_bit_index = tx_bit_index - 1;
      end
    end
  end
endmodule
