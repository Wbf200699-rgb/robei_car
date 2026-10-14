module sd_spi_block_device (
  input wire clk,
  input wire rst_n,
  input wire init_start,
  input wire block_start,
  input wire block_write,
  input wire [31:0] block_lba,
  input wire [7:0] buffer_read_data,
  input wire sd_miso,
  output wire [8:0] buffer_addr,
  output wire buffer_write_enable,
  output wire [7:0] buffer_write_data,
  output wire sd_cs_n,
  output wire sd_sclk,
  output wire sd_mosi,
  output wire initialized,
  output wire init_busy,
  output wire init_done,
  output wire block_busy,
  output wire block_done,
  output wire [7:0] error_code,
  output wire [7:0] last_r1,
  output wire [7:0] write_data_response,
  output wire [15:0] acmd41_attempts,
  output wire [31:0] ocr,
  output wire high_capacity
);

  parameter integer CLK_HZ = 50000000;
  parameter integer STARTUP_WAIT_CYCLES = CLK_HZ / 100;
  parameter integer ACMD_RETRY_WAIT_CYCLES = CLK_HZ / 1000;
  parameter integer INIT_SPI_HALF_CYCLES = 125;
  parameter integer DATA_SPI_HALF_CYCLES = 5;
  parameter integer TOKEN_TIMEOUT_BYTES = 65535;
  parameter integer WRITE_BUSY_TIMEOUT_BYTES = 65535;

  // Phone-hosted mapping leaves the SD bus electrically idle.  CS stays high,
  // clock stays low and MOSI stays high, which is safe even when a card remains
  // inserted while the competition firmware is running.
  assign buffer_addr = 9'd0;
  assign buffer_write_enable = 1'b0;
  assign buffer_write_data = 8'd0;
  assign sd_cs_n = 1'b1;
  assign sd_sclk = 1'b0;
  assign sd_mosi = 1'b1;
  assign initialized = 1'b0;
  assign init_busy = 1'b0;
  assign init_done = 1'b0;
  assign block_busy = 1'b0;
  assign block_done = 1'b0;
  assign error_code = 8'd0;
  assign last_r1 = 8'hff;
  assign write_data_response = 8'hff;
  assign acmd41_attempts = 16'd0;
  assign ocr = 32'd0;
  assign high_capacity = 1'b0;

endmodule
