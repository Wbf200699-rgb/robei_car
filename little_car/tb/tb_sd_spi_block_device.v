`timescale 1ns/1ps

module tb_sd_spi_block_device;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg init_start = 1'b0;
  reg block_start = 1'b0;
  reg block_write = 1'b0;
  reg [31:0] block_lba = 32'd0;
  wire [8:0] buffer_addr;
  wire [7:0] buffer_read_data;
  wire buffer_write_enable;
  wire [7:0] buffer_write_data;
  wire sd_cs_n;
  wire sd_sclk;
  wire sd_mosi;
  wire sd_miso;
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

  reg [7:0] block_buffer [0:511];
  reg [7:0] expected_data [0:511];
  integer index;
  integer failures = 0;

  always #10 clk = ~clk;
  assign buffer_read_data = block_buffer[buffer_addr];

  always @(posedge clk) begin
    if (buffer_write_enable)
      block_buffer[buffer_addr] <= buffer_write_data;
  end

  sd_spi_block_device #(
    .STARTUP_WAIT_CYCLES(20),
    .ACMD_RETRY_WAIT_CYCLES(8),
    .INIT_SPI_HALF_CYCLES(2),
    .DATA_SPI_HALF_CYCLES(1),
    .TOKEN_TIMEOUT_BYTES(64),
    .WRITE_BUSY_TIMEOUT_BYTES(64)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .init_start(init_start),
    .block_start(block_start),
    .block_write(block_write),
    .block_lba(block_lba),
    .buffer_addr(buffer_addr),
    .buffer_read_data(buffer_read_data),
    .buffer_write_enable(buffer_write_enable),
    .buffer_write_data(buffer_write_data),
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

  sd_spi_writable_card_model card (
    .sd_cs_n(sd_cs_n),
    .sd_sclk(sd_sclk),
    .sd_mosi(sd_mosi),
    .sd_miso(sd_miso)
  );

  task pulse_init;
    begin
      @(negedge clk); init_start = 1'b1;
      @(negedge clk); init_start = 1'b0;
    end
  endtask

  task pulse_block;
    input write_operation;
    input [31:0] lba;
    begin
      @(negedge clk);
      block_write = write_operation;
      block_lba = lba;
      block_start = 1'b1;
      @(negedge clk);
      block_start = 1'b0;
      wait (block_done);
      @(posedge clk);
    end
  endtask

  initial begin
    $dumpfile("temp/tb_sd_spi_block_device.vcd");
    $dumpvars(0, tb_sd_spi_block_device);
    for (index = 0; index < 512; index = index + 1) begin
      expected_data[index] = ((index * 37) + 11) & 8'hff;
      block_buffer[index] = expected_data[index];
    end

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    pulse_init();
    wait (init_done);
    if (!initialized || error_code != 8'h00 || !high_capacity ||
        ocr != 32'hc0ff8000) begin
      $display("ERROR init initialized=%0d error=%02x hc=%0d ocr=%08x",
               initialized, error_code, high_capacity, ocr);
      failures = failures + 1;
    end

    pulse_block(1'b1, 32'd3072);
    if (error_code != 8'h00 || write_data_response[4:0] != 5'h05) begin
      $display("ERROR write error=%02x response=%02x r1=%02x",
               error_code, write_data_response, last_r1);
      failures = failures + 1;
    end
    for (index = 0; index < 512; index = index + 1) begin
      if (card.sector[index] !== expected_data[index]) begin
        $display("ERROR card byte index=%0d got=%02x expected=%02x",
                 index, card.sector[index], expected_data[index]);
        failures = failures + 1;
        index = 512;
      end
    end

    for (index = 0; index < 512; index = index + 1)
      block_buffer[index] = 8'h00;
    pulse_block(1'b0, 32'd3072);
    repeat (3) @(posedge clk);
    if (error_code != 8'h00) begin
      $display("ERROR read error=%02x r1=%02x", error_code, last_r1);
      failures = failures + 1;
    end
    for (index = 0; index < 512; index = index + 1) begin
      if (block_buffer[index] !== expected_data[index]) begin
        $display("ERROR readback byte index=%0d got=%02x expected=%02x",
                 index, block_buffer[index], expected_data[index]);
        failures = failures + 1;
        index = 512;
      end
    end

    $display("RESULT init attempts=%0d ocr=%08x high_capacity=%0d",
             acmd41_attempts, ocr, high_capacity);
    $display("RESULT commands cmd24=%0d cmd17=%0d last_write_lba=%0d last_read_lba=%0d",
             card.cmd24_count, card.cmd17_count,
             card.last_write_lba, card.last_read_lba);
    $display("RESULT write_response=%02x busy_bytes=%0d sample=%02x,%02x,%02x,%02x",
             write_data_response, card.write_busy_bytes_sent,
             block_buffer[0], block_buffer[1],
             block_buffer[510], block_buffer[511]);
    if ((card.cmd24_count != 1) || (card.cmd17_count != 1) ||
        (card.last_write_lba != 32'd3072) ||
        (card.last_read_lba != 32'd3072)) begin
      $display("ERROR command accounting");
      failures = failures + 1;
    end

    if (failures == 0)
      $display("RESULT TEST_PASS sd_spi_block_device");
    else
      $display("RESULT TEST_FAIL sd_spi_block_device failures=%0d", failures);
    $finish;
  end

  initial begin
    repeat (3000000) @(posedge clk);
    $display("RESULT TEST_FAIL sd_spi_block_timeout state=%0d error=%02x",
             dut.state, error_code);
    $finish;
  end
endmodule

module sd_spi_writable_card_model (
  input wire sd_cs_n,
  input wire sd_sclk,
  input wire sd_mosi,
  output reg sd_miso
);
  reg [7:0] sector [0:511];
  reg [7:0] tx_queue [0:4095];
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
  integer index;
  reg [7:0] completed_byte;
  reg [5:0] decoded_command;
  reg [31:0] decoded_argument;
  integer cmd24_count;
  integer cmd17_count;
  integer write_busy_bytes_sent;
  reg [31:0] last_write_lba;
  reg [31:0] last_read_lba;

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
          cmd17_count = cmd17_count + 1;
          last_read_lba = decoded_argument;
          enqueue(8'h00);
          enqueue(8'hff);
          enqueue(8'hfe);
          for (index = 0; index < 512; index = index + 1)
            enqueue(sector[index]);
          enqueue(8'h12);
          enqueue(8'h34);
        end
        6'd24: begin
          cmd24_count = cmd24_count + 1;
          last_write_lba = decoded_argument;
          write_wait_token = 1'b1;
          enqueue(8'h00);
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
    write_wait_token = 1'b0;
    write_collecting = 1'b0;
    write_byte_index = 0;
    write_crc_index = 0;
    cmd24_count = 0;
    cmd17_count = 0;
    write_busy_bytes_sent = 0;
    last_write_lba = 32'd0;
    last_read_lba = 32'd0;
    for (index = 0; index < 512; index = index + 1)
      sector[index] = 8'hff;
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
            sector[write_byte_index] = completed_byte;
            write_byte_index = write_byte_index + 1;
          end else begin
            write_crc_index = write_crc_index + 1;
            if (write_crc_index == 2) begin
              write_collecting = 1'b0;
              write_crc_index = 0;
              enqueue(8'h05);
              enqueue(8'h00);
              enqueue(8'h00);
              enqueue(8'h00);
              enqueue(8'hff);
              write_busy_bytes_sent = write_busy_bytes_sent + 3;
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
