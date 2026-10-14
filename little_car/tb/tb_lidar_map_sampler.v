`timescale 1ns/1ps

module tb_lidar_map_sampler;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg scan_enable = 1'b0;
  reg sample_valid = 1'b0;
  reg new_scan = 1'b0;
  reg [5:0] quality = 6'd0;
  reg [14:0] angle_q6 = 15'd0;
  reg [15:0] distance_q2 = 16'd0;
  reg map_chunk_accept = 1'b0;

  wire map_chunk_valid;
  wire [7:0] map_scan_seq;
  wire [15:0] map_tick_10ms;
  wire [2:0] map_part;
  wire [143:0] map_chunk_data;
  wire [7:0] map_chunk_crc;
  wire [15:0] map_drop_count;

  integer cycle_count = 0;
  integer part_index;
  integer previous_accept_cycle;
  reg [7:0] expected_crc;

  always #10 clk = ~clk;
  always @(posedge clk) cycle_count <= cycle_count + 1;

  lidar_map_sampler dut (
    .clk(clk),
    .rst_n(rst_n),
    .scan_enable(scan_enable),
    .sample_valid(sample_valid),
    .new_scan(new_scan),
    .quality(quality),
    .angle_q6(angle_q6),
    .distance_q2(distance_q2),
    .map_chunk_accept(map_chunk_accept),
    .map_chunk_valid(map_chunk_valid),
    .map_scan_seq(map_scan_seq),
    .map_tick_10ms(map_tick_10ms),
    .map_part(map_part),
    .map_chunk_data(map_chunk_data),
    .map_chunk_crc(map_chunk_crc),
    .map_drop_count(map_drop_count)
  );

  defparam dut.CLK_HZ = 100000;
  defparam dut.REPORT_INTERVAL_TICKS = 2;
  defparam dut.CHUNK_GAP_MS = 1;

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      hex_ascii = value + ((value < 4'd10) ? 8'd48 : 8'd55);
    end
  endfunction

  function [7:0] crc8_next;
    input [7:0] crc_in;
    input [7:0] data_in;
    integer bit_index;
    reg [7:0] value;
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
        value = value[7] ? ((value << 1) ^ 8'h07) : (value << 1);
      crc8_next = value;
    end
  endfunction

  function [7:0] expected_chunk_crc;
    input [7:0] sequence_in;
    input [15:0] tick_in;
    input [2:0] part_in;
    input [143:0] data_in;
    integer word_index;
    reg [11:0] word_value;
    reg [7:0] value;
    begin
      value = crc8_next(8'h00, "R");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, hex_ascii(tick_in[15:12]));
      value = crc8_next(value, hex_ascii(tick_in[11:8]));
      value = crc8_next(value, hex_ascii(tick_in[7:4]));
      value = crc8_next(value, hex_ascii(tick_in[3:0]));
      value = crc8_next(value, hex_ascii({1'b0, part_in}));
      value = crc8_next(value, "6");
      for (word_index = 0; word_index < 12; word_index = word_index + 1) begin
        word_value = {
          data_in[word_index * 12 +: 4],
          data_in[word_index * 12 + 4 +: 4],
          data_in[word_index * 12 + 8 +: 4]
        };
        value = crc8_next(value, hex_ascii(word_value[11:8]));
        value = crc8_next(value, hex_ascii(word_value[7:4]));
        value = crc8_next(value, hex_ascii(word_value[3:0]));
      end
      expected_chunk_crc = value;
    end
  endfunction

  function [11:0] chunk_word;
    input [143:0] data_in;
    input integer word_index;
    begin
      chunk_word = {
        data_in[word_index * 12 +: 4],
        data_in[word_index * 12 + 4 +: 4],
        data_in[word_index * 12 + 8 +: 4]
      };
    end
  endfunction

  task send_sample;
    input start_flag;
    input [5:0] quality_in;
    input [14:0] angle_in_q6;
    input [15:0] distance_mm;
    begin
      @(negedge clk);
      new_scan = start_flag;
      quality = quality_in;
      angle_q6 = angle_in_q6;
      distance_q2 = distance_mm << 2;
      sample_valid = 1'b1;
      @(negedge clk);
      sample_valid = 1'b0;
      new_scan = 1'b0;
      // A real C1 node spans five 460800-baud UART bytes (>500 clocks at
      // 50 MHz). Keep enough separation for the RAM lookup/clear pipeline.
      repeat (100) @(posedge clk);
    end
  endtask

  task accept_chunk;
    begin
      @(negedge clk);
      map_chunk_accept = 1'b1;
      @(negedge clk);
      map_chunk_accept = 1'b0;
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    scan_enable = 1'b1;

    // Scan 0. Bin 0 receives three points; the exact 2.5-degree sample must
    // win even though an earlier sample has a shorter distance.
    send_sample(1'b1, 6'd20, 15'd64, 16'd1000);
    send_sample(1'b0, 6'd30, 15'd160, 16'd1200);
    send_sample(1'b0, 6'd45, 15'd480, 16'd2000);
    send_sample(1'b0, 6'd0, 15'd800, 16'd500);
    send_sample(1'b0, 6'd10, 15'd1760, 16'd1600);
    send_sample(1'b0, 6'd40, 15'd22880, 16'd3000);

    // The first point of scan 1 finalizes scan 0 and starts report part 0.
    send_sample(1'b1, 6'd8, 15'd0, 16'd800);

    previous_accept_cycle = -1000;
    for (part_index = 0; part_index < 6; part_index = part_index + 1) begin
      wait (map_chunk_valid === 1'b1);
      if (map_scan_seq !== 8'd0)
        $fatal(1, "unexpected scan sequence %0d", map_scan_seq);
      if (map_part !== part_index[2:0])
        $fatal(1, "unexpected report part %0d, expected %0d", map_part, part_index);
      if ((part_index != 0) && (cycle_count - previous_accept_cycle < 100))
        $fatal(1, "chunk gap shorter than configured minimum");

      expected_crc = expected_chunk_crc(map_scan_seq, map_tick_10ms,
                                        map_part, map_chunk_data);
      if (map_chunk_crc !== expected_crc)
        $fatal(1, "CRC mismatch part=%0d actual=%02x expected=%02x",
               part_index, map_chunk_crc, expected_crc);
      $display("RESULT chunk part=%0d seq=%0d tick=%0d crc=%02x first=%03x last=%03x gap_cycles=%0d",
               map_part, map_scan_seq, map_tick_10ms, map_chunk_crc,
               chunk_word(map_chunk_data, 0), chunk_word(map_chunk_data, 11),
               cycle_count - previous_accept_cycle);

      if (part_index == 0) begin
        if (chunk_word(map_chunk_data, 0) !== 12'h84b)
          $fatal(1, "bin 0 mismatch: %03x", chunk_word(map_chunk_data, 0));
        if (chunk_word(map_chunk_data, 1) !== 12'hc7d)
          $fatal(1, "bin 1 mismatch: %03x", chunk_word(map_chunk_data, 1));
        if (chunk_word(map_chunk_data, 2) !== 12'hfff)
          $fatal(1, "invalid bin 2 mismatch: %03x", chunk_word(map_chunk_data, 2));
        if (chunk_word(map_chunk_data, 5) !== 12'h464)
          $fatal(1, "bin 5 mismatch: %03x", chunk_word(map_chunk_data, 5));
      end
      if ((part_index == 5) &&
          (chunk_word(map_chunk_data, 11) !== 12'hcbc))
        $fatal(1, "bin 71 mismatch: %03x", chunk_word(map_chunk_data, 11));

      previous_accept_cycle = cycle_count;
      accept_chunk();
    end

    repeat (5) @(posedge clk);
    if (map_chunk_valid !== 1'b0)
      $fatal(1, "map chunk remained valid after part 5 acceptance");
    if (map_drop_count !== 16'd0)
      $fatal(1, "unexpected map report drops: %0d", map_drop_count);

    $display("RESULT scan_seq=%0d tick_10ms=%0d parts=6", map_scan_seq, map_tick_10ms);
    $display("RESULT bin0=84B bin1=C7D bin2=FFF bin5=464 bin71=CBC");
    $display("RESULT TEST_PASS lidar_map_sampler");
    $finish;
  end

  initial begin
    #2000000;
    $fatal(1, "timeout waiting for lidar map sampler");
  end
endmodule
