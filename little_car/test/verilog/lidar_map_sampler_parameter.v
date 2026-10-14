`timescale 1ns/1ps
module lidar_map_sampler(
  input wire clk,
  input wire rst_n,
  input wire scan_enable,
  input wire sample_valid,
  input wire new_scan,
  input wire [5:0] quality,
  input wire [14:0] angle_q6,
  input wire [15:0] distance_q2,
  input wire map_chunk_accept,
  output reg map_chunk_valid,
  output reg [7:0] map_scan_seq,
  output reg [15:0] map_tick_10ms,
  output reg [2:0] map_part,
  output reg [143:0] map_chunk_data,
  output reg [7:0] map_chunk_crc,
  output reg [15:0] map_drop_count
);
  parameter integer CLK_HZ = 50000000;







  parameter integer REPORT_INTERVAL_TICKS = 100;
  parameter integer CHUNK_GAP_MS = 40;

  localparam integer TICK_CYCLES = CLK_HZ / 100;
  localparam integer CHUNK_GAP_CYCLES = (CLK_HZ / 1000) * CHUNK_GAP_MS;
  localparam [5:0] MAP_MIN_QUALITY = 6'd8;
  localparam [15:0] MAP_MIN_DISTANCE_Q2 = 16'd200;
  localparam [15:0] MAP_MAX_DISTANCE_Q2 = 16'd48000;
  localparam [14:0] FULL_CIRCLE_Q6 = 15'd23040;

  localparam [1:0] LOOKUP_IDLE = 2'd0;
  localparam [1:0] LOOKUP_WAIT = 2'd1;
  localparam [1:0] LOOKUP_CAPTURE = 2'd2;
  localparam [1:0] LOAD_IDLE = 2'd0;
  localparam [1:0] LOAD_SET_ADDRESS = 2'd1;
  localparam [1:0] LOAD_WAIT = 2'd2;
  localparam [1:0] LOAD_CAPTURE = 2'd3;




  (* ram_style = "block" *) reg [20:0] bank0_mem [0:71];
  (* ram_style = "block" *) reg [20:0] bank1_mem [0:71];
  reg [6:0] bank0_read_address;
  reg [6:0] bank1_read_address;
  reg [20:0] bank0_read_data;
  reg [20:0] bank1_read_data;
  reg bank0_write_enable;
  reg bank1_write_enable;
  reg [6:0] bank0_write_address;
  reg [6:0] bank1_write_address;
  reg [20:0] bank0_write_data;
  reg [20:0] bank1_write_data;

  always @(posedge clk) begin
    if (bank0_write_enable)
      bank0_mem[bank0_write_address] <= bank0_write_data;
    if (bank1_write_enable)
      bank1_mem[bank1_write_address] <= bank1_write_data;
    bank0_read_data <= bank0_mem[bank0_read_address];
    bank1_read_data <= bank1_mem[bank1_read_address];
  end

  reg collect_bank;
  reg report_bank;
  reg scan_seen;
  reg [7:0] current_scan_seq;
  reg [15:0] current_scan_tick;
  reg report_active;
  reg report_started;
  reg [15:0] last_report_tick;

  reg [18:0] tick_divider;
  reg [15:0] tick_10ms;
  reg [21:0] chunk_gap_count;

  reg clear_active;
  reg clear_bank;
  reg [6:0] clear_index;
  reg first_sample_pending;
  reg first_sample_good;
  reg [6:0] first_sample_bin;
  reg [7:0] first_sample_error;
  reg [11:0] first_sample_data;

  reg sample_write_pending;
  reg sample_write_bank;
  reg [6:0] sample_write_bin;
  reg [7:0] sample_write_error;
  reg [11:0] sample_write_data;

  reg [1:0] lookup_state;
  reg lookup_bank;
  reg [6:0] lookup_bin;
  reg [7:0] lookup_error;
  reg [11:0] lookup_data;

  reg [1:0] load_state;
  reg [3:0] load_index;
  reg [7:0] load_crc;

  wire [8:0] angle_degree = angle_q6[14:6];

  wire [16:0] degree_times_205 =
      ({8'd0, angle_degree} << 7) +
      ({8'd0, angle_degree} << 6) +
      ({8'd0, angle_degree} << 3) +
      ({8'd0, angle_degree} << 2) +
      {8'd0, angle_degree};
  wire [6:0] sample_bin = degree_times_205[16:10];
  wire [14:0] sample_bin_center_q6 =
      ({8'd0, sample_bin} << 8) +
      ({8'd0, sample_bin} << 6) + 15'd160;
  wire [14:0] sample_error_full =
      (angle_q6 >= sample_bin_center_q6) ?
      (angle_q6 - sample_bin_center_q6) :
      (sample_bin_center_q6 - angle_q6);
  wire [7:0] sample_error = sample_error_full[7:0];

  wire sample_good = (quality >= MAP_MIN_QUALITY) &&
                     (angle_q6 < FULL_CIRCLE_Q6) &&
                     (distance_q2 >= MAP_MIN_DISTANCE_Q2) &&
                     (distance_q2 <= MAP_MAX_DISTANCE_Q2) &&
                     (sample_bin < 7'd72);
  wire [16:0] rounded_distance_q2 = {1'b0, distance_q2} + 17'd32;
  wire [10:0] rounded_distance_code = rounded_distance_q2[16:6];
  wire [9:0] map_distance_code =
      (rounded_distance_code >= 11'd1023) ? 10'd1022 :
      rounded_distance_code[9:0];
  wire [1:0] map_quality_class =
      (quality >= 6'd40) ? 2'd3 :
      (quality >= 6'd20) ? 2'd2 :
      (quality >= MAP_MIN_QUALITY) ? 2'd1 : 2'd0;
  wire [11:0] packed_sample = {map_quality_class, map_distance_code};

  wire [15:0] ticks_since_report = tick_10ms - last_report_tick;
  wire report_due = !report_started ||
                    (ticks_since_report >= REPORT_INTERVAL_TICKS[15:0]);
  wire start_report = scan_enable && sample_valid && new_scan && scan_seen &&
                      report_due && !report_active;
  wire next_collect_bank = start_report ? ~collect_bank : collect_bank;

  wire [6:0] report_part_base =
      ({4'd0, map_part} << 3) + ({4'd0, map_part} << 2);
  wire [6:0] load_address = report_part_base + {3'd0, load_index};
  wire [20:0] report_read_entry = report_bank ? bank1_read_data : bank0_read_data;
  wire [11:0] load_word = report_read_entry[20] ?
                          report_read_entry[11:0] : 12'hfff;
  wire [20:0] lookup_entry = lookup_bank ? bank1_read_data : bank0_read_data;
  wire lookup_should_write = !lookup_entry[20] ||
                             (lookup_error < lookup_entry[19:12]) ||
                             ((lookup_error == lookup_entry[19:12]) &&
                              (lookup_data[11:10] > lookup_entry[11:10]));

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
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[7])
          value = (value << 1) ^ 8'h07;
        else
          value = value << 1;
      end
      crc8_next = value;
    end
  endfunction

  function [7:0] map_header_crc;
    input [7:0] sequence_in;
    input [15:0] tick_in;
    input [2:0] part_in;
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
      map_header_crc = value;
    end
  endfunction

  wire [7:0] load_crc_after_word =
      crc8_next(
        crc8_next(
          crc8_next(load_crc, hex_ascii(load_word[11:8])),
          hex_ascii(load_word[7:4])),
        hex_ascii(load_word[3:0]));



  always @* begin
    bank0_write_enable = 1'b0;
    bank1_write_enable = 1'b0;
    bank0_write_address = 7'd0;
    bank1_write_address = 7'd0;
    bank0_write_data = 21'd0;
    bank1_write_data = 21'd0;

    if (clear_active) begin
      if (!clear_bank) begin
        bank0_write_enable = 1'b1;
        bank0_write_address = clear_index;
      end else begin
        bank1_write_enable = 1'b1;
        bank1_write_address = clear_index;
      end
    end else if (sample_write_pending) begin
      if (!sample_write_bank) begin
        bank0_write_enable = 1'b1;
        bank0_write_address = sample_write_bin;
        bank0_write_data = {1'b1, sample_write_error, sample_write_data};
      end else begin
        bank1_write_enable = 1'b1;
        bank1_write_address = sample_write_bin;
        bank1_write_data = {1'b1, sample_write_error, sample_write_data};
      end
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tick_divider <= 19'd0;
      tick_10ms <= 16'd0;
    end else if (tick_divider >= TICK_CYCLES - 1) begin
      tick_divider <= 19'd0;
      tick_10ms <= tick_10ms + 16'd1;
    end else begin
      tick_divider <= tick_divider + 19'd1;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bank0_read_address <= 7'd0;
      bank1_read_address <= 7'd0;
      collect_bank <= 1'b0;
      report_bank <= 1'b0;
      scan_seen <= 1'b0;
      current_scan_seq <= 8'd0;
      current_scan_tick <= 16'd0;
      report_active <= 1'b0;
      report_started <= 1'b0;
      last_report_tick <= 16'd0;
      chunk_gap_count <= 22'd0;
      clear_active <= 1'b0;
      clear_bank <= 1'b0;
      clear_index <= 7'd0;
      first_sample_pending <= 1'b0;
      first_sample_good <= 1'b0;
      first_sample_bin <= 7'd0;
      first_sample_error <= 8'd0;
      first_sample_data <= 12'd0;
      sample_write_pending <= 1'b0;
      sample_write_bank <= 1'b0;
      sample_write_bin <= 7'd0;
      sample_write_error <= 8'd0;
      sample_write_data <= 12'd0;
      lookup_state <= LOOKUP_IDLE;
      lookup_bank <= 1'b0;
      lookup_bin <= 7'd0;
      lookup_error <= 8'd0;
      lookup_data <= 12'd0;
      load_state <= LOAD_IDLE;
      load_index <= 4'd0;
      load_crc <= 8'd0;
      map_chunk_valid <= 1'b0;
      map_scan_seq <= 8'd0;
      map_tick_10ms <= 16'd0;
      map_part <= 3'd0;
      map_chunk_data <= 144'd0;
      map_chunk_crc <= 8'd0;
      map_drop_count <= 16'd0;
    end else if (!scan_enable) begin
      collect_bank <= 1'b0;
      scan_seen <= 1'b0;
      current_scan_seq <= 8'd0;
      current_scan_tick <= tick_10ms;
      report_active <= 1'b0;
      report_started <= 1'b0;
      last_report_tick <= tick_10ms;
      chunk_gap_count <= 22'd0;
      clear_active <= 1'b0;
      first_sample_pending <= 1'b0;
      sample_write_pending <= 1'b0;
      lookup_state <= LOOKUP_IDLE;
      load_state <= LOAD_IDLE;
      load_index <= 4'd0;
      load_crc <= 8'd0;
      map_chunk_valid <= 1'b0;
      map_part <= 3'd0;
    end else begin

      if (sample_write_pending)
        sample_write_pending <= 1'b0;

      if (clear_active) begin
        if (clear_index == 7'd71) begin
          clear_active <= 1'b0;
          clear_index <= 7'd0;
          if (first_sample_pending && first_sample_good) begin
            sample_write_pending <= 1'b1;
            sample_write_bank <= clear_bank;
            sample_write_bin <= first_sample_bin;
            sample_write_error <= first_sample_error;
            sample_write_data <= first_sample_data;
          end
          first_sample_pending <= 1'b0;
        end else begin
          clear_index <= clear_index + 7'd1;
        end
      end

      if (chunk_gap_count != 0) begin
        chunk_gap_count <= chunk_gap_count - 22'd1;
        if (chunk_gap_count == 22'd1) begin
          load_state <= LOAD_SET_ADDRESS;
          load_index <= 4'd0;
          load_crc <= map_header_crc(map_scan_seq, map_tick_10ms, map_part);
        end
      end

      case (load_state)
        LOAD_SET_ADDRESS: begin
          if (!report_bank)
            bank0_read_address <= load_address;
          else
            bank1_read_address <= load_address;
          load_state <= LOAD_WAIT;
        end
        LOAD_WAIT: load_state <= LOAD_CAPTURE;
        LOAD_CAPTURE: begin


          map_chunk_data[load_index * 12 +: 4] <= load_word[11:8];
          map_chunk_data[load_index * 12 + 4 +: 4] <= load_word[7:4];
          map_chunk_data[load_index * 12 + 8 +: 4] <= load_word[3:0];
          load_crc <= load_crc_after_word;
          if (load_index == 4'd11) begin
            load_state <= LOAD_IDLE;
            load_index <= 4'd0;
            map_chunk_crc <= load_crc_after_word;
            map_chunk_valid <= 1'b1;
          end else begin
            load_index <= load_index + 4'd1;
            load_state <= LOAD_SET_ADDRESS;
          end
        end
        default: begin end
      endcase

      if (map_chunk_accept && map_chunk_valid) begin
        map_chunk_valid <= 1'b0;
        if (map_part == 3'd5) begin
          report_active <= 1'b0;
          map_part <= 3'd0;
        end else begin
          map_part <= map_part + 3'd1;
          if (CHUNK_GAP_CYCLES <= 1) begin
            load_state <= LOAD_SET_ADDRESS;
            load_index <= 4'd0;
            load_crc <= map_header_crc(map_scan_seq, map_tick_10ms,
                                       map_part + 3'd1);
          end else begin
            chunk_gap_count <= CHUNK_GAP_CYCLES[21:0];
          end
        end
      end

      case (lookup_state)
        LOOKUP_WAIT: lookup_state <= LOOKUP_CAPTURE;
        LOOKUP_CAPTURE: begin
          lookup_state <= LOOKUP_IDLE;
          if (lookup_should_write && !sample_write_pending && !clear_active) begin
            sample_write_pending <= 1'b1;
            sample_write_bank <= lookup_bank;
            sample_write_bin <= lookup_bin;
            sample_write_error <= lookup_error;
            sample_write_data <= lookup_data;
          end
        end
        default: begin end
      endcase

      if (sample_valid && new_scan) begin
        lookup_state <= LOOKUP_IDLE;
        sample_write_pending <= 1'b0;
        clear_active <= 1'b1;
        clear_bank <= next_collect_bank;
        clear_index <= 7'd0;
        first_sample_pending <= 1'b1;
        first_sample_good <= sample_good;
        first_sample_bin <= sample_bin;
        first_sample_error <= sample_error;
        first_sample_data <= packed_sample;
        collect_bank <= next_collect_bank;

        if (scan_seen) begin
          current_scan_seq <= current_scan_seq + 8'd1;
          current_scan_tick <= tick_10ms;
        end else begin
          scan_seen <= 1'b1;
          current_scan_seq <= 8'd0;
          current_scan_tick <= tick_10ms;
        end

        if (start_report) begin
          report_bank <= collect_bank;
          report_active <= 1'b1;
          report_started <= 1'b1;
          last_report_tick <= tick_10ms;
          map_scan_seq <= current_scan_seq;
          map_tick_10ms <= current_scan_tick;
          map_part <= 3'd0;
          map_chunk_valid <= 1'b0;
          load_state <= LOAD_SET_ADDRESS;
          load_index <= 4'd0;
          load_crc <= map_header_crc(current_scan_seq, current_scan_tick, 3'd0);
          chunk_gap_count <= 22'd0;
        end else if (scan_seen && report_due && report_active &&
                     (map_drop_count != 16'hffff)) begin
          map_drop_count <= map_drop_count + 16'd1;
        end
      end else if (sample_valid && scan_seen && sample_good &&
                   !clear_active && !sample_write_pending &&
                   (lookup_state == LOOKUP_IDLE)) begin
        lookup_bank <= collect_bank;
        lookup_bin <= sample_bin;
        lookup_error <= sample_error;
        lookup_data <= packed_sample;
        if (!collect_bank)
          bank0_read_address <= sample_bin;
        else
          bank1_read_address <= sample_bin;
        lookup_state <= LOOKUP_WAIT;
      end
    end
  end
endmodule

module lidar_map_sampler_parameter;
  reg clk;
  reg rst_n;
  reg scan_enable;
  reg sample_valid;
  reg new_scan;
  reg [5:0] quality;
  reg [14:0] angle_q6;
  reg [15:0] distance_q2;
  reg map_chunk_accept;
  wire map_chunk_valid;
  wire [7:0] map_scan_seq;
  wire [15:0] map_tick_10ms;
  wire [2:0] map_part;
  wire [143:0] map_chunk_data;
  wire [7:0] map_chunk_crc;
  wire [15:0] map_drop_count;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_lidar_map_sampler.vcd");
  $dumpvars(0, rst_n, scan_enable, sample_valid, new_scan, quality, angle_q6, distance_q2, map_chunk_valid, map_scan_seq, map_part, map_chunk_crc, map_drop_count);
end
`endif

initial begin
  clk = 1'b0;
  rst_n = 1'b0;
  scan_enable = 1'b0;
  sample_valid = 1'b0;
  new_scan = 1'b0;
  quality = 6'd0;
  angle_q6 = 15'd0;
  distance_q2 = 16'd0;
  map_chunk_accept = 1'b0;
end

integer cycle_count = 0;
  integer part_index;
  integer previous_accept_cycle;
  reg [7:0] expected_crc;

  always #10 clk = ~clk;
  always @(posedge clk) cycle_count <= cycle_count + 1;



  defparam lidar_map_sampler0.CLK_HZ = 100000;
  defparam lidar_map_sampler0.REPORT_INTERVAL_TICKS = 2;
  defparam lidar_map_sampler0.CHUNK_GAP_MS = 1;

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


  lidar_map_sampler lidar_map_sampler0(
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
endmodule
