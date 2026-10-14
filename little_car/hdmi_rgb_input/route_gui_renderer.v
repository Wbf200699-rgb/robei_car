module route_gui_renderer(
input wire clk_pix,
input wire rst_n,
input wire [11:0] pixel_x,
input wire [11:0] pixel_y,
input wire pixel_de,
input wire [1:0] menu_info,
input wire [4:0] state_info,
input wire [63:0] route_info,
input wire [63:0] old_route_info,
input wire [5:0] active_route_steps,
input wire [5:0] completed_route_steps,
input wire [15:0] step_length,
input wire [15:0] speed,
input wire [15:0] total_route_length,
input wire [15:0] complete_route_length,
input wire [15:0] remaining_route_length,
input wire [7:0] now_x,
input wire [7:0] now_y,
input wire [7:0] target_x,
input wire [7:0] target_y,
output reg [23:0] rgb_data
);

  localparam [1:0] MENU_FOCUS_CHANGE = 2'b00;
  localparam [1:0] MENU_IN_CHANGE    = 2'b01;
  localparam [1:0] MENU_FOCUS_RUN    = 2'b10;
  localparam [1:0] MENU_IN_RUN       = 2'b11;

  localparam [4:0] STATE_MENU   = 5'b00000;
  localparam [4:0] STATE_CHANGE = 5'b00001;
  localparam [4:0] STATE_RUN    = 5'b00011;
  localparam [4:0] STATE_STOP   = 5'b00100;

  localparam [1:0] ROUTE_STOP    = 2'b00;
  localparam [1:0] ROUTE_RIGHT   = 2'b01;
  localparam [1:0] ROUTE_LEFT    = 2'b10;
  localparam [1:0] ROUTE_FORWARD = 2'b11;

  localparam [1:0] DIR_UP    = 2'd0;
  localparam [1:0] DIR_RIGHT = 2'd1;
  localparam [1:0] DIR_DOWN  = 2'd2;
  localparam [1:0] DIR_LEFT  = 2'd3;

  localparam integer ROUTE_WORDS = 16;

  localparam [4:0] WORD_MENU   = 5'd0;
  localparam [4:0] WORD_EDIT   = 5'd1;
  localparam [4:0] WORD_RUN    = 5'd2;
  localparam [4:0] WORD_STOP   = 5'd3;
  localparam [4:0] WORD_CHANGE = 5'd4;
  localparam [4:0] WORD_TOTAL  = 5'd5;
  localparam [4:0] WORD_DONE   = 5'd6;
  localparam [4:0] WORD_LEFT   = 5'd7;
  localparam [4:0] WORD_SPEED  = 5'd8;
  localparam [4:0] WORD_ACT    = 5'd9;
  localparam [4:0] WORD_STEP   = 5'd10;
  localparam [4:0] WORD_NX     = 5'd11;
  localparam [4:0] WORD_NY     = 5'd12;
  localparam [4:0] WORD_TX     = 5'd13;
  localparam [4:0] WORD_TY     = 5'd14;
  localparam [4:0] WORD_RT     = 5'd15;
  localparam [4:0] WORD_OLD    = 5'd16;
  localparam [4:0] WORD_NEXT   = 5'd17;
  localparam [4:0] WORD_PATH   = 5'd18;

  integer i;
  integer cur_x;
  integer cur_y;
  integer next_x;
  integer next_y;

  reg [1:0] cmd;
  reg [1:0] dir;

  reg [6:0] step_pix;
  reg [8:0] total_bar_w;
  reg [8:0] done_bar_w;
  reg [8:0] remain_bar_w;
  reg [8:0] speed_bar_w;
  reg [11:0] car_x;
  reg [11:0] car_y;
  reg [1:0] car_dir;
  reg [11:0] target_draw_x;
  reg [11:0] target_draw_y;
  reg [11:0] stop_seg_sx;
  reg [11:0] stop_seg_sy;
  reg [11:0] stop_seg_ex;
  reg [11:0] stop_seg_ey;
  reg stop_seg_valid;

  reg [11:0] old_seg_sx [0:ROUTE_WORDS-1];
  reg [11:0] old_seg_sy [0:ROUTE_WORDS-1];
  reg [11:0] old_seg_ex [0:ROUTE_WORDS-1];
  reg [11:0] old_seg_ey [0:ROUTE_WORDS-1];
  reg [11:0] active_seg_sx [0:ROUTE_WORDS-1];
  reg [11:0] active_seg_sy [0:ROUTE_WORDS-1];
  reg [11:0] active_seg_ex [0:ROUTE_WORDS-1];
  reg [11:0] active_seg_ey [0:ROUTE_WORDS-1];
  reg [ROUTE_WORDS-1:0] old_seg_valid;
  reg [ROUTE_WORDS-1:0] active_seg_valid;

  reg old_route_pixel;
  reg active_route_pixel;
  reg stop_line_pixel;
  reg [23:0] rgb_next;
  reg grid_x_line;
  reg grid_y_line;

  reg txt_state0;
  reg txt_state1;
  reg txt_state2;
  reg txt_state3;
  reg txt_change_box;
  reg txt_run_box;
  reg txt_info_dark;
  reg txt_route_dark;
  reg txt_route_hex;
  reg txt_menu_hint;
  reg txt_route_hint;

  integer text_idx;
  integer text_base_x;
  reg [7:0] text_char;

  wire status_panel;
  wire menu_panel;
  wire route_panel;
  wire change_box;
  wire run_box;
  wire map_area;
  wire border_line;
  wire grid_line;
  wire change_select;
  wire run_select;
  wire state_stop;
  wire state_menu_sel;
  wire state_change_sel;
  wire state_run_sel;
  wire state_stop_sel;
  wire state_bar_0;
  wire state_bar_1;
  wire state_bar_2;
  wire state_bar_3;
  wire info_bar_total;
  wire info_bar_done;
  wire info_bar_remain;
  wire info_bar_speed;
  wire car_body;
  wire car_arrow;
  wire target_marker;
  wire now_marker;

  function integer calc_step_pix;
    input [5:0] steps;
    begin
      if (steps <= 4)
        calc_step_pix = 58;
      else if (steps <= 8)
        calc_step_pix = 44;
      else if (steps <= 12)
        calc_step_pix = 34;
      else if (steps <= 18)
        calc_step_pix = 26;
      else if (steps <= 24)
        calc_step_pix = 20;
      else
        calc_step_pix = 16;
    end
  endfunction

  function integer clamp_bar_w;
    input [15:0] value;
    integer bar_w;
    begin
      bar_w = value;
      if (bar_w > 320)
        bar_w = 320;
      if (bar_w < 0)
        bar_w = 0;
      clamp_bar_w = bar_w;
    end
  endfunction

  function integer scaled_offset;
    input [7:0] value;
    input integer pix;
    integer diff;
    integer mag;
    integer scaled;
    begin
      if (value >= 8'd128)
        mag = value - 8'd128;
      else
        mag = 8'd128 - value;

      case (pix)
        58: scaled = (mag << 5) + (mag << 4) + (mag << 3) + (mag << 1);
        44: scaled = (mag << 5) + (mag << 3) + (mag << 2);
        34: scaled = (mag << 5) + (mag << 1);
        26: scaled = (mag << 4) + (mag << 3) + (mag << 1);
        20: scaled = (mag << 4) + (mag << 2);
        default: scaled = mag << 4;
      endcase

      if (value >= 8'd128)
        diff = scaled;
      else
        diff = 0 - scaled;

      scaled_offset = diff;
    end
  endfunction

  function segment_hit;
    input integer sx;
    input integer sy;
    input integer ex;
    input integer ey;
    input integer width;
    integer lx;
    integer hx;
    integer ly;
    integer hy;
    begin
      if (sx < ex) begin lx = sx; hx = ex; end else begin lx = ex; hx = sx; end
      if (sy < ey) begin ly = sy; hy = ey; end else begin ly = ey; hy = sy; end
      segment_hit = (pixel_x >= lx - width) && (pixel_x <= hx + width) &&
                    (pixel_y >= ly - width) && (pixel_y <= hy + width) &&
                    (((sx == ex) && (pixel_x >= sx - width) && (pixel_x <= sx + width)) ||
                     ((sy == ey) && (pixel_y >= sy - width) && (pixel_y <= sy + width)));
    end
  endfunction

  function glyph5x7_2x_hit;
    input [7:0] ch;
    input [11:0] px;
    input [11:0] py;
    input integer base_x;
    input integer base_y;
    reg [2:0] row;
    reg [2:0] col;
    reg [4:0] bits;
    begin
      glyph5x7_2x_hit = 1'b0;
      bits = 5'b00000;
      if ((ch != " ") &&
          (px >= base_x) && (px < base_x + 10) &&
          (py >= base_y) && (py < base_y + 14)) begin
        row = (py - base_y) >> 1;
        case (ch)
          "0": case (row) 0: bits=5'b01110; 1: bits=5'b10011; 2: bits=5'b10101; 3: bits=5'b11001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
          "1": case (row) 0: bits=5'b00100; 1: bits=5'b01100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b01110; endcase
          "2": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b00001; 3: bits=5'b00010; 4: bits=5'b00100; 5: bits=5'b01000; default: bits=5'b11111; endcase
          "3": case (row) 0: bits=5'b11110; 1: bits=5'b00001; 2: bits=5'b00001; 3: bits=5'b01110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
          "4": case (row) 0: bits=5'b00010; 1: bits=5'b00110; 2: bits=5'b01010; 3: bits=5'b10010; 4: bits=5'b11111; 5: bits=5'b00010; default: bits=5'b00010; endcase
          "5": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b11110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
          "6": case (row) 0: bits=5'b01110; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b11110; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
          "7": case (row) 0: bits=5'b11111; 1: bits=5'b00001; 2: bits=5'b00010; 3: bits=5'b00100; 4: bits=5'b01000; 5: bits=5'b01000; default: bits=5'b01000; endcase
          "8": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b01110; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
          "9": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b01111; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b01110; endcase
          "A": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11111; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
          "C": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b01111; endcase
          "D": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b11110; endcase
          "E": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b11110; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b11111; endcase
          "F": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b11110; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b10000; endcase
          "G": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10111; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01111; endcase
          "H": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b11111; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
          "I": case (row) 0: bits=5'b11111; 1: bits=5'b00100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b11111; endcase
          "L": case (row) 0: bits=5'b10000; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b11111; endcase
          "M": case (row) 0: bits=5'b10001; 1: bits=5'b11011; 2: bits=5'b10101; 3: bits=5'b10101; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
          "N": case (row) 0: bits=5'b10001; 1: bits=5'b11001; 2: bits=5'b10101; 3: bits=5'b10011; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
          "O": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
          "P": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11110; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b10000; endcase
          "R": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11110; 4: bits=5'b10100; 5: bits=5'b10010; default: bits=5'b10001; endcase
          "S": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b01110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
          "T": case (row) 0: bits=5'b11111; 1: bits=5'b00100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b00100; endcase
          "U": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
          "W": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10101; 4: bits=5'b10101; 5: bits=5'b10101; default: bits=5'b01010; endcase
          "X": case (row) 0: bits=5'b10001; 1: bits=5'b01010; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b01010; default: bits=5'b10001; endcase
          "Y": case (row) 0: bits=5'b10001; 1: bits=5'b01010; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b00100; endcase
          default: bits = 5'b00000;
        endcase
        col = 4 - ((px - base_x) >> 1);
        glyph5x7_2x_hit = bits[col];
      end
    end
  endfunction

  function [7:0] dec5_char;
    input [15:0] value;
    input [2:0] idx;
    integer digit;
    integer rem;
    integer d0;
    integer d1;
    integer d2;
    integer d3;
    integer d4;
    integer n;
    begin
      rem = value;
      d0 = 0;
      d1 = 0;
      d2 = 0;
      d3 = 0;
      d4 = 0;
      for (n = 0; n < 6; n = n + 1)
        if (rem >= 10000) begin rem = rem - 10000; d0 = d0 + 1; end
      for (n = 0; n < 10; n = n + 1)
        if (rem >= 1000) begin rem = rem - 1000; d1 = d1 + 1; end
      for (n = 0; n < 10; n = n + 1)
        if (rem >= 100) begin rem = rem - 100; d2 = d2 + 1; end
      for (n = 0; n < 10; n = n + 1)
        if (rem >= 10) begin rem = rem - 10; d3 = d3 + 1; end
      d4 = rem;
      case (idx)
        3'd0: digit = d0;
        3'd1: digit = d1;
        3'd2: digit = d2;
        3'd3: digit = d3;
        default: digit = d4;
      endcase
      dec5_char = 8'd48 + digit;
    end
  endfunction

  function [7:0] dec3_char;
    input [7:0] value;
    input [1:0] idx;
    integer digit;
    integer rem;
    integer d0;
    integer d1;
    integer d2;
    integer n;
    begin
      rem = value;
      d0 = 0;
      d1 = 0;
      d2 = 0;
      for (n = 0; n < 2; n = n + 1)
        if (rem >= 100) begin rem = rem - 100; d0 = d0 + 1; end
      for (n = 0; n < 10; n = n + 1)
        if (rem >= 10) begin rem = rem - 10; d1 = d1 + 1; end
      d2 = rem;
      case (idx)
        2'd0: digit = d0;
        2'd1: digit = d1;
        default: digit = d2;
      endcase
      dec3_char = 8'd48 + digit;
    end
  endfunction

  function [7:0] dec2_char;
    input [7:0] value;
    input integer idx;
    integer digit;
    integer rem;
    integer d0;
    integer d1;
    integer n;
    begin
      rem = value;
      d0 = 0;
      d1 = 0;
      for (n = 0; n < 10; n = n + 1)
        if (rem >= 10) begin rem = rem - 10; d0 = d0 + 1; end
      d1 = rem;
      if (idx == 0)
        digit = d0;
      else
        digit = d1;
      dec2_char = 8'd48 + digit;
    end
  endfunction

  function integer slot12;
    input integer offset;
    begin
      if (offset < 12)
        slot12 = 0;
      else if (offset < 24)
        slot12 = 1;
      else if (offset < 36)
        slot12 = 2;
      else if (offset < 48)
        slot12 = 3;
      else if (offset < 60)
        slot12 = 4;
      else if (offset < 72)
        slot12 = 5;
      else if (offset < 84)
        slot12 = 6;
      else if (offset < 96)
        slot12 = 7;
      else if (offset < 108)
        slot12 = 8;
      else if (offset < 120)
        slot12 = 9;
      else if (offset < 132)
        slot12 = 10;
      else if (offset < 144)
        slot12 = 11;
      else if (offset < 156)
        slot12 = 12;
      else if (offset < 168)
        slot12 = 13;
      else if (offset < 180)
        slot12 = 14;
      else
        slot12 = 15;
    end
  endfunction

  function [7:0] hex16_char;
    input [63:0] value;
    input integer idx;
    reg [3:0] nibble;
    begin
      nibble = value[((15 - idx) * 4) +: 4];
      if (nibble < 10)
        hex16_char = 8'd48 + nibble;
      else
        hex16_char = 8'd55 + nibble;
    end
  endfunction

  function [7:0] text_word_char;
    input [4:0] word;
    input integer idx;
    begin
      text_word_char = " ";
      case (word)
        WORD_MENU:   case (idx) 0: text_word_char="M"; 1: text_word_char="E"; 2: text_word_char="N"; 3: text_word_char="U"; endcase
        WORD_EDIT:   case (idx) 0: text_word_char="E"; 1: text_word_char="D"; 2: text_word_char="I"; 3: text_word_char="T"; endcase
        WORD_RUN:    case (idx) 0: text_word_char="R"; 1: text_word_char="U"; 2: text_word_char="N"; endcase
        WORD_STOP:   case (idx) 0: text_word_char="S"; 1: text_word_char="T"; 2: text_word_char="O"; 3: text_word_char="P"; endcase
        WORD_CHANGE: case (idx) 0: text_word_char="C"; 1: text_word_char="H"; 2: text_word_char="A"; 3: text_word_char="N"; 4: text_word_char="G"; 5: text_word_char="E"; endcase
        WORD_TOTAL:  case (idx) 0: text_word_char="T"; 1: text_word_char="O"; 2: text_word_char="T"; 3: text_word_char="A"; 4: text_word_char="L"; endcase
        WORD_DONE:   case (idx) 0: text_word_char="D"; 1: text_word_char="O"; 2: text_word_char="N"; 3: text_word_char="E"; endcase
        WORD_LEFT:   case (idx) 0: text_word_char="L"; 1: text_word_char="E"; 2: text_word_char="F"; 3: text_word_char="T"; endcase
        WORD_SPEED:  case (idx) 0: text_word_char="S"; 1: text_word_char="P"; 2: text_word_char="E"; 3: text_word_char="E"; 4: text_word_char="D"; endcase
        WORD_ACT:    case (idx) 0: text_word_char="A"; 1: text_word_char="C"; 2: text_word_char="T"; endcase
        WORD_STEP:   case (idx) 0: text_word_char="S"; 1: text_word_char="T"; 2: text_word_char="E"; 3: text_word_char="P"; endcase
        WORD_NX:     case (idx) 0: text_word_char="N"; 1: text_word_char="X"; endcase
        WORD_NY:     case (idx) 0: text_word_char="N"; 1: text_word_char="Y"; endcase
        WORD_TX:     case (idx) 0: text_word_char="T"; 1: text_word_char="X"; endcase
        WORD_TY:     case (idx) 0: text_word_char="T"; 1: text_word_char="Y"; endcase
        WORD_RT:     case (idx) 0: text_word_char="R"; 1: text_word_char="T"; endcase
        WORD_OLD:    case (idx) 0: text_word_char="O"; 1: text_word_char="L"; 2: text_word_char="D"; endcase
        WORD_NEXT:   case (idx) 0: text_word_char="N"; 1: text_word_char="E"; 2: text_word_char="X"; 3: text_word_char="T"; endcase
        WORD_PATH:   case (idx) 0: text_word_char="P"; 1: text_word_char="A"; 2: text_word_char="T"; 3: text_word_char="H"; endcase
      endcase
    end
  endfunction

  function [7:0] cmd_word_char;
    input [1:0] route_cmd;
    input integer idx;
    begin
      cmd_word_char = " ";
      case (route_cmd)
        ROUTE_LEFT:
          case (idx) 0: cmd_word_char="L"; 1: cmd_word_char="E"; 2: cmd_word_char="F"; 3: cmd_word_char="T"; endcase
        ROUTE_RIGHT:
          case (idx) 0: cmd_word_char="R"; 1: cmd_word_char="I"; 2: cmd_word_char="G"; 3: cmd_word_char="H"; endcase
        ROUTE_FORWARD:
          case (idx) 0: cmd_word_char="F"; 1: cmd_word_char="W"; 2: cmd_word_char="D"; 3: cmd_word_char=" "; endcase
        default:
          case (idx) 0: cmd_word_char="S"; 1: cmd_word_char="T"; 2: cmd_word_char="O"; 3: cmd_word_char="P"; endcase
      endcase
    end
  endfunction

  assign state_stop = (state_info == STATE_STOP);
  assign state_menu_sel   = (state_info == STATE_MENU);
  assign state_change_sel = (state_info == STATE_CHANGE);
  assign state_run_sel    = (state_info == STATE_RUN);
  assign state_stop_sel   = (state_info == STATE_STOP);

  assign status_panel = (pixel_x >= 12'd20) && (pixel_x < 12'd1000) &&
                        (pixel_y >= 12'd16) && (pixel_y < 12'd128);
  assign menu_panel  = (pixel_x >= 12'd40)  && (pixel_x < 12'd330) &&
                       (pixel_y >= 12'd150) && (pixel_y < 12'd590);
  assign route_panel = (pixel_x >= 12'd360) && (pixel_x < 12'd990) &&
                       (pixel_y >= 12'd150) && (pixel_y < 12'd590);
  assign change_box = (pixel_x >= 12'd78) && (pixel_x < 12'd292) &&
                      (pixel_y >= 12'd230) && (pixel_y < 12'd305);
  assign run_box    = (pixel_x >= 12'd78) && (pixel_x < 12'd292) &&
                      (pixel_y >= 12'd360) && (pixel_y < 12'd435);
  assign map_area = (pixel_x >= 12'd405) && (pixel_x < 12'd945) &&
                    (pixel_y >= 12'd180) && (pixel_y < 12'd560);

  assign change_select = (menu_info == MENU_FOCUS_CHANGE) || (menu_info == MENU_IN_CHANGE);
  assign run_select = (menu_info == MENU_FOCUS_RUN) || (menu_info == MENU_IN_RUN);

  assign border_line =
      ((pixel_x == 12'd20 || pixel_x == 12'd999) && (pixel_y >= 12'd16 && pixel_y < 12'd128)) ||
      ((pixel_y == 12'd16 || pixel_y == 12'd127) && (pixel_x >= 12'd20 && pixel_x < 12'd1000)) ||
      ((pixel_x == 12'd40 || pixel_x == 12'd329) && (pixel_y >= 12'd150 && pixel_y < 12'd590)) ||
      ((pixel_y == 12'd150 || pixel_y == 12'd589) && (pixel_x >= 12'd40 && pixel_x < 12'd330)) ||
      ((pixel_x == 12'd360 || pixel_x == 12'd989) && (pixel_y >= 12'd150 && pixel_y < 12'd590)) ||
      ((pixel_y == 12'd150 || pixel_y == 12'd589) && (pixel_x >= 12'd360 && pixel_x < 12'd990)) ||
      ((pixel_x == 12'd78 || pixel_x == 12'd291) && (pixel_y >= 12'd230 && pixel_y < 12'd305)) ||
      ((pixel_y == 12'd230 || pixel_y == 12'd304) && (pixel_x >= 12'd78 && pixel_x < 12'd292)) ||
      ((pixel_x == 12'd78 || pixel_x == 12'd291) && (pixel_y >= 12'd360 && pixel_y < 12'd435)) ||
      ((pixel_y == 12'd360 || pixel_y == 12'd434) && (pixel_x >= 12'd78 && pixel_x < 12'd292)) ||
      ((pixel_x == 12'd405 || pixel_x == 12'd944) && (pixel_y >= 12'd180 && pixel_y < 12'd560)) ||
      ((pixel_y == 12'd180 || pixel_y == 12'd559) && (pixel_x >= 12'd405 && pixel_x < 12'd945));

  assign grid_line = map_area && (grid_x_line || grid_y_line);

  assign state_bar_0 = status_panel &&
                       (pixel_x >= 12'd52) && (pixel_x < 12'd172) &&
                       (pixel_y >= 12'd34) && (pixel_y < 12'd56);
  assign state_bar_1 = status_panel &&
                       (pixel_x >= 12'd192) && (pixel_x < 12'd312) &&
                       (pixel_y >= 12'd34) && (pixel_y < 12'd56);
  assign state_bar_2 = status_panel &&
                       (pixel_x >= 12'd332) && (pixel_x < 12'd452) &&
                       (pixel_y >= 12'd34) && (pixel_y < 12'd56);
  assign state_bar_3 = status_panel &&
                       (pixel_x >= 12'd472) && (pixel_x < 12'd592) &&
                       (pixel_y >= 12'd34) && (pixel_y < 12'd56);

  assign info_bar_total = status_panel &&
                          (pixel_x >= 12'd640) && (pixel_x < 12'd640 + total_bar_w) &&
                          (pixel_y >= 12'd30) && (pixel_y < 12'd42);
  assign info_bar_done = status_panel &&
                         (pixel_x >= 12'd640) && (pixel_x < 12'd640 + done_bar_w) &&
                         (pixel_y >= 12'd50) && (pixel_y < 12'd62);
  assign info_bar_remain = status_panel &&
                           (pixel_x >= 12'd640) && (pixel_x < 12'd640 + remain_bar_w) &&
                           (pixel_y >= 12'd70) && (pixel_y < 12'd82);
  assign info_bar_speed = status_panel &&
                          (pixel_x >= 12'd640) && (pixel_x < 12'd640 + speed_bar_w) &&
                          (pixel_y >= 12'd90) && (pixel_y < 12'd102);

  assign car_body = (pixel_x >= car_x - 8) && (pixel_x <= car_x + 8) &&
                    (pixel_y >= car_y - 8) && (pixel_y <= car_y + 8);

  assign car_arrow =
      (car_dir == DIR_UP)    ? ((pixel_x >= car_x - 2) && (pixel_x <= car_x + 2) &&
                                (pixel_y >= car_y - 18) && (pixel_y <= car_y - 8)) :
      (car_dir == DIR_RIGHT) ? ((pixel_x >= car_x + 8) && (pixel_x <= car_x + 18) &&
                                (pixel_y >= car_y - 2) && (pixel_y <= car_y + 2)) :
      (car_dir == DIR_DOWN)  ? ((pixel_x >= car_x - 2) && (pixel_x <= car_x + 2) &&
                                (pixel_y >= car_y + 8) && (pixel_y <= car_y + 18)) :
                               ((pixel_x >= car_x - 18) && (pixel_x <= car_x - 8) &&
                                (pixel_y >= car_y - 2) && (pixel_y <= car_y + 2));

  assign target_marker =
      ((pixel_x >= target_draw_x - 5) && (pixel_x <= target_draw_x + 5) &&
       (pixel_y >= target_draw_y - 1) && (pixel_y <= target_draw_y + 1)) ||
      ((pixel_x >= target_draw_x - 1) && (pixel_x <= target_draw_x + 1) &&
       (pixel_y >= target_draw_y - 5) && (pixel_y <= target_draw_y + 5));

  assign now_marker =
      ((pixel_x >= 12'd670) && (pixel_x <= 12'd680) &&
       (pixel_y >= 12'd365) && (pixel_y <= 12'd375)) ||
      ((pixel_x >= 12'd673) && (pixel_x <= 12'd677) &&
       (pixel_y >= 12'd362) && (pixel_y <= 12'd378));

  always @(posedge clk_pix or negedge rst_n) begin
    if (!rst_n) begin
      step_pix <= 7'd16;
      total_bar_w <= 9'd0;
      done_bar_w <= 9'd0;
      remain_bar_w <= 9'd0;
      speed_bar_w <= 9'd0;
      car_x <= 12'd675;
      car_y <= 12'd370;
      car_dir <= DIR_UP;
      target_draw_x <= 12'd675;
      target_draw_y <= 12'd370;
      stop_seg_sx <= 12'd0;
      stop_seg_sy <= 12'd0;
      stop_seg_ex <= 12'd0;
      stop_seg_ey <= 12'd0;
      stop_seg_valid <= 1'b0;
      old_seg_valid <= {ROUTE_WORDS{1'b0}};
      active_seg_valid <= {ROUTE_WORDS{1'b0}};
      for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
        old_seg_sx[i] <= 12'd0;
        old_seg_sy[i] <= 12'd0;
        old_seg_ex[i] <= 12'd0;
        old_seg_ey[i] <= 12'd0;
        active_seg_sx[i] <= 12'd0;
        active_seg_sy[i] <= 12'd0;
        active_seg_ex[i] <= 12'd0;
        active_seg_ey[i] <= 12'd0;
      end
    end else begin
      step_pix <= calc_step_pix(active_route_steps);
      total_bar_w <= clamp_bar_w(total_route_length[15:3]);
      done_bar_w <= clamp_bar_w(complete_route_length[15:3]);
      remain_bar_w <= clamp_bar_w(remaining_route_length[15:3]);
      speed_bar_w <= clamp_bar_w(speed[15:1]);

      target_draw_x <= 675 + scaled_offset(target_x, calc_step_pix(active_route_steps));
      target_draw_y <= 370 + scaled_offset(target_y, calc_step_pix(active_route_steps));
      stop_seg_valid <= 1'b0;
      old_seg_valid <= {ROUTE_WORDS{1'b0}};
      active_seg_valid <= {ROUTE_WORDS{1'b0}};

      cur_x = 675;
      cur_y = 370;
      dir = DIR_UP;

      for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
        old_seg_sx[i] <= 12'd0;
        old_seg_sy[i] <= 12'd0;
        old_seg_ex[i] <= 12'd0;
        old_seg_ey[i] <= 12'd0;

        cmd = old_route_info[i*2 +: 2];
        if ((cmd != ROUTE_STOP) && (i < completed_route_steps)) begin
          if (cmd == ROUTE_RIGHT)
            dir = dir + 2'd1;
          else if (cmd == ROUTE_LEFT)
            dir = dir - 2'd1;
          else begin
            next_x = cur_x;
            next_y = cur_y;
            case (dir)
              DIR_UP:    next_y = cur_y - calc_step_pix(active_route_steps);
              DIR_RIGHT: next_x = cur_x + calc_step_pix(active_route_steps);
              DIR_DOWN:  next_y = cur_y + calc_step_pix(active_route_steps);
              default:   next_x = cur_x - calc_step_pix(active_route_steps);
            endcase
            old_seg_valid[i] <= 1'b1;
            old_seg_sx[i] <= cur_x[11:0];
            old_seg_sy[i] <= cur_y[11:0];
            old_seg_ex[i] <= next_x[11:0];
            old_seg_ey[i] <= next_y[11:0];
            cur_x = next_x;
            cur_y = next_y;
          end
        end
      end

      car_x <= cur_x[11:0];
      car_y <= cur_y[11:0];
      car_dir <= dir;

      for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
        active_seg_sx[i] <= 12'd0;
        active_seg_sy[i] <= 12'd0;
        active_seg_ex[i] <= 12'd0;
        active_seg_ey[i] <= 12'd0;

        cmd = route_info[i*2 +: 2];
        if ((cmd != ROUTE_STOP) && (i < active_route_steps)) begin
          if (cmd == ROUTE_RIGHT)
            dir = dir + 2'd1;
          else if (cmd == ROUTE_LEFT)
            dir = dir - 2'd1;
          else begin
            next_x = cur_x;
            next_y = cur_y;
            case (dir)
              DIR_UP:    next_y = cur_y - calc_step_pix(active_route_steps);
              DIR_RIGHT: next_x = cur_x + calc_step_pix(active_route_steps);
              DIR_DOWN:  next_y = cur_y + calc_step_pix(active_route_steps);
              default:   next_x = cur_x - calc_step_pix(active_route_steps);
            endcase
            active_seg_valid[i] <= 1'b1;
            active_seg_sx[i] <= cur_x[11:0];
            active_seg_sy[i] <= cur_y[11:0];
            active_seg_ex[i] <= next_x[11:0];
            active_seg_ey[i] <= next_y[11:0];
            cur_x = next_x;
            cur_y = next_y;
          end
        end
      end

      if (state_info == STATE_STOP) begin
        stop_seg_valid <= 1'b1;
        case (car_dir)
          DIR_UP: begin
            stop_seg_sx <= car_x - 14;
            stop_seg_sy <= car_y - step_pix;
            stop_seg_ex <= car_x + 14;
            stop_seg_ey <= car_y - step_pix;
          end
          DIR_RIGHT: begin
            stop_seg_sx <= car_x + step_pix;
            stop_seg_sy <= car_y - 14;
            stop_seg_ex <= car_x + step_pix;
            stop_seg_ey <= car_y + 14;
          end
          DIR_DOWN: begin
            stop_seg_sx <= car_x - 14;
            stop_seg_sy <= car_y + step_pix;
            stop_seg_ex <= car_x + 14;
            stop_seg_ey <= car_y + step_pix;
          end
          default: begin
            stop_seg_sx <= car_x - step_pix;
            stop_seg_sy <= car_y - 14;
            stop_seg_ex <= car_x - step_pix;
            stop_seg_ey <= car_y + 14;
          end
        endcase
      end
    end
  end

  always @(*) begin
    grid_x_line = (pixel_x == 12'd405) || (pixel_x == 12'd445) ||
                  (pixel_x == 12'd485) || (pixel_x == 12'd525) ||
                  (pixel_x == 12'd565) || (pixel_x == 12'd605) ||
                  (pixel_x == 12'd645) || (pixel_x == 12'd685) ||
                  (pixel_x == 12'd725) || (pixel_x == 12'd765) ||
                  (pixel_x == 12'd805) || (pixel_x == 12'd845) ||
                  (pixel_x == 12'd885) || (pixel_x == 12'd925);
    grid_y_line = (pixel_y == 12'd180) || (pixel_y == 12'd220) ||
                  (pixel_y == 12'd260) || (pixel_y == 12'd300) ||
                  (pixel_y == 12'd340) || (pixel_y == 12'd380) ||
                  (pixel_y == 12'd420) || (pixel_y == 12'd460) ||
                  (pixel_y == 12'd500) || (pixel_y == 12'd540);
  end

  always @(*) begin
    old_route_pixel = 1'b0;
    active_route_pixel = 1'b0;
    stop_line_pixel = 1'b0;

    for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
      if (old_seg_valid[i] &&
          segment_hit(old_seg_sx[i], old_seg_sy[i], old_seg_ex[i], old_seg_ey[i], 1) &&
          pixel_x[3] && pixel_y[3])
        old_route_pixel = 1'b1;
      if (active_seg_valid[i] &&
          segment_hit(active_seg_sx[i], active_seg_sy[i], active_seg_ex[i], active_seg_ey[i], 2))
        active_route_pixel = 1'b1;
    end

    if (stop_seg_valid)
      stop_line_pixel = segment_hit(stop_seg_sx, stop_seg_sy, stop_seg_ex, stop_seg_ey, 1);
  end

  always @(*) begin
    txt_state0 = 1'b0;
    txt_state1 = 1'b0;
    txt_state2 = 1'b0;
    txt_state3 = 1'b0;
    txt_change_box = 1'b0;
    txt_run_box = 1'b0;
    txt_info_dark = 1'b0;
    txt_route_dark = 1'b0;
    txt_route_hex = 1'b0;
    txt_menu_hint = 1'b0;
    txt_route_hint = 1'b0;
    text_idx = 0;
    text_base_x = 0;
    text_char = " ";

    if ((pixel_y >= 12'd38) && (pixel_y < 12'd52)) begin
      if ((pixel_x >= 12'd78) && (pixel_x < 12'd124)) begin
        text_idx = slot12(pixel_x - 12'd78);
        text_base_x = 78 + text_idx * 12;
        txt_state0 = glyph5x7_2x_hit(text_word_char(WORD_MENU, text_idx), pixel_x, pixel_y, text_base_x, 38);
      end
      if ((pixel_x >= 12'd220) && (pixel_x < 12'd266)) begin
        text_idx = slot12(pixel_x - 12'd220);
        text_base_x = 220 + text_idx * 12;
        txt_state1 = glyph5x7_2x_hit(text_word_char(WORD_EDIT, text_idx), pixel_x, pixel_y, text_base_x, 38);
      end
      if ((pixel_x >= 12'd372) && (pixel_x < 12'd406)) begin
        text_idx = slot12(pixel_x - 12'd372);
        text_base_x = 372 + text_idx * 12;
        txt_state2 = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 38);
      end
      if ((pixel_x >= 12'd500) && (pixel_x < 12'd546)) begin
        text_idx = slot12(pixel_x - 12'd500);
        text_base_x = 500 + text_idx * 12;
        txt_state3 = glyph5x7_2x_hit(text_word_char(WORD_STOP, text_idx), pixel_x, pixel_y, text_base_x, 38);
      end
      if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
        text_idx = slot12(pixel_x - 12'd604);
        text_base_x = 604 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 38);
      end
      if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
        text_idx = slot12(pixel_x - 12'd890);
        text_base_x = 890 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(dec5_char(complete_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 38);
      end
    end

    if ((pixel_y >= 12'd18) && (pixel_y < 12'd32)) begin
      if ((pixel_x >= 12'd598) && (pixel_x < 12'd656)) begin
        text_idx = slot12(pixel_x - 12'd598);
        text_base_x = 598 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_TOTAL, text_idx), pixel_x, pixel_y, text_base_x, 18);
      end
      if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
        text_idx = slot12(pixel_x - 12'd890);
        text_base_x = 890 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(dec5_char(total_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 18);
      end
    end

    if ((pixel_y >= 12'd58) && (pixel_y < 12'd72)) begin
      if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
        text_idx = slot12(pixel_x - 12'd604);
        text_base_x = 604 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 58);
      end
      if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
        text_idx = slot12(pixel_x - 12'd890);
        text_base_x = 890 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(dec5_char(remaining_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 58);
      end
    end

    if ((pixel_y >= 12'd78) && (pixel_y < 12'd92)) begin
      if ((pixel_x >= 12'd592) && (pixel_x < 12'd650)) begin
        text_idx = slot12(pixel_x - 12'd592);
        text_base_x = 592 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_SPEED, text_idx), pixel_x, pixel_y, text_base_x, 78);
      end
      if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
        text_idx = slot12(pixel_x - 12'd890);
        text_base_x = 890 + text_idx * 12;
        txt_info_dark = glyph5x7_2x_hit(dec5_char(speed, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 78);
      end
    end

    if ((pixel_y >= 12'd260) && (pixel_y < 12'd274) &&
        (pixel_x >= 12'd112) && (pixel_x < 12'd184)) begin
      text_idx = slot12(pixel_x - 12'd112);
      text_base_x = 112 + text_idx * 12;
      txt_change_box = glyph5x7_2x_hit(text_word_char(WORD_CHANGE, text_idx), pixel_x, pixel_y, text_base_x, 260);
    end

    if ((pixel_y >= 12'd390) && (pixel_y < 12'd404) &&
        (pixel_x >= 12'd164) && (pixel_x < 12'd198)) begin
      text_idx = slot12(pixel_x - 12'd164);
      text_base_x = 164 + text_idx * 12;
      txt_run_box = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 390);
    end

    if ((pixel_y >= 12'd156) && (pixel_y < 12'd170)) begin
      if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
        text_idx = slot12(pixel_x - 12'd382);
        text_base_x = 382 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd430) && (pixel_x < 12'd454)) begin
        text_idx = slot12(pixel_x - 12'd430);
        text_base_x = 430 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd478) && (pixel_x < 12'd524)) begin
        text_idx = slot12(pixel_x - 12'd478);
        text_base_x = 478 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd538) && (pixel_x < 12'd562)) begin
        text_idx = slot12(pixel_x - 12'd538);
        text_base_x = 538 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, completed_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd586) && (pixel_x < 12'd634)) begin
        text_idx = slot12(pixel_x - 12'd586);
        text_base_x = 586 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_STEP, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd658) && (pixel_x < 12'd718)) begin
        text_idx = slot12(pixel_x - 12'd658);
        text_base_x = 658 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec5_char(step_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd742) && (pixel_x < 12'd766)) begin
        text_idx = slot12(pixel_x - 12'd742);
        text_base_x = 742 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NX, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd778) && (pixel_x < 12'd814)) begin
        text_idx = slot12(pixel_x - 12'd778);
        text_base_x = 778 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec3_char(now_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
        text_idx = slot12(pixel_x - 12'd826);
        text_base_x = 826 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NY, text_idx), pixel_x, pixel_y, text_base_x, 156);
      end
      if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
        text_idx = slot12(pixel_x - 12'd862);
        text_base_x = 862 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec3_char(now_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
      end
    end

    if ((pixel_y >= 12'd172) && (pixel_y < 12'd186)) begin
      if ((pixel_x >= 12'd730) && (pixel_x < 12'd754)) begin
        text_idx = slot12(pixel_x - 12'd730);
        text_base_x = 730 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TX, text_idx), pixel_x, pixel_y, text_base_x, 172);
      end
      if ((pixel_x >= 12'd766) && (pixel_x < 12'd802)) begin
        text_idx = slot12(pixel_x - 12'd766);
        text_base_x = 766 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec3_char(target_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
      end
      if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
        text_idx = slot12(pixel_x - 12'd826);
        text_base_x = 826 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TY, text_idx), pixel_x, pixel_y, text_base_x, 172);
      end
      if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
        text_idx = slot12(pixel_x - 12'd862);
        text_base_x = 862 + text_idx * 12;
        txt_route_dark = glyph5x7_2x_hit(dec3_char(target_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
      end
    end

    if ((pixel_y >= 12'd520) && (pixel_y < 12'd534)) begin
      if ((pixel_x >= 12'd382) && (pixel_x < 12'd406)) begin
        text_idx = slot12(pixel_x - 12'd382);
        text_base_x = 382 + text_idx * 12;
        txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_RT, text_idx), pixel_x, pixel_y, text_base_x, 520);
      end
      if ((pixel_x >= 12'd418) && (pixel_x < 12'd610)) begin
        text_idx = slot12(pixel_x - 12'd418);
        text_base_x = 418 + text_idx * 12;
        txt_route_hex = glyph5x7_2x_hit(hex16_char(route_info, text_idx), pixel_x, pixel_y, text_base_x, 520);
      end
    end

    if ((pixel_y >= 12'd538) && (pixel_y < 12'd552)) begin
      if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
        text_idx = slot12(pixel_x - 12'd382);
        text_base_x = 382 + text_idx * 12;
        txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 538);
      end
      if ((pixel_x >= 12'd430) && (pixel_x < 12'd622)) begin
        text_idx = slot12(pixel_x - 12'd430);
        text_base_x = 430 + text_idx * 12;
        txt_route_hex = glyph5x7_2x_hit(hex16_char(old_route_info, text_idx), pixel_x, pixel_y, text_base_x, 538);
      end
    end

    if ((pixel_y >= 12'd458) && (pixel_y < 12'd472)) begin
      if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
        text_idx = slot12(pixel_x - 12'd84);
        text_base_x = 84 + text_idx * 12;
        txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 458);
      end
      if ((pixel_x >= 12'd156) && (pixel_x < 12'd180)) begin
        text_idx = slot12(pixel_x - 12'd156);
        text_base_x = 156 + text_idx * 12;
        txt_menu_hint = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 458);
      end
    end

    if ((pixel_y >= 12'd478) && (pixel_y < 12'd492)) begin
      if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
        text_idx = slot12(pixel_x - 12'd84);
        text_base_x = 84 + text_idx * 12;
        txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_NEXT, text_idx), pixel_x, pixel_y, text_base_x, 478);
      end
      if ((pixel_x >= 12'd156) && (pixel_x < 12'd204)) begin
        text_idx = slot12(pixel_x - 12'd156);
        text_base_x = 156 + text_idx * 12;
        txt_menu_hint = glyph5x7_2x_hit(cmd_word_char(route_info[1:0], text_idx), pixel_x, pixel_y, text_base_x, 478);
      end
    end

    if ((pixel_y >= 12'd194) && (pixel_y < 12'd208)) begin
      if ((pixel_x >= 12'd382) && (pixel_x < 12'd418)) begin
        text_idx = slot12(pixel_x - 12'd382);
        text_base_x = 382 + text_idx * 12;
        txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 194);
      end
      if ((pixel_x >= 12'd450) && (pixel_x < 12'd498)) begin
        text_idx = slot12(pixel_x - 12'd450);
        text_base_x = 450 + text_idx * 12;
        txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
      end
      if ((pixel_x >= 12'd542) && (pixel_x < 12'd578)) begin
        text_idx = slot12(pixel_x - 12'd542);
        text_base_x = 542 + text_idx * 12;
        txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 194);
      end
      if ((pixel_x >= 12'd578) && (pixel_x < 12'd626)) begin
        text_idx = slot12(pixel_x - 12'd578);
        text_base_x = 578 + text_idx * 12;
        txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
      end
    end
  end

  always @(*) begin
    if (!pixel_de) begin
      rgb_next = 24'h000000;
    end else begin
      rgb_next = 24'hF4F7FB;

      if (status_panel) rgb_next = 24'hFFFFFF;
      if (menu_panel) rgb_next = 24'hEAF2FF;
      if (route_panel) rgb_next = 24'hF8FAFC;
      if (map_area) rgb_next = 24'hFFFFFF;
      if (grid_line) rgb_next = 24'hE5E7EB;

      if (change_box) rgb_next = change_select ? 24'h1D4ED8 : 24'hDCEAFE;
      if (run_box) rgb_next = run_select ? 24'h16A34A : 24'hDCFCE7;

      if (state_bar_0) rgb_next = state_menu_sel   ? 24'h2563EB : 24'hCBD5E1;
      if (state_bar_1) rgb_next = state_change_sel ? 24'h2563EB : 24'hCBD5E1;
      if (state_bar_2) rgb_next = state_run_sel    ? 24'h16A34A : 24'hCBD5E1;
      if (state_bar_3) rgb_next = state_stop_sel   ? 24'hDC2626 : 24'hCBD5E1;

      if (info_bar_total) rgb_next = 24'h2563EB;
      if (info_bar_done) rgb_next = 24'h16A34A;
      if (info_bar_remain) rgb_next = 24'hF59E0B;
      if (info_bar_speed) rgb_next = 24'h7C3AED;

      if (txt_info_dark || txt_route_dark || txt_route_hex || txt_menu_hint || txt_route_hint)
        rgb_next = 24'h0F172A;
      if (txt_state0) rgb_next = state_menu_sel ? 24'hFFFFFF : 24'h334155;
      if (txt_state1) rgb_next = state_change_sel ? 24'hFFFFFF : 24'h334155;
      if (txt_state2) rgb_next = state_run_sel ? 24'hFFFFFF : 24'h334155;
      if (txt_state3) rgb_next = state_stop_sel ? 24'hFFFFFF : 24'h334155;
      if (txt_change_box) rgb_next = change_select ? 24'hFFFFFF : 24'h1D4ED8;
      if (txt_run_box) rgb_next = run_select ? 24'hFFFFFF : 24'h166534;

      if (old_route_pixel) rgb_next = 24'h94A3B8;
      if (active_route_pixel) rgb_next = 24'h2563EB;
      if (target_marker) rgb_next = 24'h0891B2;
      if (now_marker) rgb_next = 24'h64748B;
      if (car_body) rgb_next = 24'h111827;
      if (car_arrow) rgb_next = 24'hFACC15;
      if (stop_line_pixel) rgb_next = 24'hDC2626;
      if (border_line) rgb_next = 24'h334155;
    end
  end

  always @(posedge clk_pix or negedge rst_n) begin
    if (!rst_n)
      rgb_data <= 24'h000000;
    else
      rgb_data <= rgb_next;
  end

endmodule
