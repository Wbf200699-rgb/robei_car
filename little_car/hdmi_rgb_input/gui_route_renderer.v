module gui_route_renderer(
input wire [11:0] pixel_x,
input wire [11:0] pixel_y,
input wire pixel_de,
input wire [1:0] menu_info,
input wire [63:0] route_info,
input wire clk_pix,
input wire rst_n,
input wire [4:0] state_info,
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
input wire bt_connected,
output reg [23:0] rgb_data
);

  //----Code starts here: integrated by Robei-----
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

            localparam integer ROUTE_WORDS = 8;

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

            reg route_build_active;
            reg route_build_phase;
            reg [3:0] route_build_index;
            reg [11:0] route_build_x;
            reg [11:0] route_build_y;
            reg [1:0] route_build_dir;
            reg [6:0] route_build_step;
            reg [63:0] route_info_snapshot;
            reg [63:0] old_route_info_snapshot;
            reg [5:0] active_steps_snapshot;
            reg [5:0] completed_steps_snapshot;
            reg route_snapshot_valid;
            reg [1:0] route_build_cmd;
            reg route_build_cmd_valid;
            reg route_build_draw;
            reg [11:0] route_build_next_x;
            reg [11:0] route_build_next_y;
            reg [1:0] route_build_next_dir;

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
            reg [23:0] rgb_base_next;
            reg [23:0] rgb_next;
            reg grid_x_line;
            reg grid_y_line;

            reg pixel_de_q;
            reg [11:0] pixel_x_q;
            reg [11:0] pixel_y_q;
            reg [23:0] rgb_base_q;
            reg old_route_pixel_q;
            reg active_route_pixel_q;
            reg target_marker_q;
            reg now_marker_q;
            reg car_body_q;
            reg car_arrow_q;
            reg stop_line_pixel_q;
            reg border_line_q;

            localparam [3:0] TEXT_NONE   = 4'd0;
            localparam [3:0] TEXT_DARK   = 4'd1;
            localparam [3:0] TEXT_STATE0 = 4'd2;
            localparam [3:0] TEXT_STATE1 = 4'd3;
            localparam [3:0] TEXT_STATE2 = 4'd4;
            localparam [3:0] TEXT_STATE3 = 4'd5;
            localparam [3:0] TEXT_CHANGE = 4'd6;
            localparam [3:0] TEXT_RUN    = 4'd7;

            // Legacy text flags remain tied low inside a synthesis-pruned
            // branch so older generated Robei code keeps a stable structure.
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

            reg [3:0] text_idx;
            reg [11:0] text_base_x;
            reg [7:0] text_char;
            reg [11:0] text_base_y;
            reg [3:0] text_style;
            reg [7:0] text_char_q;
            reg [11:0] text_base_x_q;
            reg [11:0] text_base_y_q;
            reg [3:0] text_style_q;
            reg [23:0] text_color_next;
            reg [23:0] text_color_q;

            reg [19:0] bcd_complete;
            reg [19:0] bcd_total;
            reg [19:0] bcd_remaining;
            reg [19:0] bcd_speed;
            reg [19:0] bcd_step;
            reg [19:0] bcd_now_x;
            reg [19:0] bcd_now_y;
            reg [19:0] bcd_target_x;
            reg [19:0] bcd_target_y;
            reg [19:0] bcd_active_steps;
            reg [19:0] bcd_completed_steps;
            reg [3:0] bcd_source;
            reg [4:0] bcd_bits_left;
            reg [15:0] bcd_binary;
            reg [19:0] bcd_work;
            reg bcd_busy;
            reg [19:0] bcd_adjusted;
            reg [19:0] bcd_shifted;
            wire text_pixel;

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
            wire bt_status_box;
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
                              ((sx == ex) || (sy == ey));
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

            function [7:0] bcd5_char;
              input [19:0] value;
              input [2:0] idx;
              reg [3:0] digit;
              begin
                case (idx)
                  3'd0: digit = value[19:16];
                  3'd1: digit = value[15:12];
                  3'd2: digit = value[11:8];
                  3'd3: digit = value[7:4];
                  default: digit = value[3:0];
                endcase
                bcd5_char = 8'd48 + digit;
              end
            endfunction

            function [7:0] bcd3_char;
              input [19:0] value;
              input [1:0] idx;
              reg [3:0] digit;
              begin
                case (idx)
                  2'd0: digit = value[11:8];
                  2'd1: digit = value[7:4];
                  default: digit = value[3:0];
                endcase
                bcd3_char = 8'd48 + digit;
              end
            endfunction

            function [7:0] bcd2_char;
              input [19:0] value;
              input integer idx;
              reg [3:0] digit;
              begin
                if (idx == 0)
                  digit = value[7:4];
                else
                  digit = value[3:0];
                bcd2_char = 8'd48 + digit;
              end
            endfunction

            // These names are referenced only by the synthesis-pruned legacy
            // selector below. Active rendering uses cached BCD values.
            function [7:0] dec5_char;
              input [15:0] unused_value;
              input [2:0] unused_idx;
              begin dec5_char = " "; end
            endfunction

            function [7:0] dec3_char;
              input [7:0] unused_value;
              input [1:0] unused_idx;
              begin dec3_char = " "; end
            endfunction

            function [7:0] dec2_char;
              input [7:0] unused_value;
              input integer unused_idx;
              begin dec2_char = " "; end
            endfunction

            function [3:0] slot12;
              input [11:0] offset;
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
            assign bt_status_box = status_panel &&
                                   (pixel_x >= 12'd52) && (pixel_x < 12'd136) &&
                                   (pixel_y >= 12'd78) && (pixel_y < 12'd102);

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

            // One shift/add-3 converter services all displayed decimal values.
            // The complete set refreshes in fewer than 200 pixel clocks, while
            // avoiding one large subtraction network for every text field.
            always @(*) begin
              bcd_adjusted = bcd_work;
              if (bcd_work[3:0] >= 5) bcd_adjusted[3:0] = bcd_work[3:0] + 4'd3;
              if (bcd_work[7:4] >= 5) bcd_adjusted[7:4] = bcd_work[7:4] + 4'd3;
              if (bcd_work[11:8] >= 5) bcd_adjusted[11:8] = bcd_work[11:8] + 4'd3;
              if (bcd_work[15:12] >= 5) bcd_adjusted[15:12] = bcd_work[15:12] + 4'd3;
              if (bcd_work[19:16] >= 5) bcd_adjusted[19:16] = bcd_work[19:16] + 4'd3;
              bcd_shifted = {bcd_adjusted[18:0], bcd_binary[15]};
            end

            always @(posedge clk_pix or negedge rst_n) begin
              if (!rst_n) begin
                bcd_complete <= 20'd0;
                bcd_total <= 20'd0;
                bcd_remaining <= 20'd0;
                bcd_speed <= 20'd0;
                bcd_step <= 20'd0;
                bcd_now_x <= 20'd0;
                bcd_now_y <= 20'd0;
                bcd_target_x <= 20'd0;
                bcd_target_y <= 20'd0;
                bcd_active_steps <= 20'd0;
                bcd_completed_steps <= 20'd0;
                bcd_source <= 4'd0;
                bcd_bits_left <= 5'd0;
                bcd_binary <= 16'd0;
                bcd_work <= 20'd0;
                bcd_busy <= 1'b0;
              end else if (!bcd_busy) begin
                case (bcd_source)
                  4'd0: bcd_binary <= complete_route_length;
                  4'd1: bcd_binary <= total_route_length;
                  4'd2: bcd_binary <= remaining_route_length;
                  4'd3: bcd_binary <= speed;
                  4'd4: bcd_binary <= step_length;
                  4'd5: bcd_binary <= {8'd0, now_x};
                  4'd6: bcd_binary <= {8'd0, now_y};
                  4'd7: bcd_binary <= {8'd0, target_x};
                  4'd8: bcd_binary <= {8'd0, target_y};
                  4'd9: bcd_binary <= {10'd0, active_route_steps};
                  default: bcd_binary <= {10'd0, completed_route_steps};
                endcase
                bcd_work <= 20'd0;
                bcd_bits_left <= 5'd16;
                bcd_busy <= 1'b1;
              end else begin
                bcd_work <= bcd_shifted;
                bcd_binary <= {bcd_binary[14:0], 1'b0};
                if (bcd_bits_left == 5'd1) begin
                  case (bcd_source)
                    4'd0: bcd_complete <= bcd_shifted;
                    4'd1: bcd_total <= bcd_shifted;
                    4'd2: bcd_remaining <= bcd_shifted;
                    4'd3: bcd_speed <= bcd_shifted;
                    4'd4: bcd_step <= bcd_shifted;
                    4'd5: bcd_now_x <= bcd_shifted;
                    4'd6: bcd_now_y <= bcd_shifted;
                    4'd7: bcd_target_x <= bcd_shifted;
                    4'd8: bcd_target_y <= bcd_shifted;
                    4'd9: bcd_active_steps <= bcd_shifted;
                    default: bcd_completed_steps <= bcd_shifted;
                  endcase
                  bcd_source <= (bcd_source == 4'd10) ? 4'd0 : bcd_source + 4'd1;
                  bcd_busy <= 1'b0;
                end else begin
                  bcd_bits_left <= bcd_bits_left - 5'd1;
                end
              end
            end

            // Expand one route command per pixel clock. Route data changes far
            // more slowly than video, so a 16-cycle rebuild is visually
            // instantaneous and removes the previous 8-command carry/mux
            // cascade from every display clock.
            always @(*) begin
              route_build_cmd = route_build_phase ?
                  route_info_snapshot[route_build_index*2 +: 2] :
                  old_route_info_snapshot[route_build_index*2 +: 2];
              route_build_cmd_valid = route_build_phase ?
                  (route_build_index < active_steps_snapshot) :
                  (route_build_index < completed_steps_snapshot);
              route_build_draw = 1'b0;
              route_build_next_x = route_build_x;
              route_build_next_y = route_build_y;
              route_build_next_dir = route_build_dir;

              if (route_build_cmd_valid && (route_build_cmd != ROUTE_STOP)) begin
                if (route_build_cmd == ROUTE_RIGHT)
                  route_build_next_dir = route_build_dir + 2'd1;
                else if (route_build_cmd == ROUTE_LEFT)
                  route_build_next_dir = route_build_dir - 2'd1;
                else begin
                  route_build_draw = 1'b1;
                  case (route_build_dir)
                    DIR_UP:    route_build_next_y = route_build_y - route_build_step;
                    DIR_RIGHT: route_build_next_x = route_build_x + route_build_step;
                    DIR_DOWN:  route_build_next_y = route_build_y + route_build_step;
                    default:   route_build_next_x = route_build_x - route_build_step;
                  endcase
                end
              end
            end

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
                route_build_active <= 1'b0;
                route_build_phase <= 1'b0;
                route_build_index <= 4'd0;
                route_build_x <= 12'd675;
                route_build_y <= 12'd370;
                route_build_dir <= DIR_UP;
                route_build_step <= 7'd16;
                route_info_snapshot <= 64'd0;
                old_route_info_snapshot <= 64'd0;
                active_steps_snapshot <= 6'd0;
                completed_steps_snapshot <= 6'd0;
                route_snapshot_valid <= 1'b0;
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

                stop_seg_valid <= (state_info == STATE_STOP);
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

                if (!route_build_active &&
                    (!route_snapshot_valid ||
                     (route_info != route_info_snapshot) ||
                     (old_route_info != old_route_info_snapshot) ||
                     (active_route_steps != active_steps_snapshot) ||
                     (completed_route_steps != completed_steps_snapshot))) begin
                  route_build_active <= 1'b1;
                  route_build_phase <= 1'b0;
                  route_build_index <= 4'd0;
                  route_build_x <= 12'd675;
                  route_build_y <= 12'd370;
                  route_build_dir <= DIR_UP;
                  route_build_step <= calc_step_pix(active_route_steps);
                  route_info_snapshot <= route_info;
                  old_route_info_snapshot <= old_route_info;
                  active_steps_snapshot <= active_route_steps;
                  completed_steps_snapshot <= completed_route_steps;
                  route_snapshot_valid <= 1'b1;
                  old_seg_valid <= {ROUTE_WORDS{1'b0}};
                  active_seg_valid <= {ROUTE_WORDS{1'b0}};
                end else if (route_build_active) begin
                  route_build_x <= route_build_next_x;
                  route_build_y <= route_build_next_y;
                  route_build_dir <= route_build_next_dir;

                  if (route_build_draw) begin
                    if (!route_build_phase) begin
                      old_seg_valid[route_build_index] <= 1'b1;
                      old_seg_sx[route_build_index] <= route_build_x;
                      old_seg_sy[route_build_index] <= route_build_y;
                      old_seg_ex[route_build_index] <= route_build_next_x;
                      old_seg_ey[route_build_index] <= route_build_next_y;
                    end else begin
                      active_seg_valid[route_build_index] <= 1'b1;
                      active_seg_sx[route_build_index] <= route_build_x;
                      active_seg_sy[route_build_index] <= route_build_y;
                      active_seg_ex[route_build_index] <= route_build_next_x;
                      active_seg_ey[route_build_index] <= route_build_next_y;
                    end
                  end

                  if (route_build_index == ROUTE_WORDS - 1) begin
                    route_build_index <= 4'd0;
                    if (!route_build_phase) begin
                      car_x <= route_build_next_x;
                      car_y <= route_build_next_y;
                      car_dir <= route_build_next_dir;
                      route_build_phase <= 1'b1;
                    end else begin
                      route_build_active <= 1'b0;
                    end
                  end else begin
                    route_build_index <= route_build_index + 4'd1;
                  end
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
              text_base_y = 12'd0;
              text_style = TEXT_NONE;

              // Kept only so old Robei-generated source remains structurally
              // recognizable. Constant folding removes every legacy glyph
              // call and the active selector below performs one shared call.
              if (1'b0) begin
              if ((pixel_y >= 12'd38) && (pixel_y < 12'd52)) begin
                if ((pixel_x >= 12'd78) && (pixel_x < 12'd124)) begin
                  text_idx = slot12(pixel_x - 12'd78);
                  text_base_x = 78 + (text_idx << 3) + (text_idx << 2);
                  txt_state0 = glyph5x7_2x_hit(text_word_char(WORD_MENU, text_idx), pixel_x, pixel_y, text_base_x, 38);
                end
                if ((pixel_x >= 12'd220) && (pixel_x < 12'd266)) begin
                  text_idx = slot12(pixel_x - 12'd220);
                  text_base_x = 220 + (text_idx << 3) + (text_idx << 2);
                  txt_state1 = glyph5x7_2x_hit(text_word_char(WORD_EDIT, text_idx), pixel_x, pixel_y, text_base_x, 38);
                end
                if ((pixel_x >= 12'd372) && (pixel_x < 12'd406)) begin
                  text_idx = slot12(pixel_x - 12'd372);
                  text_base_x = 372 + (text_idx << 3) + (text_idx << 2);
                  txt_state2 = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 38);
                end
                if ((pixel_x >= 12'd500) && (pixel_x < 12'd546)) begin
                  text_idx = slot12(pixel_x - 12'd500);
                  text_base_x = 500 + (text_idx << 3) + (text_idx << 2);
                  txt_state3 = glyph5x7_2x_hit(text_word_char(WORD_STOP, text_idx), pixel_x, pixel_y, text_base_x, 38);
                end
                if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                  text_idx = slot12(pixel_x - 12'd604);
                  text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 38);
                end
                if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                  text_idx = slot12(pixel_x - 12'd890);
                  text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(dec5_char(complete_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 38);
                end
              end

              if ((pixel_y >= 12'd18) && (pixel_y < 12'd32)) begin
                if ((pixel_x >= 12'd598) && (pixel_x < 12'd656)) begin
                  text_idx = slot12(pixel_x - 12'd598);
                  text_base_x = 598 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_TOTAL, text_idx), pixel_x, pixel_y, text_base_x, 18);
                end
                if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                  text_idx = slot12(pixel_x - 12'd890);
                  text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(dec5_char(total_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 18);
                end
              end

              if ((pixel_y >= 12'd58) && (pixel_y < 12'd72)) begin
                if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                  text_idx = slot12(pixel_x - 12'd604);
                  text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 58);
                end
                if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                  text_idx = slot12(pixel_x - 12'd890);
                  text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(dec5_char(remaining_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 58);
                end
              end

            if ((pixel_y >= 12'd78) && (pixel_y < 12'd92)) begin
              if ((pixel_x >= 12'd592) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd592);
                  text_base_x = 592 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_SPEED, text_idx), pixel_x, pixel_y, text_base_x, 78);
                end
                if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                  text_idx = slot12(pixel_x - 12'd890);
                  text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                  txt_info_dark = glyph5x7_2x_hit(dec5_char(speed, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 78);
                end
              end

              if ((pixel_y >= 12'd260) && (pixel_y < 12'd274) &&
                  (pixel_x >= 12'd112) && (pixel_x < 12'd184)) begin
                text_idx = slot12(pixel_x - 12'd112);
                text_base_x = 112 + (text_idx << 3) + (text_idx << 2);
                txt_change_box = glyph5x7_2x_hit(text_word_char(WORD_CHANGE, text_idx), pixel_x, pixel_y, text_base_x, 260);
              end

              if ((pixel_y >= 12'd390) && (pixel_y < 12'd404) &&
                  (pixel_x >= 12'd164) && (pixel_x < 12'd198)) begin
                text_idx = slot12(pixel_x - 12'd164);
                text_base_x = 164 + (text_idx << 3) + (text_idx << 2);
                txt_run_box = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 390);
              end

              if ((pixel_y >= 12'd156) && (pixel_y < 12'd170)) begin
                if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
                  text_idx = slot12(pixel_x - 12'd382);
                  text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd430) && (pixel_x < 12'd454)) begin
                  text_idx = slot12(pixel_x - 12'd430);
                  text_base_x = 430 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd478) && (pixel_x < 12'd524)) begin
                  text_idx = slot12(pixel_x - 12'd478);
                  text_base_x = 478 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd538) && (pixel_x < 12'd562)) begin
                  text_idx = slot12(pixel_x - 12'd538);
                  text_base_x = 538 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, completed_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd586) && (pixel_x < 12'd634)) begin
                  text_idx = slot12(pixel_x - 12'd586);
                  text_base_x = 586 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_STEP, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd658) && (pixel_x < 12'd718)) begin
                  text_idx = slot12(pixel_x - 12'd658);
                  text_base_x = 658 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec5_char(step_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd742) && (pixel_x < 12'd766)) begin
                  text_idx = slot12(pixel_x - 12'd742);
                  text_base_x = 742 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NX, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd778) && (pixel_x < 12'd814)) begin
                  text_idx = slot12(pixel_x - 12'd778);
                  text_base_x = 778 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec3_char(now_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                  text_idx = slot12(pixel_x - 12'd826);
                  text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NY, text_idx), pixel_x, pixel_y, text_base_x, 156);
                end
                if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                  text_idx = slot12(pixel_x - 12'd862);
                  text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec3_char(now_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
                end
              end

              if ((pixel_y >= 12'd172) && (pixel_y < 12'd186)) begin
                if ((pixel_x >= 12'd730) && (pixel_x < 12'd754)) begin
                  text_idx = slot12(pixel_x - 12'd730);
                  text_base_x = 730 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TX, text_idx), pixel_x, pixel_y, text_base_x, 172);
                end
                if ((pixel_x >= 12'd766) && (pixel_x < 12'd802)) begin
                  text_idx = slot12(pixel_x - 12'd766);
                  text_base_x = 766 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec3_char(target_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
                end
                if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                  text_idx = slot12(pixel_x - 12'd826);
                  text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TY, text_idx), pixel_x, pixel_y, text_base_x, 172);
                end
                if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                  text_idx = slot12(pixel_x - 12'd862);
                  text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                  txt_route_dark = glyph5x7_2x_hit(dec3_char(target_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
                end
              end

              if ((pixel_y >= 12'd520) && (pixel_y < 12'd534)) begin
                if ((pixel_x >= 12'd382) && (pixel_x < 12'd406)) begin
                  text_idx = slot12(pixel_x - 12'd382);
                  text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_RT, text_idx), pixel_x, pixel_y, text_base_x, 520);
                end
                if ((pixel_x >= 12'd418) && (pixel_x < 12'd610)) begin
                  text_idx = slot12(pixel_x - 12'd418);
                  text_base_x = 418 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hex = glyph5x7_2x_hit(hex16_char(route_info, text_idx), pixel_x, pixel_y, text_base_x, 520);
                end
              end

              if ((pixel_y >= 12'd538) && (pixel_y < 12'd552)) begin
                if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
                  text_idx = slot12(pixel_x - 12'd382);
                  text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 538);
                end
                if ((pixel_x >= 12'd430) && (pixel_x < 12'd622)) begin
                  text_idx = slot12(pixel_x - 12'd430);
                  text_base_x = 430 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hex = glyph5x7_2x_hit(hex16_char(old_route_info, text_idx), pixel_x, pixel_y, text_base_x, 538);
                end
              end

              if ((pixel_y >= 12'd458) && (pixel_y < 12'd472)) begin
                if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                  text_idx = slot12(pixel_x - 12'd84);
                  text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                  txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 458);
                end
                if ((pixel_x >= 12'd156) && (pixel_x < 12'd180)) begin
                  text_idx = slot12(pixel_x - 12'd156);
                  text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                  txt_menu_hint = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 458);
                end
              end

              if ((pixel_y >= 12'd478) && (pixel_y < 12'd492)) begin
                if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                  text_idx = slot12(pixel_x - 12'd84);
                  text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                  txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_NEXT, text_idx), pixel_x, pixel_y, text_base_x, 478);
                end
                if ((pixel_x >= 12'd156) && (pixel_x < 12'd204)) begin
                  text_idx = slot12(pixel_x - 12'd156);
                  text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                  txt_menu_hint = glyph5x7_2x_hit(cmd_word_char(route_info[1:0], text_idx), pixel_x, pixel_y, text_base_x, 478);
                end
              end

              if ((pixel_y >= 12'd194) && (pixel_y < 12'd208)) begin
                if ((pixel_x >= 12'd382) && (pixel_x < 12'd418)) begin
                  text_idx = slot12(pixel_x - 12'd382);
                  text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 194);
                end
                if ((pixel_x >= 12'd450) && (pixel_x < 12'd498)) begin
                  text_idx = slot12(pixel_x - 12'd450);
                  text_base_x = 450 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
                end
                if ((pixel_x >= 12'd542) && (pixel_x < 12'd578)) begin
                  text_idx = slot12(pixel_x - 12'd542);
                  text_base_x = 542 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 194);
                end
                if ((pixel_x >= 12'd578) && (pixel_x < 12'd626)) begin
                  text_idx = slot12(pixel_x - 12'd578);
                  text_base_x = 578 + (text_idx << 3) + (text_idx << 2);
                  txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
                end
              end
              end

              if ((pixel_y >= 12'd18) && (pixel_y < 12'd32) &&
                  (pixel_x >= 12'd598) && (pixel_x < 12'd656)) begin
                text_idx = slot12(pixel_x - 12'd598);
                text_base_x = 598 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd18;
                text_char = text_word_char(WORD_TOTAL, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd18) && (pixel_y < 12'd32) &&
                           (pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd18;
                text_char = bcd5_char(bcd_total, text_idx[2:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd78) && (pixel_x < 12'd124)) begin
                text_idx = slot12(pixel_x - 12'd78);
                text_base_x = 78 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = text_word_char(WORD_MENU, text_idx);
                text_style = TEXT_STATE0;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd220) && (pixel_x < 12'd266)) begin
                text_idx = slot12(pixel_x - 12'd220);
                text_base_x = 220 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = text_word_char(WORD_EDIT, text_idx);
                text_style = TEXT_STATE1;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd372) && (pixel_x < 12'd406)) begin
                text_idx = slot12(pixel_x - 12'd372);
                text_base_x = 372 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = text_word_char(WORD_RUN, text_idx);
                text_style = TEXT_STATE2;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd500) && (pixel_x < 12'd546)) begin
                text_idx = slot12(pixel_x - 12'd500);
                text_base_x = 500 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = text_word_char(WORD_STOP, text_idx);
                text_style = TEXT_STATE3;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd604);
                text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = text_word_char(WORD_DONE, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd38) && (pixel_y < 12'd52) &&
                           (pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd38;
                text_char = bcd5_char(bcd_complete, text_idx[2:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd58) && (pixel_y < 12'd72) &&
                           (pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd604);
                text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd58;
                text_char = text_word_char(WORD_LEFT, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd58) && (pixel_y < 12'd72) &&
                           (pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd58;
                text_char = bcd5_char(bcd_remaining, text_idx[2:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd78) && (pixel_y < 12'd92) &&
                           (pixel_x >= 12'd592) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd592);
                text_base_x = 592 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd78;
                text_char = text_word_char(WORD_SPEED, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd78) && (pixel_y < 12'd92) &&
                           (pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd78;
                text_char = bcd5_char(bcd_speed, text_idx[2:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = text_word_char(WORD_ACT, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd430) && (pixel_x < 12'd454)) begin
                text_idx = slot12(pixel_x - 12'd430);
                text_base_x = 430 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = bcd2_char(bcd_active_steps, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd478) && (pixel_x < 12'd524)) begin
                text_idx = slot12(pixel_x - 12'd478);
                text_base_x = 478 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = text_word_char(WORD_DONE, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd538) && (pixel_x < 12'd562)) begin
                text_idx = slot12(pixel_x - 12'd538);
                text_base_x = 538 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = bcd2_char(bcd_completed_steps, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd586) && (pixel_x < 12'd634)) begin
                text_idx = slot12(pixel_x - 12'd586);
                text_base_x = 586 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = text_word_char(WORD_STEP, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd658) && (pixel_x < 12'd718)) begin
                text_idx = slot12(pixel_x - 12'd658);
                text_base_x = 658 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = bcd5_char(bcd_step, text_idx[2:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd742) && (pixel_x < 12'd766)) begin
                text_idx = slot12(pixel_x - 12'd742);
                text_base_x = 742 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = text_word_char(WORD_NX, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd778) && (pixel_x < 12'd814)) begin
                text_idx = slot12(pixel_x - 12'd778);
                text_base_x = 778 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = bcd3_char(bcd_now_x, text_idx[1:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                text_idx = slot12(pixel_x - 12'd826);
                text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = text_word_char(WORD_NY, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd156) && (pixel_y < 12'd170) &&
                           (pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                text_idx = slot12(pixel_x - 12'd862);
                text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd156;
                text_char = bcd3_char(bcd_now_y, text_idx[1:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd172) && (pixel_y < 12'd186) &&
                           (pixel_x >= 12'd730) && (pixel_x < 12'd754)) begin
                text_idx = slot12(pixel_x - 12'd730);
                text_base_x = 730 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd172;
                text_char = text_word_char(WORD_TX, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd172) && (pixel_y < 12'd186) &&
                           (pixel_x >= 12'd766) && (pixel_x < 12'd802)) begin
                text_idx = slot12(pixel_x - 12'd766);
                text_base_x = 766 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd172;
                text_char = bcd3_char(bcd_target_x, text_idx[1:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd172) && (pixel_y < 12'd186) &&
                           (pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                text_idx = slot12(pixel_x - 12'd826);
                text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd172;
                text_char = text_word_char(WORD_TY, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd172) && (pixel_y < 12'd186) &&
                           (pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                text_idx = slot12(pixel_x - 12'd862);
                text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd172;
                text_char = bcd3_char(bcd_target_y, text_idx[1:0]);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd194) && (pixel_y < 12'd208) &&
                           (pixel_x >= 12'd382) && (pixel_x < 12'd418)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd194;
                text_char = text_word_char(WORD_OLD, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd194) && (pixel_y < 12'd208) &&
                           (pixel_x >= 12'd450) && (pixel_x < 12'd498)) begin
                text_idx = slot12(pixel_x - 12'd450);
                text_base_x = 450 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd194;
                text_char = text_word_char(WORD_PATH, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd194) && (pixel_y < 12'd208) &&
                           (pixel_x >= 12'd542) && (pixel_x < 12'd578)) begin
                text_idx = slot12(pixel_x - 12'd542);
                text_base_x = 542 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd194;
                text_char = text_word_char(WORD_ACT, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd194) && (pixel_y < 12'd208) &&
                           (pixel_x >= 12'd578) && (pixel_x < 12'd626)) begin
                text_idx = slot12(pixel_x - 12'd578);
                text_base_x = 578 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd194;
                text_char = text_word_char(WORD_PATH, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd260) && (pixel_y < 12'd274) &&
                           (pixel_x >= 12'd112) && (pixel_x < 12'd184)) begin
                text_idx = slot12(pixel_x - 12'd112);
                text_base_x = 112 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd260;
                text_char = text_word_char(WORD_CHANGE, text_idx);
                text_style = TEXT_CHANGE;
              end else if ((pixel_y >= 12'd390) && (pixel_y < 12'd404) &&
                           (pixel_x >= 12'd164) && (pixel_x < 12'd198)) begin
                text_idx = slot12(pixel_x - 12'd164);
                text_base_x = 164 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd390;
                text_char = text_word_char(WORD_RUN, text_idx);
                text_style = TEXT_RUN;
              end else if ((pixel_y >= 12'd458) && (pixel_y < 12'd472) &&
                           (pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                text_idx = slot12(pixel_x - 12'd84);
                text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd458;
                text_char = text_word_char(WORD_LEFT, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd458) && (pixel_y < 12'd472) &&
                           (pixel_x >= 12'd156) && (pixel_x < 12'd180)) begin
                text_idx = slot12(pixel_x - 12'd156);
                text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd458;
                text_char = bcd2_char(bcd_active_steps, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd478) && (pixel_y < 12'd492) &&
                           (pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                text_idx = slot12(pixel_x - 12'd84);
                text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd478;
                text_char = text_word_char(WORD_NEXT, text_idx);
                text_style = TEXT_DARK;
              end else if ((pixel_y >= 12'd478) && (pixel_y < 12'd492) &&
                           (pixel_x >= 12'd156) && (pixel_x < 12'd204)) begin
                text_idx = slot12(pixel_x - 12'd156);
                text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                text_base_y = 12'd478;
                text_char = cmd_word_char(route_info[1:0], text_idx);
                text_style = TEXT_DARK;
              end
            end

            always @(*) begin
              rgb_base_next = 24'h000000;
              if (pixel_de) begin
                rgb_base_next = 24'hF4F7FB;
                if (status_panel) rgb_base_next = 24'hFFFFFF;
                if (menu_panel) rgb_base_next = 24'hEAF2FF;
                if (route_panel) rgb_base_next = 24'hF8FAFC;
                if (map_area) rgb_base_next = 24'hFFFFFF;
                if (grid_line) rgb_base_next = 24'hE5E7EB;
                if (change_box) rgb_base_next = change_select ? 24'h1D4ED8 : 24'hDCEAFE;
                if (run_box) rgb_base_next = run_select ? 24'h16A34A : 24'hDCFCE7;
                if (state_bar_0) rgb_base_next = state_menu_sel   ? 24'h2563EB : 24'hCBD5E1;
                if (state_bar_1) rgb_base_next = state_change_sel ? 24'h2563EB : 24'hCBD5E1;
                if (state_bar_2) rgb_base_next = state_run_sel    ? 24'h16A34A : 24'hCBD5E1;
                if (state_bar_3) rgb_base_next = state_stop_sel   ? 24'hDC2626 : 24'hCBD5E1;
                if (bt_status_box) rgb_base_next = bt_connected ? 24'hDCFCE7 : 24'hE5E7EB;
                if (info_bar_total) rgb_base_next = 24'h2563EB;
                if (info_bar_done) rgb_base_next = 24'h16A34A;
                if (info_bar_remain) rgb_base_next = 24'hF59E0B;
                if (info_bar_speed) rgb_base_next = 24'h7C3AED;
              end
            end

            always @(*) begin
              case (text_style)
                TEXT_STATE0: text_color_next = state_menu_sel ? 24'hFFFFFF : 24'h334155;
                TEXT_STATE1: text_color_next = state_change_sel ? 24'hFFFFFF : 24'h334155;
                TEXT_STATE2: text_color_next = state_run_sel ? 24'hFFFFFF : 24'h334155;
                TEXT_STATE3: text_color_next = state_stop_sel ? 24'hFFFFFF : 24'h334155;
                TEXT_CHANGE: text_color_next = change_select ? 24'hFFFFFF : 24'h1D4ED8;
                TEXT_RUN: text_color_next = run_select ? 24'hFFFFFF : 24'h166534;
                default: text_color_next = 24'h0F172A;
              endcase
            end

            // Stage one captures all geometry and the selected glyph. The
            // second stage below only performs one 5x7 lookup and a short
            // priority color mux, keeping the 50 MHz pixel path bounded.
            always @(posedge clk_pix or negedge rst_n) begin
              if (!rst_n) begin
                pixel_de_q <= 1'b0;
                pixel_x_q <= 12'd0;
                pixel_y_q <= 12'd0;
                rgb_base_q <= 24'h000000;
                old_route_pixel_q <= 1'b0;
                active_route_pixel_q <= 1'b0;
                target_marker_q <= 1'b0;
                now_marker_q <= 1'b0;
                car_body_q <= 1'b0;
                car_arrow_q <= 1'b0;
                stop_line_pixel_q <= 1'b0;
                border_line_q <= 1'b0;
                text_char_q <= " ";
                text_base_x_q <= 12'd0;
                text_base_y_q <= 12'd0;
                text_style_q <= TEXT_NONE;
                text_color_q <= 24'h0F172A;
              end else begin
                pixel_de_q <= pixel_de;
                pixel_x_q <= pixel_x;
                pixel_y_q <= pixel_y;
                rgb_base_q <= rgb_base_next;
                old_route_pixel_q <= old_route_pixel;
                active_route_pixel_q <= active_route_pixel;
                target_marker_q <= target_marker;
                now_marker_q <= now_marker;
                car_body_q <= car_body;
                car_arrow_q <= car_arrow;
                stop_line_pixel_q <= stop_line_pixel;
                border_line_q <= border_line;
                text_char_q <= text_char;
                text_base_x_q <= text_base_x;
                text_base_y_q <= text_base_y;
                text_style_q <= text_style;
                text_color_q <= text_color_next;
              end
            end

            assign text_pixel = (text_style_q != TEXT_NONE) &&
                glyph5x7_2x_hit(text_char_q, pixel_x_q, pixel_y_q,
                                text_base_x_q, text_base_y_q);

            always @(*) begin
              if (!pixel_de_q) begin
                rgb_next = 24'h000000;
              end else begin
                rgb_next = rgb_base_q;
                if (text_pixel) rgb_next = text_color_q;
                if (old_route_pixel_q) rgb_next = 24'h94A3B8;
                if (active_route_pixel_q) rgb_next = 24'h2563EB;
                if (target_marker_q) rgb_next = 24'h0891B2;
                if (now_marker_q) rgb_next = 24'h64748B;
                if (car_body_q) rgb_next = 24'h111827;
                if (car_arrow_q) rgb_next = 24'hFACC15;
                if (stop_line_pixel_q) rgb_next = 24'hDC2626;
                if (border_line_q) rgb_next = 24'h334155;
              end
            end

            always @(posedge clk_pix or negedge rst_n) begin
              if (!rst_n)
                rgb_data <= 24'h000000;
              else
                rgb_data <= rgb_next;
            end
endmodule    //gui_route_renderer
