module bt_route_uart (
  input wire clk,
  input wire rst_n,
  input wire bt_uart_rx,
  input wire [1:0] menu_info,
  input wire [4:0] state_info,
  input wire [63:0] route_info,
  input wire [5:0] active_route_steps,
  input wire [5:0] completed_route_steps,
  input wire [7:0] now_x,
  input wire [7:0] now_y,
  input wire [7:0] target_x,
  input wire [7:0] target_y,
  input wire setting_mode,
  input wire [2:0] selected_param,
  input wire [15:0] step_length_set,
  input wire [9:0] overall_speed,
  input wire [9:0] lf_default_speed,
  input wire [9:0] rf_default_speed,
  input wire [9:0] lr_default_speed,
  input wire [9:0] rr_default_speed,
  input wire lidar_online,
  input wire lidar_obstacle_stop,
  input wire ultrasonic_obstacle_stop,
  input wire merged_obstacle_stop,
  input wire motor_blocked,
  input wire [15:0] ultrasonic_distance_cm,
  input wire ultrasonic_distance_valid,
  input wire ultrasonic_echo_timeout,
  input wire [15:0] lidar_front_distance_mm,
  input wire [15:0] lidar_safe_mm,
  input wire [15:0] lidar_sector_front_mm,
  input wire [15:0] lidar_sector_front_right_mm,
  input wire [15:0] lidar_sector_right_mm,
  input wire [15:0] lidar_sector_rear_right_mm,
  input wire [15:0] lidar_sector_rear_mm,
  input wire [15:0] lidar_sector_rear_left_mm,
  input wire [15:0] lidar_sector_left_mm,
  input wire [15:0] lidar_sector_front_left_mm,
  input wire [15:0] c5_frame_count,
  input wire [11:0] c5_last_lf_pulse,
  input wire [11:0] c5_last_rf_pulse,
  input wire [11:0] c5_last_lr_pulse,
  input wire [11:0] c5_last_rr_pulse,
  input wire c5_stop_frames_complete,
  input wire [1:0] c5_stop_frames_remaining,
  input wire c5_frame_busy,
  input wire map_chunk_valid,
  input wire [7:0] map_scan_seq,
  input wire [15:0] map_tick_10ms,
  input wire [2:0] map_part,
  input wire [143:0] map_chunk_data,
  input wire [7:0] map_chunk_crc,
  input wire grid_chunk_valid,
  input wire [7:0] grid_chunk_seq,
  input wire [13:0] grid_chunk_base,
  input wire [255:0] grid_chunk_data,
  input wire [7:0] grid_chunk_crc,
  input wire onboard_map_active,
  input wire onboard_map_paused,
  input wire onboard_map_valid,
  input wire onboard_map_busy,
  input wire [7:0] onboard_map_epoch,
  input wire [15:0] onboard_map_scan_count,
  input wire [15:0] onboard_map_occupied_count,
  input wire [15:0] onboard_map_free_count,
  input wire [15:0] onboard_map_dropped_count,
  input wire [15:0] onboard_pose_x_mm,
  input wire [15:0] onboard_pose_y_mm,
  input wire [9:0] onboard_pose_heading,
  input wire [7:0] onboard_pose_flags,
  input wire flash_ready,
  input wire flash_busy,
  input wire [1:0] flash_operation,
  input wire [7:0] flash_progress,
  input wire [7:0] flash_error,
  input wire [23:0] flash_jedec_id,
  input wire [31:0] flash_saved_sequence,
  input wire vl53_initialization_done,
  input wire [4:0] vl53_sensor_online,
  input wire [4:0] vl53_distance_valid,
  input wire [79:0] vl53_distance_mm,
  input wire [15:0] vl53_error_count,
  input wire [319:0] motion_telemetry_data,
  input wire [3:0] local_calibration_state,
  input wire [7:0] local_calibration_flags,
  input wire [10:0] learned_yaw_trim,
  input wire [10:0] yaw_correction,
  input wire [15:0] calibration_gyro_peak,
  output wire bt_uart_tx,
  output wire bt_connected,
  output wire bt_key_up_n,
  output wire bt_key_down_n,
  output wire bt_key_left_n,
  output wire bt_key_right_n,
  output wire bt_key_mid_n,
  output reg bt_cfg_mode_pulse,
  output reg bt_cfg_next_pulse,
  output reg bt_cfg_prev_pulse,
  output reg bt_cfg_inc_pulse,
  output reg bt_cfg_dec_pulse,
  output reg [2:0] bt_cfg_select,
  output reg bt_cfg_select_valid,
  output reg manual_mode,
  output reg [2:0] manual_cmd,
  output reg lidar_enable,
  output reg motor_stop_pulse,
  output reg local_calibration_start_pulse,
  output reg map_chunk_accept,
  output reg onboard_map_start_pulse,
  output reg onboard_map_pause_pulse,
  output reg onboard_map_clear_pulse,
  output reg onboard_map_save_pulse,
  output reg onboard_map_load_pulse,
  output reg onboard_map_sync_pulse,
  output reg gyro_bias_learn_pulse,
  output reg grid_chunk_accept
);

              localparam integer CLK_HZ = 50000000;
              localparam integer BAUD = 115200;
              localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
              localparam integer KEY_HOLD_CYCLES = (CLK_HZ / 1000) * 30;




              // The FPGA consumes the full-rate sensor streams locally. BLE
              // receives bounded-rate snapshots: fast enough for calibration
              // feedback and phone-side mapping, but still well below the
              // 115200-baud CH9141K UART capacity.
`ifdef BT_MAP_SIM_FAST
              localparam integer STATUS_CYCLES = 50000;
              localparam integer VL53_STATUS_CYCLES = 50000000;
              localparam integer MOTION_STATUS_CYCLES = 50000000;
              localparam integer GRID_STATUS_CYCLES = 1000;
              localparam integer TELEMETRY_GAP_CYCLES = 1000;
              localparam integer BT_STARTUP_IDLE_CYCLES = 64;
              localparam integer BT_STARTUP_COMMAND_GAP_CYCLES = 32;
              localparam integer BT_STARTUP_REBOOT_CYCLES = 128;
              localparam integer BT_RECOVERY_TRIGGER_CYCLES = 256;
              localparam integer BT_STARTUP_RECOVERY_CYCLES = 20000;
`else
              localparam integer STATUS_CYCLES = CLK_HZ * 5;
              localparam integer VL53_STATUS_CYCLES = CLK_HZ * 10;
              localparam integer MOTION_STATUS_CYCLES = CLK_HZ;
              // Pose changes may be consumed by phone-side heading fusion, so
              // report changed @G0 state at up to 5 Hz. The fixed 59-byte frame
              // still occupies only a small fraction of a 115200-baud link.
              localparam integer GRID_STATUS_CYCLES = CLK_HZ / 5;
              // Leave a short scheduling gap between complete telemetry frames.
              // Command replies use a separate higher-priority path and remain
              // immediate; the gap prevents a producer from monopolizing TX.
              // Leave the no-flow-control CH9141K time to drain its UART-to-BLE
              // queue. Command acknowledgements and PONG replies retain strict
              // priority and are not delayed by this telemetry-only pacing.
              localparam integer TELEMETRY_GAP_CYCLES = CLK_HZ / 20;
              // CH9141K accepts the serial AT-entry sequence only after at
              // least 500 ms of UART idle.  Reset it once after FPGA startup,
              // then leave enough time for slave-mode advertising to resume.
              localparam integer BT_STARTUP_IDLE_CYCLES = (CLK_HZ * 3) / 5;
              localparam integer BT_STARTUP_COMMAND_GAP_CYCLES = CLK_HZ / 20;
              localparam integer BT_STARTUP_REBOOT_CYCLES = CLK_HZ / 2;
              // link_active drops three seconds after the last heartbeat. Give
              // Android ample time to reconnect before resetting the radio;
              // any UART byte cancels and restarts this recovery timer.
              localparam integer BT_RECOVERY_TRIGGER_CYCLES = CLK_HZ * 30;
              localparam integer BT_STARTUP_RECOVERY_CYCLES = CLK_HZ * 8;
`endif
            localparam integer LINK_TIMEOUT_CYCLES = CLK_HZ * 3;
            localparam integer MANUAL_TIMEOUT_CYCLES = (CLK_HZ / 10) * 3;
          localparam [6:0] ROUTE_STATUS_LEFT = 7'd98;
            localparam [6:0] LIDAR_STATUS_LEFT = 7'd55;
            localparam [5:0] MAP_STATUS_LEFT = 6'd49;
            localparam [5:0] VL53_STATUS_LEFT = 6'd32;
            localparam [6:0] MOTION_STATUS_LEFT = 7'd83;
            localparam [6:0] GRID_STATUS_LEFT = 7'd57;
            localparam [6:0] GRID_CHUNK_LEFT = 7'd75;
              localparam [2:0] REPLY_ACK_LAST = 3'd3;
              localparam [2:0] REPLY_PONG_LAST = 3'd5;
              localparam [1:0] REPLY_NONE = 2'd0;
              localparam [1:0] REPLY_KEY  = 2'd1;
              localparam [1:0] REPLY_PONG = 2'd2;

              localparam [1:0] RX_IDLE  = 2'd0;
              localparam [1:0] RX_START = 2'd1;
              localparam [1:0] RX_DATA  = 2'd2;
              localparam [1:0] RX_STOP  = 2'd3;
              localparam [1:0] TX_IDLE  = 2'd0;
              localparam [1:0] TX_START = 2'd1;
              localparam [1:0] TX_DATA  = 2'd2;
            localparam [1:0] TX_STOP  = 2'd3;
            localparam [2:0] MANUAL_STOP    = 3'd0;
            localparam [2:0] MANUAL_FORWARD = 3'd1;
            localparam [2:0] MANUAL_REVERSE = 3'd2;
            localparam [2:0] MANUAL_LEFT    = 3'd3;
            localparam [2:0] MANUAL_RIGHT   = 3'd4;

            localparam [2:0] BT_BOOT_IDLE   = 3'd0;
            localparam [2:0] BT_BOOT_ENTER  = 3'd1;
            localparam [2:0] BT_BOOT_GAP    = 3'd2;
            localparam [2:0] BT_BOOT_RESET  = 3'd3;
            localparam [2:0] BT_BOOT_REBOOT = 3'd4;
            localparam [2:0] BT_BOOT_READY  = 3'd5;

              reg [1:0] rx_sync;
              reg [1:0] rx_state;
              reg [15:0] rx_clk_count;
              reg [2:0] rx_bit_index;
              reg [7:0] rx_shift;
              reg [7:0] rx_data;
              reg rx_valid;

              reg [21:0] up_hold_count;
              reg [21:0] down_hold_count;
              reg [21:0] left_hold_count;
              reg [21:0] right_hold_count;
              reg [21:0] mid_hold_count;

              reg [31:0] status_count;
            reg [6:0] status_left;
            reg [6:0] status_index;
              reg active_status_lidar;
              reg [7:0] tx_data;
              reg tx_start;
              reg tx_line;
              reg tx_busy;
              reg tx_busy_d;
              reg [1:0] tx_state;
              reg [15:0] tx_clk_count;
              reg [2:0] tx_bit_index;
              reg [7:0] tx_shift;
              reg [2:0] bt_boot_state;
              reg [25:0] bt_boot_count;
              reg [3:0] bt_boot_byte_index;
              reg [31:0] bt_recovery_count;
              reg bt_link_was_active;
              reg bt_startup_recovery_armed;
              reg [7:0] bt_boot_tx_data;
              reg bt_boot_tx_start;
              reg bt_boot_tx_line;
              reg bt_boot_tx_busy;
              reg bt_boot_tx_done;
              reg [1:0] bt_boot_tx_state;
              reg [15:0] bt_boot_tx_clk_count;
              reg [2:0] bt_boot_tx_bit_index;
              reg [7:0] bt_boot_tx_shift;
              reg sending_status;
              reg sending_reply;
              reg sending_map;
              reg sending_vl53;
              reg sending_motion;
              reg sending_grid_status;
              reg sending_grid_chunk;
              reg status_packet_lidar;
              reg pending_reply;
              reg [1:0] reply_kind;
              reg [1:0] active_reply_kind;
              reg [2:0] reply_index;
              reg [7:0] reply_key;
              reg [7:0] active_reply_key;
              reg [5:0] map_left;
              reg [5:0] map_index;
              reg [7:0] active_map_scan_seq;
              reg [15:0] active_map_tick_10ms;
              reg [2:0] active_map_part;
              reg [143:0] active_map_chunk_data;
              reg [7:0] active_map_chunk_crc;
              reg [31:0] grid_status_count;
              reg grid_status_pending;
              reg [7:0] grid_status_seq;
              reg [7:0] active_grid_status_seq;
              reg [6:0] grid_status_left;
              reg [6:0] grid_status_index;
              reg [7:0] active_grid_flags;
              reg [7:0] active_grid_epoch;
              reg [15:0] active_grid_scan_count;
              reg [15:0] active_grid_occupied_count;
              reg [15:0] active_grid_free_count;
              reg [15:0] active_grid_dropped_count;
              reg [15:0] active_grid_pose_x_mm;
              reg [15:0] active_grid_pose_y_mm;
              reg [9:0] active_grid_pose_heading;
              reg [7:0] active_grid_pose_flags;
              reg [1:0] active_flash_operation;
              reg [7:0] active_flash_progress;
              reg [7:0] active_flash_error;
              reg [23:0] active_flash_jedec_id;
              reg [31:0] active_flash_saved_sequence;
              reg [6:0] grid_chunk_left;
              reg [6:0] grid_chunk_index;
              reg [7:0] active_grid_chunk_seq;
              reg [13:0] active_grid_chunk_base;
              reg [255:0] active_grid_chunk_data;
              reg [7:0] active_grid_chunk_crc;
              reg grid_yield_to_status;
              reg [31:0] vl53_status_count;
              reg [5:0] vl53_left;
              reg [5:0] vl53_index;
              reg [7:0] vl53_status_seq;
              reg [7:0] active_vl53_status_seq;
              reg status_vl53_initialization_done;
              reg [4:0] status_vl53_sensor_online;
              reg [4:0] status_vl53_distance_valid;
              reg [79:0] status_vl53_distance_mm;
              reg [15:0] status_vl53_error_count;
              reg [31:0] motion_status_count;
              reg [31:0] telemetry_gap_count;
              reg [6:0] motion_left;
              reg [6:0] motion_index;
              reg [319:0] motion_shift_data;
            reg [31:0] link_idle_count;
            reg link_active;
          reg [23:0] manual_timeout_count;
          reg manual_watchdog_expired;
            reg route_status_pending;
            reg [7:0] route_status_seq;
            reg [7:0] lidar_status_seq;
            reg [7:0] active_status_seq;
            reg [1:0] prev_menu_info;
            reg [4:0] prev_state_info;
            reg [63:0] prev_route_info;
            reg [5:0] prev_active_route_steps;
            reg [5:0] prev_completed_route_steps;
            reg [7:0] prev_now_x;
            reg [7:0] prev_now_y;
          reg [7:0] prev_target_x;
          reg [7:0] prev_target_y;
          reg [15:0] prev_step_length_set;
          reg [9:0] prev_overall_speed;
          reg [9:0] prev_lf_default_speed;
          reg [9:0] prev_rf_default_speed;
          reg [9:0] prev_lr_default_speed;
          reg [9:0] prev_rr_default_speed;
          reg [15:0] prev_lidar_safe_mm;
          reg prev_setting_mode;
          reg [2:0] prev_selected_param;
          reg prev_manual_mode;
          reg [2:0] prev_manual_cmd;
          reg prev_manual_watchdog_expired;
          reg prev_lidar_enable;
          reg prev_motor_blocked;
          reg [3:0] prev_local_calibration_state;
          reg [7:0] prev_local_calibration_flags;
          reg [10:0] prev_learned_yaw_trim;
          reg prev_onboard_map_active;
          reg prev_onboard_map_paused;
          reg prev_onboard_map_valid;
          reg prev_onboard_map_busy;
          reg [7:0] prev_onboard_map_epoch;
          reg [15:0] prev_onboard_map_scan_count;
          reg [15:0] prev_onboard_pose_x_mm;
          reg [15:0] prev_onboard_pose_y_mm;
          reg [9:0] prev_onboard_pose_heading;
          reg [7:0] prev_onboard_pose_flags;
          reg prev_flash_ready;
          reg prev_flash_busy;
          reg [1:0] prev_flash_operation;
          reg [7:0] prev_flash_error;
          reg [31:0] prev_flash_saved_sequence;
            reg [1:0] status_menu_info;
            reg [4:0] status_state_info;
            reg [63:0] status_route_info;
            reg [5:0] status_active_route_steps;
            reg [5:0] status_completed_route_steps;
            reg [7:0] status_now_x;
            reg [7:0] status_now_y;
            reg [7:0] status_target_x;
            reg [7:0] status_target_y;
            reg [15:0] status_step_length_set;
          reg [9:0] status_overall_speed;
          reg [9:0] status_lf_default_speed;
          reg [9:0] status_rf_default_speed;
          reg [9:0] status_lr_default_speed;
          reg [9:0] status_rr_default_speed;
          reg [15:0] status_lidar_safe_mm_route;
          reg status_setting_mode;
            reg [2:0] status_selected_param;
            reg status_lidar_online;
            reg status_manual_mode;
            reg status_lidar_enable;
          reg status_lidar_obstacle_stop;
          reg status_ultrasonic_obstacle_stop;
          reg status_merged_obstacle_stop;
          reg status_motor_blocked;
          reg status_ultrasonic_distance_valid;
          reg status_ultrasonic_echo_timeout;
          reg [15:0] status_ultrasonic_distance_cm;
          reg [2:0] status_manual_cmd;
          reg status_manual_watchdog_expired;
          reg [15:0] status_c5_frame_count;
          reg [11:0] status_c5_last_lf_pulse;
          reg [11:0] status_c5_last_rf_pulse;
          reg [11:0] status_c5_last_lr_pulse;
          reg [11:0] status_c5_last_rr_pulse;
          reg status_c5_stop_frames_complete;
          reg [1:0] status_c5_stop_frames_remaining;
          reg status_c5_frame_busy;
          reg [3:0] status_local_calibration_state;
          reg [7:0] status_local_calibration_flags;
          reg [10:0] status_learned_yaw_trim;
          reg [10:0] status_yaw_correction;
          reg [15:0] status_calibration_gyro_peak;
          reg [15:0] ultrasonic_last_distance_cm;
          reg ultrasonic_last_valid;
          reg ultrasonic_last_timeout;
          reg ultrasonic_last_stop;
            reg [15:0] status_lidar_safe_mm;
            reg [15:0] status_lidar_front_distance_mm;
            reg [15:0] status_lidar_sector_mm [0:7];

              wire status_due;
              wire tx_done;
              wire reply_done;
              wire [7:0] norm_key;
              wire is_cfg_command;
              wire is_onboard_map_command;
              wire route_status_changed;
              wire vl53_status_due;
              wire motion_status_due;
              wire grid_status_due;
              wire telemetry_slot_ready;
              wire onboard_map_status_changed;
              wire grid_frame_ready;
              wire bt_recovery_active;
              wire bt_startup_ready;
              wire protocol_rst_n;

              // CH9141K is configured persistently as a BLE transparent-UART
              // slave.  Sending AT+RESET on every FPGA reset can interrupt an
              // already advertising or connected module. Recovery therefore
              // runs only after a previously live heartbeat link has remained
              // absent long enough for Android's BLE supervision timeout.
              assign bt_recovery_active = (bt_boot_state != BT_BOOT_READY);
              assign bt_startup_ready = !bt_recovery_active;
              assign protocol_rst_n = rst_n && !bt_recovery_active;
              assign bt_uart_tx = bt_recovery_active ? bt_boot_tx_line : tx_line;
              assign bt_connected = link_active;
              assign bt_key_up_n = (up_hold_count == 22'd0);
              assign bt_key_down_n = (down_hold_count == 22'd0);
              assign bt_key_left_n = (left_hold_count == 22'd0);
              assign bt_key_right_n = (right_hold_count == 22'd0);
              assign bt_key_mid_n = (mid_hold_count == 22'd0);
              assign status_due = (status_count >= STATUS_CYCLES - 1);
              assign vl53_status_due =
                  (vl53_status_count >= VL53_STATUS_CYCLES - 1);
              assign motion_status_due =
                  (motion_status_count >= MOTION_STATUS_CYCLES - 1);
              assign grid_status_due =
                  (grid_status_count >= GRID_STATUS_CYCLES - 1);
              assign telemetry_slot_ready =
                  (telemetry_gap_count >= TELEMETRY_GAP_CYCLES - 1);
              assign tx_done = tx_busy_d && !tx_busy;
              assign reply_done = (active_reply_kind == REPLY_KEY) ?
                                  (reply_index == REPLY_ACK_LAST) :
                                  (reply_index == REPLY_PONG_LAST);
              assign norm_key = normalized_key(rx_data);
              assign is_cfg_command = cfg_command(rx_data);
              assign is_onboard_map_command = onboard_map_command(rx_data);
              assign route_status_changed =
                  (menu_info != prev_menu_info) ||
                  (state_info != prev_state_info) ||
                  (route_info != prev_route_info) ||
                  (active_route_steps != prev_active_route_steps) ||
                (completed_route_steps != prev_completed_route_steps) ||
                (now_x != prev_now_x) || (now_y != prev_now_y) ||
                (target_x != prev_target_x) || (target_y != prev_target_y) ||
                (step_length_set != prev_step_length_set) ||
                (overall_speed != prev_overall_speed) ||
                (lf_default_speed != prev_lf_default_speed) ||
                (rf_default_speed != prev_rf_default_speed) ||
                (lr_default_speed != prev_lr_default_speed) ||
                (rr_default_speed != prev_rr_default_speed) ||
                (lidar_safe_mm != prev_lidar_safe_mm) ||
                (setting_mode != prev_setting_mode) ||
                (selected_param != prev_selected_param) ||
                (manual_mode != prev_manual_mode) ||
                (manual_cmd != prev_manual_cmd) ||
                (manual_watchdog_expired != prev_manual_watchdog_expired) ||
                (lidar_enable != prev_lidar_enable) ||
                (motor_blocked != prev_motor_blocked) ||
                (local_calibration_state != prev_local_calibration_state) ||
                // Obstacle and sensor-ready bits can chatter while the radar
                // and IMUs settle after power-up. They remain present in every
                // status snapshot, but @L and the periodic @S frame report
                // them without turning each edge into another 99-byte frame.
                (local_calibration_flags[5:0] !=
                 prev_local_calibration_flags[5:0]) ||
                (learned_yaw_trim != prev_learned_yaw_trim);
              assign onboard_map_status_changed =
                  (onboard_map_active != prev_onboard_map_active) ||
                  (onboard_map_paused != prev_onboard_map_paused) ||
                  (onboard_map_valid != prev_onboard_map_valid) ||
                  (onboard_map_busy != prev_onboard_map_busy) ||
                  (onboard_map_epoch != prev_onboard_map_epoch) ||
                  (onboard_map_scan_count != prev_onboard_map_scan_count) ||
                  (onboard_pose_x_mm != prev_onboard_pose_x_mm) ||
                  (onboard_pose_y_mm != prev_onboard_pose_y_mm) ||
                  (onboard_pose_heading != prev_onboard_pose_heading) ||
                  (onboard_pose_flags != prev_onboard_pose_flags) ||
                  (flash_ready != prev_flash_ready) ||
                  (flash_busy != prev_flash_busy) ||
                  (flash_operation != prev_flash_operation) ||
                  (flash_error != prev_flash_error) ||
                  (flash_saved_sequence != prev_flash_saved_sequence);
              assign grid_frame_ready =
                  (grid_status_pending && grid_status_due) || grid_chunk_valid;

              function [7:0] bt_boot_command_char;
                input reset_command;
                input [3:0] index;
                begin
                  if (!reset_command) begin
                    case (index)
                      4'd0: bt_boot_command_char = "A";
                      4'd1: bt_boot_command_char = "T";
                      4'd2, 4'd3, 4'd4: bt_boot_command_char = ".";
                      4'd5: bt_boot_command_char = 8'h0d;
                      default: bt_boot_command_char = 8'h0a;
                    endcase
                  end else begin
                    case (index)
                      4'd0: bt_boot_command_char = "A";
                      4'd1: bt_boot_command_char = "T";
                      4'd2: bt_boot_command_char = "+";
                      4'd3: bt_boot_command_char = "R";
                      4'd4: bt_boot_command_char = "E";
                      4'd5: bt_boot_command_char = "S";
                      4'd6: bt_boot_command_char = "E";
                      4'd7: bt_boot_command_char = "T";
                      4'd8: bt_boot_command_char = 8'h0d;
                      default: bt_boot_command_char = 8'h0a;
                    endcase
                  end
                end
              endfunction

              function [7:0] hex_char;
                input [3:0] value;
                begin
                  hex_char = value + ((value < 4'd10) ? 8'd48 : 8'd55);
                end
              endfunction

              function [7:0] normalized_key;
                input [7:0] value;
                begin
                  case (value)
                    "U", "u", "W", "w": normalized_key = "U";
                    "D", "d", "S", "s": normalized_key = "D";
                    "L", "l", "A", "a": normalized_key = "L";
                    "R", "r":           normalized_key = "R";
                    "M", "m", "C", "c": normalized_key = "M";
                    default:             normalized_key = 8'd0;
                  endcase
                end
              endfunction

            function [7:0] status_char;
              input lidar_packet;
              input [6:0] char_index;
                begin
                  if (lidar_packet) begin
                    case (char_index)
                      7'd0: status_char = "L";
                      7'd1: status_char = hex_char({3'd0, status_lidar_online});
                      7'd2: status_char = hex_char({3'd0, status_lidar_obstacle_stop});
                      7'd3: status_char = hex_char({3'd0, status_ultrasonic_obstacle_stop});
                      7'd4: status_char = hex_char({3'd0, status_merged_obstacle_stop});
                      7'd5: status_char = hex_char({3'd0, status_motor_blocked});
                      7'd6: status_char = hex_char({3'd0, status_ultrasonic_distance_valid});
                      7'd7: status_char = hex_char({3'd0, status_ultrasonic_echo_timeout});
                      7'd8: status_char = hex_char(status_lidar_safe_mm[15:12]);
                      7'd9: status_char = hex_char(status_lidar_safe_mm[11:8]);
                      7'd10: status_char = hex_char(status_lidar_safe_mm[7:4]);
                      7'd11: status_char = hex_char(status_lidar_safe_mm[3:0]);
                      7'd12: status_char = hex_char(status_ultrasonic_distance_cm[15:12]);
                      7'd13: status_char = hex_char(status_ultrasonic_distance_cm[11:8]);
                      7'd14: status_char = hex_char(status_ultrasonic_distance_cm[7:4]);
                      7'd15: status_char = hex_char(status_ultrasonic_distance_cm[3:0]);
                      7'd16: status_char = hex_char(status_lidar_front_distance_mm[15:12]);
                      7'd17: status_char = hex_char(status_lidar_front_distance_mm[11:8]);
                      7'd18: status_char = hex_char(status_lidar_front_distance_mm[7:4]);
                      7'd19: status_char = hex_char(status_lidar_front_distance_mm[3:0]);
                      7'd20: status_char = hex_char(status_lidar_sector_mm[0][15:12]);
                      7'd21: status_char = hex_char(status_lidar_sector_mm[0][11:8]);
                      7'd22: status_char = hex_char(status_lidar_sector_mm[0][7:4]);
                      7'd23: status_char = hex_char(status_lidar_sector_mm[0][3:0]);
                      7'd24: status_char = hex_char(status_lidar_sector_mm[1][15:12]);
                      7'd25: status_char = hex_char(status_lidar_sector_mm[1][11:8]);
                      7'd26: status_char = hex_char(status_lidar_sector_mm[1][7:4]);
                      7'd27: status_char = hex_char(status_lidar_sector_mm[1][3:0]);
                      7'd28: status_char = hex_char(status_lidar_sector_mm[2][15:12]);
                      7'd29: status_char = hex_char(status_lidar_sector_mm[2][11:8]);
                      7'd30: status_char = hex_char(status_lidar_sector_mm[2][7:4]);
                      7'd31: status_char = hex_char(status_lidar_sector_mm[2][3:0]);
                      7'd32: status_char = hex_char(status_lidar_sector_mm[3][15:12]);
                      7'd33: status_char = hex_char(status_lidar_sector_mm[3][11:8]);
                      7'd34: status_char = hex_char(status_lidar_sector_mm[3][7:4]);
                      7'd35: status_char = hex_char(status_lidar_sector_mm[3][3:0]);
                      7'd36: status_char = hex_char(status_lidar_sector_mm[4][15:12]);
                      7'd37: status_char = hex_char(status_lidar_sector_mm[4][11:8]);
                      7'd38: status_char = hex_char(status_lidar_sector_mm[4][7:4]);
                      7'd39: status_char = hex_char(status_lidar_sector_mm[4][3:0]);
                      7'd40: status_char = hex_char(status_lidar_sector_mm[5][15:12]);
                      7'd41: status_char = hex_char(status_lidar_sector_mm[5][11:8]);
                      7'd42: status_char = hex_char(status_lidar_sector_mm[5][7:4]);
                      7'd43: status_char = hex_char(status_lidar_sector_mm[5][3:0]);
                      7'd44: status_char = hex_char(status_lidar_sector_mm[6][15:12]);
                      7'd45: status_char = hex_char(status_lidar_sector_mm[6][11:8]);
                      7'd46: status_char = hex_char(status_lidar_sector_mm[6][7:4]);
                      7'd47: status_char = hex_char(status_lidar_sector_mm[6][3:0]);
                      7'd48: status_char = hex_char(status_lidar_sector_mm[7][15:12]);
                      7'd49: status_char = hex_char(status_lidar_sector_mm[7][11:8]);
                      7'd50: status_char = hex_char(status_lidar_sector_mm[7][7:4]);
                      7'd51: status_char = hex_char(status_lidar_sector_mm[7][3:0]);
                      7'd52: status_char = hex_char(active_status_seq[7:4]);
                      7'd53: status_char = hex_char(active_status_seq[3:0]);
                      default: status_char = 8'd10;
                    endcase
                  end else begin
                    case (char_index)
                      6'd0: status_char = "S";
                      6'd1: status_char = hex_char({3'd0, status_state_info[4]});
                      6'd2: status_char = hex_char(status_state_info[3:0]);
                      6'd3: status_char = hex_char({2'd0, status_menu_info});
                      6'd4: status_char = hex_char({2'd0, status_active_route_steps[5:4]});
                      6'd5: status_char = hex_char(status_active_route_steps[3:0]);
                      6'd6: status_char = hex_char({2'd0, status_completed_route_steps[5:4]});
                      6'd7: status_char = hex_char(status_completed_route_steps[3:0]);
                      6'd8: status_char = hex_char(status_now_x[7:4]);
                      6'd9: status_char = hex_char(status_now_x[3:0]);
                      6'd10: status_char = hex_char(status_now_y[7:4]);
                      6'd11: status_char = hex_char(status_now_y[3:0]);
                      6'd12: status_char = hex_char(status_target_x[7:4]);
                      6'd13: status_char = hex_char(status_target_x[3:0]);
                      6'd14: status_char = hex_char(status_target_y[7:4]);
                      6'd15: status_char = hex_char(status_target_y[3:0]);
                      6'd16: status_char = hex_char(status_route_info[63:60]);
                      6'd17: status_char = hex_char(status_route_info[59:56]);
                      6'd18: status_char = hex_char(status_route_info[55:52]);
                      6'd19: status_char = hex_char(status_route_info[51:48]);
                      6'd20: status_char = hex_char(status_route_info[47:44]);
                      6'd21: status_char = hex_char(status_route_info[43:40]);
                      6'd22: status_char = hex_char(status_route_info[39:36]);
                      6'd23: status_char = hex_char(status_route_info[35:32]);
                      6'd24: status_char = hex_char(status_route_info[31:28]);
                      6'd25: status_char = hex_char(status_route_info[27:24]);
                      6'd26: status_char = hex_char(status_route_info[23:20]);
                      6'd27: status_char = hex_char(status_route_info[19:16]);
                      6'd28: status_char = hex_char(status_route_info[15:12]);
                      6'd29: status_char = hex_char(status_route_info[11:8]);
                      6'd30: status_char = hex_char(status_route_info[7:4]);
                      6'd31: status_char = hex_char(status_route_info[3:0]);
                      6'd32: status_char = hex_char(status_step_length_set[15:12]);
                      6'd33: status_char = hex_char(status_step_length_set[11:8]);
                      6'd34: status_char = hex_char(status_step_length_set[7:4]);
                      6'd35: status_char = hex_char(status_step_length_set[3:0]);
                      6'd36: status_char = hex_char({2'd0, status_overall_speed[9:8]});
                      6'd37: status_char = hex_char(status_overall_speed[7:4]);
                      6'd38: status_char = hex_char(status_overall_speed[3:0]);
                      7'd39: status_char = hex_char({2'd0, status_lf_default_speed[9:8]});
                      7'd40: status_char = hex_char(status_lf_default_speed[7:4]);
                      7'd41: status_char = hex_char(status_lf_default_speed[3:0]);
                      7'd42: status_char = hex_char({2'd0, status_rf_default_speed[9:8]});
                      7'd43: status_char = hex_char(status_rf_default_speed[7:4]);
                      7'd44: status_char = hex_char(status_rf_default_speed[3:0]);
                      7'd45: status_char = hex_char({2'd0, status_lr_default_speed[9:8]});
                      7'd46: status_char = hex_char(status_lr_default_speed[7:4]);
                      7'd47: status_char = hex_char(status_lr_default_speed[3:0]);
                      7'd48: status_char = hex_char({2'd0, status_rr_default_speed[9:8]});
                      7'd49: status_char = hex_char(status_rr_default_speed[7:4]);
                      7'd50: status_char = hex_char(status_rr_default_speed[3:0]);
                      7'd51: status_char = hex_char(status_lidar_safe_mm_route[15:12]);
                      7'd52: status_char = hex_char(status_lidar_safe_mm_route[11:8]);
                      7'd53: status_char = hex_char(status_lidar_safe_mm_route[7:4]);
                      7'd54: status_char = hex_char(status_lidar_safe_mm_route[3:0]);
                      7'd55: status_char = hex_char({3'd0, status_setting_mode});
                      7'd56: status_char = hex_char({1'd0, status_selected_param});
                      7'd57: status_char = hex_char({3'd0, status_lidar_online});
                      7'd58: status_char = hex_char({3'd0, status_manual_mode});
                      7'd59: status_char = hex_char({3'd0, status_lidar_enable});
                      7'd60: status_char = hex_char({1'd0, status_manual_cmd});
                      7'd61: status_char = hex_char({3'd0, status_manual_watchdog_expired});
                      7'd62: status_char = hex_char({3'd0, status_motor_blocked});
                      7'd63: status_char = hex_char(status_c5_frame_count[15:12]);
                      7'd64: status_char = hex_char(status_c5_frame_count[11:8]);
                      7'd65: status_char = hex_char(status_c5_frame_count[7:4]);
                      7'd66: status_char = hex_char(status_c5_frame_count[3:0]);
                      7'd67: status_char = hex_char(status_c5_last_lf_pulse[11:8]);
                      7'd68: status_char = hex_char(status_c5_last_lf_pulse[7:4]);
                      7'd69: status_char = hex_char(status_c5_last_lf_pulse[3:0]);
                      7'd70: status_char = hex_char(status_c5_last_rf_pulse[11:8]);
                      7'd71: status_char = hex_char(status_c5_last_rf_pulse[7:4]);
                      7'd72: status_char = hex_char(status_c5_last_rf_pulse[3:0]);
                      7'd73: status_char = hex_char(status_c5_last_lr_pulse[11:8]);
                      7'd74: status_char = hex_char(status_c5_last_lr_pulse[7:4]);
                      7'd75: status_char = hex_char(status_c5_last_lr_pulse[3:0]);
                      7'd76: status_char = hex_char(status_c5_last_rr_pulse[11:8]);
                      7'd77: status_char = hex_char(status_c5_last_rr_pulse[7:4]);
                      7'd78: status_char = hex_char(status_c5_last_rr_pulse[3:0]);
                      7'd79: status_char = hex_char({3'd0, status_c5_stop_frames_complete});
                      7'd80: status_char = hex_char({2'd0, status_c5_stop_frames_remaining});
                      7'd81: status_char = hex_char({3'd0, status_c5_frame_busy});
                      7'd82: status_char = hex_char(active_status_seq[7:4]);
                      7'd83: status_char = hex_char(active_status_seq[3:0]);
                      7'd84: status_char = hex_char(status_local_calibration_state);
                      7'd85: status_char = hex_char(status_local_calibration_flags[7:4]);
                      7'd86: status_char = hex_char(status_local_calibration_flags[3:0]);
                      7'd87: status_char = hex_char({status_learned_yaw_trim[10], status_learned_yaw_trim[10:8]});
                      7'd88: status_char = hex_char(status_learned_yaw_trim[7:4]);
                      7'd89: status_char = hex_char(status_learned_yaw_trim[3:0]);
                      7'd90: status_char = hex_char({status_yaw_correction[10], status_yaw_correction[10:8]});
                      7'd91: status_char = hex_char(status_yaw_correction[7:4]);
                      7'd92: status_char = hex_char(status_yaw_correction[3:0]);
                      7'd93: status_char = hex_char(status_calibration_gyro_peak[15:12]);
                      7'd94: status_char = hex_char(status_calibration_gyro_peak[11:8]);
                      7'd95: status_char = hex_char(status_calibration_gyro_peak[7:4]);
                      7'd96: status_char = hex_char(status_calibration_gyro_peak[3:0]);
                    default: status_char = 8'd10;
                    endcase
                  end
                end
              endfunction

              function [7:0] map_char;
                input [5:0] char_index;
                integer data_bit;
                begin
                  if ((char_index >= 6'd10) && (char_index <= 6'd45)) begin
                    data_bit = (char_index - 6'd10) * 4;
                    map_char = hex_char(active_map_chunk_data[data_bit +: 4]);
                  end else begin
                    case (char_index)
                      6'd0: map_char = "R";
                      6'd1: map_char = "1";
                      6'd2: map_char = hex_char(active_map_scan_seq[7:4]);
                      6'd3: map_char = hex_char(active_map_scan_seq[3:0]);
                      6'd4: map_char = hex_char(active_map_tick_10ms[15:12]);
                      6'd5: map_char = hex_char(active_map_tick_10ms[11:8]);
                      6'd6: map_char = hex_char(active_map_tick_10ms[7:4]);
                      6'd7: map_char = hex_char(active_map_tick_10ms[3:0]);
                      6'd8: map_char = hex_char({1'b0, active_map_part});
                      6'd9: map_char = "6";
                      6'd46: map_char = hex_char(active_map_chunk_crc[7:4]);
                      6'd47: map_char = hex_char(active_map_chunk_crc[3:0]);
                      default: map_char = 8'd10;
                    endcase
                  end
                end
              endfunction

              // @G0 carries the complete board-resident map/Flash state. The
              // payload is fixed-width hexadecimal so Android can recover at
              // the next newline even after a dropped BLE notification.
              function [7:0] grid_status_char;
                input [6:0] char_index;
                begin
                  case (char_index)
                    7'd0: grid_status_char = "G";
                    7'd1: grid_status_char = "0";
                    7'd2: grid_status_char = hex_char(active_grid_status_seq[7:4]);
                    7'd3: grid_status_char = hex_char(active_grid_status_seq[3:0]);
                    7'd4: grid_status_char = hex_char(active_grid_flags[7:4]);
                    7'd5: grid_status_char = hex_char(active_grid_flags[3:0]);
                    7'd6: grid_status_char = hex_char(active_grid_epoch[7:4]);
                    7'd7: grid_status_char = hex_char(active_grid_epoch[3:0]);
                    7'd8: grid_status_char = hex_char(active_grid_pose_x_mm[15:12]);
                    7'd9: grid_status_char = hex_char(active_grid_pose_x_mm[11:8]);
                    7'd10: grid_status_char = hex_char(active_grid_pose_x_mm[7:4]);
                    7'd11: grid_status_char = hex_char(active_grid_pose_x_mm[3:0]);
                    7'd12: grid_status_char = hex_char(active_grid_pose_y_mm[15:12]);
                    7'd13: grid_status_char = hex_char(active_grid_pose_y_mm[11:8]);
                    7'd14: grid_status_char = hex_char(active_grid_pose_y_mm[7:4]);
                    7'd15: grid_status_char = hex_char(active_grid_pose_y_mm[3:0]);
                    7'd16: grid_status_char = hex_char({2'd0, active_grid_pose_heading[9:8]});
                    7'd17: grid_status_char = hex_char(active_grid_pose_heading[7:4]);
                    7'd18: grid_status_char = hex_char(active_grid_pose_heading[3:0]);
                    7'd19: grid_status_char = hex_char(active_grid_scan_count[15:12]);
                    7'd20: grid_status_char = hex_char(active_grid_scan_count[11:8]);
                    7'd21: grid_status_char = hex_char(active_grid_scan_count[7:4]);
                    7'd22: grid_status_char = hex_char(active_grid_scan_count[3:0]);
                    7'd23: grid_status_char = hex_char(active_grid_occupied_count[15:12]);
                    7'd24: grid_status_char = hex_char(active_grid_occupied_count[11:8]);
                    7'd25: grid_status_char = hex_char(active_grid_occupied_count[7:4]);
                    7'd26: grid_status_char = hex_char(active_grid_occupied_count[3:0]);
                    7'd27: grid_status_char = hex_char(active_grid_free_count[15:12]);
                    7'd28: grid_status_char = hex_char(active_grid_free_count[11:8]);
                    7'd29: grid_status_char = hex_char(active_grid_free_count[7:4]);
                    7'd30: grid_status_char = hex_char(active_grid_free_count[3:0]);
                    7'd31: grid_status_char = hex_char(active_grid_dropped_count[15:12]);
                    7'd32: grid_status_char = hex_char(active_grid_dropped_count[11:8]);
                    7'd33: grid_status_char = hex_char(active_grid_dropped_count[7:4]);
                    7'd34: grid_status_char = hex_char(active_grid_dropped_count[3:0]);
                    7'd35: grid_status_char = hex_char(active_grid_pose_flags[7:4]);
                    7'd36: grid_status_char = hex_char(active_grid_pose_flags[3:0]);
                    7'd37: grid_status_char = hex_char({2'd0, active_flash_operation});
                    7'd38: grid_status_char = hex_char(active_flash_progress[7:4]);
                    7'd39: grid_status_char = hex_char(active_flash_progress[3:0]);
                    7'd40: grid_status_char = hex_char(active_flash_error[7:4]);
                    7'd41: grid_status_char = hex_char(active_flash_error[3:0]);
                    7'd42: grid_status_char = hex_char(active_flash_jedec_id[23:20]);
                    7'd43: grid_status_char = hex_char(active_flash_jedec_id[19:16]);
                    7'd44: grid_status_char = hex_char(active_flash_jedec_id[15:12]);
                    7'd45: grid_status_char = hex_char(active_flash_jedec_id[11:8]);
                    7'd46: grid_status_char = hex_char(active_flash_jedec_id[7:4]);
                    7'd47: grid_status_char = hex_char(active_flash_jedec_id[3:0]);
                    7'd48: grid_status_char = hex_char(active_flash_saved_sequence[31:28]);
                    7'd49: grid_status_char = hex_char(active_flash_saved_sequence[27:24]);
                    7'd50: grid_status_char = hex_char(active_flash_saved_sequence[23:20]);
                    7'd51: grid_status_char = hex_char(active_flash_saved_sequence[19:16]);
                    7'd52: grid_status_char = hex_char(active_flash_saved_sequence[15:12]);
                    7'd53: grid_status_char = hex_char(active_flash_saved_sequence[11:8]);
                    7'd54: grid_status_char = hex_char(active_flash_saved_sequence[7:4]);
                    7'd55: grid_status_char = hex_char(active_flash_saved_sequence[3:0]);
                    default: grid_status_char = 8'd10;
                  endcase
                end
              endfunction

              // @G1 transports 128 cells, packed as 2-bit
              // unknown/free/occupied values. CRC-8 covers ASCII "G1",
              // sequence, base and the 64 data nibbles (the leading @ and
              // trailing CRC/newline are excluded).
              function [7:0] grid_chunk_char;
                input [6:0] char_index;
                integer data_bit;
                begin
                  if ((char_index >= 7'd8) && (char_index <= 7'd71)) begin
                    data_bit = (char_index - 7'd8) * 4;
                    grid_chunk_char =
                        hex_char(active_grid_chunk_data[data_bit +: 4]);
                  end else begin
                    case (char_index)
                      7'd0: grid_chunk_char = "G";
                      7'd1: grid_chunk_char = "1";
                      7'd2: grid_chunk_char = hex_char(active_grid_chunk_seq[7:4]);
                      7'd3: grid_chunk_char = hex_char(active_grid_chunk_seq[3:0]);
                      7'd4: grid_chunk_char = hex_char({2'd0, active_grid_chunk_base[13:12]});
                      7'd5: grid_chunk_char = hex_char(active_grid_chunk_base[11:8]);
                      7'd6: grid_chunk_char = hex_char(active_grid_chunk_base[7:4]);
                      7'd7: grid_chunk_char = hex_char(active_grid_chunk_base[3:0]);
                      7'd72: grid_chunk_char = hex_char(active_grid_chunk_crc[7:4]);
                      7'd73: grid_chunk_char = hex_char(active_grid_chunk_crc[3:0]);
                      default: grid_chunk_char = 8'd10;
                    endcase
                  end
                end
              endfunction

              function [7:0] vl53_char;
                input [5:0] char_index;
                integer distance_index;
                integer distance_nibble;
                reg [15:0] selected_distance;
                begin
                  if ((char_index >= 6'd7) && (char_index <= 6'd26)) begin
                    distance_index = (char_index - 6'd7) >> 2;
                    distance_nibble = (char_index - 6'd7) & 3;



                    selected_distance =
                        status_vl53_distance_mm[distance_index * 16 +: 16];
                    case (distance_nibble)
                      0: vl53_char = hex_char(selected_distance[15:12]);
                      1: vl53_char = hex_char(selected_distance[11:8]);
                      2: vl53_char = hex_char(selected_distance[7:4]);
                      default: vl53_char = hex_char(selected_distance[3:0]);
                    endcase
                  end else begin
                    case (char_index)
                      6'd0: vl53_char = "V";
                      6'd1: vl53_char = hex_char(active_vl53_status_seq[7:4]);
                      6'd2: vl53_char = hex_char(active_vl53_status_seq[3:0]);
                      6'd3: vl53_char = hex_char(
                          {status_vl53_initialization_done, 2'b00,
                           status_vl53_sensor_online[4]});
                      6'd4: vl53_char = hex_char(status_vl53_sensor_online[3:0]);
                      6'd5: vl53_char = hex_char(
                          {3'b000, status_vl53_distance_valid[4]});
                      6'd6: vl53_char = hex_char(status_vl53_distance_valid[3:0]);
                      6'd27: vl53_char = hex_char(status_vl53_error_count[15:12]);
                      6'd28: vl53_char = hex_char(status_vl53_error_count[11:8]);
                      6'd29: vl53_char = hex_char(status_vl53_error_count[7:4]);
                      6'd30: vl53_char = hex_char(status_vl53_error_count[3:0]);
                      default: vl53_char = 8'd10;
                    endcase
                  end
                end
              endfunction

              function [7:0] motion_char;
                input [6:0] char_index;
                begin
                  case (char_index)
                    7'd0: motion_char = "M";
                    7'd1: motion_char = "2";
                    default: begin
                      if ((char_index >= 7'd2) && (char_index <= 7'd81))
                        motion_char = hex_char(motion_shift_data[319:316]);
                      else
                        motion_char = 8'd10;
                    end
                  endcase
                end
              endfunction

              function cfg_command;
                input [7:0] value;
                begin
                  case (value)
                    "G", "g", "N", "n", "B", "b", "+", "=", "-", "_",
                    "V", "v", "T", "t", "1", "2", "3", "4", "5", "Z", "z",
                    "H", "h", "O", "o", "E", "e", "Q", "q", "X", "x",
                    "J", "j", "K", "k", "6", "7", "8", "9", "Y", "y":
                      cfg_command = 1'b1;
                    default: cfg_command = 1'b0;
                  endcase
                end
              endfunction

              function onboard_map_command;
                input [7:0] value;
                begin
                  case (value)
                    "J", "j", "K", "k", "6", "7", "8", "9":
                      onboard_map_command = 1'b1;
                    default: onboard_map_command = 1'b0;
                  endcase
                end
              endfunction

              function [7:0] reply_char;
                input [1:0] kind;
                input [2:0] index;
                input [7:0] key_char;
                begin
                  case (kind)
                    REPLY_KEY: begin
                      case (index)
                        3'd0: reply_char = "@";
                        3'd1: reply_char = "K";
                        3'd2: reply_char = key_char;
                        default: reply_char = 8'd10;
                      endcase
                    end
                    REPLY_PONG: begin
                      case (index)
                        3'd0: reply_char = "@";
                        3'd1: reply_char = "P";
                        3'd2: reply_char = "O";
                        3'd3: reply_char = "N";
                        3'd4: reply_char = "G";
                        default: reply_char = 8'd10;
                      endcase
                    end
                    default: reply_char = 8'd10;
                  endcase
                end
              endfunction

              // The AT pin remains high in transparent mode. After at least
              // 500 ms UART idle, AT... enters serial configuration and
              // AT+RESET restarts the BLE stack without changing parameters.
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  // Normal FPGA resets must not disturb an already advertising
                  // CH9141K. Arm recovery only after a real P heartbeat has
                  // first proved that this FPGA instance owned the link.
                  bt_boot_state <= BT_BOOT_READY;
                  bt_boot_count <= 26'd0;
                  bt_boot_byte_index <= 4'd0;
                  bt_boot_tx_data <= 8'd0;
                  bt_boot_tx_start <= 1'b0;
                  bt_recovery_count <= 32'd0;
                  bt_link_was_active <= 1'b0;
                  // If CH9141K was already wedged before the FPGA restarted,
                  // clearing bt_link_was_active would otherwise disable every
                  // recovery path. Arm one delayed startup recovery. Any UART
                  // byte (including the first phone heartbeat) cancels it, so
                  // an operating BLE link is never reset.
                  bt_startup_recovery_armed <= 1'b1;
                end else begin
                  bt_boot_tx_start <= 1'b0;
                  if (link_active) begin
                    bt_link_was_active <= 1'b1;
                    bt_recovery_count <= 32'd0;
                    bt_startup_recovery_armed <= 1'b0;
                  end
                  case (bt_boot_state)
                    BT_BOOT_IDLE: begin
                      if (bt_boot_count >= BT_STARTUP_IDLE_CYCLES - 1) begin
                        bt_boot_count <= 26'd0;
                        bt_boot_byte_index <= 4'd0;
                        bt_boot_tx_data <= bt_boot_command_char(1'b0, 4'd0);
                        bt_boot_tx_start <= 1'b1;
                        bt_boot_state <= BT_BOOT_ENTER;
                      end else begin
                        bt_boot_count <= bt_boot_count + 26'd1;
                      end
                    end

                    BT_BOOT_ENTER: begin
                      if (bt_boot_tx_done) begin
                        if (bt_boot_byte_index == 4'd6) begin
                          bt_boot_byte_index <= 4'd0;
                          bt_boot_count <= 26'd0;
                          bt_boot_state <= BT_BOOT_GAP;
                        end else begin
                          bt_boot_byte_index <= bt_boot_byte_index + 4'd1;
                          bt_boot_tx_data <= bt_boot_command_char(
                              1'b0, bt_boot_byte_index + 4'd1);
                          bt_boot_tx_start <= 1'b1;
                        end
                      end
                    end

                    BT_BOOT_GAP: begin
                      if (bt_boot_count >=
                          BT_STARTUP_COMMAND_GAP_CYCLES - 1) begin
                        bt_boot_count <= 26'd0;
                        bt_boot_byte_index <= 4'd0;
                        bt_boot_tx_data <= bt_boot_command_char(1'b1, 4'd0);
                        bt_boot_tx_start <= 1'b1;
                        bt_boot_state <= BT_BOOT_RESET;
                      end else begin
                        bt_boot_count <= bt_boot_count + 26'd1;
                      end
                    end

                    BT_BOOT_RESET: begin
                      if (bt_boot_tx_done) begin
                        if (bt_boot_byte_index == 4'd9) begin
                          bt_boot_byte_index <= 4'd0;
                          bt_boot_count <= 26'd0;
                          bt_boot_state <= BT_BOOT_REBOOT;
                        end else begin
                          bt_boot_byte_index <= bt_boot_byte_index + 4'd1;
                          bt_boot_tx_data <= bt_boot_command_char(
                              1'b1, bt_boot_byte_index + 4'd1);
                          bt_boot_tx_start <= 1'b1;
                        end
                      end
                    end

                    BT_BOOT_REBOOT: begin
                      if (bt_boot_count >= BT_STARTUP_REBOOT_CYCLES - 1) begin
                        bt_boot_count <= 26'd0;
                        bt_boot_state <= BT_BOOT_READY;
                      end else begin
                        bt_boot_count <= bt_boot_count + 26'd1;
                      end
                    end

                    BT_BOOT_READY: begin
                      bt_boot_count <= 26'd0;
                      if (rx_valid) begin
                        // Any byte proves that the BLE-to-UART path is alive.
                        // Do not reset CH9141K while Android is reconnecting or
                        // before its first heartbeat reaches the FPGA.
                        bt_recovery_count <= 32'd0;
                        bt_startup_recovery_armed <= 1'b0;
                      end else if (!link_active && bt_link_was_active) begin
                        if (bt_recovery_count >=
                            BT_RECOVERY_TRIGGER_CYCLES - 1) begin
                          bt_boot_state <= BT_BOOT_IDLE;
                          bt_recovery_count <= 32'd0;
                          bt_link_was_active <= 1'b0;
                        end else begin
                          bt_recovery_count <= bt_recovery_count + 32'd1;
                        end
                      end else if (bt_startup_recovery_armed) begin
                        if (bt_recovery_count >=
                            BT_STARTUP_RECOVERY_CYCLES - 1) begin
                          bt_boot_state <= BT_BOOT_IDLE;
                          bt_recovery_count <= 32'd0;
                          bt_startup_recovery_armed <= 1'b0;
                        end else begin
                          bt_recovery_count <= bt_recovery_count + 32'd1;
                        end
                      end
                    end

                    default: begin
                      bt_boot_state <= BT_BOOT_READY;
                      bt_boot_count <= 26'd0;
                      bt_recovery_count <= 32'd0;
                      bt_startup_recovery_armed <= 1'b0;
                    end
                  endcase
                end
              end

              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  bt_boot_tx_line <= 1'b1;
                  bt_boot_tx_busy <= 1'b0;
                  bt_boot_tx_done <= 1'b0;
                  bt_boot_tx_state <= TX_IDLE;
                  bt_boot_tx_clk_count <= 16'd0;
                  bt_boot_tx_bit_index <= 3'd0;
                  bt_boot_tx_shift <= 8'd0;
                end else begin
                  bt_boot_tx_done <= 1'b0;
                  case (bt_boot_tx_state)
                    TX_IDLE: begin
                      bt_boot_tx_line <= 1'b1;
                      bt_boot_tx_busy <= 1'b0;
                      bt_boot_tx_clk_count <= 16'd0;
                      bt_boot_tx_bit_index <= 3'd0;
                      if (bt_boot_tx_start) begin
                        bt_boot_tx_busy <= 1'b1;
                        bt_boot_tx_shift <= bt_boot_tx_data;
                        bt_boot_tx_state <= TX_START;
                        bt_boot_tx_line <= 1'b0;
                      end
                    end

                    TX_START: begin
                      bt_boot_tx_busy <= 1'b1;
                      bt_boot_tx_line <= 1'b0;
                      if (bt_boot_tx_clk_count == CLKS_PER_BIT - 1) begin
                        bt_boot_tx_clk_count <= 16'd0;
                        bt_boot_tx_state <= TX_DATA;
                        bt_boot_tx_line <= bt_boot_tx_shift[0];
                      end else begin
                        bt_boot_tx_clk_count <= bt_boot_tx_clk_count + 16'd1;
                      end
                    end

                    TX_DATA: begin
                      bt_boot_tx_busy <= 1'b1;
                      bt_boot_tx_line <= bt_boot_tx_shift[bt_boot_tx_bit_index];
                      if (bt_boot_tx_clk_count == CLKS_PER_BIT - 1) begin
                        bt_boot_tx_clk_count <= 16'd0;
                        if (bt_boot_tx_bit_index == 3'd7) begin
                          bt_boot_tx_bit_index <= 3'd0;
                          bt_boot_tx_state <= TX_STOP;
                          bt_boot_tx_line <= 1'b1;
                        end else begin
                          bt_boot_tx_bit_index <= bt_boot_tx_bit_index + 3'd1;
                          bt_boot_tx_line <=
                              bt_boot_tx_shift[bt_boot_tx_bit_index + 3'd1];
                        end
                      end else begin
                        bt_boot_tx_clk_count <= bt_boot_tx_clk_count + 16'd1;
                      end
                    end

                    default: begin
                      bt_boot_tx_busy <= 1'b1;
                      bt_boot_tx_line <= 1'b1;
                      if (bt_boot_tx_clk_count == CLKS_PER_BIT - 1) begin
                        bt_boot_tx_clk_count <= 16'd0;
                        bt_boot_tx_busy <= 1'b0;
                        bt_boot_tx_done <= 1'b1;
                        bt_boot_tx_state <= TX_IDLE;
                      end else begin
                        bt_boot_tx_clk_count <= bt_boot_tx_clk_count + 16'd1;
                      end
                    end
                  endcase
                end
              end

            always @(posedge clk or negedge protocol_rst_n) begin
              if (!protocol_rst_n) begin
                  rx_sync <= 2'b11;
                  rx_state <= RX_IDLE;
                  rx_clk_count <= 16'd0;
                  rx_bit_index <= 3'd0;
                  rx_shift <= 8'd0;
                  rx_data <= 8'd0;
                  rx_valid <= 1'b0;
                end else begin
                  rx_sync <= {rx_sync[0], bt_uart_rx};
                  rx_valid <= 1'b0;
                  case (rx_state)
                    RX_IDLE: begin
                      rx_clk_count <= 16'd0;
                      rx_bit_index <= 3'd0;
                      if (!rx_sync[1]) rx_state <= RX_START;
                    end
                    RX_START: begin
                      if (rx_clk_count == (CLKS_PER_BIT / 2)) begin
                        rx_clk_count <= 16'd0;
                        rx_state <= !rx_sync[1] ? RX_DATA : RX_IDLE;
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
              end
            end

                    RX_DATA: begin
                      if (rx_clk_count == CLKS_PER_BIT - 1) begin
                        rx_clk_count <= 16'd0;
                        rx_shift[rx_bit_index] <= rx_sync[1];
                        if (rx_bit_index == 3'd7) begin
                          rx_bit_index <= 3'd0;
                          rx_state <= RX_STOP;
                        end else begin
                          rx_bit_index <= rx_bit_index + 3'd1;
                        end
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
                      end
                    end
                    default: begin
                      if (rx_clk_count == CLKS_PER_BIT - 1) begin
                        rx_clk_count <= 16'd0;
                        rx_data <= rx_shift;
                        rx_valid <= rx_sync[1];
                        rx_state <= RX_IDLE;
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
                      end
                    end
                  endcase
                end
              end

              // Board-resident map commands are independent of the route
              // editor and manual-drive keys. Each decoded byte produces one
              // clock pulse; long Flash work is owned by map_flash_store.
              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  onboard_map_start_pulse <= 1'b0;
                  onboard_map_pause_pulse <= 1'b0;
                  onboard_map_clear_pulse <= 1'b0;
                  onboard_map_save_pulse <= 1'b0;
                  onboard_map_load_pulse <= 1'b0;
                  onboard_map_sync_pulse <= 1'b0;
                  gyro_bias_learn_pulse <= 1'b0;
                end else begin
                  onboard_map_start_pulse <= 1'b0;
                  onboard_map_pause_pulse <= 1'b0;
                  onboard_map_clear_pulse <= 1'b0;
                  onboard_map_save_pulse <= 1'b0;
                  onboard_map_load_pulse <= 1'b0;
                  onboard_map_sync_pulse <= 1'b0;
                  gyro_bias_learn_pulse <= 1'b0;
                  if (rx_valid) begin
                    case (rx_data)
                      "J", "j": onboard_map_start_pulse <= 1'b1;
                      "K", "k": onboard_map_pause_pulse <= 1'b1;
                      "6": onboard_map_clear_pulse <= 1'b1;
                      "7": onboard_map_save_pulse <= 1'b1;
                      "8": onboard_map_load_pulse <= 1'b1;
                      "9": onboard_map_sync_pulse <= 1'b1;
                      "Y", "y": gyro_bias_learn_pulse <= 1'b1;
                      default: begin end
                    endcase
                  end
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  manual_mode <= 1'b0;
                  manual_cmd <= MANUAL_STOP;
                  lidar_enable <= 1'b1;
                  manual_timeout_count <= 24'd0;
                  manual_watchdog_expired <= 1'b0;
                  motor_stop_pulse <= 1'b0;
                  local_calibration_start_pulse <= 1'b0;
                end else begin
                  motor_stop_pulse <= 1'b0;
                  local_calibration_start_pulse <= 1'b0;
                  if (manual_mode && (manual_cmd != MANUAL_STOP)) begin
                    if (manual_timeout_count >= MANUAL_TIMEOUT_CYCLES - 1) begin
                      manual_cmd <= MANUAL_STOP;
                      motor_stop_pulse <= 1'b1;
                      manual_timeout_count <= 24'd0;
                      manual_watchdog_expired <= 1'b1;
                    end else begin
                      manual_timeout_count <= manual_timeout_count + 24'd1;
                    end
                  end else begin
                    manual_timeout_count <= 24'd0;
                  end

                  if (rx_valid) begin
                    case (rx_data)
                      "H", "h": begin
                        manual_mode <= 1'b1;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "J", "j": begin
                        // Start exploration from a known stopped state, then
                        // release manual ownership to the board auto controller.
                        manual_mode <= 1'b0;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "K", "k": begin
                        // Pausing exploration must atomically take motor ownership.
                        // Otherwise the finite stop burst can finish while auto_mode
                        // is still asserted and a forward frame is sent again.
                        manual_mode <= 1'b1;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "O", "o": begin
                        manual_mode <= 1'b0;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "E", "e": lidar_enable <= 1'b1;
                      "Q", "q": lidar_enable <= 1'b0;
                      "W", "w", "U", "u": if (manual_mode) begin
                        if (local_calibration_flags[0]) begin
                          manual_cmd <= MANUAL_STOP;
                          motor_stop_pulse <= 1'b1;
                        end else
                          manual_cmd <= MANUAL_FORWARD;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "S", "s": if (manual_mode) begin
                        if (local_calibration_flags[0]) begin
                          manual_cmd <= MANUAL_STOP;
                          motor_stop_pulse <= 1'b1;
                        end else
                          manual_cmd <= MANUAL_REVERSE;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "A", "a", "L", "l": if (manual_mode) begin
                        if (local_calibration_flags[0]) begin
                          manual_cmd <= MANUAL_STOP;
                          motor_stop_pulse <= 1'b1;
                        end else
                          manual_cmd <= MANUAL_LEFT;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "D", "d", "R", "r": if (manual_mode) begin
                        if (local_calibration_flags[0]) begin
                          manual_cmd <= MANUAL_STOP;
                          motor_stop_pulse <= 1'b1;
                        end else
                          manual_cmd <= MANUAL_RIGHT;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "C", "c", "M", "m": begin
                        // Stop without changing ownership.  In manual mode this
                        // is the normal drive stop; in automatic mode the same
                        // byte also drives bt_key_mid_n as the route confirm
                        // key.  Forcing manual mode here would make the route
                        // key gate discard that confirm pulse.
                        manual_mode <= manual_mode;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "X", "x": begin
                        manual_mode <= 1'b1;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        local_calibration_start_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      default: begin end
                    endcase
                  end
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  link_idle_count <= 32'd0;
                  link_active <= 1'b0;
                end else if (rx_valid && (rx_data == "P" || rx_data == "p")) begin
                  link_idle_count <= 32'd0;
                  link_active <= 1'b1;
                end else if (link_active) begin
                  if (link_idle_count >= LINK_TIMEOUT_CYCLES - 1) begin
                    link_active <= 1'b0;
                    link_idle_count <= LINK_TIMEOUT_CYCLES - 1;
                  end else begin
                    link_idle_count <= link_idle_count + 32'd1;
                  end
                end else begin
                  link_idle_count <= 32'd0;
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  ultrasonic_last_distance_cm <= 16'hffff;
                  ultrasonic_last_valid <= 1'b0;
                  ultrasonic_last_timeout <= 1'b0;
                  ultrasonic_last_stop <= 1'b0;
                end else if (ultrasonic_distance_valid) begin
                  ultrasonic_last_distance_cm <= ultrasonic_distance_cm;
                  ultrasonic_last_valid <= !ultrasonic_echo_timeout &&
                                           (ultrasonic_distance_cm != 16'hffff);
                  ultrasonic_last_timeout <= ultrasonic_echo_timeout;
                  ultrasonic_last_stop <= ultrasonic_obstacle_stop;
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  pending_reply <= 1'b0;
                  reply_kind <= REPLY_NONE;
                  reply_key <= 8'd0;
                end else if (rx_valid) begin
                  if (norm_key != 8'd0) begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_KEY;
                    reply_key <= norm_key;
                  end else if (rx_data == "P" || rx_data == "p") begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_PONG;
                    reply_key <= 8'd0;
                  end else if (is_cfg_command) begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_KEY;
                    reply_key <= rx_data;
                  end
                end else if (!sending_status && !sending_reply && !sending_map &&
                             !sending_vl53 && !sending_motion &&
                             !sending_grid_status && !sending_grid_chunk &&
                             pending_reply && !tx_busy) begin
                  pending_reply <= 1'b0;
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  bt_cfg_mode_pulse <= 1'b0;
                  bt_cfg_next_pulse <= 1'b0;
                  bt_cfg_prev_pulse <= 1'b0;
                  bt_cfg_inc_pulse <= 1'b0;
                  bt_cfg_dec_pulse <= 1'b0;
                  bt_cfg_select <= 3'd0;
                  bt_cfg_select_valid <= 1'b0;
                end else begin
                  bt_cfg_mode_pulse <= 1'b0;
                  bt_cfg_next_pulse <= 1'b0;
                  bt_cfg_prev_pulse <= 1'b0;
                  bt_cfg_inc_pulse <= 1'b0;
                  bt_cfg_dec_pulse <= 1'b0;
                  bt_cfg_select_valid <= 1'b0;
                  if (rx_valid) begin
                    case (rx_data)
                      "G", "g": bt_cfg_mode_pulse <= 1'b1;
                      "N", "n": bt_cfg_next_pulse <= 1'b1;
                      "B", "b": bt_cfg_prev_pulse <= 1'b1;
                      "+", "=": bt_cfg_inc_pulse <= 1'b1;
                      "-", "_": bt_cfg_dec_pulse <= 1'b1;
                      "V", "v": begin bt_cfg_select <= 3'd0; bt_cfg_select_valid <= 1'b1; end
                      "T", "t": begin bt_cfg_select <= 3'd1; bt_cfg_select_valid <= 1'b1; end
                      "1": begin bt_cfg_select <= 3'd2; bt_cfg_select_valid <= 1'b1; end
                      "2": begin bt_cfg_select <= 3'd3; bt_cfg_select_valid <= 1'b1; end
                      "3": begin bt_cfg_select <= 3'd4; bt_cfg_select_valid <= 1'b1; end
                      "4": begin bt_cfg_select <= 3'd5; bt_cfg_select_valid <= 1'b1; end
                      "5", "Z", "z": begin bt_cfg_select <= 3'd6; bt_cfg_select_valid <= 1'b1; end
                      default: begin end
                    endcase
                  end
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  up_hold_count <= 22'd0;
                  down_hold_count <= 22'd0;
                  left_hold_count <= 22'd0;
                  right_hold_count <= 22'd0;
                  mid_hold_count <= 22'd0;
                end else begin
                  if (up_hold_count != 22'd0) up_hold_count <= up_hold_count - 22'd1;
                  if (down_hold_count != 22'd0) down_hold_count <= down_hold_count - 22'd1;
                  if (left_hold_count != 22'd0) left_hold_count <= left_hold_count - 22'd1;
                  if (right_hold_count != 22'd0) right_hold_count <= right_hold_count - 22'd1;
                  if (mid_hold_count != 22'd0) mid_hold_count <= mid_hold_count - 22'd1;
                  if (rx_valid && !manual_mode) begin
                    case (rx_data)
                      "U", "u", "W", "w": up_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "D", "d", "S", "s": down_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "L", "l", "A", "a": left_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "R", "r":           right_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "M", "m", "C", "c": mid_hold_count <= KEY_HOLD_CYCLES[21:0];
                      default: begin end
                    endcase
                  end
                end
              end

              always @(posedge clk or negedge protocol_rst_n) begin
                if (!protocol_rst_n) begin
                  tx_line <= 1'b1;
                  tx_busy <= 1'b0;
                  tx_busy_d <= 1'b0;
                  tx_state <= TX_IDLE;
                  tx_clk_count <= 16'd0;
                  tx_bit_index <= 3'd0;
                  tx_shift <= 8'd0;
                  tx_data <= 8'd0;
                  tx_start <= 1'b0;
                  status_count <= 32'd0;
                  status_left <= 7'd0;
                  status_index <= 7'd0;
                  active_status_lidar <= 1'b0;
                  sending_status <= 1'b0;
                  status_packet_lidar <= 1'b0;
                  sending_reply <= 1'b0;
                  sending_map <= 1'b0;
                  sending_vl53 <= 1'b0;
                  sending_motion <= 1'b0;
                  sending_grid_status <= 1'b0;
                  sending_grid_chunk <= 1'b0;
                  active_reply_kind <= REPLY_NONE;
                  active_reply_key <= 8'd0;
                  reply_index <= 3'd0;
                  map_left <= 6'd0;
                  map_index <= 6'd0;
                  active_map_scan_seq <= 8'd0;
                  active_map_tick_10ms <= 16'd0;
                  active_map_part <= 3'd0;
                  active_map_chunk_data <= 144'd0;
                  active_map_chunk_crc <= 8'd0;
                  grid_status_count <= GRID_STATUS_CYCLES - 1;
                  grid_status_pending <= 1'b1;
                  grid_status_seq <= 8'd0;
                  active_grid_status_seq <= 8'd0;
                  grid_status_left <= 7'd0;
                  grid_status_index <= 7'd0;
                  active_grid_flags <= 8'd0;
                  active_grid_epoch <= 8'd0;
                  active_grid_scan_count <= 16'd0;
                  active_grid_occupied_count <= 16'd0;
                  active_grid_free_count <= 16'd0;
                  active_grid_dropped_count <= 16'd0;
                  active_grid_pose_x_mm <= 16'd0;
                  active_grid_pose_y_mm <= 16'd0;
                  active_grid_pose_heading <= 10'd0;
                  active_grid_pose_flags <= 8'd0;
                  active_flash_operation <= 2'd0;
                  active_flash_progress <= 8'd0;
                  active_flash_error <= 8'd0;
                  active_flash_jedec_id <= 24'd0;
                  active_flash_saved_sequence <= 32'd0;
                  grid_chunk_left <= 7'd0;
                  grid_chunk_index <= 7'd0;
                  active_grid_chunk_seq <= 8'd0;
                  active_grid_chunk_base <= 14'd0;
                  active_grid_chunk_data <= 256'd0;
                  active_grid_chunk_crc <= 8'd0;
                  grid_yield_to_status <= 1'b0;
                  vl53_status_count <= 32'd0;
                  vl53_left <= 6'd0;
                  vl53_index <= 6'd0;
                  vl53_status_seq <= 8'd0;
                  active_vl53_status_seq <= 8'd0;
                  status_vl53_initialization_done <= 1'b0;
                  status_vl53_sensor_online <= 5'd0;
                  status_vl53_distance_valid <= 5'd0;
                  status_vl53_distance_mm <= 80'd0;
                  status_vl53_error_count <= 16'd0;
                  motion_status_count <= 32'd0;
                  // Allow the power-on status frame without an extra delay.
                  telemetry_gap_count <= TELEMETRY_GAP_CYCLES - 1;
                  motion_left <= 7'd0;
                  motion_index <= 7'd0;
                  motion_shift_data <= 320'd0;
                  map_chunk_accept <= 1'b0;
                  grid_chunk_accept <= 1'b0;
                  route_status_pending <= 1'b1;
                  route_status_seq <= 8'd0;
                  lidar_status_seq <= 8'd0;
                  active_status_seq <= 8'd0;
                  prev_menu_info <= 2'd0;
                  prev_state_info <= 5'd0;
                  prev_route_info <= 64'd0;
                  prev_active_route_steps <= 6'd0;
                  prev_completed_route_steps <= 6'd0;
                  prev_now_x <= 8'd0;
                  prev_now_y <= 8'd0;
                  prev_target_x <= 8'd0;
                  prev_target_y <= 8'd0;
                  prev_step_length_set <= 16'd0;
                  prev_overall_speed <= 10'd0;
                  prev_lf_default_speed <= 10'd0;
                  prev_rf_default_speed <= 10'd0;
                  prev_lr_default_speed <= 10'd0;
                  prev_rr_default_speed <= 10'd0;
                  prev_lidar_safe_mm <= 16'd0;
                  prev_setting_mode <= 1'b0;
                  prev_selected_param <= 3'd0;
                  prev_manual_mode <= 1'b0;
                  prev_manual_cmd <= MANUAL_STOP;
                  prev_manual_watchdog_expired <= 1'b0;
                  prev_lidar_enable <= 1'b0;
                  prev_motor_blocked <= 1'b0;
                  prev_local_calibration_state <= 4'd0;
                  prev_local_calibration_flags <= 8'd0;
                  prev_learned_yaw_trim <= 11'd0;
                  prev_onboard_map_active <= 1'b0;
                  prev_onboard_map_paused <= 1'b1;
                  prev_onboard_map_valid <= 1'b0;
                  prev_onboard_map_busy <= 1'b0;
                  prev_onboard_map_epoch <= 8'd0;
                  prev_onboard_map_scan_count <= 16'd0;
                  prev_onboard_pose_x_mm <= 16'd0;
                  prev_onboard_pose_y_mm <= 16'd0;
                  prev_onboard_pose_heading <= 10'd0;
                  prev_onboard_pose_flags <= 8'd0;
                  prev_flash_ready <= 1'b0;
                  prev_flash_busy <= 1'b0;
                  prev_flash_operation <= 2'd0;
                  prev_flash_error <= 8'd0;
                  prev_flash_saved_sequence <= 32'd0;
                  status_menu_info <= 2'd0;
                  status_state_info <= 5'd0;
                  status_route_info <= 64'd0;
                  status_active_route_steps <= 6'd0;
                  status_completed_route_steps <= 6'd0;
                  status_now_x <= 8'd0;
                  status_now_y <= 8'd0;
                  status_target_x <= 8'd0;
                  status_target_y <= 8'd0;
                  status_step_length_set <= 16'd0;
                  status_overall_speed <= 10'd0;
                  status_lf_default_speed <= 10'd0;
                  status_rf_default_speed <= 10'd0;
                  status_lr_default_speed <= 10'd0;
                  status_rr_default_speed <= 10'd0;
                  status_lidar_safe_mm_route <= 16'd0;
                  status_setting_mode <= 1'b0;
                  status_selected_param <= 3'd0;
                  status_lidar_online <= 1'b0;
                  status_manual_mode <= 1'b0;
                  status_lidar_enable <= 1'b0;
                  status_lidar_obstacle_stop <= 1'b0;
                  status_ultrasonic_obstacle_stop <= 1'b0;
                  status_merged_obstacle_stop <= 1'b0;
                  status_motor_blocked <= 1'b0;
                  status_ultrasonic_distance_valid <= 1'b0;
                  status_ultrasonic_echo_timeout <= 1'b0;
                  status_ultrasonic_distance_cm <= 16'hffff;
                  status_manual_cmd <= MANUAL_STOP;
                  status_manual_watchdog_expired <= 1'b0;
                  status_c5_frame_count <= 16'd0;
                  status_c5_last_lf_pulse <= 12'd1500;
                  status_c5_last_rf_pulse <= 12'd1500;
                  status_c5_last_lr_pulse <= 12'd1500;
                  status_c5_last_rr_pulse <= 12'd1500;
                  status_c5_stop_frames_complete <= 1'b0;
                  status_c5_stop_frames_remaining <= 2'd0;
                  status_c5_frame_busy <= 1'b0;
                  status_local_calibration_state <= 4'd0;
                  status_local_calibration_flags <= 8'd0;
                  status_learned_yaw_trim <= 11'd0;
                  status_yaw_correction <= 11'd0;
                  status_calibration_gyro_peak <= 16'd0;
                  status_lidar_safe_mm <= 16'd0;
                  status_lidar_front_distance_mm <= 16'hffff;
                  status_lidar_sector_mm[0] <= 16'hffff;
                  status_lidar_sector_mm[1] <= 16'hffff;
                  status_lidar_sector_mm[2] <= 16'hffff;
                  status_lidar_sector_mm[3] <= 16'hffff;
                  status_lidar_sector_mm[4] <= 16'hffff;
                  status_lidar_sector_mm[5] <= 16'hffff;
                  status_lidar_sector_mm[6] <= 16'hffff;
                  status_lidar_sector_mm[7] <= 16'hffff;
                end else begin
                  tx_start <= 1'b0;
                  tx_busy_d <= tx_busy;
                  map_chunk_accept <= 1'b0;
                  grid_chunk_accept <= 1'b0;

                  if (route_status_changed) begin
                    prev_menu_info <= menu_info;
                    prev_state_info <= state_info;
                    prev_route_info <= route_info;
                    prev_active_route_steps <= active_route_steps;
                    prev_completed_route_steps <= completed_route_steps;
                    prev_now_x <= now_x;
                    prev_now_y <= now_y;
                    prev_target_x <= target_x;
                    prev_target_y <= target_y;
                    prev_step_length_set <= step_length_set;
                    prev_overall_speed <= overall_speed;
                    prev_lf_default_speed <= lf_default_speed;
                    prev_rf_default_speed <= rf_default_speed;
                    prev_lr_default_speed <= lr_default_speed;
                    prev_rr_default_speed <= rr_default_speed;
                    prev_lidar_safe_mm <= lidar_safe_mm;
                    prev_setting_mode <= setting_mode;
                    prev_selected_param <= selected_param;
                    prev_manual_mode <= manual_mode;
                    prev_manual_cmd <= manual_cmd;
                    prev_manual_watchdog_expired <= manual_watchdog_expired;
                    prev_lidar_enable <= lidar_enable;
                    prev_motor_blocked <= motor_blocked;
                    prev_local_calibration_state <= local_calibration_state;
                    prev_local_calibration_flags <= local_calibration_flags;
                    prev_learned_yaw_trim <= learned_yaw_trim;
                    route_status_pending <= 1'b1;
                  end

                  if (onboard_map_status_changed) begin
                    prev_onboard_map_active <= onboard_map_active;
                    prev_onboard_map_paused <= onboard_map_paused;
                    prev_onboard_map_valid <= onboard_map_valid;
                    prev_onboard_map_busy <= onboard_map_busy;
                    prev_onboard_map_epoch <= onboard_map_epoch;
                    prev_onboard_map_scan_count <= onboard_map_scan_count;
                    prev_onboard_pose_x_mm <= onboard_pose_x_mm;
                    prev_onboard_pose_y_mm <= onboard_pose_y_mm;
                    prev_onboard_pose_heading <= onboard_pose_heading;
                    prev_onboard_pose_flags <= onboard_pose_flags;
                    prev_flash_ready <= flash_ready;
                    prev_flash_busy <= flash_busy;
                    prev_flash_operation <= flash_operation;
                    prev_flash_error <= flash_error;
                    prev_flash_saved_sequence <= flash_saved_sequence;
                    grid_status_pending <= 1'b1;
                  end

                  case (tx_state)
                    TX_IDLE: begin
                      tx_line <= 1'b1;
                      tx_busy <= 1'b0;
                      tx_clk_count <= 16'd0;
                      tx_bit_index <= 3'd0;
                      if (tx_start) begin
                        tx_busy <= 1'b1;
                        tx_shift <= tx_data;
                        tx_state <= TX_START;
                        tx_line <= 1'b0;
                      end
                    end
                    TX_START: begin
                      tx_busy <= 1'b1;
                      tx_line <= 1'b0;
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        tx_state <= TX_DATA;
                        tx_line <= tx_shift[0];
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                    TX_DATA: begin
                      tx_busy <= 1'b1;
                      tx_line <= tx_shift[tx_bit_index];
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        if (tx_bit_index == 3'd7) begin
                          tx_bit_index <= 3'd0;
                          tx_state <= TX_STOP;
                          tx_line <= 1'b1;
                        end else begin
                          tx_bit_index <= tx_bit_index + 3'd1;
                          tx_line <= tx_shift[tx_bit_index + 3'd1];
                        end
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                    default: begin
                      tx_busy <= 1'b1;
                      tx_line <= 1'b1;
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        tx_busy <= 1'b0;
                        tx_state <= TX_IDLE;
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                  endcase

                  if (rx_valid && ((rx_data == "?") ||
                                   (norm_key != 8'd0) ||
                                   is_cfg_command))
                    route_status_pending <= 1'b1;
                  // A status query is a synchronization barrier: return both
                  // the route snapshot and the latest board pose/map snapshot.
                  if (rx_valid && (rx_data == "?")) begin
                    grid_status_pending <= 1'b1;
                    grid_status_count <= GRID_STATUS_CYCLES - 1;
                  end
                  if (rx_valid && is_onboard_map_command) begin
                    grid_status_pending <= 1'b1;
                    grid_status_count <= GRID_STATUS_CYCLES - 1;
                  end
                  if (!sending_status && !sending_reply && !sending_map &&
                      !sending_vl53 && !sending_motion &&
                      !sending_grid_status && !sending_grid_chunk && !status_due)
                    status_count <= status_count + 32'd1;
                  if (!vl53_status_due)
                    vl53_status_count <= vl53_status_count + 32'd1;
                  if (!motion_status_due)
                    motion_status_count <= motion_status_count + 32'd1;
                  if (!grid_status_due && !(rx_valid &&
                      (is_onboard_map_command || (rx_data == "?"))))
                    grid_status_count <= grid_status_count + 32'd1;
                  if (!telemetry_slot_ready)
                    telemetry_gap_count <= telemetry_gap_count + 32'd1;

                  if (!sending_status && !sending_reply && !sending_map &&
                      !sending_vl53 && !sending_motion &&
                      !sending_grid_status && !sending_grid_chunk &&
                      pending_reply && !tx_busy) begin
                    active_reply_kind <= reply_kind;
                    active_reply_key <= reply_key;
                    reply_index <= 3'd0;
                    sending_reply <= 1'b1;
                    tx_data <= reply_char(reply_kind, 3'd0, reply_key);
                    tx_start <= 1'b1;
                  end else if (sending_reply) begin
                    if (tx_done) begin
                      if (reply_done) begin
                        sending_reply <= 1'b0;
                        active_reply_kind <= REPLY_NONE;
                      end else begin
                        reply_index <= reply_index + 3'd1;
                        tx_data <= reply_char(active_reply_kind, reply_index + 3'd1, active_reply_key);
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (sending_map) begin
                    if (tx_done) begin
                      if (map_left == 6'd0) begin
                        sending_map <= 1'b0;
                      end else begin
                        tx_data <= map_char(map_index);
                        map_index <= map_index + 6'd1;
                        map_left <= map_left - 6'd1;
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (sending_vl53) begin
                    if (tx_done) begin
                      if (vl53_left == 6'd0) begin
                        sending_vl53 <= 1'b0;
                      end else begin
                        tx_data <= vl53_char(vl53_index);
                        vl53_index <= vl53_index + 6'd1;
                        vl53_left <= vl53_left - 6'd1;
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (sending_motion) begin
                    if (tx_done) begin
                      if (motion_left == 7'd0) begin
                        sending_motion <= 1'b0;
                      end else begin
                        tx_data <= motion_char(motion_index);
                        if ((motion_index >= 7'd2) &&
                            (motion_index <= 7'd81))
                          motion_shift_data <= {motion_shift_data[315:0], 4'd0};
                        motion_index <= motion_index + 7'd1;
                        motion_left <= motion_left - 7'd1;
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (sending_grid_status) begin
                    if (tx_done) begin
                      if (grid_status_left == 7'd0) begin
                        sending_grid_status <= 1'b0;
                      end else begin
                        tx_data <= grid_status_char(grid_status_index);
                        grid_status_index <= grid_status_index + 7'd1;
                        grid_status_left <= grid_status_left - 7'd1;
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (sending_grid_chunk) begin
                    if (tx_done) begin
                      if (grid_chunk_left == 7'd0) begin
                        sending_grid_chunk <= 1'b0;
                      end else begin
                        tx_data <= grid_chunk_char(grid_chunk_index);
                        grid_chunk_index <= grid_chunk_index + 7'd1;
                        grid_chunk_left <= grid_chunk_left - 7'd1;
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (!sending_status) begin





                    if (link_active && motion_status_due &&
                        telemetry_slot_ready && !tx_busy) begin
                      motion_status_count <= 32'd0;
                      telemetry_gap_count <= 32'd0;
                      sending_motion <= 1'b1;
                      motion_left <= MOTION_STATUS_LEFT;
                      motion_index <= 7'd0;
                      motion_shift_data <= {
                          motion_telemetry_data[23:8],
                          motion_telemetry_data[7:0],
                          motion_telemetry_data[31:24],
                          motion_telemetry_data[47:32],
                          motion_telemetry_data[63:48],
                          motion_telemetry_data[95:64],
                          motion_telemetry_data[127:96],
                          motion_telemetry_data[143:128],
                          motion_telemetry_data[159:144],
                          motion_telemetry_data[175:160],
                          motion_telemetry_data[191:176],
                          motion_telemetry_data[207:192],
                          motion_telemetry_data[223:208],
                          motion_telemetry_data[239:224],
                          motion_telemetry_data[255:240],
                          motion_telemetry_data[271:256],
                          motion_telemetry_data[287:272],
                          motion_telemetry_data[303:288],
                          motion_telemetry_data[319:304]
                      };
                      tx_data <= "@";
                      tx_start <= 1'b1;
                    end else if (link_active && vl53_status_due &&
                                 telemetry_slot_ready && !tx_busy) begin
                      vl53_status_count <= 32'd0;
                      telemetry_gap_count <= 32'd0;
                      sending_vl53 <= 1'b1;
                      vl53_left <= VL53_STATUS_LEFT;
                      vl53_index <= 6'd0;
                      active_vl53_status_seq <= vl53_status_seq;
                      vl53_status_seq <= vl53_status_seq + 8'd1;
                      status_vl53_initialization_done <= vl53_initialization_done;
                      status_vl53_sensor_online <= vl53_sensor_online;
                      status_vl53_distance_valid <= vl53_distance_valid;
                      status_vl53_distance_mm <= vl53_distance_mm;
                      status_vl53_error_count <= vl53_error_count;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                    end else if (link_active && map_chunk_valid &&
                                 telemetry_slot_ready && !tx_busy) begin





                      map_chunk_accept <= 1'b1;
                      telemetry_gap_count <= 32'd0;
                      sending_map <= 1'b1;
                      map_left <= MAP_STATUS_LEFT;
                      map_index <= 6'd0;
                      active_map_scan_seq <= map_scan_seq;
                      active_map_tick_10ms <= map_tick_10ms;
                      active_map_part <= map_part;
                      active_map_chunk_data <= map_chunk_data;
                      active_map_chunk_crc <= map_chunk_crc;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                    end else if (link_active &&
                                 (route_status_pending || status_due) &&
                                 (!grid_frame_ready || grid_yield_to_status) &&
                                 telemetry_slot_ready && !tx_busy) begin
                      status_count <= 32'd0;
                      telemetry_gap_count <= 32'd0;
                      grid_yield_to_status <= 1'b0;
                      sending_status <= 1'b1;
                      status_index <= 7'd0;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                      if (route_status_pending) begin
                        route_status_pending <= 1'b0;
                        active_status_lidar <= 1'b0;
                        active_status_seq <= route_status_seq;
                        route_status_seq <= route_status_seq + 8'd1;
                        status_left <= ROUTE_STATUS_LEFT;
                        status_menu_info <= menu_info;
                        status_state_info <= state_info;
                        status_route_info <= route_info;
                        status_active_route_steps <= active_route_steps;
                        status_completed_route_steps <= completed_route_steps;
                        status_now_x <= now_x;
                        status_now_y <= now_y;
                        status_target_x <= target_x;
                        status_target_y <= target_y;
                        status_step_length_set <= step_length_set;
                        status_overall_speed <= overall_speed;
                        status_lf_default_speed <= lf_default_speed;
                        status_rf_default_speed <= rf_default_speed;
                        status_lr_default_speed <= lr_default_speed;
                        status_rr_default_speed <= rr_default_speed;
                        status_lidar_safe_mm_route <= lidar_safe_mm;
                        status_setting_mode <= setting_mode;
                        status_selected_param <= selected_param;
                        status_lidar_online <= lidar_online;
                        status_manual_mode <= manual_mode;
                        status_lidar_enable <= lidar_enable;
                        status_manual_cmd <= manual_cmd;
                        status_manual_watchdog_expired <= manual_watchdog_expired;
                        status_motor_blocked <= motor_blocked;
                        status_c5_frame_count <= c5_frame_count;
                        status_c5_last_lf_pulse <= c5_last_lf_pulse;
                        status_c5_last_rf_pulse <= c5_last_rf_pulse;
                        status_c5_last_lr_pulse <= c5_last_lr_pulse;
                        status_c5_last_rr_pulse <= c5_last_rr_pulse;
                        status_c5_stop_frames_complete <= c5_stop_frames_complete;
                        status_c5_stop_frames_remaining <= c5_stop_frames_remaining;
                        status_c5_frame_busy <= c5_frame_busy;
                        status_local_calibration_state <= local_calibration_state;
                        status_local_calibration_flags <= local_calibration_flags;
                        status_learned_yaw_trim <= learned_yaw_trim;
                        status_yaw_correction <= yaw_correction;
                        status_calibration_gyro_peak <= calibration_gyro_peak;
                      end else if (status_packet_lidar) begin
                        status_packet_lidar <= 1'b0;
                        active_status_lidar <= 1'b1;
                        active_status_seq <= lidar_status_seq;
                        lidar_status_seq <= lidar_status_seq + 8'd1;
                        status_left <= LIDAR_STATUS_LEFT;
                        status_lidar_online <= lidar_online;
                        status_lidar_obstacle_stop <= lidar_obstacle_stop;
                        status_ultrasonic_obstacle_stop <= ultrasonic_last_stop;
                        status_merged_obstacle_stop <= merged_obstacle_stop | ultrasonic_last_stop;
                        status_motor_blocked <= motor_blocked;
                        status_c5_frame_count <= c5_frame_count;
                        status_c5_last_lf_pulse <= c5_last_lf_pulse;
                        status_c5_last_rf_pulse <= c5_last_rf_pulse;
                        status_c5_last_lr_pulse <= c5_last_lr_pulse;
                        status_c5_last_rr_pulse <= c5_last_rr_pulse;
                        status_c5_stop_frames_complete <= c5_stop_frames_complete;
                        status_c5_stop_frames_remaining <= c5_stop_frames_remaining;
                        status_c5_frame_busy <= c5_frame_busy;
                        status_ultrasonic_distance_valid <= ultrasonic_last_valid;
                        status_ultrasonic_echo_timeout <= ultrasonic_last_timeout;
                        status_ultrasonic_distance_cm <= ultrasonic_last_distance_cm;
                        status_lidar_safe_mm <= lidar_safe_mm;
                        status_lidar_front_distance_mm <= lidar_front_distance_mm;
                        status_lidar_sector_mm[0] <= lidar_sector_front_mm;
                        status_lidar_sector_mm[1] <= lidar_sector_front_right_mm;
                        status_lidar_sector_mm[2] <= lidar_sector_right_mm;
                        status_lidar_sector_mm[3] <= lidar_sector_rear_right_mm;
                        status_lidar_sector_mm[4] <= lidar_sector_rear_mm;
                        status_lidar_sector_mm[5] <= lidar_sector_rear_left_mm;
                        status_lidar_sector_mm[6] <= lidar_sector_left_mm;
                        status_lidar_sector_mm[7] <= lidar_sector_front_left_mm;
                      end else begin
                        status_packet_lidar <= 1'b1;
                        active_status_lidar <= 1'b0;
                        active_status_seq <= route_status_seq;
                        route_status_seq <= route_status_seq + 8'd1;
                        status_left <= ROUTE_STATUS_LEFT;
                        status_menu_info <= menu_info;
                        status_state_info <= state_info;
                        status_route_info <= route_info;
                        status_active_route_steps <= active_route_steps;
                        status_completed_route_steps <= completed_route_steps;
                        status_now_x <= now_x;
                        status_now_y <= now_y;
                        status_target_x <= target_x;
                        status_target_y <= target_y;
                        status_step_length_set <= step_length_set;
                        status_overall_speed <= overall_speed;
                        status_lf_default_speed <= lf_default_speed;
                        status_rf_default_speed <= rf_default_speed;
                        status_lr_default_speed <= lr_default_speed;
                        status_rr_default_speed <= rr_default_speed;
                        status_lidar_safe_mm_route <= lidar_safe_mm;
                        status_setting_mode <= setting_mode;
                        status_selected_param <= selected_param;
                        status_lidar_online <= lidar_online;
                        status_manual_mode <= manual_mode;
                        status_lidar_enable <= lidar_enable;
                        status_manual_cmd <= manual_cmd;
                        status_manual_watchdog_expired <= manual_watchdog_expired;
                        status_motor_blocked <= motor_blocked;
                        status_local_calibration_state <= local_calibration_state;
                        status_local_calibration_flags <= local_calibration_flags;
                        status_learned_yaw_trim <= learned_yaw_trim;
                        status_yaw_correction <= yaw_correction;
                        status_calibration_gyro_peak <= calibration_gyro_peak;
                      end
                    end else if (link_active && grid_status_pending &&
                                 grid_status_due &&
                                 telemetry_slot_ready && !tx_busy) begin
                      grid_status_pending <= 1'b0;
                      grid_status_count <= 32'd0;
                      telemetry_gap_count <= 32'd0;
                      sending_grid_status <= 1'b1;
                      grid_yield_to_status <= 1'b1;
                      grid_status_left <= GRID_STATUS_LEFT;
                      grid_status_index <= 7'd0;
                      active_grid_status_seq <= grid_status_seq;
                      grid_status_seq <= grid_status_seq + 8'd1;
                      active_grid_flags <= {
                          local_calibration_flags[6], lidar_online,
                          flash_busy, flash_ready, onboard_map_busy,
                          onboard_map_valid, onboard_map_paused,
                          onboard_map_active
                      };
                      active_grid_epoch <= onboard_map_epoch;
                      active_grid_scan_count <= onboard_map_scan_count;
                      active_grid_occupied_count <= onboard_map_occupied_count;
                      active_grid_free_count <= onboard_map_free_count;
                      active_grid_dropped_count <= onboard_map_dropped_count;
                      active_grid_pose_x_mm <= onboard_pose_x_mm;
                      active_grid_pose_y_mm <= onboard_pose_y_mm;
                      active_grid_pose_heading <= onboard_pose_heading;
                      active_grid_pose_flags <= onboard_pose_flags;
                      active_flash_operation <= flash_operation;
                      active_flash_progress <= flash_progress;
                      active_flash_error <= flash_error;
                      active_flash_jedec_id <= flash_jedec_id;
                      active_flash_saved_sequence <= flash_saved_sequence;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                    end else if (link_active && grid_chunk_valid &&
                                 telemetry_slot_ready &&
                                 !tx_busy) begin
                      grid_chunk_accept <= 1'b1;
                      telemetry_gap_count <= 32'd0;
                      sending_grid_chunk <= 1'b1;
                      grid_yield_to_status <= 1'b1;
                      grid_chunk_left <= GRID_CHUNK_LEFT;
                      grid_chunk_index <= 7'd0;
                      active_grid_chunk_seq <= grid_chunk_seq;
                      active_grid_chunk_base <= grid_chunk_base;
                      active_grid_chunk_data <= grid_chunk_data;
                      active_grid_chunk_crc <= grid_chunk_crc;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                    end
                  end else if (tx_done) begin
                    if (status_left == 7'd0) begin
                      sending_status <= 1'b0;
                    end else begin
                      tx_data <= status_char(active_status_lidar, status_index);
                      status_index <= status_index + 7'd1;
                      status_left <= status_left - 7'd1;
                      tx_start <= 1'b1;
                    end
                  end
                end
              end

endmodule
