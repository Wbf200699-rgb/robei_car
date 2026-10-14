module motion_sensor_subsystem (
  input wire clk,
  input wire rst_n,
  input wire vehicle_motion_expected,
  input wire sensor_spi_miso,
  output reg sensor_spi_sclk,
  output reg sensor_spi_mosi,
  output wire pmw3901_cs_n,
  output wire bmi0_acc_cs_n,
  output wire bmi0_gyro_cs_n,
  output wire bmi1_acc_cs_n,
  output wire bmi1_gyro_cs_n,
  output reg sensor_debug_uart_tx,
  output reg pmw3901_online,
  output reg bmi0_online,
  output reg bmi1_online,
  output reg calibration_done,
  output reg fused_sample_valid,
  output reg imu_degraded,
  output reg imu_disagreement,
  output reg [1:0] imu_source,
  output reg [7:0] pmw3901_product_id,
  output reg [7:0] pmw3901_inverse_id,
  output reg [7:0] bmi0_accel_id,
  output reg [7:0] bmi0_gyro_id,
  output reg [7:0] bmi1_accel_id,
  output reg [7:0] bmi1_gyro_id,
  output reg flow_sample_valid,
  output reg [7:0] flow_squal,
  output reg [15:0] flow_delta_x,
  output reg [15:0] flow_delta_y,
  output reg [31:0] flow_accum_x,
  output reg [31:0] flow_accum_y,
  output reg [15:0] fused_accel_x,
  output reg [15:0] fused_accel_y,
  output reg [15:0] fused_accel_z,
  output reg [15:0] fused_gyro_x,
  output reg [15:0] fused_gyro_y,
  output reg [15:0] fused_gyro_z,
  output reg [15:0] sample_sequence,
  output wire [319:0] telemetry_data
);

`ifdef SENSOR_SIM_FAST
  localparam integer SPI_HALF_CYCLES = 2;
  localparam integer ONE_MS_CYCLES = 10;
  localparam integer BOOT_WAIT_CYCLES = 40;
  localparam integer RUN_WAIT_CYCLES = 100;
  localparam integer PMW_SETUP_CYCLES = 2;
  localparam integer PMW_READ_GAP_CYCLES = 2;
  localparam integer PMW_WRITE_HOLD_CYCLES = 2;
  localparam integer PMW_READ_HOLD_CYCLES = 2;
  localparam integer PMW_WRITE_POST_CYCLES = 3;
  localparam integer BMI_SETUP_CYCLES = 2;
  localparam integer BMI_HOLD_CYCLES = 2;
  localparam integer BMI_POST_CYCLES = 2;
  localparam integer CALIBRATION_SAMPLES = 8;
  localparam integer CALIBRATION_SHIFT = 3;
  localparam integer DEBUG_PERIOD_CYCLES = 500;
  localparam integer UART_CLKS_PER_BIT = 4;
`else
  localparam integer SPI_HALF_CYCLES = 13;
  localparam integer ONE_MS_CYCLES = 50000;
  localparam integer BOOT_WAIT_CYCLES = 5000000;
  localparam integer RUN_WAIT_CYCLES = 250000;
  localparam integer PMW_SETUP_CYCLES = 2500;
  localparam integer PMW_READ_GAP_CYCLES = 2500;
  localparam integer PMW_WRITE_HOLD_CYCLES = 2500;
  localparam integer PMW_READ_HOLD_CYCLES = 5000;
  localparam integer PMW_WRITE_POST_CYCLES = 10000;
  localparam integer BMI_SETUP_CYCLES = 4;
  localparam integer BMI_HOLD_CYCLES = 4;
  localparam integer BMI_POST_CYCLES = 100;
  localparam integer CALIBRATION_SAMPLES = 512;
  localparam integer CALIBRATION_SHIFT = 9;
  localparam integer DEBUG_PERIOD_CYCLES = 5000000;
  localparam integer UART_CLKS_PER_BIT = 434;
`endif

  localparam [2:0] DEV_PMW       = 3'd0;
  localparam [2:0] DEV_BMI0_ACC  = 3'd1;
  localparam [2:0] DEV_BMI0_GYRO = 3'd2;
  localparam [2:0] DEV_BMI1_ACC  = 3'd3;
  localparam [2:0] DEV_BMI1_GYRO = 3'd4;

  localparam [3:0] SPI_IDLE   = 4'd0;
  localparam [3:0] SPI_SETUP  = 4'd1;
  localparam [3:0] SPI_DRIVE  = 4'd2;
  localparam [3:0] SPI_SAMPLE = 4'd3;
  localparam [3:0] SPI_GAP    = 4'd4;
  localparam [3:0] SPI_HOLD   = 4'd5;
  localparam [3:0] SPI_POST   = 4'd6;

  localparam [7:0] ST_BOOT               = 8'd0;
  localparam [7:0] ST_PMW_CS_LOW         = 8'd1;
  localparam [7:0] ST_PMW_CS_HIGH        = 8'd2;
  localparam [7:0] ST_PMW_RESET          = 8'd3;
  localparam [7:0] ST_PMW_RESET_WAIT     = 8'd4;
  localparam [7:0] ST_PMW_ID             = 8'd5;
  localparam [7:0] ST_PMW_INVERSE_ID     = 8'd6;
  localparam [7:0] ST_PMW_RETRY_WAIT     = 8'd7;
  localparam [7:0] ST_PMW_CLEAR          = 8'd8;
  localparam [7:0] ST_PMW_INIT           = 8'd9;
  localparam [7:0] ST_PMW_INIT_LONG_WAIT = 8'd10;
  localparam [7:0] ST_BMI_INIT           = 8'd11;
  localparam [7:0] ST_RUN_WAIT           = 8'd12;
  localparam [7:0] ST_IMU0_ACCEL         = 8'd13;
  localparam [7:0] ST_IMU0_GYRO          = 8'd14;
  localparam [7:0] ST_IMU1_ACCEL         = 8'd15;
  localparam [7:0] ST_IMU1_GYRO          = 8'd16;
  localparam [7:0] ST_FUSION_STROBE      = 8'd17;
  localparam [7:0] ST_PMW_MOTION         = 8'd18;
  localparam [7:0] ST_PMW_DX_LOW         = 8'd19;
  localparam [7:0] ST_PMW_DX_HIGH        = 8'd20;
  localparam [7:0] ST_PMW_DY_LOW         = 8'd21;
  localparam [7:0] ST_PMW_DY_HIGH        = 8'd22;
  localparam [7:0] ST_PMW_SQUAL          = 8'd23;
  localparam [7:0] ST_GYRO_RECOVERY_RESET      = 8'd24;
  localparam [7:0] ST_GYRO_RECOVERY_RESET_WAIT = 8'd25;
  localparam [7:0] ST_GYRO_RECOVERY_ID         = 8'd26;
  localparam [7:0] ST_GYRO_RECOVERY_POWER      = 8'd27;
  localparam [7:0] ST_GYRO_RECOVERY_POWER_WAIT = 8'd28;
  localparam [7:0] ST_GYRO_RECOVERY_BW         = 8'd29;
  localparam [7:0] ST_GYRO_RECOVERY_RANGE      = 8'd30;
  localparam [7:0] ST_GYRO_RECOVERY_VERIFY     = 8'd31;
  localparam [7:0] ST_GYRO_RANGE_GUARD0        = 8'd32;
  localparam [7:0] ST_GYRO_RANGE_GUARD1        = 8'd33;

  localparam [1:0] UART_IDLE  = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA  = 2'd2;
  localparam [1:0] UART_STOP  = 2'd3;

  localparam [16:0] ACCEL_DISAGREE_LIMIT = 17'd1500;
  localparam [17:0] GYRO_DISAGREE_LIMIT  = 18'd2000;
  localparam [15:0] CALIBRATION_GYRO_LIMIT = 16'd2000;
  // BMI088 contains independent accelerometer and gyroscope dies.  A valid
  // accelerometer sample therefore cannot prove that the matching gyro data
  // path is live.  If one corrected gyro remains essentially unchanged while
  // its peer observes clear motion, remove the stuck gyro from fusion.
  localparam [16:0] GYRO_MISSING_SELF_LIMIT = 17'd16;
  localparam [16:0] GYRO_MISSING_PEER_LIMIT = 17'd256;
  localparam [4:0] GYRO_MISSING_ASSERT_SAMPLES = 5'd7;
  // A recovered gyroscope has just completed a soft reset, so its old zero
  // offset is no longer trustworthy.  Average 64 quiet samples from only that
  // die before allowing it back into fusion.  The peer limit corresponds to
  // roughly 3.9 degrees/s at the configured +/-250 degrees/s range: normal
  // stationary noise is accepted, while an obvious chassis turn restarts the
  // collection window instead of being learned as bias.
  localparam [6:0] GYRO_REBIAS_SAMPLES = 7'd64;
  localparam integer GYRO_REBIAS_SHIFT = 6;
  // A range rewrite or a single-sensor rail transient can leave the saved
  // offset several thousand counts away from the live raw value.  The old
  // 2000-count limit made that state unrecoverable: the channel stayed online
  // and continuously reported a false turn, but every re-bias window reset.
  // 8192 counts still rejects a real fast spin when combined with the optical
  // stillness gate below, while covering the observed warm-start offset.
  localparam [15:0] GYRO_REBIAS_RAW_LIMIT = 16'd8192;
  localparam [16:0] GYRO_REBIAS_PEER_LIMIT = 17'd512;
  localparam [16:0] GYRO_SINGLE_REBIAS_TRIGGER = 17'd512;
  localparam [3:0] GYRO_SINGLE_REBIAS_ASSERT_SAMPLES = 4'd8;
  localparam [7:0] GYRO_REBIAS_FLOW_SQUAL_MIN = 8'd20;
  localparam [15:0] GYRO_REBIAS_FLOW_DELTA_MAX = 16'd2;
  localparam [32:0] GYRO_REBIAS_FLOW_ACCUM_MAX = 33'd8;
  // A BMI088 can reset independently during a short sensor-rail disturbance
  // while the FPGA keeps running. Its gyro range then returns to 0x00
  // (+/-2000 dps), whose sensitivity is exactly one eighth of the configured
  // 0x03 (+/-250 dps) range. Rewriting 0x03 is idempotent and is safer than a
  // live readback audit: an occasional bad SPI read must never reset or remove
  // the only healthy IMU while the vehicle is turning.
  localparam [4:0] GYRO_RANGE_GUARD_SAMPLES = 5'd16;

  reg [4:0] spi_cs_n_raw;
  reg pmw_force_cs_low;
  reg spi_start;
  reg spi_busy;
  reg spi_done;
  reg [3:0] spi_state;
  reg [3:0] spi_length_latched;
  reg [3:0] spi_byte_count;
  reg [2:0] spi_bit_index;
  reg [15:0] spi_half_count;
  reg [15:0] spi_delay_count;
  reg [15:0] spi_setup_latched;
  reg [15:0] spi_gap_latched;
  reg [15:0] spi_hold_latched;
  reg [15:0] spi_post_latched;
  reg [71:0] spi_tx_shift;
  reg [71:0] spi_rx_shift;
  reg [71:0] spi_rx_data;

  reg [7:0] main_state;
  reg command_inflight;
  reg [31:0] state_wait_count;
  reg [6:0] pmw_init_index;
  reg [2:0] pmw_clear_index;
  reg [1:0] pmw_retry_count;
  reg bmi_index;
  reg [4:0] bmi_init_step;
  reg [7:0] current_bmi_accel_id;
  reg [7:0] current_bmi_gyro_id;
  reg [7:0] current_bmi_gyro_range;
  reg [7:0] bmi0_gyro_range;
  reg [7:0] bmi1_gyro_range;
  reg [1:0] bmi_accel_id_retry;
  reg [1:0] bmi_gyro_range_retry;
  reg gyro_recovery_target;
  reg [1:0] gyro_recovery_retry_count;
  reg gyro_recovery_done_pulse;
  reg gyro_recovery_success;
  reg bmi0_gyro_recovery_attempted;
  reg bmi1_gyro_recovery_attempted;
  reg bmi0_gyro_quarantined;
  reg bmi1_gyro_quarantined;
  reg [4:0] gyro_range_guard_count;
  reg pmw_poll_toggle;
  reg pair_sample_strobe;
  reg flow_data_seen;
  reg fused_data_ready;

  reg signed [15:0] bmi0_accel_x_raw;
  reg signed [15:0] bmi0_accel_y_raw;
  reg signed [15:0] bmi0_accel_z_raw;
  reg signed [15:0] bmi0_gyro_x_raw;
  reg signed [15:0] bmi0_gyro_y_raw;
  reg signed [15:0] bmi0_gyro_z_raw;
  reg signed [15:0] bmi1_accel_x_raw;
  reg signed [15:0] bmi1_accel_y_raw;
  reg signed [15:0] bmi1_accel_z_raw;
  reg signed [15:0] bmi1_gyro_x_raw;
  reg signed [15:0] bmi1_gyro_y_raw;
  reg signed [15:0] bmi1_gyro_z_raw;
  reg [5:0] bmi0_bad_sample_count;
  reg [5:0] bmi1_bad_sample_count;
  reg [3:0] bmi0_good_sample_count;
  reg [3:0] bmi1_good_sample_count;
  reg [4:0] bmi0_gyro_missing_count;
  reg [4:0] bmi1_gyro_missing_count;
  reg bmi0_gyro_stuck;
  reg bmi1_gyro_stuck;
  reg bmi0_rebias_pending;
  reg bmi1_rebias_pending;
  reg [6:0] bmi0_rebias_count;
  reg [6:0] bmi1_rebias_count;
  reg signed [31:0] bmi0_rebias_sum_x;
  reg signed [31:0] bmi0_rebias_sum_y;
  reg signed [31:0] bmi0_rebias_sum_z;
  reg signed [31:0] bmi1_rebias_sum_x;
  reg signed [31:0] bmi1_rebias_sum_y;
  reg signed [31:0] bmi1_rebias_sum_z;
  reg [3:0] single_imu_rebias_candidate_count;
  reg bmi0_auto_rebias;
  reg bmi1_auto_rebias;
  reg [31:0] single_rebias_flow_start_x;
  reg [31:0] single_rebias_flow_start_y;

  reg [7:0] pmw_dx_low;
  reg [7:0] pmw_dx_high;
  reg [7:0] pmw_dy_low;
  reg [7:0] pmw_dy_high;

  reg [9:0] calibration_count;
  reg signed [31:0] bmi0_bias_sum_x;
  reg signed [31:0] bmi0_bias_sum_y;
  reg signed [31:0] bmi0_bias_sum_z;
  reg signed [31:0] bmi1_bias_sum_x;
  reg signed [31:0] bmi1_bias_sum_y;
  reg signed [31:0] bmi1_bias_sum_z;
  reg signed [15:0] bmi0_gyro_bias_x;
  reg signed [15:0] bmi0_gyro_bias_y;
  reg signed [15:0] bmi0_gyro_bias_z;
  reg signed [15:0] bmi1_gyro_bias_x;
  reg signed [15:0] bmi1_gyro_bias_y;
  reg signed [15:0] bmi1_gyro_bias_z;

  reg cmd_valid;
  reg [2:0] cmd_device;
  reg [3:0] cmd_length;
  reg [71:0] cmd_tx_data;
  reg [15:0] cmd_setup_cycles;
  reg [15:0] cmd_gap_cycles;
  reg [15:0] cmd_hold_cycles;
  reg [15:0] cmd_post_cycles;

  reg [31:0] debug_period_count;
  reg debug_sending;
  reg [6:0] debug_char_index;
  reg [15:0] debug_sequence;
  reg [7:0] uart_data_byte;
  reg uart_start_pulse;
  reg uart_busy;
  reg uart_busy_d;
  reg [1:0] uart_state;
  reg [15:0] uart_clock_count;
  reg [2:0] uart_bit_index;
  reg [7:0] uart_shift;

  reg [15:0] snap_sequence;
  reg [7:0] snap_status;
  reg [7:0] snap_squal;
  reg [7:0] snap_pmw_id;
  reg [7:0] snap_pmw_inverse_id;
  reg [7:0] snap_bmi0_accel_id;
  reg [7:0] snap_bmi0_gyro_id;
  reg [7:0] snap_bmi1_accel_id;
  reg [7:0] snap_bmi1_gyro_id;
  reg [15:0] snap_flow_dx;
  reg [15:0] snap_flow_dy;
  reg [31:0] snap_flow_accum_x;
  reg [31:0] snap_flow_accum_y;
  reg [15:0] snap_accel_x;
  reg [15:0] snap_accel_y;
  reg [15:0] snap_accel_z;
  reg [15:0] snap_gyro_x;
  reg [15:0] snap_gyro_y;
  reg [15:0] snap_gyro_z;
  reg [15:0] snap_bmi0_gyro_z;
  reg [15:0] snap_bmi1_gyro_z;
`ifdef SENSOR_CAL_PROBE
  reg static_cal_start;
  reg static_cal_launched;
  reg [6:0] static_cal_stable_samples;
  reg [15:0] snap_bmi0_accel_x;
  reg [15:0] snap_bmi0_accel_y;
  reg [15:0] snap_bmi0_accel_z;
  reg [15:0] snap_bmi0_gyro_x_corrected;
  reg [15:0] snap_bmi0_gyro_y_corrected;
  reg [15:0] snap_bmi0_gyro_z_corrected;
  reg [15:0] snap_bmi1_accel_x;
  reg [15:0] snap_bmi1_accel_y;
  reg [15:0] snap_bmi1_accel_z;
  reg [15:0] snap_bmi1_gyro_x_corrected;
  reg [15:0] snap_bmi1_gyro_y_corrected;
  reg [15:0] snap_bmi1_gyro_z_corrected;
  reg [7:0] snap_static_cal_flags;
  reg [7:0] snap_static_cal_axes;
  reg [15:0] snap_static_cal_imu0_peak;
  reg [15:0] snap_static_cal_imu1_peak;
  reg [15:0] snap_static_cal_imu0_mean_x;
  reg [15:0] snap_static_cal_imu0_mean_y;
  reg [15:0] snap_static_cal_imu0_mean_z;
  reg [15:0] snap_static_cal_imu1_mean_x;
  reg [15:0] snap_static_cal_imu1_mean_y;
  reg [15:0] snap_static_cal_imu1_mean_z;
`endif

  wire uart_done;
  wire bmi0_bad_sample;
  wire bmi1_bad_sample;
  wire calibration_stationary;
  wire signed [16:0] bmi0_gyro_x_corrected;
  wire signed [16:0] bmi0_gyro_y_corrected;
  wire signed [16:0] bmi0_gyro_z_corrected;
  wire signed [16:0] bmi1_gyro_x_corrected;
  wire signed [16:0] bmi1_gyro_y_corrected;
  wire signed [16:0] bmi1_gyro_z_corrected;
  wire [16:0] accel_diff_x;
  wire [16:0] accel_diff_y;
  wire [16:0] accel_diff_z;
  wire [17:0] gyro_diff_x;
  wire [17:0] gyro_diff_y;
  wire [17:0] gyro_diff_z;
  wire [15:0] pmw_init_entry;
  wire [15:0] bmi0_accel_peak;
  wire [15:0] bmi1_accel_peak;
  wire [16:0] bmi0_gyro_peak;
  wire [16:0] bmi1_gyro_peak;
  wire [15:0] bmi0_raw_gyro_peak;
  wire [15:0] bmi1_raw_gyro_peak;
  wire bmi0_runtime_sample_plausible;
  wire bmi1_runtime_sample_plausible;
  wire bmi0_gyro_missing_now;
  wire bmi1_gyro_missing_now;
  wire bmi0_gyro_unusable;
  wire bmi1_gyro_unusable;
  wire accel_pair_agrees;
  wire gyro_pair_agrees;
`ifdef SENSOR_CAL_PROBE
  wire static_cal_busy;
  wire static_cal_done;
  wire static_cal_pass;
  wire [7:0] static_cal_flags;
  wire [1:0] static_cal_imu0_axis;
  wire static_cal_imu0_positive;
  wire [1:0] static_cal_imu1_axis;
  wire static_cal_imu1_positive;
  wire signed [15:0] static_cal_imu0_mean_x;
  wire signed [15:0] static_cal_imu0_mean_y;
  wire signed [15:0] static_cal_imu0_mean_z;
  wire signed [15:0] static_cal_imu1_mean_x;
  wire signed [15:0] static_cal_imu1_mean_y;
  wire signed [15:0] static_cal_imu1_mean_z;
  wire [15:0] static_cal_imu0_peak;
  wire [15:0] static_cal_imu1_peak;
  wire [7:0] static_cal_axes = {
      2'b00,
      static_cal_imu1_positive,
      static_cal_imu1_axis,
      static_cal_imu0_positive,
      static_cal_imu0_axis
  };
`endif

  assign pmw3901_cs_n = pmw_force_cs_low ? 1'b0 : spi_cs_n_raw[0];
  assign bmi0_acc_cs_n = spi_cs_n_raw[1];
  assign bmi0_gyro_cs_n = spi_cs_n_raw[2];
  assign bmi1_acc_cs_n = spi_cs_n_raw[3];
  assign bmi1_gyro_cs_n = spi_cs_n_raw[4];
  assign uart_done = uart_busy_d && !uart_busy;

  assign bmi0_bad_sample = ((bmi0_accel_x_raw == 16'hffff) &&
                            (bmi0_accel_y_raw == 16'hffff) &&
                            (bmi0_accel_z_raw == 16'hffff) &&
                            (bmi0_gyro_x_raw == 16'hffff) &&
                            (bmi0_gyro_y_raw == 16'hffff) &&
                            (bmi0_gyro_z_raw == 16'hffff)) ||
                           ((bmi0_accel_x_raw == 16'h0000) &&
                            (bmi0_accel_y_raw == 16'h0000) &&
                            (bmi0_accel_z_raw == 16'h0000) &&
                            (bmi0_gyro_x_raw == 16'h0000) &&
                            (bmi0_gyro_y_raw == 16'h0000) &&
                            (bmi0_gyro_z_raw == 16'h0000));

  assign bmi1_bad_sample = ((bmi1_accel_x_raw == 16'hffff) &&
                            (bmi1_accel_y_raw == 16'hffff) &&
                            (bmi1_accel_z_raw == 16'hffff) &&
                            (bmi1_gyro_x_raw == 16'hffff) &&
                            (bmi1_gyro_y_raw == 16'hffff) &&
                            (bmi1_gyro_z_raw == 16'hffff)) ||
                           ((bmi1_accel_x_raw == 16'h0000) &&
                            (bmi1_accel_y_raw == 16'h0000) &&
                            (bmi1_accel_z_raw == 16'h0000) &&
                            (bmi1_gyro_x_raw == 16'h0000) &&
                            (bmi1_gyro_y_raw == 16'h0000) &&
                            (bmi1_gyro_z_raw == 16'h0000));

  assign calibration_stationary =
      (!bmi0_online || ((abs16(bmi0_gyro_x_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi0_gyro_y_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi0_gyro_z_raw) <= CALIBRATION_GYRO_LIMIT))) &&
      (!bmi1_online || ((abs16(bmi1_gyro_x_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi1_gyro_y_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi1_gyro_z_raw) <= CALIBRATION_GYRO_LIMIT)));

  assign bmi0_gyro_x_corrected = $signed(bmi0_gyro_x_raw) - $signed(bmi0_gyro_bias_x);
  assign bmi0_gyro_y_corrected = $signed(bmi0_gyro_y_raw) - $signed(bmi0_gyro_bias_y);
  assign bmi0_gyro_z_corrected = $signed(bmi0_gyro_z_raw) - $signed(bmi0_gyro_bias_z);
  assign bmi1_gyro_x_corrected = $signed(bmi1_gyro_x_raw) - $signed(bmi1_gyro_bias_x);
  assign bmi1_gyro_y_corrected = $signed(bmi1_gyro_y_raw) - $signed(bmi1_gyro_bias_y);
  assign bmi1_gyro_z_corrected = $signed(bmi1_gyro_z_raw) - $signed(bmi1_gyro_bias_z);

  assign accel_diff_x = abs_diff16(bmi0_accel_x_raw, bmi1_accel_x_raw);
  assign accel_diff_y = abs_diff16(bmi0_accel_y_raw, bmi1_accel_y_raw);
  assign accel_diff_z = abs_diff16(bmi0_accel_z_raw, bmi1_accel_z_raw);
  assign gyro_diff_x = abs_diff17(bmi0_gyro_x_corrected, bmi1_gyro_x_corrected);
  assign gyro_diff_y = abs_diff17(bmi0_gyro_y_corrected, bmi1_gyro_y_corrected);
  assign gyro_diff_z = abs_diff17(bmi0_gyro_z_corrected, bmi1_gyro_z_corrected);
  assign pmw_init_entry = pmw_init_word(pmw_init_index);
  assign bmi0_accel_peak = max3_abs16(
      bmi0_accel_x_raw, bmi0_accel_y_raw, bmi0_accel_z_raw);
  assign bmi1_accel_peak = max3_abs16(
      bmi1_accel_x_raw, bmi1_accel_y_raw, bmi1_accel_z_raw);
  assign bmi0_gyro_peak = max3_abs17(
      bmi0_gyro_x_corrected, bmi0_gyro_y_corrected, bmi0_gyro_z_corrected);
  assign bmi1_gyro_peak = max3_abs17(
      bmi1_gyro_x_corrected, bmi1_gyro_y_corrected, bmi1_gyro_z_corrected);
  assign bmi0_raw_gyro_peak = max3_abs16(
      bmi0_gyro_x_raw, bmi0_gyro_y_raw, bmi0_gyro_z_raw);
  assign bmi1_raw_gyro_peak = max3_abs16(
      bmi1_gyro_x_raw, bmi1_gyro_y_raw, bmi1_gyro_z_raw);
  // Optical flow is used only as evidence that a large single-IMU gyro value
  // is a zero-offset fault rather than a deliberate hand rotation.  Requiring
  // an online PMW3901 with good surface quality keeps the recovery conservative.
  wire optical_stillness_valid = pmw3901_online && flow_data_seen &&
      (flow_squal >= GYRO_REBIAS_FLOW_SQUAL_MIN) &&
      (abs16(flow_delta_x) <= GYRO_REBIAS_FLOW_DELTA_MAX) &&
      (abs16(flow_delta_y) <= GYRO_REBIAS_FLOW_DELTA_MAX);
  wire single_imu_rebias_candidate = calibration_done &&
      !vehicle_motion_expected &&
      (bmi0_online ^ bmi1_online) && optical_stillness_valid &&
      ((bmi0_online && (bmi0_gyro_peak >= GYRO_SINGLE_REBIAS_TRIGGER)) ||
       (bmi1_online && (bmi1_gyro_peak >= GYRO_SINGLE_REBIAS_TRIGGER)));
  wire signed [32:0] single_rebias_flow_dx =
      $signed({flow_accum_x[31], flow_accum_x}) -
      $signed({single_rebias_flow_start_x[31], single_rebias_flow_start_x});
  wire signed [32:0] single_rebias_flow_dy =
      $signed({flow_accum_y[31], flow_accum_y}) -
      $signed({single_rebias_flow_start_y[31], single_rebias_flow_start_y});
  wire [32:0] single_rebias_flow_abs_x = single_rebias_flow_dx[32] ?
      $unsigned(-single_rebias_flow_dx) : $unsigned(single_rebias_flow_dx);
  wire [32:0] single_rebias_flow_abs_y = single_rebias_flow_dy[32] ?
      $unsigned(-single_rebias_flow_dy) : $unsigned(single_rebias_flow_dy);
  wire auto_rebias_flow_still = optical_stillness_valid &&
      (single_rebias_flow_abs_x <= GYRO_REBIAS_FLOW_ACCUM_MAX) &&
      (single_rebias_flow_abs_y <= GYRO_REBIAS_FLOW_ACCUM_MAX);
  assign bmi0_runtime_sample_plausible = !bmi0_bad_sample &&
      (bmi0_accel_peak >= 16'd2000) && (bmi0_accel_peak <= 16'd16000);
  assign bmi1_runtime_sample_plausible = !bmi1_bad_sample &&
      (bmi1_accel_peak >= 16'd2000) && (bmi1_accel_peak <= 16'd16000);
  assign bmi0_gyro_missing_now = bmi0_online && bmi1_online &&
      (bmi0_gyro_peak <= GYRO_MISSING_SELF_LIMIT) &&
      (bmi1_gyro_peak >= GYRO_MISSING_PEER_LIMIT);
  assign bmi1_gyro_missing_now = bmi0_online && bmi1_online &&
      (bmi1_gyro_peak <= GYRO_MISSING_SELF_LIMIT) &&
      (bmi0_gyro_peak >= GYRO_MISSING_PEER_LIMIT);
  // Withdraw a channel while its bias is being replaced. The C5 calibration
  // controller now waits and resumes the interrupted phase, so keeping a
  // known-biased stream visible would only corrupt heading and map state.
  assign bmi0_gyro_unusable = bmi0_gyro_stuck || bmi0_rebias_pending ||
      bmi0_gyro_missing_now;
  assign bmi1_gyro_unusable = bmi1_gyro_stuck || bmi1_rebias_pending ||
      bmi1_gyro_missing_now;
  assign accel_pair_agrees =
      (accel_diff_x <= ACCEL_DISAGREE_LIMIT) &&
      (accel_diff_y <= ACCEL_DISAGREE_LIMIT) &&
      (accel_diff_z <= ACCEL_DISAGREE_LIMIT);
  assign gyro_pair_agrees =
      (gyro_diff_x <= GYRO_DISAGREE_LIMIT) &&
      (gyro_diff_y <= GYRO_DISAGREE_LIMIT) &&
      (gyro_diff_z <= GYRO_DISAGREE_LIMIT);




  assign telemetry_data = {
      bmi1_gyro_z_corrected[15:0],
      bmi1_gyro_y_corrected[15:0],
      bmi1_gyro_x_corrected[15:0],
      bmi1_accel_z_raw,
      bmi1_accel_y_raw,
      bmi1_accel_x_raw,
      bmi0_gyro_z_corrected[15:0],
      bmi0_gyro_y_corrected[15:0],
      bmi0_gyro_x_corrected[15:0],
      bmi0_accel_z_raw,
      bmi0_accel_y_raw,
      bmi0_accel_x_raw,
      flow_accum_y,
      flow_accum_x,
      flow_delta_y,
      flow_delta_x,
      flow_squal,
      sample_sequence,
      {flow_data_seen, imu_disagreement, imu_degraded, fused_data_ready,
       calibration_done, bmi1_online, bmi0_online, pmw3901_online}
  };

`ifdef SENSOR_CAL_PROBE



  dual_imu_static_calibrator static_calibrator (
    .clk(clk),
    .rst_n(rst_n),
    .start(static_cal_start),
    .sample_valid(pair_sample_strobe),
    .imu0_online(bmi0_online),
    .imu1_online(bmi1_online),
    .imu0_accel_x(bmi0_accel_x_raw),
    .imu0_accel_y(bmi0_accel_y_raw),
    .imu0_accel_z(bmi0_accel_z_raw),
    .imu0_gyro_x(bmi0_gyro_x_corrected[15:0]),
    .imu0_gyro_y(bmi0_gyro_y_corrected[15:0]),
    .imu0_gyro_z(bmi0_gyro_z_corrected[15:0]),
    .imu1_accel_x(bmi1_accel_x_raw),
    .imu1_accel_y(bmi1_accel_y_raw),
    .imu1_accel_z(bmi1_accel_z_raw),
    .imu1_gyro_x(bmi1_gyro_x_corrected[15:0]),
    .imu1_gyro_y(bmi1_gyro_y_corrected[15:0]),
    .imu1_gyro_z(bmi1_gyro_z_corrected[15:0]),
    .busy(static_cal_busy),
    .done(static_cal_done),
    .pass(static_cal_pass),
    .result_flags(static_cal_flags),
    .imu0_vertical_axis(static_cal_imu0_axis),
    .imu0_vertical_positive(static_cal_imu0_positive),
    .imu1_vertical_axis(static_cal_imu1_axis),
    .imu1_vertical_positive(static_cal_imu1_positive),
    .imu0_accel_mean_x(),
    .imu0_accel_mean_y(),
    .imu0_accel_mean_z(),
    .imu0_gyro_mean_x(static_cal_imu0_mean_x),
    .imu0_gyro_mean_y(static_cal_imu0_mean_y),
    .imu0_gyro_mean_z(static_cal_imu0_mean_z),
    .imu1_accel_mean_x(),
    .imu1_accel_mean_y(),
    .imu1_accel_mean_z(),
    .imu1_gyro_mean_x(static_cal_imu1_mean_x),
    .imu1_gyro_mean_y(static_cal_imu1_mean_y),
    .imu1_gyro_mean_z(static_cal_imu1_mean_z),
    .imu0_gyro_peak(static_cal_imu0_peak),
    .imu1_gyro_peak(static_cal_imu1_peak)
  );

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      static_cal_start <= 1'b0;
      static_cal_launched <= 1'b0;
      static_cal_stable_samples <= 7'd0;
    end else begin
      static_cal_start <= 1'b0;
      if (!calibration_done || !bmi0_online || !bmi1_online) begin
        static_cal_launched <= 1'b0;
        static_cal_stable_samples <= 7'd0;
      end else if (static_cal_busy) begin
        static_cal_launched <= 1'b1;
        static_cal_stable_samples <= 7'd0;
      end else if (static_cal_done && static_cal_pass) begin
        static_cal_launched <= 1'b1;
        static_cal_stable_samples <= 7'd0;
      end else if (pair_sample_strobe) begin
        if (!calibration_stationary) begin
          static_cal_launched <= 1'b0;
          static_cal_stable_samples <= 7'd0;
        end else if (static_cal_stable_samples >= 7'd63) begin
          static_cal_start <= 1'b1;
          static_cal_launched <= 1'b1;
          static_cal_stable_samples <= 7'd0;
        end else begin
          static_cal_launched <= 1'b0;
          static_cal_stable_samples <= static_cal_stable_samples + 7'd1;
        end
      end
    end
  end
`endif

  function [15:0] abs16;
    input [15:0] value;
    begin
      abs16 = value[15] ? ((~value) + 16'd1) : value;
    end
  endfunction

  function [15:0] max3_abs16;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    reg [15:0] xy_max;
    begin
      xy_max = (abs16(x) >= abs16(y)) ? abs16(x) : abs16(y);
      max3_abs16 = (xy_max >= abs16(z)) ? xy_max : abs16(z);
    end
  endfunction

  function [16:0] abs_diff16;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [16:0] difference;
    begin
      difference = a - b;
      abs_diff16 = difference[16] ? -difference : difference;
    end
  endfunction

  function [16:0] abs17;
    input signed [16:0] value;
    begin
      abs17 = value[16] ? -value : value;
    end
  endfunction

  function [16:0] max3_abs17;
    input signed [16:0] x;
    input signed [16:0] y;
    input signed [16:0] z;
    reg [16:0] xy_max;
    begin
      xy_max = (abs17(x) >= abs17(y)) ? abs17(x) : abs17(y);
      max3_abs17 = (xy_max >= abs17(z)) ? xy_max : abs17(z);
    end
  endfunction

  function [17:0] abs_diff17;
    input signed [16:0] a;
    input signed [16:0] b;
    reg signed [17:0] difference;
    begin
      difference = a - b;
      abs_diff17 = difference[17] ? -difference : difference;
    end
  endfunction

  function [15:0] saturate16;
    input signed [17:0] value;
    begin
      if (value > 18'sd32767)
        saturate16 = 16'h7fff;
      else if (value < -18'sd32768)
        saturate16 = 16'h8000;
      else
        saturate16 = value[15:0];
    end
  endfunction



  function [15:0] pmw_init_word;
    input [6:0] index;
    begin
      case (index)
        7'd0:  pmw_init_word = 16'h7f00;
        7'd1:  pmw_init_word = 16'h61ad;
        7'd2:  pmw_init_word = 16'h7f03;
        7'd3:  pmw_init_word = 16'h4000;
        7'd4:  pmw_init_word = 16'h7f05;
        7'd5:  pmw_init_word = 16'h41b3;
        7'd6:  pmw_init_word = 16'h43f1;
        7'd7:  pmw_init_word = 16'h4514;
        7'd8:  pmw_init_word = 16'h5b32;
        7'd9:  pmw_init_word = 16'h5f34;
        7'd10: pmw_init_word = 16'h7b08;
        7'd11: pmw_init_word = 16'h7f06;
        7'd12: pmw_init_word = 16'h441b;
        7'd13: pmw_init_word = 16'h40bf;
        7'd14: pmw_init_word = 16'h4e3f;
        7'd15: pmw_init_word = 16'h7f08;
        7'd16: pmw_init_word = 16'h6520;
        7'd17: pmw_init_word = 16'h6a18;
        7'd18: pmw_init_word = 16'h7f09;
        7'd19: pmw_init_word = 16'h4faf;
        7'd20: pmw_init_word = 16'h5f40;
        7'd21: pmw_init_word = 16'h4880;
        7'd22: pmw_init_word = 16'h4980;
        7'd23: pmw_init_word = 16'h5777;
        7'd24: pmw_init_word = 16'h6078;
        7'd25: pmw_init_word = 16'h6178;
        7'd26: pmw_init_word = 16'h6208;
        7'd27: pmw_init_word = 16'h6350;
        7'd28: pmw_init_word = 16'h7f0a;
        7'd29: pmw_init_word = 16'h4560;
        7'd30: pmw_init_word = 16'h7f00;
        7'd31: pmw_init_word = 16'h4d11;
        7'd32: pmw_init_word = 16'h5580;
        7'd33: pmw_init_word = 16'h741f;
        7'd34: pmw_init_word = 16'h751f;
        7'd35: pmw_init_word = 16'h4a78;
        7'd36: pmw_init_word = 16'h4b78;
        7'd37: pmw_init_word = 16'h4408;
        7'd38: pmw_init_word = 16'h4550;
        7'd39: pmw_init_word = 16'h64ff;
        7'd40: pmw_init_word = 16'h651f;
        7'd41: pmw_init_word = 16'h7f14;
        7'd42: pmw_init_word = 16'h6560;
        7'd43: pmw_init_word = 16'h6608;
        7'd44: pmw_init_word = 16'h6378;
        7'd45: pmw_init_word = 16'h7f15;
        7'd46: pmw_init_word = 16'h4858;
        7'd47: pmw_init_word = 16'h7f07;
        7'd48: pmw_init_word = 16'h410d;
        7'd49: pmw_init_word = 16'h4314;
        7'd50: pmw_init_word = 16'h4b0e;
        7'd51: pmw_init_word = 16'h450f;
        7'd52: pmw_init_word = 16'h4442;
        7'd53: pmw_init_word = 16'h4c80;
        7'd54: pmw_init_word = 16'h7f10;
        7'd55: pmw_init_word = 16'h5b02;
        7'd56: pmw_init_word = 16'h7f07;
        7'd57: pmw_init_word = 16'h4041;
        7'd58: pmw_init_word = 16'h7000;
        7'd59: pmw_init_word = 16'h3244;
        7'd60: pmw_init_word = 16'h7f07;
        7'd61: pmw_init_word = 16'h4040;
        7'd62: pmw_init_word = 16'h7f06;
        7'd63: pmw_init_word = 16'h62f0;
        7'd64: pmw_init_word = 16'h6300;
        7'd65: pmw_init_word = 16'h7f0d;
        7'd66: pmw_init_word = 16'h48c0;
        7'd67: pmw_init_word = 16'h6fd5;
        7'd68: pmw_init_word = 16'h7f00;
        7'd69: pmw_init_word = 16'h5ba0;
        7'd70: pmw_init_word = 16'h4ea8;
        7'd71: pmw_init_word = 16'h5a50;
        default: pmw_init_word = 16'h4080;
      endcase
    end
  endfunction

  function [7:0] hex_ascii;
    input [3:0] nibble;
    begin
      hex_ascii = (nibble < 4'd10) ? (8'h30 + nibble) : (8'h41 + nibble - 4'd10);
    end
  endfunction

  function [7:0] hex8_char;
    input [7:0] value;
    input index;
    begin
      hex8_char = index ? hex_ascii(value[3:0]) : hex_ascii(value[7:4]);
    end
  endfunction

  function [7:0] hex16_char;
    input [15:0] value;
    input [1:0] index;
    begin
      case (index)
        2'd0: hex16_char = hex_ascii(value[15:12]);
        2'd1: hex16_char = hex_ascii(value[11:8]);
        2'd2: hex16_char = hex_ascii(value[7:4]);
        default: hex16_char = hex_ascii(value[3:0]);
      endcase
    end
  endfunction

  function [7:0] hex32_char;
    input [31:0] value;
    input [2:0] index;
    begin
      case (index)
        3'd0: hex32_char = hex_ascii(value[31:28]);
        3'd1: hex32_char = hex_ascii(value[27:24]);
        3'd2: hex32_char = hex_ascii(value[23:20]);
        3'd3: hex32_char = hex_ascii(value[19:16]);
        3'd4: hex32_char = hex_ascii(value[15:12]);
        3'd5: hex32_char = hex_ascii(value[11:8]);
        3'd6: hex32_char = hex_ascii(value[7:4]);
        default: hex32_char = hex_ascii(value[3:0]);
      endcase
    end
  endfunction




  function [7:0] debug_frame_char;
    input [6:0] index;
    begin
`ifdef SENSOR_CAL_PROBE






      if (index == 7'd0) debug_frame_char = "@";
      else if (index == 7'd1) debug_frame_char = "I";
      else if ((index >= 7'd2) && (index <= 7'd5)) debug_frame_char = hex16_char(snap_sequence, index - 7'd2);
      else if (index == 7'd6) debug_frame_char = ",";
      else if ((index >= 7'd7) && (index <= 7'd8)) debug_frame_char = hex8_char(snap_status, index - 7'd7);
      else if (index == 7'd9) debug_frame_char = ",";
      else if ((index >= 7'd10) && (index <= 7'd11)) debug_frame_char = hex8_char(snap_static_cal_flags, index - 7'd10);
      else if (index == 7'd12) debug_frame_char = ",";
      else if ((index >= 7'd13) && (index <= 7'd14)) debug_frame_char = hex8_char(snap_static_cal_axes, index - 7'd13);
      else if (index == 7'd15) debug_frame_char = ",";
      else if ((index >= 7'd16) && (index <= 7'd19)) debug_frame_char = hex16_char(snap_bmi0_accel_x, index - 7'd16);
      else if (index == 7'd20) debug_frame_char = ",";
      else if ((index >= 7'd21) && (index <= 7'd24)) debug_frame_char = hex16_char(snap_bmi0_accel_y, index - 7'd21);
      else if (index == 7'd25) debug_frame_char = ",";
      else if ((index >= 7'd26) && (index <= 7'd29)) debug_frame_char = hex16_char(snap_bmi0_accel_z, index - 7'd26);
      else if (index == 7'd30) debug_frame_char = ",";
      else if ((index >= 7'd31) && (index <= 7'd34)) debug_frame_char = hex16_char(snap_bmi0_gyro_x_corrected, index - 7'd31);
      else if (index == 7'd35) debug_frame_char = ",";
      else if ((index >= 7'd36) && (index <= 7'd39)) debug_frame_char = hex16_char(snap_bmi0_gyro_y_corrected, index - 7'd36);
      else if (index == 7'd40) debug_frame_char = ",";
      else if ((index >= 7'd41) && (index <= 7'd44)) debug_frame_char = hex16_char(snap_bmi0_gyro_z_corrected, index - 7'd41);
      else if (index == 7'd45) debug_frame_char = ",";
      else if ((index >= 7'd46) && (index <= 7'd49)) debug_frame_char = hex16_char(snap_bmi1_accel_x, index - 7'd46);
      else if (index == 7'd50) debug_frame_char = ",";
      else if ((index >= 7'd51) && (index <= 7'd54)) debug_frame_char = hex16_char(snap_bmi1_accel_y, index - 7'd51);
      else if (index == 7'd55) debug_frame_char = ",";
      else if ((index >= 7'd56) && (index <= 7'd59)) debug_frame_char = hex16_char(snap_bmi1_accel_z, index - 7'd56);
      else if (index == 7'd60) debug_frame_char = ",";
      else if ((index >= 7'd61) && (index <= 7'd64)) debug_frame_char = hex16_char(snap_bmi1_gyro_x_corrected, index - 7'd61);
      else if (index == 7'd65) debug_frame_char = ",";
      else if ((index >= 7'd66) && (index <= 7'd69)) debug_frame_char = hex16_char(snap_bmi1_gyro_y_corrected, index - 7'd66);
      else if (index == 7'd70) debug_frame_char = ",";
      else if ((index >= 7'd71) && (index <= 7'd74)) debug_frame_char = hex16_char(snap_bmi1_gyro_z_corrected, index - 7'd71);
      else if (index == 7'd75) debug_frame_char = ",";
      else if ((index >= 7'd76) && (index <= 7'd79)) debug_frame_char = hex16_char(snap_static_cal_imu0_peak, index - 7'd76);
      else if (index == 7'd80) debug_frame_char = ",";
      else if ((index >= 7'd81) && (index <= 7'd84)) debug_frame_char = hex16_char(snap_static_cal_imu1_peak, index - 7'd81);
      else if (index == 7'd85) debug_frame_char = ",";
      else if ((index >= 7'd86) && (index <= 7'd89)) debug_frame_char = hex16_char(snap_static_cal_imu0_mean_x, index - 7'd86);
      else if (index == 7'd90) debug_frame_char = ",";
      else if ((index >= 7'd91) && (index <= 7'd94)) debug_frame_char = hex16_char(snap_static_cal_imu0_mean_y, index - 7'd91);
      else if (index == 7'd95) debug_frame_char = ",";
      else if ((index >= 7'd96) && (index <= 7'd99)) debug_frame_char = hex16_char(snap_static_cal_imu0_mean_z, index - 7'd96);
      else if (index == 7'd100) debug_frame_char = ",";
      else if ((index >= 7'd101) && (index <= 7'd104)) debug_frame_char = hex16_char(snap_static_cal_imu1_mean_x, index - 7'd101);
      else if (index == 7'd105) debug_frame_char = ",";
      else if ((index >= 7'd106) && (index <= 7'd109)) debug_frame_char = hex16_char(snap_static_cal_imu1_mean_y, index - 7'd106);
      else if (index == 7'd110) debug_frame_char = ",";
      else if ((index >= 7'd111) && (index <= 7'd114)) debug_frame_char = hex16_char(snap_static_cal_imu1_mean_z, index - 7'd111);
      else debug_frame_char = 8'h0a;
`else
      if (index == 7'd0) debug_frame_char = "@";
      else if (index == 7'd1) debug_frame_char = "M";
      else if ((index >= 7'd2) && (index <= 7'd5)) debug_frame_char = hex16_char(snap_sequence, index - 7'd2);
      else if (index == 7'd6) debug_frame_char = ",";
      else if ((index >= 7'd7) && (index <= 7'd8)) debug_frame_char = hex8_char(snap_status, index - 7'd7);
      else if (index == 7'd9) debug_frame_char = ",";
      else if ((index >= 7'd10) && (index <= 7'd11)) debug_frame_char = hex8_char(snap_squal, index - 7'd10);
      else if (index == 7'd12) debug_frame_char = ",";
      else if ((index >= 7'd13) && (index <= 7'd14)) debug_frame_char = hex8_char(snap_pmw_id, index - 7'd13);
      else if (index == 7'd15) debug_frame_char = ",";
      else if ((index >= 7'd16) && (index <= 7'd17)) debug_frame_char = hex8_char(snap_pmw_inverse_id, index - 7'd16);
      else if (index == 7'd18) debug_frame_char = ",";
      else if ((index >= 7'd19) && (index <= 7'd20)) debug_frame_char = hex8_char(snap_bmi0_accel_id, index - 7'd19);
      else if (index == 7'd21) debug_frame_char = ",";
      else if ((index >= 7'd22) && (index <= 7'd23)) debug_frame_char = hex8_char(snap_bmi0_gyro_id, index - 7'd22);
      else if (index == 7'd24) debug_frame_char = ",";
      else if ((index >= 7'd25) && (index <= 7'd26)) debug_frame_char = hex8_char(snap_bmi1_accel_id, index - 7'd25);
      else if (index == 7'd27) debug_frame_char = ",";
      else if ((index >= 7'd28) && (index <= 7'd29)) debug_frame_char = hex8_char(snap_bmi1_gyro_id, index - 7'd28);
      else if (index == 7'd30) debug_frame_char = ",";
      else if ((index >= 7'd31) && (index <= 7'd34)) debug_frame_char = hex16_char(snap_flow_dx, index - 7'd31);
      else if (index == 7'd35) debug_frame_char = ",";
      else if ((index >= 7'd36) && (index <= 7'd39)) debug_frame_char = hex16_char(snap_flow_dy, index - 7'd36);
      else if (index == 7'd40) debug_frame_char = ",";
      else if ((index >= 7'd41) && (index <= 7'd48)) debug_frame_char = hex32_char(snap_flow_accum_x, index - 7'd41);
      else if (index == 7'd49) debug_frame_char = ",";
      else if ((index >= 7'd50) && (index <= 7'd57)) debug_frame_char = hex32_char(snap_flow_accum_y, index - 7'd50);
      else if (index == 7'd58) debug_frame_char = ",";
      else if ((index >= 7'd59) && (index <= 7'd62)) debug_frame_char = hex16_char(snap_accel_x, index - 7'd59);
      else if (index == 7'd63) debug_frame_char = ",";
      else if ((index >= 7'd64) && (index <= 7'd67)) debug_frame_char = hex16_char(snap_accel_y, index - 7'd64);
      else if (index == 7'd68) debug_frame_char = ",";
      else if ((index >= 7'd69) && (index <= 7'd72)) debug_frame_char = hex16_char(snap_accel_z, index - 7'd69);
      else if (index == 7'd73) debug_frame_char = ",";
      else if ((index >= 7'd74) && (index <= 7'd77)) debug_frame_char = hex16_char(snap_gyro_x, index - 7'd74);
      else if (index == 7'd78) debug_frame_char = ",";
      else if ((index >= 7'd79) && (index <= 7'd82)) debug_frame_char = hex16_char(snap_gyro_y, index - 7'd79);
      else if (index == 7'd83) debug_frame_char = ",";
      else if ((index >= 7'd84) && (index <= 7'd87)) debug_frame_char = hex16_char(snap_gyro_z, index - 7'd84);
      else if (index == 7'd88) debug_frame_char = ",";
      else if ((index >= 7'd89) && (index <= 7'd92)) debug_frame_char = hex16_char(snap_bmi0_gyro_z, index - 7'd89);
      else if (index == 7'd93) debug_frame_char = ",";
      else if ((index >= 7'd94) && (index <= 7'd97)) debug_frame_char = hex16_char(snap_bmi1_gyro_z, index - 7'd94);
      else debug_frame_char = 8'h0a;
`endif
    end
  endfunction




  always @(*) begin
    cmd_valid = 1'b0;
    cmd_device = DEV_PMW;
    cmd_length = 4'd2;
    cmd_tx_data = 72'd0;
    cmd_setup_cycles = PMW_SETUP_CYCLES;
    cmd_gap_cycles = 16'd0;
    cmd_hold_cycles = PMW_WRITE_HOLD_CYCLES;
    cmd_post_cycles = PMW_WRITE_POST_CYCLES;

    case (main_state)
      ST_PMW_RESET: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'hba, 8'h5a, 56'd0};
      end
      ST_PMW_ID: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'h00, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_INVERSE_ID: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'h5f, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_CLEAR: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {{5'd0, pmw_clear_index + 3'd2}, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_INIT: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {pmw_init_entry[15:8] | 8'h80,
                       pmw_init_entry[7:0], 56'd0};
      end
      ST_BMI_INIT: begin
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
        case (bmi_init_step)
          5'd0, 5'd3, 5'd4: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_length = 4'd3;
            cmd_tx_data = {8'h80, 64'd0};
          end
          5'd1: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7e, 8'hb6, 56'd0};
          end
          5'd5: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h14, 8'hb6, 56'd0};
          end
          5'd7: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h80, 64'd0};
          end
          5'd8: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7c, 8'h00, 56'd0};
          end
          5'd10: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7d, 8'h04, 56'd0};
          end
          5'd12: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h40, 8'h89, 56'd0};
          end
          5'd14: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h41, 8'h01, 56'd0};
          end
          5'd15: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h11, 8'h00, 56'd0};
          end
          5'd17: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h10, 8'h04, 56'd0};
          end
          5'd18: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h0f, 8'h03, 56'd0};
          end
          5'd19: begin



            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h8f, 64'd0};
          end
          default: begin end
        endcase
      end
      ST_IMU0_ACCEL, ST_IMU1_ACCEL: begin
        cmd_valid = 1'b1;
        cmd_device = (main_state == ST_IMU0_ACCEL) ? DEV_BMI0_ACC : DEV_BMI1_ACC;
        cmd_length = 4'd8;
        cmd_tx_data = {8'h92, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_IMU0_GYRO, ST_IMU1_GYRO: begin
        cmd_valid = 1'b1;
        cmd_device = (main_state == ST_IMU0_GYRO) ? DEV_BMI0_GYRO : DEV_BMI1_GYRO;
        cmd_length = 4'd7;
        cmd_tx_data = {8'h82, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_RESET: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h14, 8'hb6, 56'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_ID: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h80, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_POWER: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h11, 8'h00, 56'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_BW: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h10, 8'h04, 56'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_RANGE: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h0f, 8'h03, 56'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RECOVERY_VERIFY: begin
        cmd_valid = 1'b1;
        cmd_device = gyro_recovery_target ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
        cmd_tx_data = {8'h8f, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_GYRO_RANGE_GUARD0, ST_GYRO_RANGE_GUARD1: begin
        cmd_valid = 1'b1;
        cmd_device = (main_state == ST_GYRO_RANGE_GUARD0) ?
            DEV_BMI0_GYRO : DEV_BMI1_GYRO;
        cmd_tx_data = {8'h0f, 8'h03, 56'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_PMW_MOTION, ST_PMW_DX_LOW, ST_PMW_DX_HIGH,
      ST_PMW_DY_LOW, ST_PMW_DY_HIGH, ST_PMW_SQUAL: begin
        cmd_valid = 1'b1;
        case (main_state)
          ST_PMW_MOTION:  cmd_tx_data = {8'h02, 64'd0};
          ST_PMW_DX_LOW:  cmd_tx_data = {8'h03, 64'd0};
          ST_PMW_DX_HIGH: cmd_tx_data = {8'h04, 64'd0};
          ST_PMW_DY_LOW:  cmd_tx_data = {8'h05, 64'd0};
          ST_PMW_DY_HIGH: cmd_tx_data = {8'h06, 64'd0};
          default:        cmd_tx_data = {8'h07, 64'd0};
        endcase
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      default: begin end
    endcase
  end



  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sensor_spi_sclk <= 1'b1;
      sensor_spi_mosi <= 1'b0;
      spi_cs_n_raw <= 5'b11111;
      spi_busy <= 1'b0;
      spi_done <= 1'b0;
      spi_state <= SPI_IDLE;
      spi_length_latched <= 4'd0;
      spi_byte_count <= 4'd0;
      spi_bit_index <= 3'd7;
      spi_half_count <= 16'd0;
      spi_delay_count <= 16'd0;
      spi_setup_latched <= 16'd0;
      spi_gap_latched <= 16'd0;
      spi_hold_latched <= 16'd0;
      spi_post_latched <= 16'd0;
      spi_tx_shift <= 72'd0;
      spi_rx_shift <= 72'd0;
      spi_rx_data <= 72'd0;
    end else begin
      spi_done <= 1'b0;
      case (spi_state)
        SPI_IDLE: begin
          sensor_spi_sclk <= 1'b1;
          sensor_spi_mosi <= 1'b0;
          spi_cs_n_raw <= 5'b11111;
          spi_busy <= 1'b0;
          spi_half_count <= 16'd0;
          spi_delay_count <= 16'd0;
          if (spi_start) begin
            spi_busy <= 1'b1;
            spi_length_latched <= cmd_length;
            spi_byte_count <= 4'd0;
            spi_bit_index <= 3'd7;
            spi_setup_latched <= cmd_setup_cycles;
            spi_gap_latched <= cmd_gap_cycles;
            spi_hold_latched <= cmd_hold_cycles;
            spi_post_latched <= cmd_post_cycles;
            spi_tx_shift <= cmd_tx_data;
            spi_rx_shift <= 72'd0;
            case (cmd_device)
              DEV_PMW:       spi_cs_n_raw <= 5'b11110;
              DEV_BMI0_ACC:  spi_cs_n_raw <= 5'b11101;
              DEV_BMI0_GYRO: spi_cs_n_raw <= 5'b11011;
              DEV_BMI1_ACC:  spi_cs_n_raw <= 5'b10111;
              default:       spi_cs_n_raw <= 5'b01111;
            endcase
            spi_state <= SPI_SETUP;
          end
        end
        SPI_SETUP: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_setup_latched) begin
            spi_delay_count <= 16'd0;
            spi_half_count <= 16'd0;
            spi_state <= SPI_DRIVE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        SPI_DRIVE: begin
          spi_busy <= 1'b1;
          if (spi_half_count >= SPI_HALF_CYCLES - 1) begin
            spi_half_count <= 16'd0;
            sensor_spi_sclk <= 1'b0;
            sensor_spi_mosi <= spi_tx_shift[71];
            spi_state <= SPI_SAMPLE;
          end else begin
            spi_half_count <= spi_half_count + 16'd1;
          end
        end
        SPI_SAMPLE: begin
          spi_busy <= 1'b1;
          if (spi_half_count >= SPI_HALF_CYCLES - 1) begin
            spi_half_count <= 16'd0;
            sensor_spi_sclk <= 1'b1;
            spi_rx_shift <= {spi_rx_shift[70:0], sensor_spi_miso};
            spi_tx_shift <= {spi_tx_shift[70:0], 1'b0};
            if (spi_bit_index == 3'd0) begin
              spi_bit_index <= 3'd7;
              if (spi_byte_count + 4'd1 >= spi_length_latched) begin
                spi_rx_data <= {spi_rx_shift[70:0], sensor_spi_miso};
                spi_delay_count <= 16'd0;
                spi_state <= SPI_HOLD;
              end else begin
                spi_byte_count <= spi_byte_count + 4'd1;
                spi_delay_count <= 16'd0;
                spi_state <= SPI_GAP;
              end
            end else begin
              spi_bit_index <= spi_bit_index - 3'd1;
              spi_state <= SPI_DRIVE;
            end
          end else begin
            spi_half_count <= spi_half_count + 16'd1;
          end
        end
        SPI_GAP: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_gap_latched) begin
            spi_delay_count <= 16'd0;
            spi_state <= SPI_DRIVE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        SPI_HOLD: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_hold_latched) begin
            spi_delay_count <= 16'd0;
            spi_cs_n_raw <= 5'b11111;
            if (spi_post_latched == 16'd0) begin
              spi_busy <= 1'b0;
              spi_done <= 1'b1;
              spi_state <= SPI_IDLE;
            end else begin
              spi_state <= SPI_POST;
            end
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        default: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_post_latched) begin
            spi_delay_count <= 16'd0;
            spi_busy <= 1'b0;
            spi_done <= 1'b1;
            spi_state <= SPI_IDLE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
      endcase
    end
  end


  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      main_state <= ST_BOOT;
      spi_start <= 1'b0;
      command_inflight <= 1'b0;
      state_wait_count <= 32'd0;
      pmw_force_cs_low <= 1'b0;
      pmw_init_index <= 7'd0;
      pmw_clear_index <= 3'd0;
      pmw_retry_count <= 2'd0;
      bmi_index <= 1'b0;
      bmi_init_step <= 5'd0;
      current_bmi_accel_id <= 8'd0;
      current_bmi_gyro_id <= 8'd0;
      current_bmi_gyro_range <= 8'd0;
      bmi0_gyro_range <= 8'd0;
      bmi1_gyro_range <= 8'd0;
      bmi_accel_id_retry <= 2'd0;
      bmi_gyro_range_retry <= 2'd0;
      gyro_recovery_target <= 1'b0;
      gyro_recovery_retry_count <= 2'd0;
      gyro_recovery_done_pulse <= 1'b0;
      gyro_recovery_success <= 1'b0;
      bmi0_gyro_recovery_attempted <= 1'b0;
      bmi1_gyro_recovery_attempted <= 1'b0;
      bmi0_gyro_quarantined <= 1'b0;
      bmi1_gyro_quarantined <= 1'b0;
      gyro_range_guard_count <= 5'd0;
      pmw_poll_toggle <= 1'b0;
      pair_sample_strobe <= 1'b0;
      flow_data_seen <= 1'b0;
      pmw3901_online <= 1'b0;
      bmi0_online <= 1'b0;
      bmi1_online <= 1'b0;
      pmw3901_product_id <= 8'd0;
      pmw3901_inverse_id <= 8'd0;
      bmi0_accel_id <= 8'd0;
      bmi0_gyro_id <= 8'd0;
      bmi1_accel_id <= 8'd0;
      bmi1_gyro_id <= 8'd0;
      bmi0_accel_x_raw <= 16'sd0;
      bmi0_accel_y_raw <= 16'sd0;
      bmi0_accel_z_raw <= 16'sd0;
      bmi0_gyro_x_raw <= 16'sd0;
      bmi0_gyro_y_raw <= 16'sd0;
      bmi0_gyro_z_raw <= 16'sd0;
      bmi1_accel_x_raw <= 16'sd0;
      bmi1_accel_y_raw <= 16'sd0;
      bmi1_accel_z_raw <= 16'sd0;
      bmi1_gyro_x_raw <= 16'sd0;
      bmi1_gyro_y_raw <= 16'sd0;
      bmi1_gyro_z_raw <= 16'sd0;
      bmi0_bad_sample_count <= 6'd0;
      bmi1_bad_sample_count <= 6'd0;
      bmi0_good_sample_count <= 4'd0;
      bmi1_good_sample_count <= 4'd0;
      pmw_dx_low <= 8'd0;
      pmw_dx_high <= 8'd0;
      pmw_dy_low <= 8'd0;
      pmw_dy_high <= 8'd0;
      flow_sample_valid <= 1'b0;
      flow_squal <= 8'd0;
      flow_delta_x <= 16'sd0;
      flow_delta_y <= 16'sd0;
      flow_accum_x <= 32'sd0;
      flow_accum_y <= 32'sd0;
      sample_sequence <= 16'd0;
    end else begin
      spi_start <= 1'b0;
      pair_sample_strobe <= 1'b0;
      flow_sample_valid <= 1'b0;
      gyro_recovery_done_pulse <= 1'b0;

      if (cmd_valid && !command_inflight && !spi_busy) begin
        spi_start <= 1'b1;
        command_inflight <= 1'b1;
      end

      if (cmd_valid && command_inflight && spi_done) begin
        command_inflight <= 1'b0;
        state_wait_count <= 32'd0;
        case (main_state)
          ST_PMW_RESET: main_state <= ST_PMW_RESET_WAIT;
          ST_PMW_ID: begin
            pmw3901_product_id <= spi_rx_data[7:0];
            main_state <= ST_PMW_INVERSE_ID;
          end
          ST_PMW_INVERSE_ID: begin
            pmw3901_inverse_id <= spi_rx_data[7:0];
            if ((pmw3901_product_id == 8'h49) && (spi_rx_data[7:0] == 8'hb6)) begin
              pmw_clear_index <= 3'd0;
              main_state <= ST_PMW_CLEAR;
            end else if (pmw_retry_count < 2'd2) begin
              pmw_retry_count <= pmw_retry_count + 2'd1;
              main_state <= ST_PMW_RETRY_WAIT;
            end else begin
              pmw3901_online <= 1'b0;
              bmi_index <= 1'b0;
              bmi_init_step <= 5'd0;
              current_bmi_accel_id <= 8'd0;
               current_bmi_gyro_id <= 8'd0;
               current_bmi_gyro_range <= 8'd0;
               bmi_accel_id_retry <= 2'd0;
               bmi_gyro_range_retry <= 2'd0;
              main_state <= ST_BMI_INIT;
            end
          end
          ST_PMW_CLEAR: begin
            if (pmw_clear_index == 3'd4) begin
              pmw_init_index <= 7'd0;
              main_state <= ST_PMW_INIT;
            end else begin
              pmw_clear_index <= pmw_clear_index + 3'd1;
            end
          end
          ST_PMW_INIT: begin
            if (pmw_init_index == 7'd58) begin
              main_state <= ST_PMW_INIT_LONG_WAIT;
            end else if (pmw_init_index == 7'd72) begin
              pmw3901_online <= 1'b1;
              bmi_index <= 1'b0;
              bmi_init_step <= 5'd0;
              current_bmi_accel_id <= 8'd0;
               current_bmi_gyro_id <= 8'd0;
               current_bmi_gyro_range <= 8'd0;
               bmi_accel_id_retry <= 2'd0;
               bmi_gyro_range_retry <= 2'd0;
              main_state <= ST_BMI_INIT;
            end else begin
              pmw_init_index <= pmw_init_index + 7'd1;
            end
          end
          ST_BMI_INIT: begin
            if (bmi_init_step == 5'd4) begin
              current_bmi_accel_id <= spi_rx_data[7:0];
              if ((spi_rx_data[7:0] != 8'h1e) &&
                  (bmi_accel_id_retry < 2'd3)) begin



                bmi_accel_id_retry <= bmi_accel_id_retry + 2'd1;
                bmi_init_step <= 5'd21;
              end else begin
                bmi_init_step <= 5'd5;
              end
            end else if (bmi_init_step == 5'd7) begin
              current_bmi_gyro_id <= spi_rx_data[7:0];
              bmi_init_step <= 5'd8;
            end else if (bmi_init_step == 5'd19) begin
              current_bmi_gyro_range <= spi_rx_data[7:0];
              if ((spi_rx_data[7:0] != 8'h03) &&
                  (bmi_gyro_range_retry < 2'd2)) begin
                bmi_gyro_range_retry <= bmi_gyro_range_retry + 2'd1;
                bmi_init_step <= 5'd18;
              end else begin
                bmi_init_step <= 5'd20;
              end
            end else begin
              bmi_init_step <= bmi_init_step + 5'd1;
            end
          end
          ST_IMU0_ACCEL: begin
            bmi0_accel_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi0_accel_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi0_accel_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU0_GYRO;
          end
          ST_IMU0_GYRO: begin
            bmi0_gyro_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi0_gyro_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi0_gyro_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU1_ACCEL;
          end
          ST_IMU1_ACCEL: begin
            bmi1_accel_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi1_accel_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi1_accel_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU1_GYRO;
          end
          ST_IMU1_GYRO: begin
            bmi1_gyro_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi1_gyro_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi1_gyro_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_FUSION_STROBE;
          end
          ST_GYRO_RECOVERY_RESET: begin
            main_state <= ST_GYRO_RECOVERY_RESET_WAIT;
          end
          ST_GYRO_RECOVERY_ID: begin
            if (gyro_recovery_target)
              bmi1_gyro_id <= spi_rx_data[7:0];
            else
              bmi0_gyro_id <= spi_rx_data[7:0];
            if (spi_rx_data[7:0] == 8'h0f) begin
              main_state <= ST_GYRO_RECOVERY_POWER;
            end else if (gyro_recovery_retry_count < 2'd2) begin
              gyro_recovery_retry_count <= gyro_recovery_retry_count + 2'd1;
              main_state <= ST_GYRO_RECOVERY_RESET;
            end else begin
              if (gyro_recovery_target) bmi1_online <= 1'b0;
              else bmi0_online <= 1'b0;
              if (gyro_recovery_target) bmi1_gyro_quarantined <= 1'b1;
              else bmi0_gyro_quarantined <= 1'b1;
              gyro_recovery_done_pulse <= 1'b1;
              gyro_recovery_success <= 1'b0;
              main_state <= ST_RUN_WAIT;
            end
          end
          ST_GYRO_RECOVERY_POWER: begin
            main_state <= ST_GYRO_RECOVERY_POWER_WAIT;
          end
          ST_GYRO_RECOVERY_BW: begin
            main_state <= ST_GYRO_RECOVERY_RANGE;
          end
          ST_GYRO_RECOVERY_RANGE: begin
            main_state <= ST_GYRO_RECOVERY_VERIFY;
          end
          ST_GYRO_RECOVERY_VERIFY: begin
            if (gyro_recovery_target)
              bmi1_gyro_range <= spi_rx_data[7:0];
            else
              bmi0_gyro_range <= spi_rx_data[7:0];
            if (spi_rx_data[7:0] == 8'h03) begin
              gyro_recovery_retry_count <= 2'd0;
              gyro_recovery_done_pulse <= 1'b1;
              gyro_recovery_success <= 1'b1;
              main_state <= ST_RUN_WAIT;
            end else if (gyro_recovery_retry_count < 2'd2) begin
              gyro_recovery_retry_count <= gyro_recovery_retry_count + 2'd1;
              main_state <= ST_GYRO_RECOVERY_RESET;
            end else begin
              if (gyro_recovery_target) bmi1_online <= 1'b0;
              else bmi0_online <= 1'b0;
              if (gyro_recovery_target) bmi1_gyro_quarantined <= 1'b1;
              else bmi0_gyro_quarantined <= 1'b1;
              gyro_recovery_done_pulse <= 1'b1;
              gyro_recovery_success <= 1'b0;
              main_state <= ST_RUN_WAIT;
            end
          end
          ST_GYRO_RANGE_GUARD0: begin
            bmi0_gyro_range <= 8'h03;
            if (bmi1_online)
              main_state <= ST_GYRO_RANGE_GUARD1;
            else
              main_state <= ST_RUN_WAIT;
          end
          ST_GYRO_RANGE_GUARD1: begin
            bmi1_gyro_range <= 8'h03;
            main_state <= ST_RUN_WAIT;
          end
          ST_PMW_MOTION: main_state <= ST_PMW_DX_LOW;
          ST_PMW_DX_LOW: begin
            pmw_dx_low <= spi_rx_data[7:0];
            main_state <= ST_PMW_DX_HIGH;
          end
          ST_PMW_DX_HIGH: begin
            pmw_dx_high <= spi_rx_data[7:0];
            main_state <= ST_PMW_DY_LOW;
          end
          ST_PMW_DY_LOW: begin
            pmw_dy_low <= spi_rx_data[7:0];
            main_state <= ST_PMW_DY_HIGH;
          end
          ST_PMW_DY_HIGH: begin
            pmw_dy_high <= spi_rx_data[7:0];
            main_state <= ST_PMW_SQUAL;
          end
          default: begin
            flow_squal <= spi_rx_data[7:0];
            flow_delta_x <= {pmw_dx_high, pmw_dx_low};
            flow_delta_y <= {pmw_dy_high, pmw_dy_low};
            flow_accum_x <= $signed(flow_accum_x) + $signed({pmw_dx_high, pmw_dx_low});
            flow_accum_y <= $signed(flow_accum_y) + $signed({pmw_dy_high, pmw_dy_low});
            flow_sample_valid <= pmw3901_online;
            flow_data_seen <= pmw3901_online;
            main_state <= ST_RUN_WAIT;
          end
        endcase
      end else if (!cmd_valid) begin
        case (main_state)
          ST_BOOT: begin
            pmw_force_cs_low <= 1'b0;
            if (state_wait_count >= BOOT_WAIT_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b1;
              main_state <= ST_PMW_CS_LOW;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_CS_LOW: begin
            if (state_wait_count >= ONE_MS_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b0;
              main_state <= ST_PMW_CS_HIGH;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_CS_HIGH: begin
            if (state_wait_count >= ONE_MS_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_PMW_RESET;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_RESET_WAIT: begin
            if (state_wait_count >= (5 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_PMW_ID;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_RETRY_WAIT: begin
            if (state_wait_count >= (100 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b1;
              main_state <= ST_PMW_CS_LOW;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_INIT_LONG_WAIT: begin
            if (state_wait_count >= (100 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              pmw_init_index <= 7'd59;
              main_state <= ST_PMW_INIT;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_BMI_INIT: begin
            case (bmi_init_step)
              5'd2: begin
                if (state_wait_count >= (2 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= 5'd3;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd6, 5'd16: begin
                if (state_wait_count >= (30 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= bmi_init_step + 5'd1;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd9, 5'd11: begin
                if (state_wait_count >= (5 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= bmi_init_step + 5'd1;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd13: begin
                if (state_wait_count >= (40 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= 5'd14;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd21: begin
                if (state_wait_count >= (2 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= 5'd3;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd20: begin
                if (!bmi_index) begin
                  bmi0_accel_id <= current_bmi_accel_id;
                  bmi0_gyro_id <= current_bmi_gyro_id;
                  bmi0_gyro_range <= current_bmi_gyro_range;
                  bmi0_online <= (current_bmi_accel_id == 8'h1e) &&
                                 (current_bmi_gyro_id == 8'h0f) &&
                                 (current_bmi_gyro_range == 8'h03);
                  bmi_index <= 1'b1;
                  bmi_init_step <= 5'd0;
                  current_bmi_accel_id <= 8'd0;
                  current_bmi_gyro_id <= 8'd0;
                  current_bmi_gyro_range <= 8'd0;
                  bmi_accel_id_retry <= 2'd0;
                  bmi_gyro_range_retry <= 2'd0;
                end else begin
                  bmi1_accel_id <= current_bmi_accel_id;
                  bmi1_gyro_id <= current_bmi_gyro_id;
                  bmi1_gyro_range <= current_bmi_gyro_range;
                  bmi1_online <= (current_bmi_accel_id == 8'h1e) &&
                                 (current_bmi_gyro_id == 8'h0f) &&
                                 (current_bmi_gyro_range == 8'h03);
                  state_wait_count <= 32'd0;
                  main_state <= ST_RUN_WAIT;
                end
              end
              default: begin end
            endcase
          end
          ST_RUN_WAIT: begin
            if (state_wait_count >= RUN_WAIT_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_IMU0_ACCEL;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_GYRO_RECOVERY_RESET_WAIT: begin
            // BMI088 gyro soft reset needs 30 ms before CHIP_ID is reliable.
            if (state_wait_count >= (30 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_GYRO_RECOVERY_ID;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_GYRO_RECOVERY_POWER_WAIT: begin
            // Normal-mode power-up has the same 30 ms settling requirement.
            if (state_wait_count >= (30 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_GYRO_RECOVERY_BW;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_FUSION_STROBE: begin
            pair_sample_strobe <= 1'b1;
            sample_sequence <= sample_sequence + 16'd1;
            if (gyro_range_guard_count < GYRO_RANGE_GUARD_SAMPLES - 1'b1)
              gyro_range_guard_count <= gyro_range_guard_count + 5'd1;

            if (bmi0_online) begin
              bmi0_good_sample_count <= 4'd0;
              if (!bmi0_runtime_sample_plausible) begin
                if (bmi0_bad_sample_count >= 6'd31) begin
                  bmi0_bad_sample_count <= 6'd32;
                  bmi0_online <= 1'b0;
                end else bmi0_bad_sample_count <= bmi0_bad_sample_count + 6'd1;
              end else bmi0_bad_sample_count <= 6'd0;
            end else if (bmi0_runtime_sample_plausible &&
                         (!bmi0_gyro_quarantined ||
                          (bmi0_gyro_peak > GYRO_MISSING_PEER_LIMIT))) begin
              bmi0_bad_sample_count <= 6'd0;
              if (bmi0_good_sample_count >= 4'd7) begin
                bmi0_good_sample_count <= 4'd0;
                bmi0_online <= 1'b1;
                bmi0_gyro_quarantined <= 1'b0;
                bmi0_gyro_recovery_attempted <= 1'b0;
              end else bmi0_good_sample_count <= bmi0_good_sample_count + 4'd1;
            end else begin
              bmi0_good_sample_count <= 4'd0;
            end
            if (bmi1_online) begin
              bmi1_good_sample_count <= 4'd0;
              if (!bmi1_runtime_sample_plausible) begin
                if (bmi1_bad_sample_count >= 6'd31) begin
                  bmi1_bad_sample_count <= 6'd32;
                  bmi1_online <= 1'b0;
                end else bmi1_bad_sample_count <= bmi1_bad_sample_count + 6'd1;
              end else bmi1_bad_sample_count <= 6'd0;
            end else if (bmi1_runtime_sample_plausible &&
                         (!bmi1_gyro_quarantined ||
                          (bmi1_gyro_peak > GYRO_MISSING_PEER_LIMIT))) begin
              bmi1_bad_sample_count <= 6'd0;
              if (bmi1_good_sample_count >= 4'd7) begin
                bmi1_good_sample_count <= 4'd0;
                bmi1_online <= 1'b1;
                bmi1_gyro_quarantined <= 1'b0;
                bmi1_gyro_recovery_attempted <= 1'b0;
              end else bmi1_good_sample_count <= bmi1_good_sample_count + 4'd1;
            end else begin
              bmi1_good_sample_count <= 4'd0;
            end

            // A BMI088 package contains independent accel and gyro dies.  A
            // live accelerometer therefore cannot repair a gyro die that has
            // stopped updating.  Recover only the failed gyro so the healthy
            // peer and the rest of the sensor subsystem remain operational.
            if (bmi0_online && bmi0_gyro_stuck && !bmi0_rebias_pending) begin
              if (bmi0_gyro_recovery_attempted) begin
                // The channel failed again after a complete reset/re-bias
                // cycle.  Quarantine it so every later turn does not pause the
                // healthy peer for another 60 ms recovery transaction.
                bmi0_online <= 1'b0;
                bmi0_gyro_quarantined <= 1'b1;
                bmi0_good_sample_count <= 4'd0;
                main_state <= ST_RUN_WAIT;
              end else begin
                bmi0_gyro_recovery_attempted <= 1'b1;
                bmi0_gyro_quarantined <= 1'b0;
                gyro_recovery_target <= 1'b0;
                gyro_recovery_retry_count <= 2'd0;
                main_state <= ST_GYRO_RECOVERY_RESET;
              end
            end else if (bmi1_online && bmi1_gyro_stuck &&
                         !bmi1_rebias_pending) begin
              if (bmi1_gyro_recovery_attempted) begin
                bmi1_online <= 1'b0;
                bmi1_gyro_quarantined <= 1'b1;
                bmi1_good_sample_count <= 4'd0;
                main_state <= ST_RUN_WAIT;
              end else begin
                bmi1_gyro_recovery_attempted <= 1'b1;
                bmi1_gyro_quarantined <= 1'b0;
                gyro_recovery_target <= 1'b1;
                gyro_recovery_retry_count <= 2'd0;
                main_state <= ST_GYRO_RECOVERY_RESET;
              end
            end else if ((gyro_range_guard_count >=
                          GYRO_RANGE_GUARD_SAMPLES - 1'b1) &&
                         !bmi0_rebias_pending && !bmi1_rebias_pending &&
                         (bmi0_online || bmi1_online)) begin
              gyro_range_guard_count <= 5'd0;
              if (bmi0_online)
                main_state <= ST_GYRO_RANGE_GUARD0;
              else
                main_state <= ST_GYRO_RANGE_GUARD1;
            end else if (pmw3901_online && !pmw_poll_toggle) begin
              pmw_poll_toggle <= 1'b1;
              main_state <= ST_PMW_MOTION;
            end else begin
              pmw_poll_toggle <= 1'b0;
              main_state <= ST_RUN_WAIT;
            end
          end
          default: begin end
        endcase
      end
    end
  end




  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      calibration_count <= 10'd0;
      bmi0_bias_sum_x <= 32'sd0;
      bmi0_bias_sum_y <= 32'sd0;
      bmi0_bias_sum_z <= 32'sd0;
      bmi1_bias_sum_x <= 32'sd0;
      bmi1_bias_sum_y <= 32'sd0;
      bmi1_bias_sum_z <= 32'sd0;
      bmi0_gyro_bias_x <= 16'sd0;
      bmi0_gyro_bias_y <= 16'sd0;
      bmi0_gyro_bias_z <= 16'sd0;
      bmi1_gyro_bias_x <= 16'sd0;
      bmi1_gyro_bias_y <= 16'sd0;
      bmi1_gyro_bias_z <= 16'sd0;
      bmi0_gyro_missing_count <= 5'd0;
      bmi1_gyro_missing_count <= 5'd0;
      bmi0_gyro_stuck <= 1'b0;
      bmi1_gyro_stuck <= 1'b0;
      bmi0_rebias_pending <= 1'b0;
      bmi1_rebias_pending <= 1'b0;
      bmi0_rebias_count <= 7'd0;
      bmi1_rebias_count <= 7'd0;
      bmi0_rebias_sum_x <= 32'sd0;
      bmi0_rebias_sum_y <= 32'sd0;
      bmi0_rebias_sum_z <= 32'sd0;
      bmi1_rebias_sum_x <= 32'sd0;
      bmi1_rebias_sum_y <= 32'sd0;
      bmi1_rebias_sum_z <= 32'sd0;
      single_imu_rebias_candidate_count <= 4'd0;
      bmi0_auto_rebias <= 1'b0;
      bmi1_auto_rebias <= 1'b0;
      single_rebias_flow_start_x <= 32'd0;
      single_rebias_flow_start_y <= 32'd0;
      calibration_done <= 1'b0;
      fused_sample_valid <= 1'b0;
      fused_data_ready <= 1'b0;
      imu_degraded <= 1'b1;
      imu_disagreement <= 1'b0;
      imu_source <= 2'd3;
      fused_accel_x <= 16'sd0;
      fused_accel_y <= 16'sd0;
      fused_accel_z <= 16'sd0;
      fused_gyro_x <= 16'sd0;
      fused_gyro_y <= 16'sd0;
      fused_gyro_z <= 16'sd0;
    end else begin
      fused_sample_valid <= 1'b0;
      if (gyro_recovery_done_pulse) begin
        // Keep the recovered channel excluded until a fresh, channel-local
        // zero-offset estimate has completed.  Failure leaves that channel
        // offline; the healthy peer continues to feed the fused output.
        if (!gyro_recovery_target) begin
          bmi0_gyro_missing_count <= 5'd0;
          bmi0_rebias_count <= 7'd0;
          bmi0_rebias_sum_x <= 32'sd0;
          bmi0_rebias_sum_y <= 32'sd0;
          bmi0_rebias_sum_z <= 32'sd0;
          bmi0_rebias_pending <= gyro_recovery_success;
          bmi0_gyro_stuck <= gyro_recovery_success;
          bmi0_auto_rebias <= 1'b0;
        end else begin
          bmi1_gyro_missing_count <= 5'd0;
          bmi1_rebias_count <= 7'd0;
          bmi1_rebias_sum_x <= 32'sd0;
          bmi1_rebias_sum_y <= 32'sd0;
          bmi1_rebias_sum_z <= 32'sd0;
          bmi1_rebias_pending <= gyro_recovery_success;
          bmi1_gyro_stuck <= gyro_recovery_success;
          bmi1_auto_rebias <= 1'b0;
        end
      end else if (pair_sample_strobe) begin
        // If only one IMU survives, there is no peer vote capable of detecting
        // a bad but changing zero offset.  Eight consecutive optically-still
        // samples arm the existing channel-local 64-sample re-bias sequence.
        // A real hand/chassis turn produces optical motion and clears the
        // candidate before the bias can be changed.
        if (bmi0_rebias_pending || bmi1_rebias_pending ||
            !single_imu_rebias_candidate) begin
          single_imu_rebias_candidate_count <= 4'd0;
        end else if (single_imu_rebias_candidate_count >=
                     GYRO_SINGLE_REBIAS_ASSERT_SAMPLES - 1'b1) begin
          single_imu_rebias_candidate_count <= 4'd0;
          // Per-sample deltas alone can hide a slow real turn.  Arm only if
          // the total flow displacement across the confirmation window is also
          // small, then retain the same origin through the averaging window.
          if (auto_rebias_flow_still) begin
            if (bmi0_online) begin
              bmi0_rebias_pending <= 1'b1;
              bmi0_auto_rebias <= 1'b1;
              bmi0_rebias_count <= 7'd0;
              bmi0_rebias_sum_x <= 32'sd0;
              bmi0_rebias_sum_y <= 32'sd0;
              bmi0_rebias_sum_z <= 32'sd0;
            end else begin
              bmi1_rebias_pending <= 1'b1;
              bmi1_auto_rebias <= 1'b1;
              bmi1_rebias_count <= 7'd0;
              bmi1_rebias_sum_x <= 32'sd0;
              bmi1_rebias_sum_y <= 32'sd0;
              bmi1_rebias_sum_z <= 32'sd0;
            end
          end
        end else begin
          if (single_imu_rebias_candidate_count == 4'd0) begin
            single_rebias_flow_start_x <= flow_accum_x;
            single_rebias_flow_start_y <= flow_accum_y;
          end
          single_imu_rebias_candidate_count <=
              single_imu_rebias_candidate_count + 4'd1;
        end

        if (bmi0_rebias_pending) begin
          bmi0_gyro_missing_count <= 5'd0;
          if (!bmi0_auto_rebias)
            bmi0_gyro_stuck <= 1'b1;
          if (bmi0_auto_rebias &&
              (vehicle_motion_expected || !auto_rebias_flow_still)) begin
            // Movement or loss of trustworthy optical evidence cancels only an
            // automatically armed re-bias.  The old bias remains active and a
            // later genuinely still window may try again.
            bmi0_rebias_pending <= 1'b0;
            bmi0_auto_rebias <= 1'b0;
            bmi0_gyro_stuck <= 1'b0;
            bmi0_rebias_count <= 7'd0;
            bmi0_rebias_sum_x <= 32'sd0;
            bmi0_rebias_sum_y <= 32'sd0;
            bmi0_rebias_sum_z <= 32'sd0;
          end else if ((bmi0_raw_gyro_peak <= GYRO_REBIAS_RAW_LIMIT) &&
              ((bmi1_online && (bmi1_gyro_peak <= GYRO_REBIAS_PEER_LIMIT)) ||
               (!bmi1_online && optical_stillness_valid))) begin
            if (bmi0_rebias_count >= GYRO_REBIAS_SAMPLES - 1'b1) begin
              bmi0_gyro_bias_x <= $signed(
                  bmi0_rebias_sum_x +
                  {{16{bmi0_gyro_x_raw[15]}}, bmi0_gyro_x_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi0_gyro_bias_y <= $signed(
                  bmi0_rebias_sum_y +
                  {{16{bmi0_gyro_y_raw[15]}}, bmi0_gyro_y_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi0_gyro_bias_z <= $signed(
                  bmi0_rebias_sum_z +
                  {{16{bmi0_gyro_z_raw[15]}}, bmi0_gyro_z_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi0_rebias_pending <= 1'b0;
              bmi0_auto_rebias <= 1'b0;
              bmi0_gyro_stuck <= 1'b0;
              bmi0_rebias_count <= 7'd0;
              bmi0_rebias_sum_x <= 32'sd0;
              bmi0_rebias_sum_y <= 32'sd0;
              bmi0_rebias_sum_z <= 32'sd0;
            end else begin
              bmi0_rebias_count <= bmi0_rebias_count + 7'd1;
              bmi0_rebias_sum_x <= bmi0_rebias_sum_x +
                  {{16{bmi0_gyro_x_raw[15]}}, bmi0_gyro_x_raw};
              bmi0_rebias_sum_y <= bmi0_rebias_sum_y +
                  {{16{bmi0_gyro_y_raw[15]}}, bmi0_gyro_y_raw};
              bmi0_rebias_sum_z <= bmi0_rebias_sum_z +
                  {{16{bmi0_gyro_z_raw[15]}}, bmi0_gyro_z_raw};
            end
          end else begin
            bmi0_rebias_count <= 7'd0;
            bmi0_rebias_sum_x <= 32'sd0;
            bmi0_rebias_sum_y <= 32'sd0;
            bmi0_rebias_sum_z <= 32'sd0;
          end
        end else if (!calibration_done || !bmi0_online || !bmi1_online) begin
          bmi0_gyro_missing_count <= 5'd0;
          bmi0_gyro_stuck <= 1'b0;
        end else if (bmi0_gyro_missing_now) begin
          if (bmi0_gyro_missing_count >= GYRO_MISSING_ASSERT_SAMPLES) begin
            bmi0_gyro_missing_count <= GYRO_MISSING_ASSERT_SAMPLES + 5'd1;
            bmi0_gyro_stuck <= 1'b1;
          end else begin
            bmi0_gyro_missing_count <= bmi0_gyro_missing_count + 5'd1;
          end
        end else if (bmi0_gyro_peak > GYRO_MISSING_SELF_LIMIT) begin
          bmi0_gyro_missing_count <= 5'd0;
          bmi0_gyro_stuck <= 1'b0;
        end

        if (bmi1_rebias_pending) begin
          bmi1_gyro_missing_count <= 5'd0;
          if (!bmi1_auto_rebias)
            bmi1_gyro_stuck <= 1'b1;
          if (bmi1_auto_rebias &&
              (vehicle_motion_expected || !auto_rebias_flow_still)) begin
            bmi1_rebias_pending <= 1'b0;
            bmi1_auto_rebias <= 1'b0;
            bmi1_gyro_stuck <= 1'b0;
            bmi1_rebias_count <= 7'd0;
            bmi1_rebias_sum_x <= 32'sd0;
            bmi1_rebias_sum_y <= 32'sd0;
            bmi1_rebias_sum_z <= 32'sd0;
          end else if ((bmi1_raw_gyro_peak <= GYRO_REBIAS_RAW_LIMIT) &&
              ((bmi0_online && (bmi0_gyro_peak <= GYRO_REBIAS_PEER_LIMIT)) ||
               (!bmi0_online && optical_stillness_valid))) begin
            if (bmi1_rebias_count >= GYRO_REBIAS_SAMPLES - 1'b1) begin
              bmi1_gyro_bias_x <= $signed(
                  bmi1_rebias_sum_x +
                  {{16{bmi1_gyro_x_raw[15]}}, bmi1_gyro_x_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi1_gyro_bias_y <= $signed(
                  bmi1_rebias_sum_y +
                  {{16{bmi1_gyro_y_raw[15]}}, bmi1_gyro_y_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi1_gyro_bias_z <= $signed(
                  bmi1_rebias_sum_z +
                  {{16{bmi1_gyro_z_raw[15]}}, bmi1_gyro_z_raw}) >>>
                  GYRO_REBIAS_SHIFT;
              bmi1_rebias_pending <= 1'b0;
              bmi1_auto_rebias <= 1'b0;
              bmi1_gyro_stuck <= 1'b0;
              bmi1_rebias_count <= 7'd0;
              bmi1_rebias_sum_x <= 32'sd0;
              bmi1_rebias_sum_y <= 32'sd0;
              bmi1_rebias_sum_z <= 32'sd0;
            end else begin
              bmi1_rebias_count <= bmi1_rebias_count + 7'd1;
              bmi1_rebias_sum_x <= bmi1_rebias_sum_x +
                  {{16{bmi1_gyro_x_raw[15]}}, bmi1_gyro_x_raw};
              bmi1_rebias_sum_y <= bmi1_rebias_sum_y +
                  {{16{bmi1_gyro_y_raw[15]}}, bmi1_gyro_y_raw};
              bmi1_rebias_sum_z <= bmi1_rebias_sum_z +
                  {{16{bmi1_gyro_z_raw[15]}}, bmi1_gyro_z_raw};
            end
          end else begin
            bmi1_rebias_count <= 7'd0;
            bmi1_rebias_sum_x <= 32'sd0;
            bmi1_rebias_sum_y <= 32'sd0;
            bmi1_rebias_sum_z <= 32'sd0;
          end
        end else if (!calibration_done || !bmi0_online || !bmi1_online) begin
          bmi1_gyro_missing_count <= 5'd0;
          bmi1_gyro_stuck <= 1'b0;
        end else if (bmi1_gyro_missing_now) begin
          if (bmi1_gyro_missing_count >= GYRO_MISSING_ASSERT_SAMPLES) begin
            bmi1_gyro_missing_count <= GYRO_MISSING_ASSERT_SAMPLES + 5'd1;
            bmi1_gyro_stuck <= 1'b1;
          end else begin
            bmi1_gyro_missing_count <= bmi1_gyro_missing_count + 5'd1;
          end
        end else if (bmi1_gyro_peak > GYRO_MISSING_SELF_LIMIT) begin
          bmi1_gyro_missing_count <= 5'd0;
          bmi1_gyro_stuck <= 1'b0;
        end

        if (!calibration_done) begin
          fused_data_ready <= 1'b0;
          imu_degraded <= 1'b1;
          imu_source <= 2'd3;
          if ((bmi0_online || bmi1_online) && calibration_stationary) begin
            if (bmi0_online) begin
              bmi0_bias_sum_x <= bmi0_bias_sum_x + $signed(bmi0_gyro_x_raw);
              bmi0_bias_sum_y <= bmi0_bias_sum_y + $signed(bmi0_gyro_y_raw);
              bmi0_bias_sum_z <= bmi0_bias_sum_z + $signed(bmi0_gyro_z_raw);
            end
            if (bmi1_online) begin
              bmi1_bias_sum_x <= bmi1_bias_sum_x + $signed(bmi1_gyro_x_raw);
              bmi1_bias_sum_y <= bmi1_bias_sum_y + $signed(bmi1_gyro_y_raw);
              bmi1_bias_sum_z <= bmi1_bias_sum_z + $signed(bmi1_gyro_z_raw);
            end
            if (calibration_count >= CALIBRATION_SAMPLES - 1) begin
              calibration_done <= 1'b1;
              if (bmi0_online) begin
                bmi0_gyro_bias_x <= $signed(bmi0_bias_sum_x + $signed(bmi0_gyro_x_raw)) >>> CALIBRATION_SHIFT;
                bmi0_gyro_bias_y <= $signed(bmi0_bias_sum_y + $signed(bmi0_gyro_y_raw)) >>> CALIBRATION_SHIFT;
                bmi0_gyro_bias_z <= $signed(bmi0_bias_sum_z + $signed(bmi0_gyro_z_raw)) >>> CALIBRATION_SHIFT;
              end
              if (bmi1_online) begin
                bmi1_gyro_bias_x <= $signed(bmi1_bias_sum_x + $signed(bmi1_gyro_x_raw)) >>> CALIBRATION_SHIFT;
                bmi1_gyro_bias_y <= $signed(bmi1_bias_sum_y + $signed(bmi1_gyro_y_raw)) >>> CALIBRATION_SHIFT;
                bmi1_gyro_bias_z <= $signed(bmi1_bias_sum_z + $signed(bmi1_gyro_z_raw)) >>> CALIBRATION_SHIFT;
              end
            end else calibration_count <= calibration_count + 10'd1;
          end
        end else if (bmi0_online && bmi1_online) begin
          if (bmi0_gyro_unusable && !bmi1_gyro_unusable) begin
            imu_degraded <= 1'b1;
            imu_disagreement <= 1'b1;
            imu_source <= 2'd2;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi1_accel_x_raw;
            fused_accel_y <= bmi1_accel_y_raw;
            fused_accel_z <= bmi1_accel_z_raw;
            fused_gyro_x <= saturate16(bmi1_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi1_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi1_gyro_z_corrected);
          end else if (bmi1_gyro_unusable && !bmi0_gyro_unusable) begin
            imu_degraded <= 1'b1;
            imu_disagreement <= 1'b1;
            imu_source <= 2'd1;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi0_accel_x_raw;
            fused_accel_y <= bmi0_accel_y_raw;
            fused_accel_z <= bmi0_accel_z_raw;
            fused_gyro_x <= saturate16(bmi0_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi0_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi0_gyro_z_corrected);
          end else if (bmi0_gyro_unusable && bmi1_gyro_unusable) begin
            imu_degraded <= 1'b1;
            imu_disagreement <= 1'b1;
            imu_source <= 2'd3;
            fused_data_ready <= 1'b0;
          end else if (gyro_pair_agrees) begin
            // Chassis vibration can make the two accelerometers disagree
            // during an otherwise valid yaw observation. Keep the averaged
            // gyroscope sample alive; accel disagreement is still reported so
            // higher-level pose logic can reduce its confidence.
            imu_degraded <= 1'b0;
            imu_source <= 2'd0;
            imu_disagreement <= !accel_pair_agrees;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            if (accel_pair_agrees) begin
              fused_accel_x <= ($signed(bmi0_accel_x_raw) + $signed(bmi1_accel_x_raw)) >>> 1;
              fused_accel_y <= ($signed(bmi0_accel_y_raw) + $signed(bmi1_accel_y_raw)) >>> 1;
              fused_accel_z <= ($signed(bmi0_accel_z_raw) + $signed(bmi1_accel_z_raw)) >>> 1;
            end else begin
              fused_accel_x <= bmi0_accel_x_raw;
              fused_accel_y <= bmi0_accel_y_raw;
              fused_accel_z <= bmi0_accel_z_raw;
            end
            fused_gyro_x <= saturate16(($signed(bmi0_gyro_x_corrected) + $signed(bmi1_gyro_x_corrected)) >>> 1);
            fused_gyro_y <= saturate16(($signed(bmi0_gyro_y_corrected) + $signed(bmi1_gyro_y_corrected)) >>> 1);
            fused_gyro_z <= saturate16(($signed(bmi0_gyro_z_corrected) + $signed(bmi1_gyro_z_corrected)) >>> 1);
          end else if (bmi0_gyro_peak >= bmi1_gyro_peak) begin
            // With only two healthy sensors there is no majority vote. Preserve
            // the stronger motion observation instead of selecting a fixed
            // primary that may be the channel which stopped updating.
            imu_degraded <= 1'b1;
            imu_disagreement <= 1'b1;
            imu_source <= 2'd1;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi0_accel_x_raw;
            fused_accel_y <= bmi0_accel_y_raw;
            fused_accel_z <= bmi0_accel_z_raw;
            fused_gyro_x <= saturate16(bmi0_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi0_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi0_gyro_z_corrected);
          end else begin
            imu_degraded <= 1'b1;
            imu_disagreement <= 1'b1;
            imu_source <= 2'd2;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi1_accel_x_raw;
            fused_accel_y <= bmi1_accel_y_raw;
            fused_accel_z <= bmi1_accel_z_raw;
            fused_gyro_x <= saturate16(bmi1_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi1_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi1_gyro_z_corrected);
          end
        end else if (bmi0_online) begin
          imu_degraded <= 1'b1;
          if (bmi0_gyro_unusable) begin
            // Never feed a known-invalid zero reference into heading or PID.
            imu_disagreement <= 1'b1;
            imu_source <= 2'd3;
            fused_data_ready <= 1'b0;
          end else begin
            imu_disagreement <= 1'b0;
            imu_source <= 2'd1;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi0_accel_x_raw;
            fused_accel_y <= bmi0_accel_y_raw;
            fused_accel_z <= bmi0_accel_z_raw;
            fused_gyro_x <= saturate16(bmi0_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi0_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi0_gyro_z_corrected);
          end
        end else if (bmi1_online) begin
          imu_degraded <= 1'b1;
          if (bmi1_gyro_unusable) begin
            imu_disagreement <= 1'b1;
            imu_source <= 2'd3;
            fused_data_ready <= 1'b0;
          end else begin
            imu_disagreement <= 1'b0;
            imu_source <= 2'd2;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= bmi1_accel_x_raw;
            fused_accel_y <= bmi1_accel_y_raw;
            fused_accel_z <= bmi1_accel_z_raw;
            fused_gyro_x <= saturate16(bmi1_gyro_x_corrected);
            fused_gyro_y <= saturate16(bmi1_gyro_y_corrected);
            fused_gyro_z <= saturate16(bmi1_gyro_z_corrected);
          end
        end else begin
          imu_degraded <= 1'b1;
          imu_disagreement <= 1'b0;
          imu_source <= 2'd3;
          fused_data_ready <= 1'b0;
        end
      end
    end
  end


  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sensor_debug_uart_tx <= 1'b1;
      uart_busy <= 1'b0;
      uart_busy_d <= 1'b0;
      uart_state <= UART_IDLE;
      uart_clock_count <= 16'd0;
      uart_bit_index <= 3'd0;
      uart_shift <= 8'd0;
    end else begin
      uart_busy_d <= uart_busy;
      case (uart_state)
        UART_IDLE: begin
          sensor_debug_uart_tx <= 1'b1;
          uart_busy <= 1'b0;
          uart_clock_count <= 16'd0;
          uart_bit_index <= 3'd0;
          if (uart_start_pulse) begin
            uart_shift <= uart_data_byte;
            sensor_debug_uart_tx <= 1'b0;
            uart_busy <= 1'b1;
            uart_state <= UART_START;
          end
        end
        UART_START: begin
          sensor_debug_uart_tx <= 1'b0;
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            sensor_debug_uart_tx <= uart_shift[0];
            uart_state <= UART_DATA;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        UART_DATA: begin
          sensor_debug_uart_tx <= uart_shift[uart_bit_index];
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            if (uart_bit_index == 3'd7) begin
              uart_bit_index <= 3'd0;
              sensor_debug_uart_tx <= 1'b1;
              uart_state <= UART_STOP;
            end else begin
              uart_bit_index <= uart_bit_index + 3'd1;
              sensor_debug_uart_tx <= uart_shift[uart_bit_index + 3'd1];
            end
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        default: begin
          sensor_debug_uart_tx <= 1'b1;
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            uart_busy <= 1'b0;
            uart_state <= UART_IDLE;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
      endcase
    end
  end


  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      debug_period_count <= 32'd0;
      debug_sending <= 1'b0;
      debug_char_index <= 7'd0;
      debug_sequence <= 16'd0;
      uart_data_byte <= 8'd0;
      uart_start_pulse <= 1'b0;
      snap_sequence <= 16'd0;
      snap_status <= 8'd0;
      snap_squal <= 8'd0;
      snap_pmw_id <= 8'd0;
      snap_pmw_inverse_id <= 8'd0;
      snap_bmi0_accel_id <= 8'd0;
      snap_bmi0_gyro_id <= 8'd0;
      snap_bmi1_accel_id <= 8'd0;
      snap_bmi1_gyro_id <= 8'd0;
      snap_flow_dx <= 16'd0;
      snap_flow_dy <= 16'd0;
      snap_flow_accum_x <= 32'd0;
      snap_flow_accum_y <= 32'd0;
      snap_accel_x <= 16'd0;
      snap_accel_y <= 16'd0;
      snap_accel_z <= 16'd0;
      snap_gyro_x <= 16'd0;
      snap_gyro_y <= 16'd0;
      snap_gyro_z <= 16'd0;
      snap_bmi0_gyro_z <= 16'd0;
      snap_bmi1_gyro_z <= 16'd0;
`ifdef SENSOR_CAL_PROBE
      snap_bmi0_accel_x <= 16'd0;
      snap_bmi0_accel_y <= 16'd0;
      snap_bmi0_accel_z <= 16'd0;
      snap_bmi0_gyro_x_corrected <= 16'd0;
      snap_bmi0_gyro_y_corrected <= 16'd0;
      snap_bmi0_gyro_z_corrected <= 16'd0;
      snap_bmi1_accel_x <= 16'd0;
      snap_bmi1_accel_y <= 16'd0;
      snap_bmi1_accel_z <= 16'd0;
      snap_bmi1_gyro_x_corrected <= 16'd0;
      snap_bmi1_gyro_y_corrected <= 16'd0;
      snap_bmi1_gyro_z_corrected <= 16'd0;
      snap_static_cal_flags <= 8'd0;
      snap_static_cal_axes <= 8'd0;
      snap_static_cal_imu0_peak <= 16'd0;
      snap_static_cal_imu1_peak <= 16'd0;
      snap_static_cal_imu0_mean_x <= 16'd0;
      snap_static_cal_imu0_mean_y <= 16'd0;
      snap_static_cal_imu0_mean_z <= 16'd0;
      snap_static_cal_imu1_mean_x <= 16'd0;
      snap_static_cal_imu1_mean_y <= 16'd0;
      snap_static_cal_imu1_mean_z <= 16'd0;
`endif
    end else begin
      uart_start_pulse <= 1'b0;
      if (!debug_sending) begin
        if (debug_period_count >= DEBUG_PERIOD_CYCLES - 1) begin
          debug_period_count <= 32'd0;
          debug_sending <= 1'b1;
          debug_char_index <= 7'd0;
          uart_data_byte <= "@";
          uart_start_pulse <= 1'b1;
          snap_sequence <= debug_sequence;
          debug_sequence <= debug_sequence + 16'd1;
          snap_status <= {flow_data_seen, imu_disagreement, imu_degraded,
                          fused_data_ready, calibration_done, bmi1_online,
                          bmi0_online, pmw3901_online};
          snap_squal <= flow_squal;
          snap_pmw_id <= pmw3901_product_id;
          snap_pmw_inverse_id <= pmw3901_inverse_id;
          snap_bmi0_accel_id <= bmi0_accel_id;
          snap_bmi0_gyro_id <= bmi0_gyro_id;
          snap_bmi1_accel_id <= bmi1_accel_id;
          snap_bmi1_gyro_id <= bmi1_gyro_id;
          snap_flow_dx <= flow_delta_x;
          snap_flow_dy <= flow_delta_y;
          snap_flow_accum_x <= flow_accum_x;
          snap_flow_accum_y <= flow_accum_y;
          snap_accel_x <= fused_accel_x;
          snap_accel_y <= fused_accel_y;
          snap_accel_z <= fused_accel_z;
          snap_gyro_x <= fused_gyro_x;
          snap_gyro_y <= fused_gyro_y;
          snap_gyro_z <= fused_gyro_z;
          snap_bmi0_gyro_z <= bmi0_gyro_z_raw;
          snap_bmi1_gyro_z <= bmi1_gyro_z_raw;
`ifdef SENSOR_CAL_PROBE
          snap_bmi0_accel_x <= bmi0_accel_x_raw;
          snap_bmi0_accel_y <= bmi0_accel_y_raw;
          snap_bmi0_accel_z <= bmi0_accel_z_raw;
          snap_bmi0_gyro_x_corrected <= bmi0_gyro_x_corrected[15:0];
          snap_bmi0_gyro_y_corrected <= bmi0_gyro_y_corrected[15:0];
          snap_bmi0_gyro_z_corrected <= bmi0_gyro_z_corrected[15:0];
          snap_bmi1_accel_x <= bmi1_accel_x_raw;
          snap_bmi1_accel_y <= bmi1_accel_y_raw;
          snap_bmi1_accel_z <= bmi1_accel_z_raw;
          snap_bmi1_gyro_x_corrected <= bmi1_gyro_x_corrected[15:0];
          snap_bmi1_gyro_y_corrected <= bmi1_gyro_y_corrected[15:0];
          snap_bmi1_gyro_z_corrected <= bmi1_gyro_z_corrected[15:0];
          snap_static_cal_flags <= static_cal_flags;
          snap_static_cal_axes <= static_cal_axes;
          snap_static_cal_imu0_peak <= static_cal_imu0_peak;
          snap_static_cal_imu1_peak <= static_cal_imu1_peak;
          snap_static_cal_imu0_mean_x <= static_cal_imu0_mean_x;
          snap_static_cal_imu0_mean_y <= static_cal_imu0_mean_y;
          snap_static_cal_imu0_mean_z <= static_cal_imu0_mean_z;
          snap_static_cal_imu1_mean_x <= static_cal_imu1_mean_x;
          snap_static_cal_imu1_mean_y <= static_cal_imu1_mean_y;
          snap_static_cal_imu1_mean_z <= static_cal_imu1_mean_z;
`endif
        end else debug_period_count <= debug_period_count + 32'd1;
      end else if (uart_done) begin
`ifdef SENSOR_CAL_PROBE
        if (debug_char_index >= 7'd115) begin
`else
        if (debug_char_index >= 7'd98) begin
`endif
          debug_sending <= 1'b0;
          debug_char_index <= 7'd0;
        end else begin
          debug_char_index <= debug_char_index + 7'd1;
          uart_data_byte <= debug_frame_char(debug_char_index + 7'd1);
          uart_start_pulse <= 1'b1;
        end
      end
    end
  end

  initial begin
    sensor_spi_sclk = 1'b1;
    sensor_spi_mosi = 1'b0;
    spi_cs_n_raw = 5'b11111;
    pmw_force_cs_low = 1'b0;
    sensor_debug_uart_tx = 1'b1;
  end

endmodule
