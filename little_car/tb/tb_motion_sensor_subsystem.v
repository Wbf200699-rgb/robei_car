`timescale 1ns/1ps

module tb_motion_sensor_subsystem;
  reg clk;
  reg rst_n;
  reg sensor_spi_miso;
  wire sensor_spi_sclk;
  wire sensor_spi_mosi;
  wire pmw3901_cs_n;
  wire bmi0_acc_cs_n;
  wire bmi0_gyro_cs_n;
  wire bmi1_acc_cs_n;
  wire bmi1_gyro_cs_n;
  wire sensor_debug_uart_tx;
  wire pmw3901_online;
  wire bmi0_online;
  wire bmi1_online;
  wire calibration_done;
  wire fused_sample_valid;
  wire imu_degraded;
  wire imu_disagreement;
  wire [1:0] imu_source;
  wire [7:0] pmw3901_product_id;
  wire [7:0] pmw3901_inverse_id;
  wire [7:0] bmi0_accel_id;
  wire [7:0] bmi0_gyro_id;
  wire [7:0] bmi1_accel_id;
  wire [7:0] bmi1_gyro_id;
  wire flow_sample_valid;
  wire [7:0] flow_squal;
  wire signed [15:0] flow_delta_x;
  wire signed [15:0] flow_delta_y;
  wire signed [31:0] flow_accum_x;
  wire signed [31:0] flow_accum_y;
  wire signed [15:0] fused_accel_x;
  wire signed [15:0] fused_accel_y;
  wire signed [15:0] fused_accel_z;
  wire signed [15:0] fused_gyro_x;
  wire signed [15:0] fused_gyro_y;
  wire signed [15:0] fused_gyro_z;
  wire [15:0] sample_sequence;

  reg [7:0] captured_byte;
  reg [7:0] transaction_address;
  integer spi_bit_count;
  integer spi_byte_index;
  integer pmw_write_count;
  integer bmi0_range_write_count;
  integer bmi1_range_write_count;
  integer bmi1_accel_chip_id_read_count;
  integer bmi0_gyro_reset_count;
  integer bmi1_gyro_reset_count;
  integer bmi0_gyro_reset_count_before_fault;
  integer bmi1_gyro_reset_count_before_range_loss;
  integer bmi1_range_write_count_before_loss;
  integer selected_device;
  reg motion_phase;
  reg bmi0_gyro_stuck_model;
  reg bmi1_runtime_dropout;
  reg bmi1_accel_disagreement;
  reg bmi1_gyro_disagreement;
  reg [7:0] outgoing_byte;
  reg [7:0] bmi0_gyro_range_register;
  reg [7:0] bmi1_gyro_range_register;

  motion_sensor_subsystem dut(
    .clk(clk),
    .rst_n(rst_n),
    .vehicle_motion_expected(1'b0),
    .sensor_spi_miso(sensor_spi_miso),
    .sensor_spi_sclk(sensor_spi_sclk),
    .sensor_spi_mosi(sensor_spi_mosi),
    .pmw3901_cs_n(pmw3901_cs_n),
    .bmi0_acc_cs_n(bmi0_acc_cs_n),
    .bmi0_gyro_cs_n(bmi0_gyro_cs_n),
    .bmi1_acc_cs_n(bmi1_acc_cs_n),
    .bmi1_gyro_cs_n(bmi1_gyro_cs_n),
    .sensor_debug_uart_tx(sensor_debug_uart_tx),
    .pmw3901_online(pmw3901_online),
    .bmi0_online(bmi0_online),
    .bmi1_online(bmi1_online),
    .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement),
    .imu_source(imu_source),
    .pmw3901_product_id(pmw3901_product_id),
    .pmw3901_inverse_id(pmw3901_inverse_id),
    .bmi0_accel_id(bmi0_accel_id),
    .bmi0_gyro_id(bmi0_gyro_id),
    .bmi1_accel_id(bmi1_accel_id),
    .bmi1_gyro_id(bmi1_gyro_id),
    .flow_sample_valid(flow_sample_valid),
    .flow_squal(flow_squal),
    .flow_delta_x(flow_delta_x),
    .flow_delta_y(flow_delta_y),
    .flow_accum_x(flow_accum_x),
    .flow_accum_y(flow_accum_y),
    .fused_accel_x(fused_accel_x),
    .fused_accel_y(fused_accel_y),
    .fused_accel_z(fused_accel_z),
    .fused_gyro_x(fused_gyro_x),
    .fused_gyro_y(fused_gyro_y),
    .fused_gyro_z(fused_gyro_z),
    .sample_sequence(sample_sequence)
  );

  function [7:0] response_byte;
    input integer device;
    input [7:0] address;
    input integer byte_index;
    reg [7:0] reg_address;
    reg signed [15:0] gx;
    reg signed [15:0] gy;
    reg signed [15:0] gz;
    begin
      response_byte = 8'h00;
      reg_address = address & 8'h7f;
      if (device == 0) begin
        if (byte_index == 1) begin
          case (reg_address)
            8'h00: response_byte = 8'h49;
            8'h5f: response_byte = 8'hb6;
            8'h02: response_byte = 8'h80;
            8'h03: response_byte = motion_phase ? 8'h20 : 8'h10;
            8'h04: response_byte = 8'h00;
            8'h05: response_byte = 8'hf8;
            8'h06: response_byte = 8'hff;
            8'h07: response_byte = 8'h45;
            default: response_byte = 8'h00;
          endcase
        end
      end else if ((device == 1) || (device == 3)) begin
        // BMI088 accelerometer: byte 1 is the mandatory unpredictable dummy.
        if (byte_index >= 2) begin
          if (reg_address == 8'h00) begin
            // Reproduce the field symptom on BMI1: the first checked ID read
            // after soft reset returns 0x1c. A robust initializer must wait,
            // issue another dummy read, and then obtain the valid 0x1e ID.
            response_byte = (byte_index == 2) ?
                ((device == 3 && bmi1_accel_chip_id_read_count == 3) ?
                 8'h1c : 8'h1e) : 8'h00;
          end else if (reg_address == 8'h12) begin
            if ((device == 3) && bmi1_runtime_dropout) begin
              response_byte = 8'h00;
            end else if (device == 1) begin
              case (byte_index)
                2: response_byte = 8'h64;  // +100 X LSB
                3: response_byte = 8'h00;
                4: response_byte = 8'h38;  // -200 Y LSB
                5: response_byte = 8'hff;
                6: response_byte = 8'h7c;  // +5500 Z LSB
                7: response_byte = 8'h15;
                default: response_byte = 8'h00;
              endcase
            end else begin
              case (byte_index)
                2: response_byte = bmi1_accel_disagreement ? 8'hb8 : 8'h78;
                3: response_byte = bmi1_accel_disagreement ? 8'h0b : 8'h00;
                4: response_byte = 8'h4c;  // -180 Y LSB
                5: response_byte = 8'hff;
                6: response_byte = 8'h90;  // +5520 Z LSB
                7: response_byte = 8'h15;
                default: response_byte = 8'h00;
              endcase
            end
          end
        end
      end else begin
        if (device == 2) begin
          // Reproduce the observed board fault: BMI0 acceleration remains
          // live, but its independent gyro die keeps returning the last quiet
          // sample while BMI1 sees the chassis rotate.  A soft reset clears
          // the modelled fault so the DUT can prove its recovery sequence.
          if (bmi0_gyro_stuck_model) begin
            gx = 16'sd100;
            gy = -16'sd50;
            gz = 16'sd200;
          end else begin
            gx = motion_phase ? 16'sd400 : 16'sd100;
            gy = motion_phase ? 16'sd50 : -16'sd50;
            gz = motion_phase ? 16'sd500 : 16'sd200;
          end
        end else begin
          if (bmi1_runtime_dropout) begin
            gx = 16'sd0;
            gy = 16'sd0;
            gz = 16'sd0;
          end else begin
            gx = motion_phase ? 16'sd420 : 16'sd120;
            gy = motion_phase ? 16'sd70 : -16'sd30;
            gz = bmi1_gyro_disagreement ? -16'sd4000 :
                 (motion_phase ? 16'sd520 : 16'sd220);
          end
        end
        if ((reg_address == 8'h00) && (byte_index == 1)) begin
          response_byte = 8'h0f;
        end else if ((reg_address == 8'h0f) && (byte_index == 1)) begin
          response_byte = (device == 2) ? bmi0_gyro_range_register
                                        : bmi1_gyro_range_register;
        end else if ((reg_address == 8'h02) && (byte_index >= 1)) begin
          case (byte_index)
            1: response_byte = gx[7:0];
            2: response_byte = gx[15:8];
            3: response_byte = gy[7:0];
            4: response_byte = gy[15:8];
            5: response_byte = gz[7:0];
            6: response_byte = gz[15:8];
            default: response_byte = 8'h00;
          endcase
        end
      end
    end
  endfunction

  task begin_transaction;
    input integer device;
    begin
      selected_device = device;
      captured_byte = 8'h00;
      transaction_address = 8'h00;
      spi_bit_count = 0;
      spi_byte_index = 0;
      sensor_spi_miso = 1'b0;
    end
  endtask

  always #10 clk = ~clk;

  always @(negedge pmw3901_cs_n) begin_transaction(0);
  always @(negedge bmi0_acc_cs_n) begin_transaction(1);
  always @(negedge bmi0_gyro_cs_n) begin_transaction(2);
  always @(negedge bmi1_acc_cs_n) begin_transaction(3);
  always @(negedge bmi1_gyro_cs_n) begin_transaction(4);

  always @(posedge sensor_spi_sclk) begin
    if (!pmw3901_cs_n || !bmi0_acc_cs_n || !bmi0_gyro_cs_n ||
        !bmi1_acc_cs_n || !bmi1_gyro_cs_n) begin
      captured_byte = {captured_byte[6:0], sensor_spi_mosi};
      if (spi_bit_count == 7) begin
        if (spi_byte_index == 0)
          transaction_address = captured_byte;
        if ((spi_byte_index == 0) && (selected_device == 3) &&
            ((captured_byte & 8'h7f) == 8'h00) && captured_byte[7])
          bmi1_accel_chip_id_read_count =
              bmi1_accel_chip_id_read_count + 1;
        else if ((selected_device == 0) && transaction_address[7])
          pmw_write_count = pmw_write_count + 1;
        else if ((spi_byte_index == 1) && !transaction_address[7] &&
                 ((selected_device == 2) || (selected_device == 4)) &&
                 ((transaction_address & 8'h7f) == 8'h14) &&
                 (captured_byte == 8'hb6)) begin
          if (selected_device == 2) begin
            bmi0_gyro_reset_count = bmi0_gyro_reset_count + 1;
            bmi0_gyro_range_register = 8'h00;
            bmi0_gyro_stuck_model = 1'b0;
          end else begin
            bmi1_gyro_reset_count = bmi1_gyro_reset_count + 1;
            bmi1_gyro_range_register = 8'h00;
          end
        end else if ((spi_byte_index == 1) && !transaction_address[7] &&
                 ((transaction_address & 8'h7f) == 8'h0f)) begin
          if (selected_device == 2) begin
            bmi0_gyro_range_register = captured_byte;
            bmi0_range_write_count = bmi0_range_write_count + 1;
          end else if (selected_device == 4) begin
            // Model the field failure: the first BMI1 range write is lost.
            // The DUT must detect 0x00 on readback and issue another write.
            if (bmi1_range_write_count != 0)
              bmi1_gyro_range_register = captured_byte;
            bmi1_range_write_count = bmi1_range_write_count + 1;
          end
        end
        spi_bit_count = 0;
        spi_byte_index = spi_byte_index + 1;
        captured_byte = 8'h00;
      end else begin
        spi_bit_count = spi_bit_count + 1;
      end
    end
  end

  always @(negedge sensor_spi_sclk) begin
    if (!pmw3901_cs_n || !bmi0_acc_cs_n || !bmi0_gyro_cs_n ||
        !bmi1_acc_cs_n || !bmi1_gyro_cs_n) begin
      outgoing_byte = response_byte(selected_device, transaction_address,
                                    spi_byte_index);
      sensor_spi_miso = outgoing_byte[7 - spi_bit_count];
    end else begin
      sensor_spi_miso = 1'b1;
    end
  end

  initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    sensor_spi_miso = 1'b1;
    captured_byte = 8'h00;
    transaction_address = 8'h00;
    spi_bit_count = 0;
    spi_byte_index = 0;
    pmw_write_count = 0;
    bmi0_range_write_count = 0;
    bmi1_range_write_count = 0;
    bmi1_accel_chip_id_read_count = 0;
    bmi0_gyro_reset_count = 0;
    bmi1_gyro_reset_count = 0;
    bmi0_gyro_reset_count_before_fault = 0;
    bmi1_gyro_reset_count_before_range_loss = 0;
    bmi1_range_write_count_before_loss = 0;
    selected_device = 0;
    motion_phase = 1'b0;
    bmi0_gyro_stuck_model = 1'b0;
    bmi1_runtime_dropout = 1'b0;
    bmi1_accel_disagreement = 1'b0;
    bmi1_gyro_disagreement = 1'b0;
    outgoing_byte = 8'h00;
    bmi0_gyro_range_register = 8'h00;
    bmi1_gyro_range_register = 8'h00;

    $dumpfile("motion_sensor_subsystem_test.vcd");
    $dumpvars(0, tb_motion_sensor_subsystem);

    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    fork
      begin
        repeat (1000000) @(posedge clk);
        $display("FAIL timeout state=%0d bmi_step=%0d seq=%0d online=%b%b%b cal=%b count=%0d stationary=%b raw0=%0d,%0d,%0d/%0d,%0d,%0d raw1=%0d,%0d,%0d/%0d,%0d,%0d",
                 dut.main_state, dut.bmi_init_step, sample_sequence,
                 pmw3901_online, bmi0_online, bmi1_online, calibration_done,
                 dut.calibration_count, dut.calibration_stationary,
                 $signed(dut.bmi0_accel_x_raw), $signed(dut.bmi0_accel_y_raw),
                 $signed(dut.bmi0_accel_z_raw), $signed(dut.bmi0_gyro_x_raw),
                 $signed(dut.bmi0_gyro_y_raw), $signed(dut.bmi0_gyro_z_raw),
                 $signed(dut.bmi1_accel_x_raw), $signed(dut.bmi1_accel_y_raw),
                 $signed(dut.bmi1_accel_z_raw), $signed(dut.bmi1_gyro_x_raw),
                 $signed(dut.bmi1_gyro_y_raw), $signed(dut.bmi1_gyro_z_raw));
        $finish;
      end
      begin
        wait (pmw3901_online && bmi0_online && bmi1_online);
        if (pmw3901_product_id !== 8'h49 || pmw3901_inverse_id !== 8'hb6 ||
            bmi0_accel_id !== 8'h1e || bmi0_gyro_id !== 8'h0f ||
            bmi1_accel_id !== 8'h1e || bmi1_gyro_id !== 8'h0f) begin
          $display("FAIL identification IDs=%h/%h %h/%h %h/%h",
                   pmw3901_product_id, pmw3901_inverse_id,
                   bmi0_accel_id, bmi0_gyro_id, bmi1_accel_id, bmi1_gyro_id);
          $finish;
        end
        if ((dut.bmi0_gyro_range !== 8'h03) ||
            (dut.bmi1_gyro_range !== 8'h03) ||
            (bmi0_range_write_count < 1) ||
            (bmi1_range_write_count < 2) ||
            (bmi1_accel_chip_id_read_count < 5)) begin
          $display("FAIL gyro range readback=%h/%h writes=%0d/%0d",
                   dut.bmi0_gyro_range, dut.bmi1_gyro_range,
                   bmi0_range_write_count, bmi1_range_write_count);
          $finish;
        end
        wait (calibration_done);
`ifdef SENSOR_CAL_PROBE
        wait (dut.static_cal_done);
        #1;
        if (!dut.static_cal_pass || (dut.static_cal_flags !== 8'hff)) begin
          $display("FAIL static calibrator flags=%h axes=%h",
                   dut.static_cal_flags, dut.static_cal_axes);
          $finish;
        end
        $display("RESULT static_calibration=pass flags=%h axes=%h",
                 dut.static_cal_flags, dut.static_cal_axes);
`endif
        // Model an independent sensor-rail reset after startup: BMI1 remains
        // electrically responsive but its GYRO_RANGE register falls back to
        // 0x00. At that range the same turn produces one eighth as many raw
        // counts. The runtime guard must restore 0x03 without resetting,
        // offlining or re-biasing the only healthy channel.
        bmi1_gyro_reset_count_before_range_loss = bmi1_gyro_reset_count;
        bmi1_range_write_count_before_loss = bmi1_range_write_count;
        bmi1_gyro_range_register = 8'h00;
        wait (bmi1_range_write_count > bmi1_range_write_count_before_loss);
        #1;
        if (!bmi1_online || (dut.bmi1_gyro_range !== 8'h03) ||
            (bmi1_gyro_range_register !== 8'h03) ||
            dut.bmi1_rebias_pending ||
            (bmi1_gyro_reset_count !=
             bmi1_gyro_reset_count_before_range_loss)) begin
          $display("FAIL BMI1 runtime range guard online=%b dut=%h model=%h reset_delta=%0d rebias=%b",
                   bmi1_online, dut.bmi1_gyro_range,
                   bmi1_gyro_range_register,
                   bmi1_gyro_reset_count -
                   bmi1_gyro_reset_count_before_range_loss,
                   dut.bmi1_rebias_pending);
          $finish;
        end
        $display("RESULT bmi1_runtime_range_guard=pass resets=0 range=%h writes=%0d",
                 dut.bmi1_gyro_range,
                 bmi1_range_write_count -
                 bmi1_range_write_count_before_loss);

        // Force the exact live-board failure: only BMI0 gyro remains at its
        // calibrated quiet value.  BMI1 sees motion, so fusion must first use
        // BMI1 alone, then issue a targeted BMI0 gyro reset.  The test returns
        // the chassis to rest while the recovered die collects 64 fresh bias
        // samples, after which normal dual-IMU averaging must resume.
        bmi0_gyro_reset_count_before_fault = bmi0_gyro_reset_count;
        bmi0_gyro_stuck_model = 1'b1;
        motion_phase = 1'b1;
        wait (dut.bmi0_gyro_stuck);
        #1;
        if (!imu_degraded || !imu_disagreement || (imu_source != 2'd2)) begin
          $display("FAIL BMI0 stuck fallback status=%b/%b source=%0d",
                   imu_degraded, imu_disagreement, imu_source);
          $finish;
        end
        motion_phase = 1'b0;
        wait (bmi0_gyro_reset_count > bmi0_gyro_reset_count_before_fault);
        wait (dut.bmi0_rebias_pending);
        wait (!dut.bmi0_rebias_pending && !dut.bmi0_gyro_stuck);
        if (($signed(dut.bmi0_gyro_bias_x) != 100) ||
            ($signed(dut.bmi0_gyro_bias_y) != -50) ||
            ($signed(dut.bmi0_gyro_bias_z) != 200) ||
            (dut.bmi0_gyro_range !== 8'h03)) begin
          $display("FAIL BMI0 recovery bias=%0d,%0d,%0d range=%h",
                   $signed(dut.bmi0_gyro_bias_x),
                   $signed(dut.bmi0_gyro_bias_y),
                   $signed(dut.bmi0_gyro_bias_z), dut.bmi0_gyro_range);
          $finish;
        end
        $display("RESULT bmi0_targeted_gyro_recovery=pass resets=%0d bias=%0d,%0d,%0d",
                 bmi0_gyro_reset_count - bmi0_gyro_reset_count_before_fault,
                 $signed(dut.bmi0_gyro_bias_x),
                 $signed(dut.bmi0_gyro_bias_y),
                 $signed(dut.bmi0_gyro_bias_z));

        motion_phase = 1'b1;
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if (imu_degraded || imu_disagreement || imu_source != 2'd0) begin
          $display("FAIL fusion status degraded=%b disagree=%b source=%0d",
                   imu_degraded, imu_disagreement, imu_source);
          $finish;
        end
        if (($signed(fused_accel_x) != 110) ||
            ($signed(fused_accel_y) != -190) ||
            ($signed(fused_accel_z) != 5510)) begin
          $display("FAIL fused accel=%0d,%0d,%0d", $signed(fused_accel_x),
                   $signed(fused_accel_y), $signed(fused_accel_z));
          $finish;
        end
        if (($signed(fused_gyro_x) != 300) ||
            ($signed(fused_gyro_y) != 100) ||
            ($signed(fused_gyro_z) != 300)) begin
          $display("FAIL fused gyro=%0d,%0d,%0d", $signed(fused_gyro_x),
                   $signed(fused_gyro_y), $signed(fused_gyro_z));
          $finish;
        end

        // Reproduce a persistent field failure after the one allowed targeted
        // recovery.  The second failure must not launch an endless sequence of
        // 60 ms reset attempts.  Instead BMI0 is quarantined, BMI1 supplies the
        // complete gyro sample (300 counts, never the erroneous 150-count
        // average), and eight dynamic BMI0 samples may later re-admit it.
        bmi0_gyro_stuck_model = 1'b1;
        wait (dut.bmi0_gyro_stuck);
        wait (!bmi0_online && dut.bmi0_gyro_quarantined);
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if ((bmi0_gyro_reset_count !=
             (bmi0_gyro_reset_count_before_fault + 1)) ||
            !imu_degraded || imu_disagreement || (imu_source != 2'd2) ||
            ($signed(fused_gyro_z) != 300)) begin
          $display("FAIL BMI0 quarantine resets=%0d online=%b quarantine=%b status=%b/%b source=%0d gyro_z=%0d",
                   bmi0_gyro_reset_count - bmi0_gyro_reset_count_before_fault,
                   bmi0_online, dut.bmi0_gyro_quarantined,
                   imu_degraded, imu_disagreement, imu_source,
                   $signed(fused_gyro_z));
          $finish;
        end
        repeat (12) @(posedge dut.pair_sample_strobe);
        #1;
        if (bmi0_online || !dut.bmi0_gyro_quarantined ||
            (bmi0_gyro_reset_count !=
             (bmi0_gyro_reset_count_before_fault + 1))) begin
          $display("FAIL BMI0 quarantine was not stable resets=%0d online=%b quarantine=%b",
                   bmi0_gyro_reset_count - bmi0_gyro_reset_count_before_fault,
                   bmi0_online, dut.bmi0_gyro_quarantined);
          $finish;
        end
        bmi0_gyro_stuck_model = 1'b0;
        wait (bmi0_online && !dut.bmi0_gyro_quarantined);
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if (dut.bmi0_gyro_recovery_attempted || imu_degraded ||
            imu_disagreement || (imu_source != 2'd0) ||
            ($signed(fused_gyro_z) != 300)) begin
          $display("FAIL BMI0 quarantine rejoin attempted=%b status=%b/%b source=%0d gyro_z=%0d",
                   dut.bmi0_gyro_recovery_attempted, imu_degraded,
                   imu_disagreement, imu_source, $signed(fused_gyro_z));
          $finish;
        end
        $display("RESULT bmi0_persistent_fault_quarantine=pass resets=1 fallback_gyro_z=%0d rejoin_samples=8",
                 $signed(fused_gyro_z));

        // Dynamic acceleration disagreement must not suppress a valid yaw
        // sample. The gyro pair still agrees, so retain averaged gyro data and
        // report only the lower-confidence acceleration condition.
        bmi1_accel_disagreement = 1'b1;
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if (imu_degraded || !imu_disagreement || (imu_source != 2'd0) ||
            ($signed(fused_gyro_z) != 300) ||
            ($signed(fused_accel_x) != 100)) begin
          $display("FAIL accel disagreement fallback status=%b/%b source=%0d accel_x=%0d gyro_z=%0d",
                   imu_degraded, imu_disagreement, imu_source,
                   $signed(fused_accel_x), $signed(fused_gyro_z));
          $finish;
        end
        $display("RESULT accel_disagreement_gyro_continuity=pass");
        bmi1_accel_disagreement = 1'b0;

        // If the gyroscopes themselves disagree, keep the stronger motion
        // observation and mark the result degraded instead of dropping the
        // stream.  This is the non-stuck fallback; a zero/stale channel is
        // handled earlier by the targeted recovery path exercised above.
        bmi1_gyro_disagreement = 1'b1;
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if (!imu_degraded || !imu_disagreement || (imu_source != 2'd2) ||
            ($signed(fused_gyro_x) != 300) ||
            ($signed(fused_gyro_y) != 100) ||
            ($signed(fused_gyro_z) != -4220)) begin
          $display("FAIL gyro disagreement fallback status=%b/%b source=%0d gyro=%0d,%0d,%0d",
                   imu_degraded, imu_disagreement, imu_source,
                   $signed(fused_gyro_x), $signed(fused_gyro_y),
                   $signed(fused_gyro_z));
          $finish;
        end
        $display("RESULT gyro_disagreement_stronger_fallback=pass source=2");
        bmi1_gyro_disagreement = 1'b0;
        @(negedge fused_sample_valid);
        @(posedge fused_sample_valid);
        #1;
        if (imu_degraded || imu_disagreement || (imu_source != 2'd0)) begin
          $display("FAIL fusion did not recover after disagreement");
          $finish;
        end
        wait (flow_sample_valid);
        #1;
        if (($signed(flow_delta_y) != -8) || (flow_squal != 8'h45) ||
            ($signed(flow_accum_x) <= 0)) begin
          $display("FAIL flow dx=%0d dy=%0d squal=%h accum=%0d,%0d",
                   $signed(flow_delta_x), $signed(flow_delta_y), flow_squal,
                   $signed(flow_accum_x), $signed(flow_accum_y));
          $finish;
        end
        if (pmw_write_count < 74) begin
          $display("FAIL PMW init write count=%0d", pmw_write_count);
          $finish;
        end
        bmi1_runtime_dropout = 1'b1;
        repeat (40) @(posedge dut.pair_sample_strobe);
        #1;
        if (bmi1_online !== 1'b0) begin
          $display("FAIL BMI1 runtime dropout was not detected bad_count=%0d",
                   dut.bmi1_bad_sample_count);
          $finish;
        end
        // Runtime recovery must be based on the live six-axis stream. A stale
        // startup ID cache must not permanently suppress an otherwise valid
        // sensor after a temporary bus dropout.
        force dut.bmi1_accel_id = 8'h1c;
        bmi1_runtime_dropout = 1'b0;
        repeat (16) @(posedge dut.pair_sample_strobe);
        #1;
        if (bmi1_online !== 1'b1) begin
          $display("FAIL BMI1 did not recover good_count=%0d",
                   dut.bmi1_good_sample_count);
          $finish;
        end
        release dut.bmi1_accel_id;
        $display("RESULT ids=%h/%h,%h/%h,%h/%h", pmw3901_product_id,
                 pmw3901_inverse_id, bmi0_accel_id, bmi0_gyro_id,
                 bmi1_accel_id, bmi1_gyro_id);
        $display("RESULT gyro_range=%h/%h writes=%0d/%0d",
                 dut.bmi0_gyro_range, dut.bmi1_gyro_range,
                 bmi0_range_write_count, bmi1_range_write_count);
        $display("RESULT bmi1_accel_id_reads=%0d recovered_from_1c=1",
                 bmi1_accel_chip_id_read_count);
        $display("RESULT bmi1_runtime_recovery=pass bad_threshold=32 good_threshold=8 stale_id_tolerated=1");
        $display("RESULT fused_accel=%0d,%0d,%0d fused_gyro=%0d,%0d,%0d",
                 $signed(fused_accel_x), $signed(fused_accel_y),
                 $signed(fused_accel_z), $signed(fused_gyro_x),
                 $signed(fused_gyro_y), $signed(fused_gyro_z));
        $display("RESULT flow=%0d,%0d accum=%0d,%0d squal=%h",
                 $signed(flow_delta_x), $signed(flow_delta_y),
                 $signed(flow_accum_x), $signed(flow_accum_y), flow_squal);
        $display("RESULT TEST_PASS motion_sensor_subsystem");
        $finish;
      end
    join
  end
endmodule
