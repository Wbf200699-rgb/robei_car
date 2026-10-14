`timescale 1ns/1ps
module car_param_controller(
  input wire clk,
  input wire rst_n,
  input wire key_up_n,
  input wire key_down_n,
  input wire key_left_n,
  input wire key_right_n,
  input wire key_mid_n,
  input wire bt_cfg_mode_pulse,
  input wire bt_cfg_next_pulse,
  input wire bt_cfg_prev_pulse,
  input wire bt_cfg_inc_pulse,
  input wire bt_cfg_dec_pulse,
  input wire [2:0] bt_cfg_select,
  input wire bt_cfg_select_valid,
  output reg route_key_lock,
  output reg setting_mode,
  output reg [2:0] selected_param,
  output reg [15:0] step_length_set,
  output wire [15:0] speed_set,
  output reg [9:0] overall_speed,
  output reg [9:0] lf_default_speed,
  output reg [9:0] rf_default_speed,
  output reg [9:0] lr_default_speed,
  output reg [9:0] rr_default_speed,
  output reg [15:0] lidar_safe_mm
);
        assign speed_set = {6'd0, overall_speed};

        localparam [2:0] PARAM_OVERALL = 3'd0;
        localparam [2:0] PARAM_STEP    = 3'd1;
        localparam [2:0] PARAM_LF      = 3'd2;
        localparam [2:0] PARAM_RF      = 3'd3;
        localparam [2:0] PARAM_LR      = 3'd4;
        localparam [2:0] PARAM_RR      = 3'd5;
        localparam [2:0] PARAM_LIDAR   = 3'd6;

        localparam [9:0] SPEED_MIN = 10'd100;
        localparam [9:0] SPEED_MAX = 10'd900;
        localparam [9:0] SPEED_STEP = 10'd25;
        localparam [15:0] STEP_MIN = 16'd5;
        localparam [15:0] STEP_MAX = 16'd80;
        localparam [15:0] STEP_STEP = 16'd5;
        localparam [15:0] LIDAR_SAFE_MIN = 16'd150;
        localparam [15:0] LIDAR_SAFE_MAX = 16'd1500;
        localparam [15:0] LIDAR_SAFE_STEP = 16'd50;

        function [2:0] next_param;
          input [2:0] value;
          begin
            next_param = (value >= PARAM_LIDAR) ? PARAM_OVERALL : (value + 3'd1);
          end
        endfunction

        function [2:0] prev_param;
          input [2:0] value;
          begin
            prev_param = (value == PARAM_OVERALL) ? PARAM_LIDAR : (value - 3'd1);
          end
        endfunction

        function [9:0] inc_speed;
          input [9:0] value;
          begin
            inc_speed = (value >= SPEED_MAX - SPEED_STEP) ? SPEED_MAX :
                        (value + SPEED_STEP);
          end
        endfunction

        function [9:0] dec_speed;
          input [9:0] value;
          begin
            dec_speed = (value <= SPEED_MIN + SPEED_STEP) ? SPEED_MIN :
                        (value - SPEED_STEP);
          end
        endfunction

        function [15:0] inc_step;
          input [15:0] value;
          begin
            inc_step = (value >= STEP_MAX - STEP_STEP) ? STEP_MAX :
                       (value + STEP_STEP);
          end
        endfunction

        function [15:0] dec_step;
          input [15:0] value;
          begin
            dec_step = (value <= STEP_MIN + STEP_STEP) ? STEP_MIN :
                       (value - STEP_STEP);
          end
        endfunction

        function [15:0] inc_lidar_safe;
          input [15:0] value;
          begin
            inc_lidar_safe = (value >= LIDAR_SAFE_MAX - LIDAR_SAFE_STEP) ? LIDAR_SAFE_MAX :
                             (value + LIDAR_SAFE_STEP);
          end
        endfunction

        function [15:0] dec_lidar_safe;
          input [15:0] value;
          begin
            dec_lidar_safe = (value <= LIDAR_SAFE_MIN + LIDAR_SAFE_STEP) ? LIDAR_SAFE_MIN :
                             (value - LIDAR_SAFE_STEP);
          end
        endfunction

        always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
            route_key_lock <= 1'b0;
            setting_mode <= 1'b0;
            selected_param <= PARAM_OVERALL;
            step_length_set <= 16'd20;
            overall_speed <= 10'd700;
            lf_default_speed <= 10'd700;
            rf_default_speed <= 10'd700;
            lr_default_speed <= 10'd700;
            rr_default_speed <= 10'd700;
            lidar_safe_mm <= 16'd450;
          end else begin
            route_key_lock <= 1'b0;

            if (bt_cfg_mode_pulse)
              setting_mode <= ~setting_mode;

            if (bt_cfg_select_valid) begin
              setting_mode <= 1'b1;
              if (bt_cfg_select <= PARAM_LIDAR)
                selected_param <= bt_cfg_select;
            end

            if (bt_cfg_next_pulse) begin
              setting_mode <= 1'b1;
              selected_param <= next_param(selected_param);
            end

            if (bt_cfg_prev_pulse) begin
              setting_mode <= 1'b1;
              selected_param <= prev_param(selected_param);
            end

            if (bt_cfg_inc_pulse) begin
              setting_mode <= 1'b1;
              case (selected_param)
                PARAM_STEP: step_length_set <= inc_step(step_length_set);
                PARAM_LF: lf_default_speed <= inc_speed(lf_default_speed);
                PARAM_RF: rf_default_speed <= inc_speed(rf_default_speed);
                PARAM_LR: lr_default_speed <= inc_speed(lr_default_speed);
                PARAM_RR: rr_default_speed <= inc_speed(rr_default_speed);
                PARAM_LIDAR: lidar_safe_mm <= inc_lidar_safe(lidar_safe_mm);
                default: overall_speed <= inc_speed(overall_speed);
              endcase
            end

            if (bt_cfg_dec_pulse) begin
              setting_mode <= 1'b1;
              case (selected_param)
                PARAM_STEP: step_length_set <= dec_step(step_length_set);
                PARAM_LF: lf_default_speed <= dec_speed(lf_default_speed);
                PARAM_RF: rf_default_speed <= dec_speed(rf_default_speed);
                PARAM_LR: lr_default_speed <= dec_speed(lr_default_speed);
                PARAM_RR: rr_default_speed <= dec_speed(rr_default_speed);
                PARAM_LIDAR: lidar_safe_mm <= dec_lidar_safe(lidar_safe_mm);
                default: overall_speed <= dec_speed(overall_speed);
              endcase
            end
          end
        end
endmodule

module car_param_controller_robei_test;
  reg clk;
  reg rst_n;
  reg key_up_n;
  reg key_down_n;
  reg key_left_n;
  reg key_right_n;
  reg key_mid_n;
  reg bt_cfg_mode_pulse;
  reg bt_cfg_next_pulse;
  reg bt_cfg_prev_pulse;
  reg bt_cfg_inc_pulse;
  reg bt_cfg_dec_pulse;
  reg [2:0] bt_cfg_select;
  reg bt_cfg_select_valid;
  wire route_key_lock;
  wire setting_mode;
  wire [2:0] selected_param;
  wire [15:0] step_length_set;
  wire [15:0] speed_set;
  wire [9:0] overall_speed;
  wire [9:0] lf_default_speed;
  wire [9:0] rf_default_speed;
  wire [9:0] lr_default_speed;
  wire [9:0] rr_default_speed;
  wire [15:0] lidar_safe_mm;


integer fd;
integer errors;
integer i;

initial clk=0;
always #10 clk=~clk;

task pulse_inc; begin bt_cfg_inc_pulse=1; #20; bt_cfg_inc_pulse=0; #20; end endtask
task pulse_dec; begin bt_cfg_dec_pulse=1; #20; bt_cfg_dec_pulse=0; #20; end endtask
task select_param; input [2:0] p; begin bt_cfg_select=p; bt_cfg_select_valid=1; #20; bt_cfg_select_valid=0; #20; end endtask
task log_values;
  input [8*20-1:0] label_text;
  begin
    $display("RESULT %0s overall=%0d step=%0d lf=%0d rf=%0d lr=%0d rr=%0d lidar=%0d",
      label_text,overall_speed,step_length_set,lf_default_speed,rf_default_speed,
      lr_default_speed,rr_default_speed,lidar_safe_mm);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",label_text,overall_speed,
      step_length_set,lf_default_speed,rf_default_speed,lr_default_speed,
      rr_default_speed,lidar_safe_mm);
  end
endtask

initial begin
  fd=$fopen("car_parameter_results.csv","w");
  $fwrite(fd,"case,overall,step,lf,rf,lr,rr,lidar_mm\n");
  errors=0; rst_n=0;
  key_up_n=1; key_down_n=1; key_left_n=1; key_right_n=1; key_mid_n=1;
  bt_cfg_mode_pulse=0; bt_cfg_next_pulse=0; bt_cfg_prev_pulse=0;
  bt_cfg_inc_pulse=0; bt_cfg_dec_pulse=0; bt_cfg_select=0; bt_cfg_select_valid=0;
  #200; rst_n=1; #100; log_values("defaults");
  if(overall_speed!=700 || step_length_set!=20 || lf_default_speed!=700 ||
     rf_default_speed!=700 || lr_default_speed!=700 || rr_default_speed!=700 ||
     lidar_safe_mm!=450) errors=errors+1;
  select_param(0); pulse_inc(); log_values("overall_plus_25");
  select_param(1); pulse_inc(); log_values("step_plus_5");
  select_param(2); pulse_dec(); log_values("lf_minus_25");
  select_param(3); pulse_inc(); log_values("rf_plus_25");
  select_param(4); pulse_inc(); pulse_inc(); log_values("lr_plus_50");
  select_param(5); pulse_dec(); pulse_dec(); log_values("rr_minus_50");
  select_param(6); pulse_inc(); log_values("lidar_plus_50");
  select_param(0); for(i=0;i<40;i=i+1) pulse_inc(); log_values("speed_max");
  for(i=0;i<40;i=i+1) pulse_dec(); log_values("speed_min");
  select_param(6); for(i=0;i<30;i=i+1) pulse_inc(); log_values("lidar_max");
  for(i=0;i<40;i=i+1) pulse_dec(); log_values("lidar_min");
  if(overall_speed!=100 || lidar_safe_mm!=150) errors=errors+1;
  if(errors==0) $display("RESULT TEST_PASS car_parameters");
  else $display("RESULT TEST_FAIL car_parameters errors=%0d",errors);
  $fclose(fd); $finish;
end


  car_param_controller car_param_controller0(
    .clk(clk),
    .rst_n(rst_n),
    .key_up_n(key_up_n),
    .key_down_n(key_down_n),
    .key_left_n(key_left_n),
    .key_right_n(key_right_n),
    .key_mid_n(key_mid_n),
    .bt_cfg_mode_pulse(bt_cfg_mode_pulse),
    .bt_cfg_next_pulse(bt_cfg_next_pulse),
    .bt_cfg_prev_pulse(bt_cfg_prev_pulse),
    .bt_cfg_inc_pulse(bt_cfg_inc_pulse),
    .bt_cfg_dec_pulse(bt_cfg_dec_pulse),
    .bt_cfg_select(bt_cfg_select),
    .bt_cfg_select_valid(bt_cfg_select_valid),
    .route_key_lock(route_key_lock),
    .setting_mode(setting_mode),
    .selected_param(selected_param),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .overall_speed(overall_speed),
    .lf_default_speed(lf_default_speed),
    .rf_default_speed(rf_default_speed),
    .lr_default_speed(lr_default_speed),
    .rr_default_speed(rr_default_speed),
    .lidar_safe_mm(lidar_safe_mm)
  );
endmodule
