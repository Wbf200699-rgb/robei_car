module motor_tb6612_driver(
input wire clk,
input wire rst_n,
input wire motor_enable,
input wire [1:0] lf_dir,
input wire [1:0] rf_dir,
input wire [1:0] lr_dir,
input wire [1:0] rr_dir,
input wire [7:0] lf_duty,
input wire [7:0] rf_duty,
input wire [7:0] lr_duty,
input wire [7:0] rr_duty,
output reg lf_in1,
output reg lf_in2,
output reg rf_in1,
output reg rf_in2,
output reg lr_in1,
output reg lr_in2,
output reg rr_in1,
output reg rr_in2,
output wire lf_pwm,
output wire rf_pwm,
output wire lr_pwm,
output wire rr_pwm,
output wire stby_a,
output wire stby_b
);

  localparam [1:0] MOTOR_STOP    = 2'b00;
  localparam [1:0] MOTOR_FORWARD = 2'b01;
  localparam [1:0] MOTOR_REVERSE = 2'b10;
  localparam [1:0] MOTOR_BRAKE   = 2'b11;

  reg [7:0] pwm_cnt;

  assign stby_a = motor_enable;
  assign stby_b = motor_enable;
  assign lf_pwm = motor_enable && (lf_dir != MOTOR_STOP) && (lf_dir != MOTOR_BRAKE) && (pwm_cnt < lf_duty);
  assign rf_pwm = motor_enable && (rf_dir != MOTOR_STOP) && (rf_dir != MOTOR_BRAKE) && (pwm_cnt < rf_duty);
  assign lr_pwm = motor_enable && (lr_dir != MOTOR_STOP) && (lr_dir != MOTOR_BRAKE) && (pwm_cnt < lr_duty);
  assign rr_pwm = motor_enable && (rr_dir != MOTOR_STOP) && (rr_dir != MOTOR_BRAKE) && (pwm_cnt < rr_duty);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pwm_cnt <= 8'd0;
    else
      pwm_cnt <= pwm_cnt + 8'd1;
  end

  always @(*) begin
    lf_in1 = 1'b0;
    lf_in2 = 1'b0;
    rf_in1 = 1'b0;
    rf_in2 = 1'b0;
    lr_in1 = 1'b0;
    lr_in2 = 1'b0;
    rr_in1 = 1'b0;
    rr_in2 = 1'b0;

    if (motor_enable) begin
      case (lf_dir)
        MOTOR_FORWARD: begin lf_in1 = 1'b1; lf_in2 = 1'b0; end
        MOTOR_REVERSE: begin lf_in1 = 1'b0; lf_in2 = 1'b1; end
        MOTOR_BRAKE: begin lf_in1 = 1'b1; lf_in2 = 1'b1; end
        default: begin lf_in1 = 1'b0; lf_in2 = 1'b0; end
      endcase

      case (rf_dir)
        MOTOR_FORWARD: begin rf_in1 = 1'b1; rf_in2 = 1'b0; end
        MOTOR_REVERSE: begin rf_in1 = 1'b0; rf_in2 = 1'b1; end
        MOTOR_BRAKE: begin rf_in1 = 1'b1; rf_in2 = 1'b1; end
        default: begin rf_in1 = 1'b0; rf_in2 = 1'b0; end
      endcase

      case (lr_dir)
        MOTOR_FORWARD: begin lr_in1 = 1'b1; lr_in2 = 1'b0; end
        MOTOR_REVERSE: begin lr_in1 = 1'b0; lr_in2 = 1'b1; end
        MOTOR_BRAKE: begin lr_in1 = 1'b1; lr_in2 = 1'b1; end
        default: begin lr_in1 = 1'b0; lr_in2 = 1'b0; end
      endcase

      case (rr_dir)
        MOTOR_FORWARD: begin rr_in1 = 1'b1; rr_in2 = 1'b0; end
        MOTOR_REVERSE: begin rr_in1 = 1'b0; rr_in2 = 1'b1; end
        MOTOR_BRAKE: begin rr_in1 = 1'b1; rr_in2 = 1'b1; end
        default: begin rr_in1 = 1'b0; rr_in2 = 1'b0; end
      endcase
    end
  end

endmodule
