module motor_tb6612_driver_test();
reg clk;
reg rst_n;
reg motor_enable;
reg [1:0] lf_dir;
reg [1:0] rf_dir;
reg [1:0] lr_dir;
reg [1:0] rr_dir;
reg [7:0] lf_duty;
reg [7:0] rf_duty;
reg [7:0] lr_duty;
reg [7:0] rr_duty;
wire lf_in1;
wire lf_in2;
wire rf_in1;
wire rf_in2;
wire lr_in1;
wire lr_in2;
wire rr_in1;
wire rr_in2;
wire lf_pwm;
wire rf_pwm;
wire lr_pwm;
wire rr_pwm;
wire stby_a;
wire stby_b;

localparam [1:0] MOTOR_STOP    = 2'b00;
localparam [1:0] MOTOR_FORWARD = 2'b01;
localparam [1:0] MOTOR_REVERSE = 2'b10;
localparam [1:0] MOTOR_BRAKE   = 2'b11;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial
  begin
    rst_n = 1'b0;
    motor_enable = 1'b0;
    lf_dir = MOTOR_STOP;
    rf_dir = MOTOR_STOP;
    lr_dir = MOTOR_STOP;
    rr_dir = MOTOR_STOP;
    lf_duty = 8'd0;
    rf_duty = 8'd0;
    lr_duty = 8'd0;
    rr_duty = 8'd0;

    #200;
    rst_n = 1'b1;
    #200;

    motor_enable = 1'b1;

    // forward
    lf_dir = MOTOR_FORWARD; lf_duty = 8'd64;
    rf_dir = MOTOR_FORWARD; rf_duty = 8'd128;
    lr_dir = MOTOR_FORWARD; lr_duty = 8'd192;
    rr_dir = MOTOR_FORWARD; rr_duty = 8'd224;
    #10000;

    // reverse
    lf_dir = MOTOR_REVERSE;
    rf_dir = MOTOR_REVERSE;
    lr_dir = MOTOR_REVERSE;
    rr_dir = MOTOR_REVERSE;
    #10000;

    // brake
    lf_dir = MOTOR_BRAKE;
    rf_dir = MOTOR_BRAKE;
    lr_dir = MOTOR_BRAKE;
    rr_dir = MOTOR_BRAKE;
    #5000;

    // stop
    lf_dir = MOTOR_STOP;
    rf_dir = MOTOR_STOP;
    lr_dir = MOTOR_STOP;
    rr_dir = MOTOR_STOP;
    #5000;

    motor_enable = 1'b0;
    #1000;
    $finish;
  end

  initial begin
    $dumpfile("tb/tb_motor_tb6612_driver_robei.vcd");
    $dumpvars(0, motor_tb6612_driver_test);
  end
//---Module instantiation---
  motor_tb6612_driver motor_tb6612_driver0(
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .lf_dir(lf_dir),
    .rf_dir(rf_dir),
    .lr_dir(lr_dir),
    .rr_dir(rr_dir),
    .lf_duty(lf_duty),
    .rf_duty(rf_duty),
    .lr_duty(lr_duty),
    .rr_duty(rr_duty),
    .lf_in1(lf_in1),
    .lf_in2(lf_in2),
    .rf_in1(rf_in1),
    .rf_in2(rf_in2),
    .lr_in1(lr_in1),
    .lr_in2(lr_in2),
    .rr_in1(rr_in1),
    .rr_in2(rr_in2),
    .lf_pwm(lf_pwm),
    .rf_pwm(rf_pwm),
    .lr_pwm(lr_pwm),
    .rr_pwm(rr_pwm),
    .stby_a(stby_a),
    .stby_b(stby_b)
);

endmodule    //motor_tb6612_driver_test
