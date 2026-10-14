module buzzer_event_gen_test();
reg clk;
reg rst_n;
reg [4:0] state_info;
reg [5:0] active_route_steps;
wire power_on_pulse;
wire route_add_pulse;
wire run_enter_pulse;
wire stop_enter_pulse;

localparam [4:0] STATE_MENU   = 5'b00000;
localparam [4:0] STATE_CHANGE = 5'b00001;
localparam [4:0] STATE_RUN    = 5'b00011;
localparam [4:0] STATE_STOP   = 5'b00100;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial
  begin
    rst_n = 1'b0;
    state_info = STATE_MENU;
    active_route_steps = 6'd0;

    #200;
    rst_n = 1'b1;

    #200;
    active_route_steps = 6'd1;
    #200;
    active_route_steps = 6'd2;

    #200;
    state_info = STATE_CHANGE;
    #200;
    state_info = STATE_RUN;
    #200;
    state_info = STATE_STOP;
    #200;
    state_info = STATE_MENU;
    #200;
    active_route_steps = 6'd3;

    #400;
    $finish;
  end

  initial begin
    $dumpfile("tb/tb_buzzer_event_gen_robei.vcd");
    $dumpvars(0, buzzer_event_gen_test);
  end
//---Module instantiation---
  buzzer_event_gen buzzer_event_gen0(
    .clk(clk),
    .rst_n(rst_n),
    .state_info(state_info),
    .active_route_steps(active_route_steps),
    .power_on_pulse(power_on_pulse),
    .route_add_pulse(route_add_pulse),
    .run_enter_pulse(run_enter_pulse),
    .stop_enter_pulse(stop_enter_pulse)
);

endmodule    //buzzer_event_gen_test
