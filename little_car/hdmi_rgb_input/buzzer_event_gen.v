module buzzer_event_gen(
  input wire clk,
  input wire rst_n,
  input wire [4:0] state_info,
  input wire [5:0] active_route_steps,
  output wire power_on_pulse,
  output wire route_add_pulse,
  output wire run_enter_pulse,
  output wire stop_enter_pulse
);

  localparam [4:0] STATE_RUN  = 5'b00011;
  localparam [4:0] STATE_STOP = 5'b00100;

  reg rst_n_d;
  reg [4:0] state_info_d;
  reg [5:0] active_route_steps_d;

  assign power_on_pulse = rst_n & ~rst_n_d;
  assign route_add_pulse = (active_route_steps > active_route_steps_d);
  assign run_enter_pulse = (state_info == STATE_RUN) && (state_info_d != STATE_RUN);
  assign stop_enter_pulse = (state_info == STATE_STOP) && (state_info_d != STATE_STOP);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_n_d <= 1'b0;
      state_info_d <= 5'd0;
      active_route_steps_d <= 6'd0;
    end else begin
      rst_n_d <= 1'b1;
      state_info_d <= state_info;
      active_route_steps_d <= active_route_steps;
    end
  end

endmodule
