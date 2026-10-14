module temp_run_status_source(
output wire run_step_pulse,
output wire obstacle_stop,
output wire [15:0] step_length_set,
output wire [15:0] speed_set
);

  assign run_step_pulse = 1'b0;
  assign obstacle_stop = 1'b0;
  assign step_length_set = 16'd20;
  assign speed_set = 16'd25;

endmodule
