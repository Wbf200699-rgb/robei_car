module vl53l1x_array5 (
  input wire clk,
  input wire rst_n,
  input wire enable,
  output wire xshut0,
  output wire xshut1,
  output wire xshut2,
  output wire xshut3,
  output wire xshut4,
  output wire i2c_scl,
  output reg initialization_done,
  output reg [4:0] sensor_online,
  output reg [4:0] distance_valid,
  output reg [79:0] distance_mm,
  output reg distance_update_pulse,
  output reg [2:0] distance_update_sensor,
  output reg [15:0] error_count,
  inout wire i2c_sda
);

  // The five VL53L1X sensors are not fitted in the final vehicle. Keep the
  // legacy ports and telemetry shape stable while synthesizing no active
  // controller logic and leaving the shared I2C bus electrically released.
  assign xshut0 = 1'b0;
  assign xshut1 = 1'b0;
  assign xshut2 = 1'b0;
  assign xshut3 = 1'b0;
  assign xshut4 = 1'b0;
  assign i2c_scl = 1'b1;
  assign i2c_sda = 1'bz;

  always @(*) begin
    initialization_done = 1'b0;
    sensor_online = 5'b00000;
    distance_valid = 5'b00000;
    distance_mm = {5{16'hFFFF}};
    distance_update_pulse = 1'b0;
    distance_update_sensor = 3'd0;
    error_count = 16'd0;
  end

endmodule
