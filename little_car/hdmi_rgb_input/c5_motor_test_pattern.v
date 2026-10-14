module c5_motor_test_pattern(
input wire clk,
input wire rst_n,
output reg [10:0] lf_speed,
output reg [10:0] rf_speed,
output reg [10:0] lr_speed,
output reg [10:0] rr_speed
);

  localparam integer CLK_HZ = 50000000;
  localparam integer DRIVE_CYCLES = CLK_HZ * 2;
  localparam integer STOP_CYCLES = CLK_HZ;
  localparam [31:0] TEST_SPEED = 32'd180;

  reg [31:0] counter;
  reg drive_phase;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      counter <= 32'd0;
      drive_phase <= 1'b1;
      lf_speed <= 11'd0;
      rf_speed <= 11'd0;
      lr_speed <= 11'd0;
      rr_speed <= 11'd0;
    end else begin
      if (drive_phase) begin
        lf_speed <= TEST_SPEED[10:0];
        rf_speed <= TEST_SPEED[10:0];
        lr_speed <= TEST_SPEED[10:0];
        rr_speed <= TEST_SPEED[10:0];

        if (counter >= DRIVE_CYCLES - 1) begin
          counter <= 32'd0;
          drive_phase <= 1'b0;
        end else begin
          counter <= counter + 32'd1;
        end
      end else begin
        lf_speed <= 11'd0;
        rf_speed <= 11'd0;
        lr_speed <= 11'd0;
        rr_speed <= 11'd0;

        if (counter >= STOP_CYCLES - 1) begin
          counter <= 32'd0;
          drive_phase <= 1'b1;
        end else begin
          counter <= counter + 32'd1;
        end
      end
    end
  end

endmodule
