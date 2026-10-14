module motor_route_controller(
input wire clk,
input wire rst_n,
input wire motor_enable,
input wire [4:0] state_info,
input wire [63:0] route_info,
input wire [5:0] active_route_steps,
input wire lf_tick,
input wire rf_tick,
input wire lr_tick,
input wire rr_tick,
input wire [15:0] forward_ticks,
input wire [15:0] turn_ticks,
input wire [7:0] forward_duty,
input wire [7:0] turn_duty,
output reg run_step_pulse,
output reg [1:0] lf_dir,
output reg [1:0] rf_dir,
output reg [1:0] lr_dir,
output reg [1:0] rr_dir,
output reg [7:0] lf_duty,
output reg [7:0] rf_duty,
output reg [7:0] lr_duty,
output reg [7:0] rr_duty,
output reg motion_busy,
output reg [1:0] current_cmd,
output reg [15:0] lf_count,
output reg [15:0] rf_count,
output reg [15:0] lr_count,
output reg [15:0] rr_count
);

  // Temporary wheel mapping note from on-board single-wheel test:
  // code signal `lr` currently drives the physical right-rear wheel,
  // and the direction observed with this mapping is correct.
  // code signal `lf` currently drives the physical left-rear wheel,
  // and the direction observed with this mapping is correct.
  // code signal `rf` currently drives the physical left-front wheel,
  // but the forward direction is inverted and must be flipped later.
  // code signal `rr` currently drives the physical right-front wheel,
  // but the forward direction is inverted and must be flipped later.
  //
  // Finalized physical mapping:
  //   lf -> left-rear   (normal polarity)
  //   lr -> right-rear  (normal polarity)
  //   rf -> left-front  (inverted polarity)
  //   rr -> right-front (inverted polarity)

  localparam [4:0] STATE_RUN = 5'b00011;

  localparam [1:0] ROUTE_STOP    = 2'b00;
  localparam [1:0] ROUTE_RIGHT   = 2'b01;
  localparam [1:0] ROUTE_LEFT    = 2'b10;
  localparam [1:0] ROUTE_FORWARD = 2'b11;

  localparam [1:0] MOTOR_STOP    = 2'b00;
  localparam [1:0] MOTOR_FORWARD = 2'b01;
  localparam [1:0] MOTOR_REVERSE = 2'b10;

  reg [15:0] target_ticks;
  reg [1:0] start_cmd;
  reg restart_pending;

  wire in_run;
  wire motion_done;

  assign in_run = (state_info == STATE_RUN);
  assign motion_done = motion_busy &&
                       (lf_count >= target_ticks) &&
                       (rf_count >= target_ticks) &&
                       (lr_count >= target_ticks) &&
                       (rr_count >= target_ticks);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run_step_pulse <= 1'b0;
      lf_dir <= MOTOR_STOP;
      rf_dir <= MOTOR_STOP;
      lr_dir <= MOTOR_STOP;
      rr_dir <= MOTOR_STOP;
      lf_duty <= 8'd0;
      rf_duty <= 8'd0;
      lr_duty <= 8'd0;
      rr_duty <= 8'd0;
      motion_busy <= 1'b0;
      current_cmd <= ROUTE_STOP;
      lf_count <= 16'd0;
      rf_count <= 16'd0;
      lr_count <= 16'd0;
      rr_count <= 16'd0;
      target_ticks <= 16'd0;
      start_cmd <= ROUTE_STOP;
      restart_pending <= 1'b0;
    end else begin
      run_step_pulse <= 1'b0;

      if (!motor_enable || !in_run || active_route_steps == 6'd0) begin
        lf_dir <= MOTOR_STOP;
        rf_dir <= MOTOR_STOP;
        lr_dir <= MOTOR_STOP;
        rr_dir <= MOTOR_STOP;
        lf_duty <= 8'd0;
        rf_duty <= 8'd0;
        lr_duty <= 8'd0;
        rr_duty <= 8'd0;
        motion_busy <= 1'b0;
        current_cmd <= ROUTE_STOP;
        lf_count <= 16'd0;
        rf_count <= 16'd0;
        lr_count <= 16'd0;
        rr_count <= 16'd0;
        target_ticks <= 16'd0;
        start_cmd <= ROUTE_STOP;
        restart_pending <= 1'b0;
      end else if (!motion_busy && restart_pending) begin
        restart_pending <= 1'b0;
      end else if (!motion_busy) begin
        start_cmd <= route_info[1:0];
        current_cmd <= route_info[1:0];
        lf_count <= 16'd0;
        rf_count <= 16'd0;
        lr_count <= 16'd0;
        rr_count <= 16'd0;
        motion_busy <= 1'b1;
        restart_pending <= 1'b0;

        case (route_info[1:0])
          ROUTE_FORWARD: begin
            lf_dir <= MOTOR_REVERSE;
            rf_dir <= MOTOR_FORWARD;
            lr_dir <= MOTOR_REVERSE;
            rr_dir <= MOTOR_FORWARD;
            lf_duty <= forward_duty;
            rf_duty <= forward_duty;
            lr_duty <= forward_duty;
            rr_duty <= forward_duty;
            target_ticks <= forward_ticks;
          end

          ROUTE_LEFT: begin
            lf_dir <= MOTOR_FORWARD;
            rf_dir <= MOTOR_REVERSE;
            lr_dir <= MOTOR_REVERSE;
            rr_dir <= MOTOR_FORWARD;
            lf_duty <= turn_duty;
            rf_duty <= turn_duty;
            lr_duty <= turn_duty;
            rr_duty <= turn_duty;
            target_ticks <= turn_ticks;
          end

          ROUTE_RIGHT: begin
            lf_dir <= MOTOR_REVERSE;
            rf_dir <= MOTOR_FORWARD;
            lr_dir <= MOTOR_FORWARD;
            rr_dir <= MOTOR_REVERSE;
            lf_duty <= turn_duty;
            rf_duty <= turn_duty;
            lr_duty <= turn_duty;
            rr_duty <= turn_duty;
            target_ticks <= turn_ticks;
          end

          default: begin
            lf_dir <= MOTOR_STOP;
            rf_dir <= MOTOR_STOP;
            lr_dir <= MOTOR_STOP;
            rr_dir <= MOTOR_STOP;
            lf_duty <= 8'd0;
            rf_duty <= 8'd0;
            lr_duty <= 8'd0;
            rr_duty <= 8'd0;
            target_ticks <= 16'd1;
          end
        endcase
      end else begin
        if (lf_tick && lf_count != 16'hFFFF)
          lf_count <= lf_count + 16'd1;

        if (rf_tick && rf_count != 16'hFFFF)
          rf_count <= rf_count + 16'd1;

        if (lr_tick && lr_count != 16'hFFFF)
          lr_count <= lr_count + 16'd1;

        if (rr_tick && rr_count != 16'hFFFF)
          rr_count <= rr_count + 16'd1;

        if (motion_done || start_cmd == ROUTE_STOP) begin
          lf_dir <= MOTOR_STOP;
          rf_dir <= MOTOR_STOP;
          lr_dir <= MOTOR_STOP;
          rr_dir <= MOTOR_STOP;
          lf_duty <= 8'd0;
          rf_duty <= 8'd0;
          lr_duty <= 8'd0;
          rr_duty <= 8'd0;
          motion_busy <= 1'b0;
          run_step_pulse <= 1'b1;
          restart_pending <= 1'b1;
        end
      end
    end
  end

endmodule
