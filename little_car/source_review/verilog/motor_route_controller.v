// =============================================================================
// 模块: motor_route_controller
// 作用: 旧 TB6612 底盘路线到左右轮控制器。
// 说明: 当前 C5 四驱链由 c5_route_controller 替代。
//
// Robei 源: little_car/model/motor_route_controller.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module motor_route_controller (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire motor_enable,  // C5 运动链总使能。
  input wire [4:0] state_info,  // 路线 FSM 状态编码。
  input wire [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  input wire lf_tick,  // 旧底盘 左前轮接收的编码器计数脉冲。
  input wire rf_tick,  // 旧底盘 右前轮接收的编码器计数脉冲。
  input wire lr_tick,  // 旧底盘 左后轮接收的编码器计数脉冲。
  input wire rr_tick,  // 旧底盘 右后轮接收的编码器计数脉冲。
  input wire [15:0] forward_ticks,  // 旧编码器底盘一次前进动作需要累计的轮脉冲目标数。
  input wire [15:0] turn_ticks,  // 旧编码器底盘一次转向动作需要累计的轮脉冲目标数。
  input wire [7:0] forward_duty,  // 旧编码器底盘执行前进动作时采用的 8 bit PWM 占空比。
  input wire [7:0] turn_duty,  // 旧编码器底盘执行转向动作时采用的 8 bit PWM 占空比。
  output reg run_step_pulse,  // 一条运动动作完成的单周期脉冲。
  output reg [1:0] lf_dir,  // 旧底盘 左前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg [1:0] rf_dir,  // 旧底盘 右前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg [1:0] lr_dir,  // 旧底盘 左后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg [1:0] rr_dir,  // 旧底盘 右后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg [7:0] lf_duty,  // 旧底盘 左前轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  output reg [7:0] rf_duty,  // 旧底盘 右前轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  output reg [7:0] lr_duty,  // 旧底盘 左后轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  output reg [7:0] rr_duty,  // 旧底盘 右后轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  output reg motion_busy,  // 自动路线动作正在计时执行；为 1 时不会重复启动同一队首命令。
  output reg [1:0] current_cmd,  // 当前执行或最近锁存的 2 bit 路线命令编码。
  output reg [15:0] lf_count,  // 旧底盘当前动作期间累计的左前轮编码器脉冲数；启动新动作时清零，达到 target_ticks 后用于 motion_done 判定。
  output reg [15:0] rf_count,  // 旧底盘当前动作期间累计的右前轮编码器脉冲数；与其余三轮计数共同决定动作是否完成。
  output reg [15:0] lr_count,  // 旧底盘当前动作期间累计的左后轮编码器脉冲数；只在 motion_busy 且 lr_tick 到达时递增。
  output reg [15:0] rr_count  // 旧底盘当前动作期间累计的右后轮编码器脉冲数；只在 motion_busy 且 rr_tick 到达时递增。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
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

  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [4:0] STATE_RUN = 5'b00011;  // 路线/业务状态“STATE / RUN”的固定编码；case 分支和状态上报必须保持一致。

  localparam [1:0] ROUTE_STOP    = 2'b00;  // 2 bit 路线命令“ROUTE / STOP”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_RIGHT   = 2'b01;  // 2 bit 路线命令“ROUTE / RIGHT”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_LEFT    = 2'b10;  // 2 bit 路线命令“ROUTE / LEFT”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_FORWARD = 2'b11;  // 2 bit 路线命令“ROUTE / FORWARD”的编码，存入 route_info 对应槽位。

  localparam [1:0] MOTOR_STOP    = 2'b00;  // 旧 TB6612 底盘电机方向“MOTOR / STOP”的 2 bit 编码。
  localparam [1:0] MOTOR_FORWARD = 2'b01;  // 旧 TB6612 底盘电机方向“MOTOR / FORWARD”的 2 bit 编码。
  localparam [1:0] MOTOR_REVERSE = 2'b10;  // 旧 TB6612 底盘电机方向“MOTOR / REVERSE”的 2 bit 编码。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [15:0] target_ticks;  // 当前旧底盘动作要求每个轮子累计达到的编码器脉冲目标。
  reg [1:0] start_cmd;  // 当前已启动并保持到动作结束的 2 bit 路线命令。
  reg restart_pending;  // 旧底盘动作结束后的单周期隔离标志，避免同一队首命令立即重复启动。

  wire in_run;  // 旧底盘路线状态是否处于 STATE_RUN。
  wire motion_done;  // 四轮编码器累计值均达到 target_ticks 的旧底盘动作完成判定。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“in_run”：旧底盘路线状态是否处于 STATE_RUN。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_RUN`（STATE / RUN）的当前值决定，不引入时钟周期延迟。
  assign in_run = (state_info == STATE_RUN);
  // [ASSIGN] 组合定义“motion_done”：四轮编码器累计值均达到 target_ticks 的旧底盘动作完成判定。 该组合结果直接由 `motion_busy`（运动动作 / 忙标志）、`lf_count`（左前轮 / 计数）、`target_ticks`（目标 / ticks）、`rf_count`（右前轮 / 计数）、`lr_count`（左后轮 / 计数）、`rr_count`（右后轮 / 计数）的当前值决定，不引入时钟周期延迟。
  assign motion_done = motion_busy &&
                       (lf_count >= target_ticks) &&
                       (rf_count >= target_ticks) &&
                       (lr_count >= target_ticks) &&
                       (rr_count >= target_ticks);

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 run_step_pulse（RUN / 步骤 / 脉冲）、lf_dir（左前轮 / 朝向）、rf_dir（右前轮 / 朝向）、lr_dir（左后轮 / 朝向）、rr_dir（右后轮 / 朝向）、lf_duty（左前轮 / duty）等信号；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
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
