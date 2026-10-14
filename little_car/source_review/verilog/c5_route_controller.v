// =============================================================================
// 模块: c5_route_controller
// 作用: 把自动路线或手机手动命令转换为四轮 11 bit 二补码有符号速度。
// 说明: 每轮速度 = clamp(整体速度 + 该轮默认速度 - 700, 100, 900)。
// 说明: 原地转向时左、右侧符号相反，并在速度大于 150 时将转向幅值减 50。
// 说明: 手动模式只阻断受障碍影响的前进，倒车和左右原地转向仍可用于脱困。
// 说明: 自动模式每条命令计时完成后输出一个 run_step_pulse，推动路线 FSM 出队。
//
// Robei 源: little_car/model/c5_route_controller.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module c5_route_controller (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire motor_enable,  // C5 运动链总使能。
  input wire [4:0] state_info,  // 路线 FSM 状态编码。
  input wire [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  input wire [15:0] forward_ms,  // 自动前进动作的后备持续时间，单位 ms；步长参数有效时由换算值覆盖。
  input wire [15:0] turn_ms,  // 自动原地转向动作的持续时间，单位 ms。
  input wire [9:0] forward_speed,  // “前进 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。
  input wire [9:0] turn_speed,  // “转向 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。
  input wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  input wire [9:0] overall_speed,  // 手机可调的公共速度基准。
  input wire [9:0] lf_default_speed,  // 左前轮校准默认速度。
  input wire [9:0] rf_default_speed,  // 右前轮校准默认速度。
  input wire [9:0] lr_default_speed,  // 左后轮校准默认速度。
  input wire [9:0] rr_default_speed,  // 右后轮校准默认速度。
  input wire manual_mode,  // 1 表示手机手动模式覆盖自动路线。
  input wire [2:0] manual_cmd,  // 手动动作：0停、1前、2后、3左转、4右转。
  input wire obstacle_stop,  // 合并或当前模块产生的停车请求。
  output reg run_step_pulse,  // 一条运动动作完成的单周期脉冲。
  output reg [10:0] lf_speed,  // 左前轮 11 bit 二补码有符号目标速度。
  output reg [10:0] rf_speed,  // 右前轮 11 bit 二补码有符号目标速度。
  output reg [10:0] lr_speed,  // 左后轮 11 bit 二补码有符号目标速度。
  output reg [10:0] rr_speed,  // 右后轮 11 bit 二补码有符号目标速度。
  output reg motion_busy,  // 自动路线动作正在计时执行；为 1 时不会重复启动同一队首命令。
  output reg [1:0] current_cmd,  // 当前执行或最近锁存的 2 bit 路线命令编码。
  output reg motor_blocked  // 当前运动命令被避障逻辑拦截。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
        // -----------------------------------------------------------------------------
        // 常量、状态编码与时序参数
        // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
        // -----------------------------------------------------------------------------
        localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
        localparam integer CYCLES_PER_MS = CLK_HZ / 1000;  // 1 ms 对应的系统时钟周期数。

        localparam [4:0] STATE_RUN = 5'b00011;  // 路线/业务状态“STATE / RUN”的固定编码；case 分支和状态上报必须保持一致。

        localparam [1:0] ROUTE_STOP    = 2'b00;  // 2 bit 路线命令“ROUTE / STOP”的编码，存入 route_info 对应槽位。
        localparam [1:0] ROUTE_RIGHT   = 2'b01;  // 2 bit 路线命令“ROUTE / RIGHT”的编码，存入 route_info 对应槽位。
        localparam [1:0] ROUTE_LEFT    = 2'b10;  // 2 bit 路线命令“ROUTE / LEFT”的编码，存入 route_info 对应槽位。
        localparam [1:0] ROUTE_FORWARD = 2'b11;  // 2 bit 路线命令“ROUTE / FORWARD”的编码，存入 route_info 对应槽位。
        localparam [2:0] MANUAL_STOP    = 3'd0;  // 手机手动动作“MANUAL / STOP”的 3 bit 协议/内部统一编码。
        localparam [2:0] MANUAL_FORWARD = 3'd1;  // 手机手动动作“MANUAL / FORWARD”的 3 bit 协议/内部统一编码。
        localparam [2:0] MANUAL_REVERSE = 3'd2;  // 手机手动动作“MANUAL / REVERSE”的 3 bit 协议/内部统一编码。
        localparam [2:0] MANUAL_LEFT    = 3'd3;  // 手机手动动作“MANUAL / LEFT”的 3 bit 协议/内部统一编码。
        localparam [2:0] MANUAL_RIGHT   = 3'd4;  // 手机手动动作“MANUAL / RIGHT”的 3 bit 协议/内部统一编码。
        localparam [9:0] CAL_BASE_SPEED = 10'd700;  // 四轮默认校准的零偏基准；默认轮速等于该值时不改变整体速度。
        localparam [9:0] SPEED_MIN = 10'd100;  // “SPEED / MIN”允许的饱和下限；继续减小时保持此值。
        localparam [9:0] SPEED_MAX = 10'd900;  // “SPEED / MAX”允许的饱和上限；继续增加时保持此值。

        // -----------------------------------------------------------------------------
        // 内部网络、寄存器和计数器
        // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
        // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
        // -----------------------------------------------------------------------------
        reg [31:0] elapsed_cycles;  // 当前自动动作已经持续的 50 MHz 时钟周期数。
        reg [31:0] target_cycles;  // 当前动作计划持续的总周期数。
        reg restart_pending;  // 动作结束后的单周期隔离标志，避免同一命令立即重复启动。

        wire in_run;  // state_info 是否等于 STATE_RUN。
        wire motion_done;  // 动作计时达到 target_cycles 的组合完成条件。
        wire [9:0] effective_forward_speed;  // 整体速度为零时回退到固定 forward_speed，否则使用手机参数。
        wire [9:0] effective_turn_speed;  // 整体速度为零时回退到固定 turn_speed，否则使用手机参数。

        // -----------------------------------------------------------------------------
        // 组合派生信号
        // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
        // -----------------------------------------------------------------------------
        // [ASSIGN] 组合定义“in_run”：state_info 是否等于 STATE_RUN。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_RUN`（STATE / RUN）的当前值决定，不引入时钟周期延迟。
        assign in_run = (state_info == STATE_RUN);
        // [ASSIGN] 组合定义“motion_done”：动作计时达到 target_cycles 的组合完成条件。 该组合结果直接由 `motion_busy`（运动动作 / 忙标志）、`elapsed_cycles`（elapsed / cycles）、`target_cycles`（目标 / cycles）的当前值决定，不引入时钟周期延迟。
        assign motion_done = motion_busy && (elapsed_cycles >= target_cycles);
        // [ASSIGN] 组合定义“effective_forward_speed”：“effective / 前进 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `overall_speed`（整体 / 速度）、`forward_speed`（前进 / 速度）的当前值决定，不引入时钟周期延迟。
        assign effective_forward_speed = (overall_speed == 10'd0) ? forward_speed : overall_speed;
        // [ASSIGN] 组合定义“effective_turn_speed”：“effective / 转向 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `overall_speed`（整体 / 速度）、`turn_speed`（转向 / 速度）的当前值决定，不引入时钟周期延迟。
        assign effective_turn_speed = (overall_speed == 10'd0) ? turn_speed : overall_speed;

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: adjusted_speed
        // 叠加公共速度和单轮校准偏置，并限制到 C5 可用范围。
        // -----------------------------------------------------------------------------
        function [9:0] adjusted_speed;
          input [9:0] wheel_default;  // 10 bit 函数 `adjusted_speed` 的“wheel / 默认值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          input [9:0] common_speed;  // 10 bit 函数 `adjusted_speed` 的“common / 速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          integer signed value;  // common_speed + wheel_default - 700 的有符号中间结果；随后饱和限制到 100..900。 该变量只在本次函数求值期间存在，不保存跨周期状态。
          begin
            value = common_speed + wheel_default - CAL_BASE_SPEED;
            if (value < SPEED_MIN)
              adjusted_speed = SPEED_MIN;
            else if (value > SPEED_MAX)
              adjusted_speed = SPEED_MAX;
            else
              adjusted_speed = value[9:0];
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: turn_mag
        // 转向降速函数，降低原地旋转的突发冲击。
        // -----------------------------------------------------------------------------
        function [9:0] turn_mag;
          input [9:0] speed;  // 10 bit 函数 `turn_mag` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            if (speed > 10'd150)
              turn_mag = speed - 10'd50;
            else
              turn_mag = speed;
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: pos_speed
        // 把无符号幅值扩展成正的 11 bit 二补码。
        // -----------------------------------------------------------------------------
        function [10:0] pos_speed;
          input [9:0] speed;  // 10 bit 函数 `pos_speed` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            pos_speed = {1'b0, speed};
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: neg_speed
        // 把无符号幅值转换为负的 11 bit 二补码。
        // -----------------------------------------------------------------------------
        function [10:0] neg_speed;
          input [9:0] speed;  // 10 bit 函数 `neg_speed` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            neg_speed = (~{1'b0, speed}) + 11'd1;
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: ms_to_cycles
        // 把毫秒动作时间换算为 50 MHz 周期终值。
        // -----------------------------------------------------------------------------
        function [31:0] ms_to_cycles;
          input [15:0] ms;  // 16 bit 函数 `ms_to_cycles` 的“MS”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            if (ms == 16'd0)
              ms_to_cycles = CYCLES_PER_MS - 1;
            else
              ms_to_cycles = ({16'd0, ms} * CYCLES_PER_MS) - 32'd1;
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: step_to_ms
        // 用移位加减实现 step_length*125，避免综合乘法器。
        // -----------------------------------------------------------------------------
        function [15:0] step_to_ms;
          input [15:0] steps;  // 16 bit 函数 `step_to_ms` 的“步数”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          reg [31:0] ms_calc;  // 32 bit step_length*125 的 32 bit 中间毫秒数；超过 60000 时输出饱和到 60 s。 该变量只在本次函数求值期间存在，不保存跨周期状态。
          begin
            if (steps == 16'd0) begin
              step_to_ms = forward_ms;
            end else begin
              ms_calc = ({16'd0, steps} << 7) - ({16'd0, steps} << 1) -
                        {16'd0, steps};
              if (ms_calc > 32'd60000)
                step_to_ms = 16'd60000;
              else
                step_to_ms = ms_calc[15:0];
            end
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [TASK] 过程任务: stop_motors
        // 原子地把四路目标速度全部写为零。
        // -----------------------------------------------------------------------------
        task stop_motors;
          begin
            lf_speed <= 11'd0;
            rf_speed <= 11'd0;
            lr_speed <= 11'd0;
            rr_speed <= 11'd0;
          end
        endtask

        // -----------------------------------------------------------------------------
        // [PROCESS] 过程块 1
        // 运动控制主时序：手动模式优先，其次避障，再执行自动路线命令和动作计时。
        // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
        // -----------------------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
            run_step_pulse <= 1'b0;
            lf_speed <= 11'd0;
            rf_speed <= 11'd0;
            lr_speed <= 11'd0;
            rr_speed <= 11'd0;
            motion_busy <= 1'b0;
            current_cmd <= ROUTE_STOP;
            elapsed_cycles <= 32'd0;
            target_cycles <= 32'd0;
            restart_pending <= 1'b0;
            motor_blocked <= 1'b0;
          end else begin
            run_step_pulse <= 1'b0;
            motor_blocked <= 1'b0;

            if (!motor_enable) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
            // -----------------------------------------------------------------------------
            // 关键分支
            // 手动模式优先于自动路线；动作持续时间由手机重复命令和蓝牙看门狗决定。
            // -----------------------------------------------------------------------------
            end else if (manual_mode) begin
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
              if (obstacle_stop && (manual_cmd == MANUAL_FORWARD)) begin
                stop_motors();
                motor_blocked <= 1'b1;
              end else begin
                case (manual_cmd)
                  MANUAL_FORWARD: begin
                    lf_speed <= pos_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                    rf_speed <= pos_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                    lr_speed <= pos_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                    rr_speed <= pos_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  end
                  MANUAL_REVERSE: begin
                    lf_speed <= neg_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                    rf_speed <= neg_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                    lr_speed <= neg_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                    rr_speed <= neg_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  end
                  MANUAL_LEFT: begin
                    lf_speed <= neg_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                    rf_speed <= pos_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                    lr_speed <= neg_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                    rr_speed <= pos_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  end
                  MANUAL_RIGHT: begin
                    lf_speed <= pos_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                    rf_speed <= neg_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                    lr_speed <= pos_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                    rr_speed <= neg_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  end
                  default: stop_motors();
                endcase
              end
            // -----------------------------------------------------------------------------
            // 关键分支
            // 自动模式遇到任一障碍源立即把四轮目标速度清零。
            // -----------------------------------------------------------------------------
            end else if (obstacle_stop) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
              motor_blocked <= 1'b1;
            end else if (!in_run || active_route_steps == 6'd0) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
            end else if (!motion_busy && restart_pending) begin
              restart_pending <= 1'b0;
            end else if (!motion_busy) begin
              current_cmd <= route_info[1:0];
              motion_busy <= 1'b1;
              elapsed_cycles <= 32'd0;
              restart_pending <= 1'b0;

              case (route_info[1:0])
                // -----------------------------------------------------------------------------
                // 关键分支
                // 自动前进：四轮同为正速度，持续 step_length*125 ms。
                // -----------------------------------------------------------------------------
                ROUTE_FORWARD: begin
                  lf_speed <= pos_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                  rf_speed <= pos_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                  lr_speed <= pos_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                  rr_speed <= pos_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  target_cycles <= ms_to_cycles(step_to_ms(step_length));
                end

                // -----------------------------------------------------------------------------
                // 关键分支
                // 自动左转：左侧反转、右侧正转，形成原地左旋。
                // -----------------------------------------------------------------------------
                ROUTE_LEFT: begin
                  lf_speed <= neg_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                  rf_speed <= pos_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                  lr_speed <= neg_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                  rr_speed <= pos_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  target_cycles <= ms_to_cycles(turn_ms);
                end

                // -----------------------------------------------------------------------------
                // 关键分支
                // 自动右转：左侧正转、右侧反转，形成原地右旋。
                // -----------------------------------------------------------------------------
                ROUTE_RIGHT: begin
                  lf_speed <= pos_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                  rf_speed <= neg_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                  lr_speed <= pos_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                  rr_speed <= neg_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  target_cycles <= ms_to_cycles(turn_ms);
                end

                default: begin
                  stop_motors();
                  target_cycles <= CYCLES_PER_MS - 1;
                end
              endcase
            end else begin
              if (motion_done || current_cmd == ROUTE_STOP) begin
                stop_motors();
                motion_busy <= 1'b0;
                elapsed_cycles <= 32'd0;
                run_step_pulse <= 1'b1;
                restart_pending <= 1'b1;
              end else begin
                elapsed_cycles <= elapsed_cycles + 32'd1;
              end
            end
          end
        end

endmodule
