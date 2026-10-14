// =============================================================================
// 模块: ultrasonic_obstacle_guard
// 作用: 前向超声波触发、回波计时、距离换算和独立停车判断。
// 说明: 每 60 ms 测量一次，触发高电平 10 us，等待及测量超时均为 30 ms。
// 说明: 距离厘米数约等于 echo_cycles/2900；超时输出 FFFF，但不会误触发停车。
//
// Robei 源: little_car/model/ultrasonic_obstacle_guard.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module ultrasonic_obstacle_guard (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire echo,  // 经板级引脚进入的异步 ECHO 脉冲，宽度表示声波往返时间。
  input wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  output reg trig,  // 每 60 ms 输出一次约 10 us 高电平以触发超声模块。
  output reg [15:0] distance_cm,  // 超声波距离，单位 cm；FFFF 表示超时无效。
  output reg distance_valid,  // 当前周期更新了超声波测距结果。
  output reg obstacle_stop,  // 合并或当前模块产生的停车请求。
  output reg echo_timeout,  // 等待或测量回波超过 30 ms。
  input wire enable  // 超声测距总使能；为 0 时终止测量并清除停车请求。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [2:0] ST_TRIG      = 3'd0;  // 超声/控制状态机“ST / TRIG”的状态编码。
  localparam [2:0] ST_WAIT_ECHO = 3'd1;  // 超声/控制状态机“ST / WAIT / ECHO”的状态编码。
  localparam [2:0] ST_MEASURE   = 3'd2;  // 超声/控制状态机“ST / MEASURE”的状态编码。
  localparam [2:0] ST_GAP       = 3'd3;  // 超声/控制状态机“ST / GAP”的状态编码。

  localparam [31:0] TRIG_CYCLES     = 32'd500;      // 10 us at 50 MHz
  localparam [31:0] ECHO_TIMEOUT    = 32'd1500000;  // 30 ms at 50 MHz
  localparam [31:0] MEASURE_PERIOD  = 32'd3000000;  // 60 ms at 50 MHz
  localparam [31:0] CYCLES_PER_CM   = 32'd2900;     // 58 us/cm * 50 cycles/us
  localparam [15:0] DEFAULT_STEP_CM = 16'd20;  // 超声避障阈值输入为零时采用的后备安全距离 20 cm。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [2:0] state;  // TRIG、WAIT_ECHO、MEASURE、GAP 四状态测距流程。
  reg [31:0] cnt;  // 超声状态机当前阶段的 50 MHz 周期计数：在 TRIG 阶段形成 10 us 脉冲，在等待/测量阶段判断 30 ms 超时，在 GAP 阶段补足 60 ms 周期。
  reg [31:0] echo_cycles;  // ECHO 高电平持续周期数，与往返声波时间成正比。
  reg [31:0] cm_cycle_cnt;  // ECHO 高电平内 0..2899 的厘米换算分频计数；每累计 2900 周期 measured_cm 加一。
  reg [15:0] measured_cm;  // echo_cycles/2900 的组合距离结果。
  reg echo_d0;  // 异步 ECHO 输入的第一级同步采样。
  reg echo_d1;  // ECHO 的第二级同步采样，对外作为 echo_sync。
  reg echo_d2;  // echo_d1 的上一周期值，与 echo_d1 比较检测上升沿和下降沿。
  reg [15:0] compare_length;  // 当前实际超声停车阈值，单位 cm；step_length 为零时使用 DEFAULT_STEP_CM。

  wire echo_sync;  // 两级同步后的 ECHO 输入。
  wire echo_rise;  // 同步后 ECHO 从 0 到 1 的单周期边沿脉冲，结束等待并开始计时。
  wire echo_fall;  // 同步后 ECHO 从 1 到 0 的单周期边沿脉冲，结束计时并换算距离。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“echo_sync”：两级同步后的 ECHO 输入。 该组合结果直接由 `echo_d1`（回波 / D1）的当前值决定，不引入时钟周期延迟。
  assign echo_sync = echo_d1;
  // [ASSIGN] 组合定义“echo_rise”：同步后 ECHO 从 0 到 1 的单周期边沿脉冲，结束等待并开始计时。 该组合结果直接由 `echo_d1`（回波 / D1）、`echo_d2`（回波 / D2）的当前值决定，不引入时钟周期延迟。
  assign echo_rise = echo_d1 & ~echo_d2;
  // [ASSIGN] 组合定义“echo_fall”：同步后 ECHO 从 1 到 0 的单周期边沿脉冲，结束计时并换算距离。 该组合结果直接由 `echo_d1`（回波 / D1）、`echo_d2`（回波 / D2）的当前值决定，不引入时钟周期延迟。
  assign echo_fall = ~echo_d1 & echo_d2;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 双触发器同步异步 ECHO 输入，降低亚稳态传播风险。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      echo_d0 <= 1'b0;
      echo_d1 <= 1'b0;
      echo_d2 <= 1'b0;
    end else begin
      echo_d0 <= echo;
      echo_d1 <= echo_d0;
      echo_d2 <= echo_d1;
    end
  end

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 超声测距状态机：TRIG、等待上升沿、测量高电平和测量间隔。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_TRIG;
      cnt <= 32'd0;
      echo_cycles <= 32'd0;
      cm_cycle_cnt <= 32'd0;
      measured_cm <= 16'd0;
      trig <= 1'b0;
      distance_cm <= 16'd0;
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;
      echo_timeout <= 1'b0;
      compare_length <= DEFAULT_STEP_CM;
    end else begin
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;

      if (step_length == 16'd0)
        compare_length <= DEFAULT_STEP_CM;
      else
        compare_length <= step_length;

      if (!enable) begin
        state <= ST_TRIG;
        cnt <= 32'd0;
        echo_cycles <= 32'd0;
        cm_cycle_cnt <= 32'd0;
        measured_cm <= 16'd0;
        trig <= 1'b0;
        obstacle_stop <= 1'b0;
        echo_timeout <= 1'b0;
      end else begin
        case (state)
        // -----------------------------------------------------------------------------
        // 关键分支
        // 输出 10 us 高电平触发超声波发射。
        // -----------------------------------------------------------------------------
        ST_TRIG: begin
          trig <= 1'b1;
          echo_timeout <= 1'b0;
          echo_cycles <= 32'd0;
          cm_cycle_cnt <= 32'd0;
          measured_cm <= 16'd0;

          if (cnt >= TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            trig <= 1'b0;
            state <= ST_WAIT_ECHO;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        // -----------------------------------------------------------------------------
        // 关键分支
        // 等待 ECHO 上升沿；30 ms 未出现则输出 FFFF 超时。
        // -----------------------------------------------------------------------------
        ST_WAIT_ECHO: begin
          trig <= 1'b0;

          if (echo_rise || echo_sync) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            state <= ST_MEASURE;
          end else if (cnt >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        // -----------------------------------------------------------------------------
        // 关键分支
        // 累计 ECHO 高电平周期，下降沿到达时换算厘米并比较阈值。
        // -----------------------------------------------------------------------------
        ST_MEASURE: begin
          trig <= 1'b0;

          if (echo_fall) begin
            if (measured_cm == 16'd0 && echo_cycles != 32'd0)
              distance_cm <= 16'd1;
            else
              distance_cm <= measured_cm;

            distance_valid <= 1'b1;
            echo_timeout <= 1'b0;

            if (((measured_cm == 16'd0 && echo_cycles != 32'd0) ? 16'd1 : measured_cm) < compare_length)
              obstacle_stop <= 1'b1;

            cnt <= 32'd0;
            state <= ST_GAP;
          end else if (echo_cycles >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            echo_cycles <= echo_cycles + 32'd1;
            if (cm_cycle_cnt >= CYCLES_PER_CM - 32'd1) begin
              cm_cycle_cnt <= 32'd0;
              if (measured_cm != 16'hFFFF)
                measured_cm <= measured_cm + 16'd1;
            end else begin
              cm_cycle_cnt <= cm_cycle_cnt + 32'd1;
            end
          end
        end

        // -----------------------------------------------------------------------------
        // 关键分支
        // 补足到 60 ms 测量周期，避免触发回波互相干扰。
        // -----------------------------------------------------------------------------
        ST_GAP: begin
          trig <= 1'b0;

          if (cnt >= MEASURE_PERIOD - TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            state <= ST_TRIG;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        default: begin
          state <= ST_TRIG;
          cnt <= 32'd0;
          trig <= 1'b0;
        end
        endcase
      end
    end
  end

endmodule
