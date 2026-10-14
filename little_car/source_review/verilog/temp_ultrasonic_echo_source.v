// =============================================================================
// 模块: temp_ultrasonic_echo_source
// 作用: 超声波回波仿真/临时源。
// 说明: 硬件顶层实际使用外部 ultrasonic_echo。
//
// Robei 源: little_car/model/temp_ultrasonic_echo_source.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module temp_ultrasonic_echo_source (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire trig,  // 临时超声模型接收的触发输入，用于启动伪回波计时。
  output reg echo,  // 临时超声模型产生的伪 ECHO 高电平。
  output reg [15:0] fake_distance_cm  // 临时超声模型用于决定伪回波宽度的目标距离，单位 cm。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [2:0] ST_IDLE  = 3'd0;  // 超声/控制状态机“ST / IDLE”的状态编码。
  localparam [2:0] ST_DELAY = 3'd1;  // 超声/控制状态机“ST / DELAY”的状态编码。
  localparam [2:0] ST_ECHO  = 3'd2;  // 超声/控制状态机“ST / ECHO”的状态编码。
  localparam [2:0] ST_WAIT  = 3'd3;  // 超声/控制状态机“ST / WAIT”的状态编码。

  localparam [31:0] RESPONSE_DELAY_CYCLES = 32'd5000;    // 100 us at 50 MHz
  localparam [31:0] FAR_ECHO_CYCLES       = 32'd174000;  // 60 cm * 2900
  localparam [31:0] NEAR_ECHO_CYCLES      = 32'd29000;   // 10 cm * 2900

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [2:0] state;  // 临时超声模型的空闲、触发延时、ECHO 高电平和结束状态。
  reg [31:0] cnt;  // 临时超声模型当前状态已经经过的系统时钟周期数。
  reg [31:0] echo_target_cycles;  // fake_distance_cm 换算得到的目标 ECHO 高电平总周期数。
  reg [3:0] trig_count;  // 用于观察触发次数的 4 bit 诊断计数器。
  reg trig_d0;  // trig 输入的第一级同步采样。
  reg trig_d1;  // trig 输入的第二级同步采样，与 trig_d0 比较检测上升沿。

  wire trig_rise;  // 临时模型对 trig 同步值检测出的上升沿单周期脉冲，用于开始伪回波。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“trig_rise”：临时模型对 trig 同步值检测出的上升沿单周期脉冲，用于开始伪回波。 该组合结果直接由 `trig_d0`（trig / D0）、`trig_d1`（trig / D1）的当前值决定，不引入时钟周期延迟。
  assign trig_rise = trig_d0 & ~trig_d1;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 trig_d0（trig / D0）、trig_d1（trig / D1）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trig_d0 <= 1'b0;
      trig_d1 <= 1'b0;
    end else begin
      trig_d0 <= trig;
      trig_d1 <= trig_d0;
    end
  end

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 时序过程：在有效时钟边沿更新 state（状态）、cnt（计数）、echo（回波）、echo_target_cycles（回波 / 目标 / cycles）、fake_distance_cm（fake / 距离 / CM）、trig_count（trig / 计数）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_IDLE;
      cnt <= 32'd0;
      echo <= 1'b0;
      echo_target_cycles <= FAR_ECHO_CYCLES;
      fake_distance_cm <= 16'd60;
      trig_count <= 4'd0;
    end else begin
      case (state)
        ST_IDLE: begin
          echo <= 1'b0;
          cnt <= 32'd0;

          if (trig_rise) begin
            if (trig_count < 4'd8) begin
              echo_target_cycles <= FAR_ECHO_CYCLES;
              fake_distance_cm <= 16'd60;
              trig_count <= trig_count + 4'd1;
            end else begin
              echo_target_cycles <= NEAR_ECHO_CYCLES;
              fake_distance_cm <= 16'd10;
            end

            state <= ST_DELAY;
          end
        end

        ST_DELAY: begin
          echo <= 1'b0;

          if (cnt >= RESPONSE_DELAY_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            echo <= 1'b1;
            state <= ST_ECHO;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_ECHO: begin
          echo <= 1'b1;

          if (cnt >= echo_target_cycles - 32'd1) begin
            cnt <= 32'd0;
            echo <= 1'b0;
            state <= ST_WAIT;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_WAIT: begin
          echo <= 1'b0;

          if (!trig) begin
            state <= ST_IDLE;
            cnt <= 32'd0;
          end
        end

        default: begin
          state <= ST_IDLE;
          cnt <= 32'd0;
          echo <= 1'b0;
        end
      endcase
    end
  end

endmodule
