// =============================================================================
// 模块: c5_motor_test_pattern
// 作用: C5 四轮固定测试序列发生器。
// 说明: 当前系统顶层保留了实例但输出未接入执行链，因此不会控制实际轮子。
//
// Robei 源: little_car/model/c5_motor_test_pattern.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module c5_motor_test_pattern (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  output reg [10:0] lf_speed,  // 左前轮 11 bit 二补码有符号目标速度。
  output reg [10:0] rf_speed,  // 右前轮 11 bit 二补码有符号目标速度。
  output reg [10:0] lr_speed,  // 左后轮 11 bit 二补码有符号目标速度。
  output reg [10:0] rr_speed  // 右后轮 11 bit 二补码有符号目标速度。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
  localparam integer DRIVE_CYCLES = CLK_HZ * 2;  // 未接入主链的固定测试图案连续输出 TEST_SPEED 的时间；50 MHz 下等于 2 s。
  localparam integer STOP_CYCLES = CLK_HZ;  // 固定测试图案四轮保持零速的时间；50 MHz 下等于 1 s，之后再次进入驱动阶段。
  localparam [31:0] TEST_SPEED = 32'd180;  // 未接入主控制链的 C5 测试图案固定轮速幅值。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [31:0] counter;  // 当前测试阶段已经持续的 50 MHz 周期数；驱动阶段计到 2 s，停车阶段计到 1 s 后切换阶段。
  reg drive_phase;  // 固定测试序列阶段标志：1 时四轮输出 TEST_SPEED，0 时四轮输出零速。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 counter（counter）、drive_phase（drive / phase）、lf_speed（左前轮 / 速度）、rf_speed（右前轮 / 速度）、lr_speed（左后轮 / 速度）、rr_speed（右后轮 / 速度）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
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
