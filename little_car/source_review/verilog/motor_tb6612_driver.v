// =============================================================================
// 模块: motor_tb6612_driver
// 作用: 旧 TB6612 双 H 桥 PWM 驱动器。
// 说明: 当前 C5 串口电机不使用此模块。
//
// Robei 源: little_car/model/motor_tb6612_driver.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module motor_tb6612_driver (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire motor_enable,  // C5 运动链总使能。
  input wire [1:0] lf_dir,  // 旧底盘 左前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  input wire [1:0] rf_dir,  // 旧底盘 右前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  input wire [1:0] lr_dir,  // 旧底盘 左后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  input wire [1:0] rr_dir,  // 旧底盘 右后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  input wire [7:0] lf_duty,  // 旧底盘 左前轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  input wire [7:0] rf_duty,  // 旧底盘 右前轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  input wire [7:0] lr_duty,  // 旧底盘 左后轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  input wire [7:0] rr_duty,  // 旧底盘 右后轮8 bit PWM 占空比命令，0 停止、255 接近全速。
  output reg lf_in1,  // 旧底盘 TB6612 左前轮方向控制 IN1 物理输出。
  output reg lf_in2,  // 旧底盘 TB6612 左前轮方向控制 IN2 物理输出。
  output reg rf_in1,  // 旧底盘 TB6612 右前轮方向控制 IN1 物理输出。
  output reg rf_in2,  // 旧底盘 TB6612 右前轮方向控制 IN2 物理输出。
  output reg lr_in1,  // 旧底盘 TB6612 左后轮方向控制 IN1 物理输出。
  output reg lr_in2,  // 旧底盘 TB6612 左后轮方向控制 IN2 物理输出。
  output reg rr_in1,  // 旧底盘 TB6612 右后轮方向控制 IN1 物理输出。
  output reg rr_in2,  // 旧底盘 TB6612 右后轮方向控制 IN2 物理输出。
  output wire lf_pwm,  // 旧底盘 TB6612 左前轮PWM 物理输出，由 8 bit 占空比比较产生。
  output wire rf_pwm,  // 旧底盘 TB6612 右前轮PWM 物理输出，由 8 bit 占空比比较产生。
  output wire lr_pwm,  // 旧底盘 TB6612 左后轮PWM 物理输出，由 8 bit 占空比比较产生。
  output wire rr_pwm,  // 旧底盘 TB6612 右后轮PWM 物理输出，由 8 bit 占空比比较产生。
  output wire stby_a,  // 旧 TB6612 驱动器待机控制输出；motor_enable=1 时拉高使能对应双通道。
  output wire stby_b  // 旧 TB6612 驱动器待机控制输出；motor_enable=1 时拉高使能对应双通道。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [1:0] MOTOR_STOP    = 2'b00;  // 旧 TB6612 底盘电机方向“MOTOR / STOP”的 2 bit 编码。
  localparam [1:0] MOTOR_FORWARD = 2'b01;  // 旧 TB6612 底盘电机方向“MOTOR / FORWARD”的 2 bit 编码。
  localparam [1:0] MOTOR_REVERSE = 2'b10;  // 旧 TB6612 底盘电机方向“MOTOR / REVERSE”的 2 bit 编码。
  localparam [1:0] MOTOR_BRAKE   = 2'b11;  // 旧 TB6612 底盘电机方向“MOTOR / BRAKE”的 2 bit 编码。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [7:0] pwm_cnt;  // 8 bit 自由运行 PWM 载波计数器；与四轮 duty 比较生成对应 PWM 输出。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“stby_a”：旧 TB6612 驱动器待机控制输出；motor_enable=1 时拉高使能对应双通道。 该组合结果直接由 `motor_enable`（电机 / 使能）的当前值决定，不引入时钟周期延迟。
  assign stby_a = motor_enable;
  // [ASSIGN] 组合定义“stby_b”：旧 TB6612 驱动器待机控制输出；motor_enable=1 时拉高使能对应双通道。 该组合结果直接由 `motor_enable`（电机 / 使能）的当前值决定，不引入时钟周期延迟。
  assign stby_b = motor_enable;
  // [ASSIGN] 组合定义“lf_pwm”：旧底盘 TB6612 左前轮PWM 物理输出，由 8 bit 占空比比较产生。 该组合结果直接由 `motor_enable`（电机 / 使能）、`lf_dir`（左前轮 / 朝向）、`MOTOR_STOP`（MOTOR / STOP）、`MOTOR_BRAKE`（MOTOR / BRAKE）、`pwm_cnt`（PWM / 计数）、`lf_duty`（左前轮 / duty）的当前值决定，不引入时钟周期延迟。
  assign lf_pwm = motor_enable && (lf_dir != MOTOR_STOP) && (lf_dir != MOTOR_BRAKE) && (pwm_cnt < lf_duty);
  // [ASSIGN] 组合定义“rf_pwm”：旧底盘 TB6612 右前轮PWM 物理输出，由 8 bit 占空比比较产生。 该组合结果直接由 `motor_enable`（电机 / 使能）、`rf_dir`（右前轮 / 朝向）、`MOTOR_STOP`（MOTOR / STOP）、`MOTOR_BRAKE`（MOTOR / BRAKE）、`pwm_cnt`（PWM / 计数）、`rf_duty`（右前轮 / duty）的当前值决定，不引入时钟周期延迟。
  assign rf_pwm = motor_enable && (rf_dir != MOTOR_STOP) && (rf_dir != MOTOR_BRAKE) && (pwm_cnt < rf_duty);
  // [ASSIGN] 组合定义“lr_pwm”：旧底盘 TB6612 左后轮PWM 物理输出，由 8 bit 占空比比较产生。 该组合结果直接由 `motor_enable`（电机 / 使能）、`lr_dir`（左后轮 / 朝向）、`MOTOR_STOP`（MOTOR / STOP）、`MOTOR_BRAKE`（MOTOR / BRAKE）、`pwm_cnt`（PWM / 计数）、`lr_duty`（左后轮 / duty）的当前值决定，不引入时钟周期延迟。
  assign lr_pwm = motor_enable && (lr_dir != MOTOR_STOP) && (lr_dir != MOTOR_BRAKE) && (pwm_cnt < lr_duty);
  // [ASSIGN] 组合定义“rr_pwm”：旧底盘 TB6612 右后轮PWM 物理输出，由 8 bit 占空比比较产生。 该组合结果直接由 `motor_enable`（电机 / 使能）、`rr_dir`（右后轮 / 朝向）、`MOTOR_STOP`（MOTOR / STOP）、`MOTOR_BRAKE`（MOTOR / BRAKE）、`pwm_cnt`（PWM / 计数）、`rr_duty`（右后轮 / duty）的当前值决定，不引入时钟周期延迟。
  assign rr_pwm = motor_enable && (rr_dir != MOTOR_STOP) && (rr_dir != MOTOR_BRAKE) && (pwm_cnt < rr_duty);

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 pwm_cnt（PWM / 计数）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pwm_cnt <= 8'd0;
    else
      pwm_cnt <= pwm_cnt + 8'd1;
  end

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 组合过程：根据当前输入和寄存状态计算 lf_in1（左前轮 / IN1）、lf_in2（左前轮 / IN2）、rf_in1（右前轮 / IN1）、rf_in2（右前轮 / IN2）、lr_in1（左后轮 / IN1）、lr_in2（左后轮 / IN2）等信号；过程内应为所有输出路径赋值，避免推断非预期锁存器。
  // -----------------------------------------------------------------------------
  always @(*) begin
    lf_in1 = 1'b0;
    lf_in2 = 1'b0;
    rf_in1 = 1'b0;
    rf_in2 = 1'b0;
    lr_in1 = 1'b0;
    lr_in2 = 1'b0;
    rr_in1 = 1'b0;
    rr_in2 = 1'b0;

    if (motor_enable) begin
      case (lf_dir)
        MOTOR_FORWARD: begin lf_in1 = 1'b1; lf_in2 = 1'b0; end
        MOTOR_REVERSE: begin lf_in1 = 1'b0; lf_in2 = 1'b1; end
        MOTOR_BRAKE: begin lf_in1 = 1'b1; lf_in2 = 1'b1; end
        default: begin lf_in1 = 1'b0; lf_in2 = 1'b0; end
      endcase

      case (rf_dir)
        MOTOR_FORWARD: begin rf_in1 = 1'b1; rf_in2 = 1'b0; end
        MOTOR_REVERSE: begin rf_in1 = 1'b0; rf_in2 = 1'b1; end
        MOTOR_BRAKE: begin rf_in1 = 1'b1; rf_in2 = 1'b1; end
        default: begin rf_in1 = 1'b0; rf_in2 = 1'b0; end
      endcase

      case (lr_dir)
        MOTOR_FORWARD: begin lr_in1 = 1'b1; lr_in2 = 1'b0; end
        MOTOR_REVERSE: begin lr_in1 = 1'b0; lr_in2 = 1'b1; end
        MOTOR_BRAKE: begin lr_in1 = 1'b1; lr_in2 = 1'b1; end
        default: begin lr_in1 = 1'b0; lr_in2 = 1'b0; end
      endcase

      case (rr_dir)
        MOTOR_FORWARD: begin rr_in1 = 1'b1; rr_in2 = 1'b0; end
        MOTOR_REVERSE: begin rr_in1 = 1'b0; rr_in2 = 1'b1; end
        MOTOR_BRAKE: begin rr_in1 = 1'b1; rr_in2 = 1'b1; end
        default: begin rr_in1 = 1'b0; rr_in2 = 1'b0; end
      endcase
    end
  end

endmodule
