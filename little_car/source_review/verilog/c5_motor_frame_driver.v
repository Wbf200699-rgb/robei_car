// =============================================================================
// 模块: c5_motor_frame_driver
// 作用: 将四轮有符号速度编码为 C5 总线 ASCII 帧，并实现启动/停车保护。
// 说明: 电机 ID 006/007/008/009；运动帧 62 字节，串口 115200 8N1。
// 说明: 左轮脉宽=1500+速度，右轮脉宽=1500-速度，用于补偿左右安装方向。
// 说明: 运动期间每 100 ms 刷新一次；上电等待 500 ms 后发送三次 #255PDST!。
// 说明: force_stop、运动到停止的边沿和剩余停车序列具有高于普通运动帧的优先级。
//
// Robei 源: little_car/model/c5_motor_frame_driver.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module c5_motor_frame_driver (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire motor_enable,  // C5 运动链总使能。
  input wire force_stop,  // 高优先级停车请求；到达后等待当前字节/帧边界并优先装载强化停车序列。
  input wire [10:0] lf_speed,  // 左前轮 11 bit 二补码有符号目标速度。
  input wire [10:0] rf_speed,  // 右前轮 11 bit 二补码有符号目标速度。
  input wire [10:0] lr_speed,  // 左后轮 11 bit 二补码有符号目标速度。
  input wire [10:0] rr_speed,  // 右后轮 11 bit 二补码有符号目标速度。
  output wire c5_bus_tx,  // C5 四轮控制总线的 115200 bit/s、8N1 串行输出。
  output reg frame_busy,  // 1 表示 C5 UART 正在发送完整帧。
  output reg [15:0] frame_count,  // 已经完整发送的 C5 帧数量。
  output reg [11:0] last_lf_pulse,  // 最近一帧发送给左前轮的 C5 脉宽诊断值，单位 us。
  output reg [11:0] last_rf_pulse,  // 最近一帧发送给右前轮的 C5 脉宽诊断值，单位 us。
  output reg [11:0] last_lr_pulse,  // 最近一帧发送给左后轮的 C5 脉宽诊断值，单位 us。
  output reg [11:0] last_rr_pulse,  // 最近一帧发送给右后轮的 C5 脉宽诊断值，单位 us。
  output reg stop_frames_complete,  // 强化停车序列三帧已经完成。
  output reg [1:0] stop_frames_remaining  // 尚未发送完成的强化停车帧数量。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
            // -----------------------------------------------------------------------------
            // 常量、状态编码与时序参数
            // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
            // -----------------------------------------------------------------------------
            localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
            localparam integer BAUD = 115200;  // UART 目标波特率，单位 bit/s。
            localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;  // 每个 UART bit 占用的系统时钟周期数；整数除法会形成实际波特率误差。
            localparam integer MOVING_REFRESH_MS = 100;  // C5 四轮处于运动状态时主动重发当前控制帧的等待时间，单位 ms；当前为 100 ms。
            localparam integer MOVING_REFRESH_CYCLES = (CLK_HZ / 1000) * MOVING_REFRESH_MS;  // C5 运动状态两次主动刷新之间的 100 ms 等待周期数。
          localparam integer STARTUP_DELAY_MS = 500;  // FPGA 复位释放后 C5 驱动保持零速并等待底盘稳定的时间，单位 ms；当前为 500 ms。
            localparam integer STARTUP_CYCLES = (CLK_HZ / 1000) * STARTUP_DELAY_MS;  // C5 上电保护 500 ms 对应的 50 MHz 周期数；计满前不会接受普通运动帧。
          localparam [6:0] FRAME_LAST = 7'd61;  // 62 字节 C5 运动帧最后一个字符的零基下标。
          localparam [6:0] STOP_FRAME_LAST = 7'd8;  // 9 字节 #255PDST! 停车帧最后一个字符的零基下标。
            localparam [1:0] UART_IDLE  = 2'd0;  // UART 发送状态“UART / IDLE”的编码。
            localparam [1:0] UART_START = 2'd1;  // UART 发送状态“UART / START”的编码。
            localparam [1:0] UART_DATA  = 2'd2;  // UART 发送状态“UART / DATA”的编码。
            localparam [1:0] UART_STOP  = 2'd3;  // UART 发送状态“UART / STOP”的编码。

            // -----------------------------------------------------------------------------
            // 内部网络、寄存器和计数器
            // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
            // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
            // -----------------------------------------------------------------------------
            wire [10:0] effective_lf_speed;  // motor_enable=1 时等于左前目标速度，否则强制为零；后续变化检测和组帧只使用该值。
            wire [10:0] effective_rf_speed;  // motor_enable=1 时等于右前目标速度，否则强制为零；后续变化检测和组帧只使用该值。
            wire [10:0] effective_lr_speed;  // motor_enable=1 时等于左后目标速度，否则强制为零；后续变化检测和组帧只使用该值。
            wire [10:0] effective_rr_speed;  // motor_enable=1 时等于右后目标速度，否则强制为零；后续变化检测和组帧只使用该值。
            wire uart_done;  // 检测到 UART busy 从 1 变 0，表示一个完整字节已经发送。
            wire command_changed;  // 任一当前有效轮速与最近一次已锁存发送轮速不同时为 1，用于立即请求新运动帧。
            wire refresh_due;  // 小车正在运动且 100 ms 刷新计数到期；即使命令未变化也补发当前四轮帧。
            wire any_motion;  // 四个当前有效轮速中至少一个非零的组合判定。
            wire last_any_motion;  // 最近一次锁存的四轮速度中至少一个非零，用于识别从运动切换到零速的边沿。
            wire stop_transition;  // 当前四轮已全为零而上一组锁存速度仍有运动时为 1，触发强化停车序列。
          wire send_stop_now;  // 强制停车、运动到停止、仍有停车帧待发或当前零速时提出的停车帧优先请求。
          wire [6:0] active_frame_last;  // 当前帧最后字符下标：停车帧选择 STOP_FRAME_LAST，四轮运动帧选择 FRAME_LAST。

            reg [10:0] last_lf_speed;  // 最近一次已锁存发送的左前目标速度，用于检测命令变化。
            reg [10:0] last_rf_speed;  // 最近一次已锁存发送的右前目标速度，用于检测命令变化。
            reg [10:0] last_lr_speed;  // 最近一次已锁存发送的左后目标速度，用于检测命令变化。
            reg [10:0] last_rr_speed;  // 最近一次已锁存发送的右后目标速度，用于检测命令变化。
            reg [11:0] frame_lf_pulse;  // 当前帧锁存的左前 C5 脉宽，帧发送中途不会随输入改变。
            reg [11:0] frame_rf_pulse;  // 当前帧锁存的右前 C5 脉宽。
            reg [11:0] frame_lr_pulse;  // 当前帧锁存的左后 C5 脉宽。
            reg [11:0] frame_rr_pulse;  // 当前帧锁存的右后 C5 脉宽。
            reg [31:0] refresh_count;  // 运动期间 100 ms 周期刷新计数器。
            reg [6:0] send_index;  // 当前完整帧正在发送的字符下标。
            reg [7:0] uart_data;  // 帧调度器准备发送的当前 ASCII 字符。
            reg uart_start;  // 请求底层 UART 发送 uart_data 的单周期脉冲。
            reg uart_tx;  // 底层 UART 状态机产生的串行输出电平。
            reg uart_busy;  // 底层 UART 正在发送一个字节。
            reg uart_busy_d;  // uart_busy 的一拍延迟，用于生成 uart_done。
            reg [1:0] uart_state;  // 底层单字节 UART 的 IDLE/START/DATA/STOP 状态。
            reg [15:0] uart_clk_count;  // UART 当前 bit 的系统时钟分频计数器。
            reg [2:0] uart_bit_index;  // UART 当前发送的数据位编号 0..7。
            reg [7:0] uart_shifter;  // UART 发送开始时锁存的 8 bit 字节。
            reg sending;  // 完整 C5 帧互斥标志；为 1 时不得切换到另一帧。
            reg pending_send;  // 发送期间检测到新命令时暂存请求，当前帧结束后处理。
            reg frame_is_stop;  // 当前锁存帧类型：1 为 #255PDST!，0 为四轮 62 字节运动帧。
            reg [24:0] startup_count;  // 上电 500 ms 安全等待计数器。
            reg startup_ready;  // 启动等待完成后锁存为 1。

            // -----------------------------------------------------------------------------
            // 组合派生信号
            // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
            // -----------------------------------------------------------------------------
            // [ASSIGN] 组合定义“effective_lf_speed”：“effective / 左前轮 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `motor_enable`（电机 / 使能）、`lf_speed`（左前轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign effective_lf_speed = motor_enable ? lf_speed : 11'd0;
            // [ASSIGN] 组合定义“effective_rf_speed”：“effective / 右前轮 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `motor_enable`（电机 / 使能）、`rf_speed`（右前轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign effective_rf_speed = motor_enable ? rf_speed : 11'd0;
            // [ASSIGN] 组合定义“effective_lr_speed”：“effective / 左后轮 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `motor_enable`（电机 / 使能）、`lr_speed`（左后轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign effective_lr_speed = motor_enable ? lr_speed : 11'd0;
            // [ASSIGN] 组合定义“effective_rr_speed”：“effective / 右后轮 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `motor_enable`（电机 / 使能）、`rr_speed`（右后轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign effective_rr_speed = motor_enable ? rr_speed : 11'd0;
            // [ASSIGN] 组合定义“command_changed”：任一当前有效轮速与最近一次已锁存发送轮速不同时为 1，用于立即请求新运动帧。 该组合结果直接由 `effective_lf_speed`（effective / 左前轮 / 速度）、`last_lf_speed`（最近一次 / 左前轮 / 速度）、`effective_rf_speed`（effective / 右前轮 / 速度）、`last_rf_speed`（最近一次 / 右前轮 / 速度）、`effective_lr_speed`（effective / 左后轮 / 速度）、`last_lr_speed`（最近一次 / 左后轮 / 速度）等当前值决定，不引入时钟周期延迟。
            assign command_changed = (effective_lf_speed != last_lf_speed) ||
                                     (effective_rf_speed != last_rf_speed) ||
                                     (effective_lr_speed != last_lr_speed) ||
                                     (effective_rr_speed != last_rr_speed);
            // [ASSIGN] 组合定义“any_motion”：四个当前有效轮速中至少一个非零的组合判定。 该组合结果直接由 `effective_lf_speed`（effective / 左前轮 / 速度）、`effective_rf_speed`（effective / 右前轮 / 速度）、`effective_lr_speed`（effective / 左后轮 / 速度）、`effective_rr_speed`（effective / 右后轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign any_motion = (effective_lf_speed != 11'd0) ||
                                (effective_rf_speed != 11'd0) ||
                                (effective_lr_speed != 11'd0) ||
                                (effective_rr_speed != 11'd0);
            // [ASSIGN] 组合定义“last_any_motion”：最近一次锁存的四轮速度中至少一个非零，用于识别从运动切换到零速的边沿。 该组合结果直接由 `last_lf_speed`（最近一次 / 左前轮 / 速度）、`last_rf_speed`（最近一次 / 右前轮 / 速度）、`last_lr_speed`（最近一次 / 左后轮 / 速度）、`last_rr_speed`（最近一次 / 右后轮 / 速度）的当前值决定，不引入时钟周期延迟。
            assign last_any_motion = (last_lf_speed != 11'd0) ||
                                     (last_rf_speed != 11'd0) ||
                                     (last_lr_speed != 11'd0) ||
                                     (last_rr_speed != 11'd0);
            // [ASSIGN] 组合定义“stop_transition”：当前四轮已全为零而上一组锁存速度仍有运动时为 1，触发强化停车序列。 该组合结果直接由 `any_motion`（ANY / 运动动作）、`last_any_motion`（最近一次 / ANY / 运动动作）的当前值决定，不引入时钟周期延迟。
            assign stop_transition = !any_motion && last_any_motion;
          // [ASSIGN] 组合定义“refresh_due”：小车正在运动且 100 ms 刷新计数到期；即使命令未变化也补发当前四轮帧。 该组合结果直接由 `any_motion`（ANY / 运动动作）、`refresh_count`（refresh / 计数）、`MOVING_REFRESH_CYCLES`（MOVING / REFRESH / CYCLES）的当前值决定，不引入时钟周期延迟。
          assign refresh_due = any_motion &&
                               (refresh_count >= MOVING_REFRESH_CYCLES - 1);
            // [ASSIGN] 组合定义“send_stop_now”：强制停车、运动到停止、仍有停车帧待发或当前零速时提出的停车帧优先请求。 该组合结果直接由 `force_stop`（强制请求 / 停车）、`stop_transition`（停车 / transition）、`stop_frames_remaining`（停车 / 帧数 / 剩余）、`any_motion`（ANY / 运动动作）的当前值决定，不引入时钟周期延迟。
            assign send_stop_now = force_stop || stop_transition ||
                                   (stop_frames_remaining != 2'd0) || !any_motion;
          // [ASSIGN] 组合定义“uart_done”：检测到 UART busy 从 1 变 0，表示一个完整字节已经发送。 该组合结果直接由 `uart_busy_d`（UART / 忙标志 / D）、`uart_busy`（UART / 忙标志）的当前值决定，不引入时钟周期延迟。
          assign uart_done = uart_busy_d && !uart_busy;
          // [ASSIGN] 组合定义“active_frame_last”：当前帧最后字符下标：停车帧选择 STOP_FRAME_LAST，四轮运动帧选择 FRAME_LAST。 该组合结果直接由 `frame_is_stop`（帧 / 判定 / 停车）、`STOP_FRAME_LAST`（STOP / FRAME / LAST）、`FRAME_LAST`（FRAME / LAST）的当前值决定，不引入时钟周期延迟。
          assign active_frame_last = frame_is_stop ? STOP_FRAME_LAST : FRAME_LAST;
            // [ASSIGN] 组合定义“c5_bus_tx”：C5 四轮控制总线的 115200 bit/s、8N1 串行输出。 该组合结果直接由 `uart_tx`（UART / 发送）的当前值决定，不引入时钟周期延迟。
            assign c5_bus_tx = uart_tx;

            // -----------------------------------------------------------------------------
            // [FUNCTION] 辅助函数: speed_abs
            // 读取 11 bit 二补码轮速的符号位，返回对应无符号幅值供左右轮脉宽映射使用。
            // -----------------------------------------------------------------------------
            function [10:0] speed_abs;
              input [10:0] speed;  // 11 bit 函数 `speed_abs` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              begin
                if (speed[10])
                  speed_abs = (~speed) + 11'd1;
                else
                  speed_abs = speed;
              end
            endfunction

            // -----------------------------------------------------------------------------
            // [FUNCTION] 辅助函数: left_motor_pulse
            // 左侧安装方向映射：1500 + signed_speed。
            // -----------------------------------------------------------------------------
            function [11:0] left_motor_pulse;
              input [10:0] speed;  // 11 bit 函数 `left_motor_pulse` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              reg [10:0] mag;  // 11 bit 输入有符号轮速的绝对幅值；用于计算左侧脉宽 1500+speed 或 1500-mag。 该变量只在本次函数求值期间存在，不保存跨周期状态。
              begin
                mag = speed_abs(speed);
                if (speed[10])
                  left_motor_pulse = 12'd1500 - {1'b0, mag};
                else
                  left_motor_pulse = 12'd1500 + {1'b0, mag};
              end
            endfunction

            // -----------------------------------------------------------------------------
            // [FUNCTION] 辅助函数: right_motor_pulse
            // 右侧安装方向映射：1500 - signed_speed。
            // -----------------------------------------------------------------------------
            function [11:0] right_motor_pulse;
              input [10:0] speed;  // 11 bit 函数 `right_motor_pulse` 的“速度”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              reg [10:0] mag;  // 11 bit 输入有符号轮速的绝对幅值；用于计算右侧反向安装对应的 1500-speed 或 1500+mag。 该变量只在本次函数求值期间存在，不保存跨周期状态。
              begin
                mag = speed_abs(speed);
                if (speed[10])
                  right_motor_pulse = 12'd1500 + {1'b0, mag};
                else
                  right_motor_pulse = 12'd1500 - {1'b0, mag};
              end
            endfunction

            // -----------------------------------------------------------------------------
            // [FUNCTION] 辅助函数: digit4
            // 把 0..2400 左右的脉宽拆成四个 ASCII 十进制数字。
            // -----------------------------------------------------------------------------
            function [7:0] digit4;
              input [11:0] value;  // 12 bit 函数 `digit4` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              input [1:0] index;  // 2 bit 函数 `digit4` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              integer digit;  // index 指定位置最终得到的 0..9 十进制数字；函数末尾加 ASCII 字符 '0' 后返回。 该变量只在本次函数求值期间存在，不保存跨周期状态。
              integer remainder;  // 逐级减去千、百、十位权值后的剩余数；最终剩余值直接作为个位。 该变量只在本次函数求值期间存在，不保存跨周期状态。
              begin
                remainder = value;
                if (remainder >= 2000) begin
                  remainder = remainder - 2000;
                  if (index == 2'd0) digit = 2;
                end else if (remainder >= 1000) begin
                  remainder = remainder - 1000;
                  if (index == 2'd0) digit = 1;
                end else if (index == 2'd0) begin
                  digit = 0;
                end

                if (index != 2'd0) begin
                  if (remainder >= 900) begin remainder = remainder - 900; digit = 9; end
                  else if (remainder >= 800) begin remainder = remainder - 800; digit = 8; end
                  else if (remainder >= 700) begin remainder = remainder - 700; digit = 7; end
                  else if (remainder >= 600) begin remainder = remainder - 600; digit = 6; end
                  else if (remainder >= 500) begin remainder = remainder - 500; digit = 5; end
                  else if (remainder >= 400) begin remainder = remainder - 400; digit = 4; end
                  else if (remainder >= 300) begin remainder = remainder - 300; digit = 3; end
                  else if (remainder >= 200) begin remainder = remainder - 200; digit = 2; end
                  else if (remainder >= 100) begin remainder = remainder - 100; digit = 1; end
                  else digit = 0;
                end

                if (index >= 2'd2) begin
                  if (remainder >= 90) begin remainder = remainder - 90; digit = 9; end
                  else if (remainder >= 80) begin remainder = remainder - 80; digit = 8; end
                  else if (remainder >= 70) begin remainder = remainder - 70; digit = 7; end
                  else if (remainder >= 60) begin remainder = remainder - 60; digit = 6; end
                  else if (remainder >= 50) begin remainder = remainder - 50; digit = 5; end
                  else if (remainder >= 40) begin remainder = remainder - 40; digit = 4; end
                  else if (remainder >= 30) begin remainder = remainder - 30; digit = 3; end
                  else if (remainder >= 20) begin remainder = remainder - 20; digit = 2; end
                  else if (remainder >= 10) begin remainder = remainder - 10; digit = 1; end
                  else digit = 0;
                end

                case (index)
                  2'd3: digit = remainder;
                  default: begin end
                endcase
                digit4 = 8'd48 + digit[7:0];
              end
            endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: frame_char
          // 按发送索引即时生成 62 字节运动帧或 9 字节停车帧字符。
          // -----------------------------------------------------------------------------
          function [7:0] frame_char;
              input [6:0] index;  // 7 bit 函数 `frame_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              begin
              if (frame_is_stop) begin
                case (index)
                  7'd0: frame_char = "#";
                  7'd1: frame_char = "2";
                  7'd2, 7'd3: frame_char = "5";
                  7'd4: frame_char = "P";
                  7'd5: frame_char = "D";
                  7'd6: frame_char = "S";
                  7'd7: frame_char = "T";
                  default: frame_char = "!";
                endcase
              end else begin
                case (index)
                  7'd0:  frame_char = "{";
                  7'd1, 7'd16, 7'd31, 7'd46: frame_char = "#";
                  7'd2, 7'd3, 7'd17, 7'd18, 7'd32, 7'd33, 7'd47, 7'd48: frame_char = "0";
                  7'd4:  frame_char = "6";
                  7'd19: frame_char = "7";
                  7'd34: frame_char = "8";
                  7'd49: frame_char = "9";
                  7'd5, 7'd20, 7'd35, 7'd50: frame_char = "P";
                  7'd6:  frame_char = digit4(frame_lf_pulse, 2'd0);
                  7'd7:  frame_char = digit4(frame_lf_pulse, 2'd1);
                  7'd8:  frame_char = digit4(frame_lf_pulse, 2'd2);
                  7'd9:  frame_char = digit4(frame_lf_pulse, 2'd3);
                  7'd21: frame_char = digit4(frame_rf_pulse, 2'd0);
                  7'd22: frame_char = digit4(frame_rf_pulse, 2'd1);
                  7'd23: frame_char = digit4(frame_rf_pulse, 2'd2);
                  7'd24: frame_char = digit4(frame_rf_pulse, 2'd3);
                  7'd36: frame_char = digit4(frame_lr_pulse, 2'd0);
                  7'd37: frame_char = digit4(frame_lr_pulse, 2'd1);
                  7'd38: frame_char = digit4(frame_lr_pulse, 2'd2);
                  7'd39: frame_char = digit4(frame_lr_pulse, 2'd3);
                  7'd51: frame_char = digit4(frame_rr_pulse, 2'd0);
                  7'd52: frame_char = digit4(frame_rr_pulse, 2'd1);
                  7'd53: frame_char = digit4(frame_rr_pulse, 2'd2);
                  7'd54: frame_char = digit4(frame_rr_pulse, 2'd3);
                  7'd10, 7'd25, 7'd40, 7'd55: frame_char = "T";
                  7'd11, 7'd12, 7'd13, 7'd14,
                  7'd26, 7'd27, 7'd28, 7'd29,
                  7'd41, 7'd42, 7'd43, 7'd44,
                  7'd56, 7'd57, 7'd58, 7'd59: frame_char = "0";
                  7'd15, 7'd30, 7'd45, 7'd60: frame_char = "!";
                default: frame_char = "}";
                endcase
              end
              end
            endfunction

            // -----------------------------------------------------------------------------
            // [PROCESS] 过程块 1
            // C5 UART 与帧调度联合状态机：先完成当前字节/帧，再处理停车或新运动命令。
            // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
            // -----------------------------------------------------------------------------
            always @(posedge clk or negedge rst_n) begin
              if (!rst_n) begin
                last_lf_speed <= 11'h7ff;
                last_rf_speed <= 11'h7ff;
                last_lr_speed <= 11'h7ff;
                last_rr_speed <= 11'h7ff;
                frame_lf_pulse <= 12'd1500;
                frame_rf_pulse <= 12'd1500;
                frame_lr_pulse <= 12'd1500;
                frame_rr_pulse <= 12'd1500;
                refresh_count <= 32'd0;
                send_index <= 7'd0;
                uart_data <= 8'd0;
                uart_start <= 1'b0;
                uart_tx <= 1'b1;
                uart_busy <= 1'b0;
                uart_busy_d <= 1'b0;
                uart_state <= UART_IDLE;
                uart_clk_count <= 16'd0;
                uart_bit_index <= 3'd0;
                uart_shifter <= 8'd0;
                sending <= 1'b0;
                pending_send <= 1'b0;
                frame_is_stop <= 1'b0;
                startup_count <= 25'd0;
                startup_ready <= 1'b0;
                frame_busy <= 1'b0;
                frame_count <= 16'd0;
                last_lf_pulse <= 12'd1500;
                last_rf_pulse <= 12'd1500;
                last_lr_pulse <= 12'd1500;
                last_rr_pulse <= 12'd1500;
                stop_frames_complete <= 1'b0;
                stop_frames_remaining <= 2'd0;
              end else begin
                uart_start <= 1'b0;
                uart_busy_d <= uart_busy;

                case (uart_state)
                  UART_IDLE: begin
                    uart_tx <= 1'b1;
                    uart_busy <= 1'b0;
                    uart_clk_count <= 16'd0;
                    uart_bit_index <= 3'd0;
                    if (uart_start) begin
                      uart_busy <= 1'b1;
                      uart_shifter <= uart_data;
                      uart_state <= UART_START;
                      uart_tx <= 1'b0;
                    end
                  end

                  UART_START: begin
                    uart_busy <= 1'b1;
                    uart_tx <= 1'b0;
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      uart_state <= UART_DATA;
                      uart_tx <= uart_shifter[0];
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end

                  UART_DATA: begin
                    uart_busy <= 1'b1;
                    uart_tx <= uart_shifter[uart_bit_index];
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      if (uart_bit_index == 3'd7) begin
                        uart_bit_index <= 3'd0;
                        uart_state <= UART_STOP;
                        uart_tx <= 1'b1;
                      end else begin
                        uart_bit_index <= uart_bit_index + 3'd1;
                        uart_tx <= uart_shifter[uart_bit_index + 3'd1];
                      end
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end

                  default: begin
                    uart_busy <= 1'b1;
                    uart_tx <= 1'b1;
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      uart_busy <= 1'b0;
                      uart_state <= UART_IDLE;
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end
                endcase

                // -----------------------------------------------------------------------------
                // 关键分支
                // 启动保护阶段：任何运动输入都不会发出，先等待 500 ms。
                // -----------------------------------------------------------------------------
                if (!startup_ready) begin
                  refresh_count <= 32'd0;
                  frame_busy <= 1'b0;
                  if (startup_count >= STARTUP_CYCLES - 1) begin
                    startup_count <= startup_count;
                    startup_ready <= 1'b1;
                    stop_frames_remaining <= 2'd3;
                    stop_frames_complete <= 1'b0;
                  end else begin
                    startup_count <= startup_count + 25'd1;
                  end
              end else if (!sending && any_motion && !refresh_due) begin
                refresh_count <= refresh_count + 32'd1;
              end else if (!sending && !any_motion) begin
                refresh_count <= 32'd0;
                end

                // -----------------------------------------------------------------------------
                // 关键分支
                // 检测强制停止或运动到零速边沿，重新装载三帧停车序列。
                // -----------------------------------------------------------------------------
                if (startup_ready && (force_stop || stop_transition)) begin
                  stop_frames_remaining <= 2'd3;
                  stop_frames_complete <= 1'b0;
                end

                if (startup_ready && !sending) begin
                  frame_busy <= 1'b0;
                  if ((force_stop || (stop_frames_remaining != 2'd0) ||
                       pending_send || command_changed || refresh_due) && !uart_busy) begin
                    // -----------------------------------------------------------------------------
                    // 关键分支
                    // 停车帧具有优先级：锁存四轮 1500 中位值并选择 #255PDST!。
                    // -----------------------------------------------------------------------------
                    if (send_stop_now) begin
                      frame_lf_pulse <= 12'd1500;
                      frame_rf_pulse <= 12'd1500;
                      frame_lr_pulse <= 12'd1500;
                      frame_rr_pulse <= 12'd1500;
                      last_lf_pulse <= 12'd1500;
                      last_rf_pulse <= 12'd1500;
                      last_lr_pulse <= 12'd1500;
                      last_rr_pulse <= 12'd1500;
                      last_lf_speed <= 11'd0;
                      last_rf_speed <= 11'd0;
                      last_lr_speed <= 11'd0;
                      last_rr_speed <= 11'd0;
                      frame_is_stop <= 1'b1;
                    end else begin
                      frame_lf_pulse <= left_motor_pulse(effective_lf_speed);
                      frame_rf_pulse <= right_motor_pulse(effective_rf_speed);
                      frame_lr_pulse <= left_motor_pulse(effective_lr_speed);
                      frame_rr_pulse <= right_motor_pulse(effective_rr_speed);
                      last_lf_pulse <= left_motor_pulse(effective_lf_speed);
                      last_rf_pulse <= right_motor_pulse(effective_rf_speed);
                      last_lr_pulse <= left_motor_pulse(effective_lr_speed);
                      last_rr_pulse <= right_motor_pulse(effective_rr_speed);
                      last_lf_speed <= effective_lf_speed;
                      last_rf_speed <= effective_rf_speed;
                      last_lr_speed <= effective_lr_speed;
                      last_rr_speed <= effective_rr_speed;
                      frame_is_stop <= 1'b0;
                      stop_frames_complete <= 1'b0;
                    end
                    refresh_count <= 32'd0;
                    pending_send <= 1'b0;
                    sending <= 1'b1;
                    frame_busy <= 1'b1;
                    send_index <= 7'd0;
                uart_data <= send_stop_now ? "#" : "{";
                    uart_start <= 1'b1;
                  end
                end else if (startup_ready) begin
                  frame_busy <= 1'b1;
                  if (uart_done) begin
                  if (send_index == active_frame_last) begin
                      sending <= 1'b0;
                      frame_busy <= 1'b0;
                      frame_count <= frame_count + 16'd1;
                      if (frame_is_stop) begin
                        if (stop_frames_remaining > 2'd1) begin
                          stop_frames_remaining <= stop_frames_remaining - 2'd1;
                        end else begin
                          stop_frames_remaining <= 2'd0;
                          stop_frames_complete <= 1'b1;
                        end
                      end
                    end else begin
                      send_index <= send_index + 7'd1;
                      uart_data <= frame_char(send_index + 7'd1);
                      uart_start <= 1'b1;
                    end
                  end
                end
              end
            end

endmodule
