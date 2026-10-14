// =============================================================================
// 模块: uart_tx_8n1
// 作用: 通用单字节 UART 8N1 发送器。
// 说明: start 接收一个周期脉冲后锁存 data，依次发送起始位、8 个数据位和停止位。
//
// Robei 源: little_car/model/uart_tx_8n1.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module uart_tx_8n1 (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire start,  // 启动发送 data 的单周期请求；busy=1 时请求不会开启第二字节。
  input wire [7:0] data,  // start 有效时锁存并按 LSB-first 发送的 8 bit 字节。
  output reg tx,  // 8N1 串行输出；空闲/停止位为高，起始位为低。
  output reg busy  // 从锁存 start 到停止位结束期间保持为 1 的发送占用标志。
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

  localparam [1:0] STATE_IDLE  = 2'd0;  // UART 空闲状态编码；tx 保持高电平，busy=0，并等待一个 start 脉冲锁存 data。
  localparam [1:0] STATE_START = 2'd1;  // UART 起始位状态编码；tx 连续保持低电平一个 CLKS_PER_BIT 时间。
  localparam [1:0] STATE_DATA  = 2'd2;  // UART 数据位状态编码；按 bit_index=0..7 以 LSB-first 顺序输出 shifter。
  localparam [1:0] STATE_STOP  = 2'd3;  // UART 停止位状态编码；tx 保持高电平一个 bit 后清除 busy 并回到 IDLE。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [1:0] state;  // UART 字节发送状态：IDLE 等待 start，START 保持起始低位，DATA 依次发送 bit0..bit7，STOP 保持停止高位。
  reg [15:0] clk_count;  // 当前 UART bit 已保持的 50 MHz 周期数；达到 CLKS_PER_BIT-1 时进入下一数据位或状态。
  reg [2:0] bit_index;  // 当前正在发送的数据位编号 0..7；从 shifter 的最低位开始按 LSB-first 递增。
  reg [7:0] shifter;  // start 被接受时锁存的 8 bit data；整个字节发送期间保持不变，避免上游改写破坏帧。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 tx（发送）、busy（忙标志）、state（状态）、clk_count（时钟 / 计数）、bit_index（BIT / 索引）、shifter（shifter）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx <= 1'b1;
      busy <= 1'b0;
      state <= STATE_IDLE;
      clk_count <= 16'd0;
      bit_index <= 3'd0;
      shifter <= 8'd0;
    end else begin
      case (state)
        STATE_IDLE: begin
          tx <= 1'b1;
          busy <= 1'b0;
          clk_count <= 16'd0;
          bit_index <= 3'd0;
          if (start) begin
            busy <= 1'b1;
            shifter <= data;
            state <= STATE_START;
            tx <= 1'b0;
          end
        end

        STATE_START: begin
          busy <= 1'b1;
          tx <= 1'b0;
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            state <= STATE_DATA;
            tx <= shifter[0];
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end

        STATE_DATA: begin
          busy <= 1'b1;
          tx <= shifter[bit_index];
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            if (bit_index == 3'd7) begin
              bit_index <= 3'd0;
              state <= STATE_STOP;
              tx <= 1'b1;
            end else begin
              bit_index <= bit_index + 3'd1;
              tx <= shifter[bit_index + 3'd1];
            end
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end

        default: begin
          busy <= 1'b1;
          tx <= 1'b1;
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            busy <= 1'b0;
            state <= STATE_IDLE;
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end
      endcase
    end
  end

endmodule
