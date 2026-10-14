// =============================================================================
// 模块: linkpg_uwb_bridge
// 作用: LinkPG 标签侧双向桥，把 UWB 数据透传帧与现有蓝牙单字符协议无侵入连接。
// 说明: UWB 与蓝牙侧均为 115200 bit/s 8N1；蓝牙物理 RX 和 UWB 注入线按线与合并。
// 说明: 下行只接受带序号和 CRC8 的白名单单字符命令，并用两秒窗口抑制重复执行。
// 说明: 仅在蓝牙 RX 连续空闲至少 12 bit 后注入命令，100 ms 等待仍冲突则返回失败 ACK。
// 说明: 上行捕获完整 @S/@L/@K/@PONG 行，最多 96 字节并拆成两段 LinkPG 透传载荷。
// 说明: 所有 LinkPG Modbus 帧校验 CRC16；截断帧空闲 10 ms 后自动复位解析器。
//
// Robei 源: little_car/model/linkpg_uwb_bridge.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module linkpg_uwb_bridge (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire uwb_uart_rx,  // LinkPG 标签 TXD 到 FPGA 的 115200 bit/s 数据透传输入。
  input wire bt_uart_rx_in,  // CH9141K 物理 RX 电平在进入 UWB 合并器前的原始输入。
  input wire bt_uart_tx_tap,  // 旁听 FPGA 蓝牙 TX 的 UART 电平，用于捕获完整状态/确认行并经 UWB 回传。
  output wire uwb_uart_tx,  // FPGA 到 LinkPG 标签 RXD 的 Modbus 数据透传/ACK 输出。
  output wire bt_uart_rx_merged,  // CH9141K RX 与 UWB 命令注入线按线与后的 UART 输入，送入 bt_route_uart。
  output reg uwb_link_online,  // 最近三秒内收到过 CRC 正确 LinkPG 帧时保持为 1。
  output reg [15:0] rx_packet_count,  // 已接收且通过 LinkPG Modbus CRC16 校验的帧累计数。
  output reg [15:0] rx_crc_error_count,  // 长度完整但 CRC16 错误的 LinkPG 接收帧累计数。
  output reg [15:0] tx_packet_count,  // 已经完整发往 LinkPG UART 的 Modbus 帧累计数。
  output reg [15:0] telemetry_drop_count  // 上行状态行过长、捕获冲突或队列占满造成的遥测丢弃累计数。
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
  localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;  // 本模块固定参数“HALF / BIT / CLKS”，RTL 取值为 CLKS_PER_BIT / 2；用于统一相关比较和计算。
  localparam integer BT_IDLE_GUARD_CLKS = CLKS_PER_BIT * 12;  // 本模块固定参数“BT / IDLE / GUARD / CLKS”，RTL 取值为 CLKS_PER_BIT * 12；用于统一相关比较和计算。
  localparam integer COMMAND_WAIT_CLKS = CLK_HZ / 10;  // 本模块固定参数“COMMAND / WAIT / CLKS”，RTL 取值为 CLK_HZ / 10；用于统一相关比较和计算。
  localparam integer DUPLICATE_WINDOW_CLKS = CLK_HZ * 2;  // 本模块固定参数“DUPLICATE / WINDOW / CLKS”，RTL 取值为 CLK_HZ * 2；用于统一相关比较和计算。
  localparam integer UWB_LINK_TIMEOUT_CLKS = CLK_HZ * 3;  // 本模块固定参数“UWB / LINK / TIMEOUT / CLKS”，RTL 取值为 CLK_HZ * 3；用于统一相关比较和计算。
  localparam integer FRAME_IDLE_TIMEOUT_CLKS = CLK_HZ / 100;  // 本模块固定参数“FRAME / IDLE / TIMEOUT / CLKS”，RTL 取值为 CLK_HZ / 100；用于统一相关比较和计算。
  localparam integer TX_GAP_CLKS = CLK_HZ / 40;  // UART 发送状态“TX / GAP / CLKS”的编码。

  localparam [1:0] UART_IDLE  = 2'd0;  // UART 发送状态“UART / IDLE”的编码。
  localparam [1:0] UART_START = 2'd1;  // UART 发送状态“UART / START”的编码。
  localparam [1:0] UART_DATA  = 2'd2;  // UART 发送状态“UART / DATA”的编码。
  localparam [1:0] UART_STOP  = 2'd3;  // UART 发送状态“UART / STOP”的编码。

  localparam [2:0] BUILD_IDLE        = 3'd0;  // 本模块固定参数“BUILD / IDLE”，RTL 取值为 3'd0；用于统一相关比较和计算。
  localparam [2:0] BUILD_REPORT_DATA = 3'd1;  // 本模块固定参数“BUILD / REPORT / DATA”，RTL 取值为 3'd1；用于统一相关比较和计算。
  localparam [2:0] BUILD_REPORT_CRC  = 3'd2;  // 本模块固定参数“BUILD / REPORT / CRC”，RTL 取值为 3'd2；用于统一相关比较和计算。

  localparam [2:0] STREAM_IDLE    = 3'd0;  // 本模块固定参数“STREAM / IDLE”，RTL 取值为 3'd0；用于统一相关比较和计算。
  localparam [2:0] STREAM_PREFIX  = 3'd1;  // 本模块固定参数“STREAM / PREFIX”，RTL 取值为 3'd1；用于统一相关比较和计算。
  localparam [2:0] STREAM_PAYLOAD = 3'd2;  // 本模块固定参数“STREAM / PAYLOAD”，RTL 取值为 3'd2；用于统一相关比较和计算。
  localparam [2:0] STREAM_SUFFIX  = 3'd3;  // 本模块固定参数“STREAM / SUFFIX”，RTL 取值为 3'd3；用于统一相关比较和计算。
  localparam [2:0] STREAM_WAIT    = 3'd4;  // 本模块固定参数“STREAM / WAIT”，RTL 取值为 3'd4；用于统一相关比较和计算。

  localparam [1:0] PAYLOAD_NONE   = 2'd0;  // 本模块固定参数“PAYLOAD / NONE”，RTL 取值为 2'd0；用于统一相关比较和计算。
  localparam [1:0] PAYLOAD_ACK    = 2'd1;  // 本模块固定参数“PAYLOAD / ACK”，RTL 取值为 2'd1；用于统一相关比较和计算。
  localparam [1:0] PAYLOAD_REPORT = 2'd2;  // 本模块固定参数“PAYLOAD / REPORT”，RTL 取值为 2'd2；用于统一相关比较和计算。

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: crc16_modbus_next
  // 逐字节更新 LinkPG 帧使用的 Modbus CRC16（反射多项式 A001）。
  // -----------------------------------------------------------------------------
  function [15:0] crc16_modbus_next;
    input [15:0] crc_in;  // 16 bit 函数 `crc16_modbus_next` 的“CRC / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [7:0] data_in;  // 8 bit 函数 `crc16_modbus_next` 的“数据 / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    // -----------------------------------------------------------------------------
    // 内部网络、寄存器和计数器
    // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
    // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
    // -----------------------------------------------------------------------------
    integer bit_index;  // 函数 `crc16_modbus_next` 内部的整数临时量“BIT / 索引”；用阻塞赋值完成循环、除法替代或坐标中间运算，不会综合成独立状态寄存器。
    reg [15:0] value;  // 16 bit 函数 `crc16_modbus_next` 内部的组合临时变量“输入数值”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[0])
          value = (value >> 1) ^ 16'hA001;
        else
          value = value >> 1;
      end
      crc16_modbus_next = value;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: crc8_next
  // 逐字节更新命令/ACK/遥测载荷内部使用的 CRC8（多项式 07）。
  // -----------------------------------------------------------------------------
  function [7:0] crc8_next;
    input [7:0] crc_in;  // 8 bit 函数 `crc8_next` 的“CRC / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [7:0] data_in;  // 8 bit 函数 `crc8_next` 的“数据 / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    integer bit_index;  // 函数 `crc8_next` 内部的整数临时量“BIT / 索引”；用阻塞赋值完成循环、除法替代或坐标中间运算，不会综合成独立状态寄存器。
    reg [7:0] value;  // 8 bit 函数 `crc8_next` 内部的组合临时变量“输入数值”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[7])
          value = (value << 1) ^ 8'h07;
        else
          value = value << 1;
      end
      crc8_next = value;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: is_hex_ascii
  // 判断载荷字符是否为十六进制 0-9/A-F/a-f。
  // -----------------------------------------------------------------------------
  function is_hex_ascii;
    input [7:0] value;  // 8 bit 函数 `is_hex_ascii` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      is_hex_ascii = ((value >= "0") && (value <= "9")) ||
                     ((value >= "A") && (value <= "F")) ||
                     ((value >= "a") && (value <= "f"));
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex_value
  // 把十六进制 ASCII 字符转换为 4 bit 数值。
  // -----------------------------------------------------------------------------
  function [3:0] hex_value;
    input [7:0] value;  // 8 bit 函数 `hex_value` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if ((value >= "0") && (value <= "9"))
        hex_value = value - "0";
      else if ((value >= "A") && (value <= "F"))
        hex_value = value - "A" + 4'd10;
      else
        hex_value = value - "a" + 4'd10;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex_ascii
  // 把 4 bit 数值转换为大写十六进制 ASCII。
  // -----------------------------------------------------------------------------
  function [7:0] hex_ascii;
    input [3:0] value;  // 4 bit 函数 `hex_ascii` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if (value < 10)
        hex_ascii = "0" + value;
      else
        hex_ascii = "A" + value - 4'd10;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: allowed_command
  // UWB 下行白名单，只允许现有蓝牙协议已经定义的单字符命令。
  // -----------------------------------------------------------------------------
  function allowed_command;
    input [7:0] value;  // 8 bit 函数 `allowed_command` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (value)
        "P", "p", "?", "W", "U", "S", "D", "A", "L", "R", "C", "M",
        "H", "O", "E", "Q", "V", "T", "1", "2", "3", "4", "5", "Z",
        "+", "-", "N", "B", "G": allowed_command = 1'b1;
        default: allowed_command = 1'b0;
      endcase
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: command_crc8
  // 计算 C+序号+命令三个字节的下行命令 CRC8。
  // -----------------------------------------------------------------------------
  function [7:0] command_crc8;
    input [7:0] sequence_in;  // 8 bit 函数 `command_crc8` 的“sequence / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [7:0] command_byte;  // 8 bit 函数 `command_crc8` 的“命令 / byte”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    reg [7:0] value;  // 8 bit 函数 `command_crc8` 内部的组合临时变量“输入数值”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      value = crc8_next(8'h00, "C");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, command_byte);
      command_crc8 = value;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: ack_crc8
  // 计算 A+序号+命令+结果四字节 ACK 的 CRC8。
  // -----------------------------------------------------------------------------
  function [7:0] ack_crc8;
    input [7:0] sequence_in;  // 8 bit 函数 `ack_crc8` 的“sequence / IN”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [7:0] command_byte;  // 8 bit 函数 `ack_crc8` 的“命令 / byte”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [1:0] result;  // 2 bit 函数 `ack_crc8` 的“result”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    reg [7:0] value;  // 8 bit 函数 `ack_crc8` 内部的组合临时变量“输入数值”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      value = crc8_next(8'h00, "A");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, command_byte);
      value = crc8_next(value, "0" + result);
      ack_crc8 = value;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: prefix_byte
  // 按字节索引生成 LinkPG Modbus 数据透传请求头。
  // -----------------------------------------------------------------------------
  function [7:0] prefix_byte;
    input [3:0] index;  // 4 bit 函数 `prefix_byte` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (index)
        4'd0: prefix_byte = "A";
        4'd1: prefix_byte = "T";
        4'd2: prefix_byte = "+";
        4'd3: prefix_byte = "D";
        4'd4: prefix_byte = "a";
        4'd5: prefix_byte = "t";
        4'd6: prefix_byte = "a";
        4'd7: prefix_byte = "S";
        4'd8: prefix_byte = "e";
        4'd9: prefix_byte = "n";
        4'd10: prefix_byte = "d";
        4'd11: prefix_byte = "=";
        default: prefix_byte = 8'h22;
      endcase
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: suffix_byte
  // 按当前载荷和 CRC 累计值生成 Modbus 帧尾 CRC16。
  // -----------------------------------------------------------------------------
  function [7:0] suffix_byte;
    input [3:0] index;  // 4 bit 函数 `suffix_byte` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (index)
        4'd0: suffix_byte = 8'h22;
        4'd1: suffix_byte = ",";
        4'd2: suffix_byte = 8'h22;
        4'd3: suffix_byte = "2";
        4'd4: suffix_byte = "5";
        4'd5: suffix_byte = "5";
        4'd6: suffix_byte = 8'h22;
        4'd7: suffix_byte = 8'h0D;
        default: suffix_byte = 8'h0A;
      endcase
    end
  endfunction



  reg [1:0] uwb_rx_sync;  // 2 bit 寄存器，保存“UWB / 接收 / 同步值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [1:0] uwb_rx_state;  // 2 bit 用于“UWB / 接收”的状态寄存器；决定状态机当前执行的分支和下一状态。
  reg [15:0] uwb_rx_clock;  // 16 bit 寄存器，保存“UWB / 接收 / clock”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [2:0] uwb_rx_bit;  // 3 bit 寄存器，保存“UWB / 接收 / BIT”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_rx_shift;  // 8 bit 寄存器，保存“UWB / 接收 / 移位缓存”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_rx_byte;  // 8 bit 寄存器，保存“UWB / 接收 / byte”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg uwb_rx_valid;  // “UWB / 接收 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // LinkPG UART 接收器：同步异步 RX，按 115200 8N1 半位确认并产生单字节有效脉冲。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_rx_sync <= 2'b11;
      uwb_rx_state <= UART_IDLE;
      uwb_rx_clock <= 16'd0;
      uwb_rx_bit <= 3'd0;
      uwb_rx_shift <= 8'd0;
      uwb_rx_byte <= 8'd0;
      uwb_rx_valid <= 1'b0;
    end else begin
      uwb_rx_sync <= {uwb_rx_sync[0], uwb_uart_rx};
      uwb_rx_valid <= 1'b0;
      case (uwb_rx_state)
        UART_IDLE: begin
          uwb_rx_clock <= 16'd0;
          uwb_rx_bit <= 3'd0;
          if (!uwb_rx_sync[1])
            uwb_rx_state <= UART_START;
        end
        UART_START: begin
          if (uwb_rx_clock >= HALF_BIT_CLKS - 1) begin
            uwb_rx_clock <= 16'd0;
            if (!uwb_rx_sync[1])
              uwb_rx_state <= UART_DATA;
            else
              uwb_rx_state <= UART_IDLE;
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (uwb_rx_clock >= CLKS_PER_BIT - 1) begin
            uwb_rx_clock <= 16'd0;
            uwb_rx_shift[uwb_rx_bit] <= uwb_rx_sync[1];
            if (uwb_rx_bit == 3'd7) begin
              uwb_rx_bit <= 3'd0;
              uwb_rx_state <= UART_STOP;
            end else begin
              uwb_rx_bit <= uwb_rx_bit + 3'd1;
            end
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
        default: begin
          if (uwb_rx_clock >= CLKS_PER_BIT - 1) begin
            uwb_rx_clock <= 16'd0;
            uwb_rx_state <= UART_IDLE;
            if (uwb_rx_sync[1]) begin
              uwb_rx_byte <= uwb_rx_shift;
              uwb_rx_valid <= 1'b1;
            end
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
      endcase
    end
  end



  reg [6:0] uwb_frame_index;  // 当前 LinkPG Modbus 帧已经接收的字节下标。
  reg [6:0] uwb_frame_expected;  // 从功能码和长度字段推导出的完整 Modbus 帧字节数。
  reg [7:0] uwb_frame_byte_count;  // 当前帧数据区声明的载荷字节数。
  reg [15:0] uwb_frame_crc;  // 从帧首开始逐字节累计的 Modbus CRC16。
  reg [7:0] uwb_marker_high;  // 8 bit 寄存器，保存“UWB / 标记 / high”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_marker_low;  // 8 bit 寄存器，保存“UWB / 标记 / LOW”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_sender_high;  // 8 bit 寄存器，保存“UWB / sender / high”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_sender_low;  // 8 bit 寄存器，保存“UWB / sender / LOW”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uwb_payload [0:7];  // 缓存下行 CxxccH 命令载荷的 8 字节数组。
  reg [18:0] uwb_frame_idle_count;  // 不完整帧最后一个字节后的空闲周期数；10 ms 后丢弃截断帧并重新找帧首。

  reg new_command_valid;  // 解析出 CRC 正确且通过白名单的新命令时产生的单周期脉冲。
  reg [7:0] new_command_sequence;  // 下行命令携带的 8 bit 序号，用于 ACK 和重复抑制。
  reg [7:0] new_command_byte;  // 准备注入现有 bt_route_uart 的单个 ASCII 命令。
  reg parser_ack_valid;  // “parser / ACK / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  reg [7:0] parser_ack_sequence;  // 8 bit 寄存器，保存“parser / ACK / sequence”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] parser_ack_command;  // 8 bit 寄存器，保存“parser / ACK / 命令”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [1:0] parser_ack_result;  // 2 bit 寄存器，保存“parser / ACK / result”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  reg last_command_valid;  // “最近一次 / 命令 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  reg [7:0] last_command_sequence;  // 最近一次已接受命令的序号；两秒窗口内相同序号不会再次执行。
  reg [7:0] last_command_byte;  // 8 bit 寄存器，保存“最近一次 / 命令 / byte”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [26:0] duplicate_age;  // 最近命令距离当前的周期数，用于界定重复抑制有效窗口。

  wire [15:0] received_crc_next = crc16_modbus_next(uwb_frame_crc, uwb_rx_byte);  // 16 bit 组合网络，传递“received / CRC / 下一值”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] received_command_sequence = {hex_value(uwb_payload[2]), hex_value(uwb_payload[3])};  // 8 bit 组合网络，传递“received / 命令 / sequence”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] received_command_crc = {hex_value(uwb_payload[5]), hex_value(uwb_payload[6])};  // 8 bit 组合网络，传递“received / 命令 / CRC”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire received_command_format_ok =  // 组合网络，传递“received / 命令 / format / OK”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
      (uwb_frame_byte_count >= 8'd11) &&
      (uwb_payload[0] == "C") && (uwb_payload[1] == "1") &&
      is_hex_ascii(uwb_payload[2]) && is_hex_ascii(uwb_payload[3]) &&
      is_hex_ascii(uwb_payload[5]) && is_hex_ascii(uwb_payload[6]) &&
      allowed_command(uwb_payload[4]) &&
      (received_command_crc == command_crc8(received_command_sequence, uwb_payload[4]));

  wire command_accept_ready;  // “命令 / accept / 就绪”控制标志；为 1 表示名称描述的状态当前成立。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // Modbus 帧解析器：识别长度、数据透传标记、目标 ID、CRC16 和 CxxccH 命令载荷。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_frame_index <= 7'd0;
      uwb_frame_expected <= 7'd0;
      uwb_frame_byte_count <= 8'd0;
      uwb_frame_crc <= 16'hFFFF;
      uwb_marker_high <= 8'd0;
      uwb_marker_low <= 8'd0;
      uwb_sender_high <= 8'd0;
      uwb_sender_low <= 8'd0;
      uwb_frame_idle_count <= 19'd0;
      new_command_valid <= 1'b0;
      new_command_sequence <= 8'd0;
      new_command_byte <= 8'd0;
      parser_ack_valid <= 1'b0;
      parser_ack_sequence <= 8'd0;
      parser_ack_command <= 8'd0;
      parser_ack_result <= 2'd0;
      last_command_valid <= 1'b0;
      last_command_sequence <= 8'd0;
      last_command_byte <= 8'd0;
      duplicate_age <= 27'd0;
      rx_packet_count <= 16'd0;
      rx_crc_error_count <= 16'd0;
    end else begin
      new_command_valid <= 1'b0;
      parser_ack_valid <= 1'b0;

      if (last_command_valid) begin
        if (duplicate_age >= DUPLICATE_WINDOW_CLKS - 1) begin
          last_command_valid <= 1'b0;
          duplicate_age <= 27'd0;
        end else begin
          duplicate_age <= duplicate_age + 27'd1;
        end
      end





      if ((uwb_frame_index != 0) && !uwb_rx_valid) begin
        if (uwb_frame_idle_count >= FRAME_IDLE_TIMEOUT_CLKS - 1) begin
          uwb_frame_index <= 7'd0;
          uwb_frame_expected <= 7'd0;
          uwb_frame_byte_count <= 8'd0;
          uwb_frame_crc <= 16'hFFFF;
          uwb_frame_idle_count <= 19'd0;
        end else begin
          uwb_frame_idle_count <= uwb_frame_idle_count + 19'd1;
        end
      end else begin
        uwb_frame_idle_count <= 19'd0;
      end

      if (uwb_rx_valid) begin
        if (uwb_frame_index == 0) begin
          if (uwb_rx_byte == 8'h01) begin
            uwb_frame_index <= 7'd1;
            uwb_frame_crc <= crc16_modbus_next(16'hFFFF, uwb_rx_byte);
          end
        end else if (uwb_frame_index == 1) begin
          if (uwb_rx_byte == 8'h03) begin
            uwb_frame_index <= 7'd2;
            uwb_frame_crc <= received_crc_next;
          end else begin
            uwb_frame_index <= (uwb_rx_byte == 8'h01) ? 7'd1 : 7'd0;
            uwb_frame_crc <= (uwb_rx_byte == 8'h01) ?
                             crc16_modbus_next(16'hFFFF, uwb_rx_byte) : 16'hFFFF;
          end
        end else if (uwb_frame_index == 2) begin
          if ((uwb_rx_byte >= 8'd4) && (uwb_rx_byte <= 8'd84)) begin
            uwb_frame_byte_count <= uwb_rx_byte;
            uwb_frame_expected <= uwb_rx_byte + 7'd5;
            uwb_frame_index <= 7'd3;
            uwb_frame_crc <= received_crc_next;
          end else begin
            uwb_frame_index <= 7'd0;
            uwb_frame_crc <= 16'hFFFF;
          end
        end else begin
          uwb_frame_crc <= received_crc_next;
          if (uwb_frame_index == 3) uwb_marker_high <= uwb_rx_byte;
          if (uwb_frame_index == 4) uwb_marker_low <= uwb_rx_byte;
          if (uwb_frame_index == 5) uwb_sender_high <= uwb_rx_byte;
          if (uwb_frame_index == 6) uwb_sender_low <= uwb_rx_byte;
          if ((uwb_frame_index >= 7) && (uwb_frame_index <= 14))
            uwb_payload[uwb_frame_index - 7] <= uwb_rx_byte;

          if (uwb_frame_index == uwb_frame_expected - 1) begin
            uwb_frame_index <= 7'd0;
            uwb_frame_crc <= 16'hFFFF;
            if (received_crc_next != 16'h0000) begin
              rx_crc_error_count <= rx_crc_error_count + 16'd1;
            end else if ((uwb_marker_high == 8'hED) && (uwb_marker_low == 8'hDA) &&
                         (uwb_sender_high == 8'h00) && (uwb_sender_low == 8'hFF)) begin
              rx_packet_count <= rx_packet_count + 16'd1;
              if (received_command_format_ok) begin
                if (last_command_valid &&
                    (received_command_sequence == last_command_sequence) &&
                    (uwb_payload[4] == last_command_byte)) begin
                  parser_ack_valid <= 1'b1;
                  parser_ack_sequence <= received_command_sequence;
                  parser_ack_command <= uwb_payload[4];
                  parser_ack_result <= 2'd1;
                end else if (command_accept_ready) begin
                  new_command_valid <= 1'b1;
                  new_command_sequence <= received_command_sequence;
                  new_command_byte <= uwb_payload[4];
                  last_command_valid <= 1'b1;
                  last_command_sequence <= received_command_sequence;
                  last_command_byte <= uwb_payload[4];
                  duplicate_age <= 27'd0;
                end else begin
                  parser_ack_valid <= 1'b1;
                  parser_ack_sequence <= received_command_sequence;
                  parser_ack_command <= uwb_payload[4];
                  parser_ack_result <= 2'd2;
                end
              end
            end
          end else begin
            uwb_frame_index <= uwb_frame_index + 7'd1;
          end
        end
      end
    end
  end



  reg [27:0] uwb_link_idle_count;  // 距离最近 CRC 正确 LinkPG 帧的周期数，达到三秒后清除在线标志。
  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 3
  // UWB 在线计时器：CRC 正确帧刷新三秒超时，超时后仅清在线标志而不影响本地安全链。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_link_online <= 1'b0;
      uwb_link_idle_count <= 28'd0;
    end else if (uwb_rx_valid && (uwb_frame_index == uwb_frame_expected - 1) &&
                 (received_crc_next == 16'h0000) &&
                 (uwb_marker_high == 8'hED) && (uwb_marker_low == 8'hDA)) begin
      uwb_link_online <= 1'b1;
      uwb_link_idle_count <= 28'd0;
    end else if (uwb_link_online) begin
      if (uwb_link_idle_count >= UWB_LINK_TIMEOUT_CLKS - 1) begin
        uwb_link_online <= 1'b0;
        uwb_link_idle_count <= 28'd0;
      end else begin
        uwb_link_idle_count <= uwb_link_idle_count + 28'd1;
      end
    end
  end





  reg command_pending;  // 已有一条 UWB 命令等待蓝牙 RX 空闲或正在注入。
  reg [7:0] pending_command_sequence;  // 注入过程锁存的命令序号，直到 ACK 完成前保持不变。
  reg [7:0] pending_command_byte;  // 注入过程锁存的 ASCII 字节。
  reg [23:0] command_wait_count;  // 等待蓝牙物理 RX 空闲的 100 ms 超时计数。
  reg [15:0] bt_idle_high_count;  // 蓝牙 RX 已连续保持高电平的周期数；至少 12 bit 才允许开始注入。
  reg [1:0] inject_state;  // 2 bit 用于“inject”的状态寄存器；决定状态机当前执行的分支和下一状态。
  reg [15:0] inject_clock;  // 16 bit 寄存器，保存“inject / clock”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [2:0] inject_bit;  // 3 bit 寄存器，保存“inject / BIT”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] inject_shift;  // 8 bit 寄存器，保存“inject / 移位缓存”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg inject_uart_line;  // 寄存器，保存“inject / UART / 输出线”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg inject_collision;  // 注入期间蓝牙物理 RX 被外部拉低，表示手机与 UWB 同时发送产生冲突。
  reg inject_ack_valid;  // “inject / ACK / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  reg [7:0] inject_ack_sequence;  // 8 bit 寄存器，保存“inject / ACK / sequence”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] inject_ack_command;  // 8 bit 寄存器，保存“inject / ACK / 命令”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [1:0] inject_ack_result;  // 2 bit 寄存器，保存“inject / ACK / result”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“command_accept_ready”：“命令 / accept / 就绪”控制标志；为 1 表示名称描述的状态当前成立。 该组合结果直接由 `command_pending`（命令 / 待处理）、`inject_state`（inject / 状态）、`UART_IDLE`（UART / IDLE）、`new_command_valid`（NEW / 命令 / 有效标志）的当前值决定，不引入时钟周期延迟。
  assign command_accept_ready = !command_pending && (inject_state == UART_IDLE) && !new_command_valid;
  // [ASSIGN] 组合定义“bt_uart_rx_merged”：CH9141K RX 与 UWB 命令注入线按线与后的 UART 输入，送入 bt_route_uart。 该组合结果直接由 `bt_uart_rx_in`（BT / UART / 接收 / IN）、`inject_uart_line`（inject / UART / 输出线）的当前值决定，不引入时钟周期延迟。
  assign bt_uart_rx_merged = bt_uart_rx_in & inject_uart_line;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 4
  // 命令注入器：等待蓝牙 RX 空闲，在位边界完整发送单字节并生成执行/冲突/超时 ACK。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      command_pending <= 1'b0;
      pending_command_sequence <= 8'd0;
      pending_command_byte <= 8'd0;
      command_wait_count <= 24'd0;
      bt_idle_high_count <= 16'd0;
      inject_state <= UART_IDLE;
      inject_clock <= 16'd0;
      inject_bit <= 3'd0;
      inject_shift <= 8'd0;
      inject_uart_line <= 1'b1;
      inject_collision <= 1'b0;
      inject_ack_valid <= 1'b0;
      inject_ack_sequence <= 8'd0;
      inject_ack_command <= 8'd0;
      inject_ack_result <= 2'd0;
    end else begin
      inject_ack_valid <= 1'b0;

      if ((inject_state == UART_IDLE) && bt_uart_rx_in) begin
        if (bt_idle_high_count < BT_IDLE_GUARD_CLKS)
          bt_idle_high_count <= bt_idle_high_count + 16'd1;
      end else begin
        bt_idle_high_count <= 16'd0;
      end

      if (new_command_valid && !command_pending) begin
        command_pending <= 1'b1;
        pending_command_sequence <= new_command_sequence;
        pending_command_byte <= new_command_byte;
        command_wait_count <= 24'd0;
      end

      if ((inject_state != UART_IDLE) && !bt_uart_rx_in)
        inject_collision <= 1'b1;

      case (inject_state)
        UART_IDLE: begin
          inject_uart_line <= 1'b1;
          inject_clock <= 16'd0;
          if (command_pending) begin
            if (bt_idle_high_count >= BT_IDLE_GUARD_CLKS - 1) begin
              inject_state <= UART_START;
              inject_clock <= 16'd0;
              inject_bit <= 3'd0;
              inject_shift <= pending_command_byte;
              inject_uart_line <= 1'b0;
              inject_collision <= 1'b0;
              command_wait_count <= 24'd0;
            end else if (command_wait_count >= COMMAND_WAIT_CLKS - 1) begin
              command_pending <= 1'b0;
              command_wait_count <= 24'd0;
              inject_ack_valid <= 1'b1;
              inject_ack_sequence <= pending_command_sequence;
              inject_ack_command <= pending_command_byte;
              inject_ack_result <= 2'd2;
            end else begin
              command_wait_count <= command_wait_count + 24'd1;
            end
          end
        end
        UART_START: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            inject_uart_line <= inject_shift[0];
            inject_state <= UART_DATA;
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            if (inject_bit == 3'd7) begin
              inject_bit <= 3'd0;
              inject_uart_line <= 1'b1;
              inject_state <= UART_STOP;
            end else begin
              inject_bit <= inject_bit + 3'd1;
              inject_uart_line <= inject_shift[inject_bit + 3'd1];
            end
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
        default: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            inject_state <= UART_IDLE;
            inject_uart_line <= 1'b1;
            command_pending <= 1'b0;
            inject_ack_valid <= 1'b1;
            inject_ack_sequence <= pending_command_sequence;
            inject_ack_command <= pending_command_byte;
            inject_ack_result <= inject_collision ? 2'd3 : 2'd0;
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
      endcase
    end
  end




  reg [1:0] tap_rx_state;  // 2 bit 用于“TAP / 接收”的状态寄存器；决定状态机当前执行的分支和下一状态。
  reg [15:0] tap_rx_clock;  // 16 bit 寄存器，保存“TAP / 接收 / clock”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [2:0] tap_rx_bit;  // 3 bit 寄存器，保存“TAP / 接收 / BIT”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] tap_rx_shift;  // 8 bit 寄存器，保存“TAP / 接收 / 移位缓存”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] tap_rx_byte;  // 8 bit 寄存器，保存“TAP / 接收 / byte”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg tap_rx_valid;  // “TAP / 接收 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 5
  // 蓝牙 TX 旁路 UART 接收器：从物理发送电平恢复 FPGA 输出的 ASCII 字节。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tap_rx_state <= UART_IDLE;
      tap_rx_clock <= 16'd0;
      tap_rx_bit <= 3'd0;
      tap_rx_shift <= 8'd0;
      tap_rx_byte <= 8'd0;
      tap_rx_valid <= 1'b0;
    end else begin
      tap_rx_valid <= 1'b0;
      case (tap_rx_state)
        UART_IDLE: begin
          tap_rx_clock <= 16'd0;
          tap_rx_bit <= 3'd0;
          if (!bt_uart_tx_tap)
            tap_rx_state <= UART_START;
        end
        UART_START: begin
          if (tap_rx_clock >= HALF_BIT_CLKS - 1) begin
            tap_rx_clock <= 16'd0;
            if (!bt_uart_tx_tap)
              tap_rx_state <= UART_DATA;
            else
              tap_rx_state <= UART_IDLE;
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (tap_rx_clock >= CLKS_PER_BIT - 1) begin
            tap_rx_clock <= 16'd0;
            tap_rx_shift[tap_rx_bit] <= bt_uart_tx_tap;
            if (tap_rx_bit == 3'd7) begin
              tap_rx_bit <= 3'd0;
              tap_rx_state <= UART_STOP;
            end else begin
              tap_rx_bit <= tap_rx_bit + 3'd1;
            end
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
        default: begin
          if (tap_rx_clock >= CLKS_PER_BIT - 1) begin
            tap_rx_clock <= 16'd0;
            tap_rx_state <= UART_IDLE;
            if (bt_uart_tx_tap) begin
              tap_rx_byte <= tap_rx_shift;
              tap_rx_valid <= 1'b1;
            end
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
      endcase
    end
  end

  reg [7:0] report_buffer [0:95];  // 缓存 FPGA 蓝牙 TX 完整 ASCII 行的 96 字节寄存器数组。
  reg report_capture_active;  // 已看到 @ 行首且正在等待换行的捕获状态。
  reg [6:0] report_capture_length;  // 当前 @S/@L/@K/@PONG 行已缓存的字节数。
  reg report_capture_event;  // 寄存器，保存“report / capture / event”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [6:0] captured_report_length;  // 7 bit “captured / report”长度结果；由步数和单步长度组合换算。
  reg capture_event_consumed;  // 寄存器，保存“capture / event / consumed”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 6
  // 状态行捕获器：从 @ 开始缓存到换行，限制 96 字节并在完整行到达时产生事件。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      report_capture_active <= 1'b0;
      report_capture_length <= 7'd0;
      report_capture_event <= 1'b0;
      captured_report_length <= 7'd0;
      telemetry_drop_count <= 16'd0;
    end else begin
      if (report_capture_event && capture_event_consumed)
        report_capture_event <= 1'b0;

      if (tap_rx_valid) begin
        if (!report_capture_active) begin
          if (tap_rx_byte == "@") begin
            if (!report_capture_event) begin
              report_capture_length <= 7'd1;
              report_capture_active <= 1'b1;
            end else begin
              telemetry_drop_count <= telemetry_drop_count + 16'd1;
            end
          end
        end else if (tap_rx_byte == 8'h0A) begin
          report_capture_active <= 1'b0;
          if (report_capture_length >= 2) begin
            captured_report_length <= report_capture_length;
            report_capture_event <= 1'b1;
          end else begin
            telemetry_drop_count <= telemetry_drop_count + 16'd1;
          end
        end else if (report_capture_length < 7'd96) begin


          report_buffer[report_capture_length - 7'd1] <= tap_rx_byte;
          report_capture_length <= report_capture_length + 7'd1;
        end else begin
          report_capture_active <= 1'b0;
          report_capture_length <= 7'd0;
          telemetry_drop_count <= telemetry_drop_count + 16'd1;
        end
      end
    end
  end




  reg ack_pending;  // “ACK / 待处理”控制标志；为 1 表示名称描述的状态当前成立。
  reg [7:0] ack_sequence;  // 8 bit 寄存器，保存“ACK / sequence”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] ack_command;  // 8 bit 寄存器，保存“ACK / 命令”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [1:0] ack_result;  // 2 bit 寄存器，保存“ACK / result”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg ack_event_consumed;  // 寄存器，保存“ACK / event / consumed”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 7
  // ACK 挂起队列：把解析和注入阶段结果锁存到统一 LinkPG 上行发送路径。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ack_pending <= 1'b0;
      ack_sequence <= 8'd0;
      ack_command <= 8'd0;
      ack_result <= 2'd0;
    end else begin
      if (ack_event_consumed)
        ack_pending <= 1'b0;

      if (inject_ack_valid) begin
        if (!ack_pending || ack_event_consumed) begin
          ack_pending <= 1'b1;
          ack_sequence <= inject_ack_sequence;
          ack_command <= inject_ack_command;
          ack_result <= inject_ack_result;
        end
      end else if (parser_ack_valid) begin
        if (!ack_pending || ack_event_consumed) begin
          ack_pending <= 1'b1;
          ack_sequence <= parser_ack_sequence;
          ack_command <= parser_ack_command;
          ack_result <= parser_ack_result;
        end
      end
    end
  end

  reg report_pending;  // 已有完整状态行等待构造一个或两个 UWB 上行分片。
  reg [7:0] report_message_sequence;  // 每条完整上行状态行自增的 8 bit 序号，两个分片共享该值。
  reg [6:0] report_length;  // 7 bit “report”长度结果；由步数和单步长度组合换算。
  reg report_part;  // 当前状态行分片编号：0 为前 64 字节，1 为剩余字节。
  reg [1:0] report_total;  // 2 bit 寄存器，保存“report / 总计”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg report_fragment_inflight;  // 寄存器，保存“report / fragment / inflight”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg report_build_started;  // 寄存器，保存“report / build / started”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg report_packet_sent;  // 寄存器，保存“report / 数据包 / sent”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  wire [6:0] report_fragment_offset = report_part ? 7'd64 : 7'd0;  // 7 bit 组合网络，传递“report / fragment / 偏移量”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [6:0] report_fragment_length =  // 7 bit “report / fragment”长度结果；由步数和单步长度组合换算。
      report_part ? (report_length - 7'd64) :
      ((report_length > 7'd64) ? 7'd64 : report_length);

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 8
  // 遥测分片调度器：将最多 96 字节状态行拆为两段，并保证上一段发送完成后再推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      report_pending <= 1'b0;
      report_message_sequence <= 8'd0;
      report_length <= 7'd0;
      report_part <= 1'b0;
      report_total <= 2'd0;
      report_fragment_inflight <= 1'b0;
      capture_event_consumed <= 1'b0;
    end else begin
      capture_event_consumed <= 1'b0;
      if (report_capture_event && !report_pending && !report_fragment_inflight) begin
        report_pending <= 1'b1;
        report_length <= captured_report_length;
        report_part <= 1'b0;
        report_total <= (captured_report_length > 7'd64) ? 2'd2 : 2'd1;
      end

      if (report_packet_sent) begin
        report_fragment_inflight <= 1'b0;
        if (report_part + 1'b1 < report_total) begin
          report_part <= report_part + 1'b1;
        end else begin
          report_pending <= 1'b0;
          report_part <= 1'b0;
          report_message_sequence <= report_message_sequence + 8'd1;



          capture_event_consumed <= 1'b1;
        end
      end

      if (report_build_started)
        report_fragment_inflight <= 1'b1;
    end
  end




  reg [7:0] tx_payload_buffer [0:73];  // 准备封装到 LinkPG Modbus 帧的数据透传 ASCII 载荷。
  reg [6:0] tx_payload_length;  // 7 bit “发送 / payload”长度结果；由步数和单步长度组合换算。
  reg [1:0] tx_payload_kind;  // 2 bit 寄存器，保存“发送 / payload / 类型”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg tx_payload_ready;  // 载荷和内部 CRC8 已构造完成，等待 Modbus 流发送器取走。
  reg [2:0] build_state;  // 3 bit 用于“build”的状态寄存器；决定状态机当前执行的分支和下一状态。
  reg [6:0] build_index;  // 7 bit 用于“build / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  reg [7:0] build_crc;  // 8 bit 寄存器，保存“build / CRC”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg payload_sent;  // 寄存器，保存“payload / sent”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  wire [7:0] current_ack_crc = ack_crc8(ack_sequence, ack_command, ack_result);  // 8 bit 组合网络，传递“当前 / ACK / CRC”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [6:0] current_report_data_index = report_fragment_offset + build_index;  // 7 bit 用于“当前 / report / 数据 / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  wire [7:0] current_report_data_byte =  // 8 bit 组合网络，传递“当前 / report / 数据 / byte”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
      (current_report_data_index == 0) ? "@" :
      report_buffer[current_report_data_index - 7'd1];

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 9
  // 上行载荷构造器：生成 A/Ack 或 R/Report ASCII 数据及 CRC8 字段。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_payload_length <= 7'd0;
      tx_payload_kind <= PAYLOAD_NONE;
      tx_payload_ready <= 1'b0;
      build_state <= BUILD_IDLE;
      build_index <= 7'd0;
      build_crc <= 8'd0;
      ack_event_consumed <= 1'b0;
      report_build_started <= 1'b0;
      report_packet_sent <= 1'b0;
    end else begin
      ack_event_consumed <= 1'b0;
      report_build_started <= 1'b0;
      report_packet_sent <= 1'b0;
      if (payload_sent) begin
        tx_payload_ready <= 1'b0;
        if (tx_payload_kind == PAYLOAD_REPORT)
          report_packet_sent <= 1'b1;
        tx_payload_kind <= PAYLOAD_NONE;
      end

      case (build_state)
        BUILD_IDLE: begin
          if (!tx_payload_ready) begin
            if (ack_pending) begin
              tx_payload_buffer[0] <= "A";
              tx_payload_buffer[1] <= "1";
              tx_payload_buffer[2] <= hex_ascii(ack_sequence[7:4]);
              tx_payload_buffer[3] <= hex_ascii(ack_sequence[3:0]);
              tx_payload_buffer[4] <= ack_command;
              tx_payload_buffer[5] <= "0" + ack_result;
              tx_payload_buffer[6] <= hex_ascii(current_ack_crc[7:4]);
              tx_payload_buffer[7] <= hex_ascii(current_ack_crc[3:0]);
              tx_payload_length <= 7'd8;
              tx_payload_kind <= PAYLOAD_ACK;
              tx_payload_ready <= 1'b1;
              ack_event_consumed <= 1'b1;
            end else if (report_pending && !report_fragment_inflight) begin
              tx_payload_buffer[0] <= "F";
              tx_payload_buffer[1] <= "1";
              tx_payload_buffer[2] <= hex_ascii(report_message_sequence[7:4]);
              tx_payload_buffer[3] <= hex_ascii(report_message_sequence[3:0]);
              tx_payload_buffer[4] <= hex_ascii({3'd0, report_part});
              tx_payload_buffer[5] <= hex_ascii({2'd0, report_total});
              tx_payload_buffer[6] <= hex_ascii({1'b0, report_fragment_length[6:4]});
              tx_payload_buffer[7] <= hex_ascii(report_fragment_length[3:0]);
              build_crc <= crc8_next(
                  crc8_next(
                    crc8_next(
                      crc8_next(
                        crc8_next(
                          crc8_next(
                            crc8_next(
                              crc8_next(8'h00, "F"), "1"),
                            hex_ascii(report_message_sequence[7:4])),
                          hex_ascii(report_message_sequence[3:0])),
                        hex_ascii({3'd0, report_part})),
                      hex_ascii({2'd0, report_total})),
                    hex_ascii({1'b0, report_fragment_length[6:4]})),
                  hex_ascii(report_fragment_length[3:0]));
              build_index <= 7'd0;
              build_state <= BUILD_REPORT_DATA;
              report_build_started <= 1'b1;
            end
          end
        end
        BUILD_REPORT_DATA: begin
          tx_payload_buffer[8 + build_index] <= current_report_data_byte;
          build_crc <= crc8_next(build_crc, current_report_data_byte);
          if (build_index + 7'd1 >= report_fragment_length) begin
            build_index <= 7'd0;
            build_state <= BUILD_REPORT_CRC;
          end else begin
            build_index <= build_index + 7'd1;
          end
        end
        default: begin
          tx_payload_buffer[8 + report_fragment_length] <= hex_ascii(build_crc[7:4]);
          tx_payload_buffer[9 + report_fragment_length] <= hex_ascii(build_crc[3:0]);
          tx_payload_length <= report_fragment_length + 7'd10;
          tx_payload_kind <= PAYLOAD_REPORT;
          tx_payload_ready <= 1'b1;
          build_state <= BUILD_IDLE;
        end
      endcase
    end
  end


  reg [7:0] byte_tx_data;  // 8 bit 寄存器，保存“byte / 发送 / 数据”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg byte_tx_start;  // 寄存器，保存“byte / 发送 / 启动”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg byte_tx_busy;  // “byte / 发送 / 忙标志”信号；为 1 时当前事务尚未结束，上层必须等待而不能启动冲突操作。
  reg [1:0] byte_tx_state;  // 2 bit 用于“byte / 发送”的状态寄存器；决定状态机当前执行的分支和下一状态。
  reg [15:0] byte_tx_clock;  // 16 bit 寄存器，保存“byte / 发送 / clock”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [2:0] byte_tx_bit;  // 3 bit 寄存器，保存“byte / 发送 / BIT”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] byte_tx_shift;  // 8 bit 寄存器，保存“byte / 发送 / 移位缓存”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg byte_tx_line;  // 寄存器，保存“byte / 发送 / 输出线”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  // [ASSIGN] 组合定义“uwb_uart_tx”：FPGA 到 LinkPG 标签 RXD 的 Modbus 数据透传/ACK 输出。 该组合结果直接由 `byte_tx_line`（byte / 发送 / 输出线）的当前值决定，不引入时钟周期延迟。
  assign uwb_uart_tx = byte_tx_line;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 10
  // LinkPG UART 字节发送器：按 115200 8N1 输出构造好的 Modbus 帧字节。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      byte_tx_busy <= 1'b0;
      byte_tx_state <= UART_IDLE;
      byte_tx_clock <= 16'd0;
      byte_tx_bit <= 3'd0;
      byte_tx_shift <= 8'd0;
      byte_tx_line <= 1'b1;
    end else if (!byte_tx_busy) begin
      byte_tx_line <= 1'b1;
      byte_tx_state <= UART_IDLE;
      byte_tx_clock <= 16'd0;
      if (byte_tx_start) begin
        byte_tx_busy <= 1'b1;
        byte_tx_state <= UART_START;
        byte_tx_shift <= byte_tx_data;
        byte_tx_bit <= 3'd0;
        byte_tx_line <= 1'b0;
      end
    end else begin
      case (byte_tx_state)
        UART_START: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            byte_tx_line <= byte_tx_shift[0];
            byte_tx_state <= UART_DATA;
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            if (byte_tx_bit == 3'd7) begin
              byte_tx_bit <= 3'd0;
              byte_tx_line <= 1'b1;
              byte_tx_state <= UART_STOP;
            end else begin
              byte_tx_bit <= byte_tx_bit + 3'd1;
              byte_tx_line <= byte_tx_shift[byte_tx_bit + 3'd1];
            end
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
        default: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            byte_tx_busy <= 1'b0;
            byte_tx_state <= UART_IDLE;
            byte_tx_line <= 1'b1;
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
      endcase
    end
  end




  reg [2:0] stream_state;  // 上行 Modbus 流状态：空闲、前缀、载荷、后缀和帧间等待。
  reg [6:0] stream_index;  // 7 bit 用于“stream / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  reg stream_saw_busy;  // “stream / SAW / 忙标志”信号；为 1 时当前事务尚未结束，上层必须等待而不能启动冲突操作。
  reg [20:0] tx_gap_count;  // 相邻 LinkPG Modbus 帧之间的 25 ms 保护间隔计数。

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 11
  // Modbus 流发送器：原子发送前缀、载荷、CRC16 和帧间隔，防止不同消息交叉。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stream_state <= STREAM_IDLE;
      stream_index <= 7'd0;
      stream_saw_busy <= 1'b0;
      tx_gap_count <= 21'd0;
      byte_tx_data <= 8'd0;
      byte_tx_start <= 1'b0;
      payload_sent <= 1'b0;
      tx_packet_count <= 16'd0;
    end else begin
      byte_tx_start <= 1'b0;
      payload_sent <= 1'b0;
      if (tx_gap_count != 0)
        tx_gap_count <= tx_gap_count - 21'd1;

      case (stream_state)
        STREAM_IDLE: begin
          stream_index <= 7'd0;
          stream_saw_busy <= 1'b0;
          if (tx_payload_ready && (tx_gap_count == 0))
            stream_state <= STREAM_PREFIX;
        end
        STREAM_PREFIX: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= prefix_byte(stream_index[3:0]);
            byte_tx_start <= 1'b1;
            if (stream_index == 7'd12) begin
              stream_index <= 7'd0;
              stream_state <= STREAM_PAYLOAD;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        STREAM_PAYLOAD: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= tx_payload_buffer[stream_index];
            byte_tx_start <= 1'b1;
            if (stream_index + 7'd1 >= tx_payload_length) begin
              stream_index <= 7'd0;
              stream_state <= STREAM_SUFFIX;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        STREAM_SUFFIX: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= suffix_byte(stream_index[3:0]);
            byte_tx_start <= 1'b1;
            if (stream_index == 7'd8) begin
              stream_index <= 7'd0;
              stream_saw_busy <= 1'b0;
              stream_state <= STREAM_WAIT;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        default: begin
          if (byte_tx_busy)
            stream_saw_busy <= 1'b1;
          else if (stream_saw_busy) begin
            payload_sent <= 1'b1;
            tx_packet_count <= tx_packet_count + 16'd1;
            tx_gap_count <= TX_GAP_CLKS;
            stream_state <= STREAM_IDLE;
            stream_saw_busy <= 1'b0;
          end
        end
      endcase
    end
  end

endmodule
