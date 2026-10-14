// =============================================================================
// 模块: rplidar_c1_uart
// 作用: RPLIDAR C1 启停、串口接收、扫描节点解析、八扇区滤波和轮廓避障。
// 说明: UART 为 460800 8N1；启动序列为等待 0.5 s、A5 25 停止、等待 0.1 s、A5 20 扫描。
// 说明: 描述符 A5 5A 05 00 00 40 81 匹配后进入连续 5 字节扫描节点解析。
// 说明: 角度采用 Q6（1/64 度），距离采用 Q2（1/4 mm）；质量为零或 50..12000 mm 外的点丢弃。
// 说明: 每圈八扇区先取最小值，再与前两圈值做 median3；无效扇区统一输出 FFFF。
// 说明: 正前、前角和侧面采用不同阈值，近似长方形车体的前角及侧边保护区。
//
// Robei 源: little_car/model/rplidar_c1_uart.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module rplidar_c1_uart (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire lidar_uart_rx,  // RPLIDAR C1 TXD 到 FPGA 的 460800 波特输入。
  input wire [15:0] safe_distance_mm,  // 正前方可调避障距离，单位 mm。
  input wire scan_enable,  // 允许雷达旋转和连续扫描。
  output wire lidar_uart_tx,  // FPGA 到 RPLIDAR C1 RXD 的命令发送线。
  output reg lidar_online,  // 收到有效雷达扫描样本且未超时。
  output reg scan_active,  // 描述符通过后允许把接收字节解释为扫描节点。
  output reg sample_valid,  // 当前周期解析出一个有效扫描节点。
  output reg new_scan,  // 扫描节点的整圈起始标志。
  output reg [5:0] quality,  // 标准扫描节点的 6 bit 回波质量；为 0 的测点直接丢弃。
  output reg [14:0] angle_q6,  // 扫描角度 Q6，单位 1/64 度。
  output reg [15:0] distance_q2,  // 扫描距离 Q2，单位 1/4 mm。
  output reg [15:0] front_distance_mm,  // 滤波后的正前方最近距离，FFFF 表示无效。
  output reg front_valid,  // “前方”有效脉冲/标志；为 1 时下游可锁存配套数据。
  output reg obstacle_stop,  // 合并或当前模块产生的停车请求。
  output reg [15:0] sector_front_mm,  // 雷达 前方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_front_right_mm,  // 雷达 前方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_right_mm,  // 雷达 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_rear_right_mm,  // 雷达 后方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_rear_mm,  // 雷达 后方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_rear_left_mm,  // 雷达 后方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_left_mm,  // 雷达 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  output reg [15:0] sector_front_left_mm  // 雷达 前方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
          // -----------------------------------------------------------------------------
          // 常量、状态编码与时序参数
          // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
          // -----------------------------------------------------------------------------
          localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
          localparam integer BAUD = 460800;  // UART 目标波特率，单位 bit/s。
          localparam integer CLKS_PER_BIT = ((CLK_HZ + (BAUD / 2)) / BAUD);  // 每个 UART bit 占用的系统时钟周期数；整数除法会形成实际波特率误差。
          localparam integer BOOT_DELAY_CYCLES = CLK_HZ / 2;  // RPLIDAR 上电后等待电源和转子稳定的 0.5 s 周期数；计满后发送 A5 25 停止命令。
          localparam integer STOP_WAIT_CYCLES = CLK_HZ / 10;  // 向 RPLIDAR 发送 A5 25 后等待旧数据流停止的 0.1 s 周期数。
          localparam integer RETRY_WAIT_CYCLES = CLK_HZ;  // 雷达描述符或数据超时后再次执行启动序列前等待的 1 s 周期数。
          localparam integer BYTE_TIMEOUT_CYCLES = CLK_HZ * 2;  // 连续 2 s 没有收到雷达串口字节时判定链路离线并进入重试。
          localparam [14:0] FRONT_HALF_Q6 = 15'd960;  // “FRONT / HALF / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] FRONT_WRAP_Q6 = 15'd22080;  // “FRONT / WRAP / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_0_END_Q6 = 15'd1440;  // “SECTOR / 0 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_1_END_Q6 = 15'd4320;  // “SECTOR / 1 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_2_END_Q6 = 15'd7200;  // “SECTOR / 2 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_3_END_Q6 = 15'd10080;  // “SECTOR / 3 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_4_END_Q6 = 15'd12960;  // “SECTOR / 4 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_5_END_Q6 = 15'd15840;  // “SECTOR / 5 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_6_END_Q6 = 15'd18720;  // “SECTOR / 6 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
          localparam [14:0] SECTOR_7_END_Q6 = 15'd21600;  // “SECTOR / 7 / END / Q6”使用 Q6 定点格式，最低六位表示 1/64 度。
        localparam [15:0] MIN_DISTANCE_Q2 = 16'd200;  // “MIN / DISTANCE / Q2”使用 Q2 定点格式，最低两位表示 1/4 mm。
        localparam [15:0] MAX_DISTANCE_Q2 = 16'd48000;  // “MAX / DISTANCE / Q2”使用 Q2 定点格式，最低两位表示 1/4 mm。
        localparam [15:0] FRONT_MIN_EMPTY = 16'hffff;  // 当前扫描圈尚未收到有效前方点时使用的无效哨兵 FFFF。
        localparam [15:0] CORNER_MARGIN_Q2 = 16'd600;  // “CORNER / MARGIN / Q2”使用 Q2 定点格式，最低两位表示 1/4 mm。
        localparam [15:0] SIDE_GUARD_Q2 = 16'd880;  // “SIDE / GUARD / Q2”使用 Q2 定点格式，最低两位表示 1/4 mm。

          localparam [1:0] RX_IDLE  = 2'd0;  // UART 接收状态“RX / IDLE”的编码。
          localparam [1:0] RX_START = 2'd1;  // UART 接收状态“RX / START”的编码。
          localparam [1:0] RX_DATA  = 2'd2;  // UART 接收状态“RX / DATA”的编码。
          localparam [1:0] RX_STOP  = 2'd3;  // UART 接收状态“RX / STOP”的编码。
          localparam [1:0] TX_IDLE  = 2'd0;  // UART 发送状态“TX / IDLE”的编码。
          localparam [1:0] TX_START = 2'd1;  // UART 发送状态“TX / START”的编码。
          localparam [1:0] TX_DATA  = 2'd2;  // UART 发送状态“TX / DATA”的编码。
          localparam [1:0] TX_STOP  = 2'd3;  // UART 发送状态“TX / STOP”的编码。
          localparam [3:0] CMD_BOOT       = 4'd0;  // 雷达命令状态机“CMD / BOOT”的状态编码。
          localparam [3:0] CMD_STOP_A5    = 4'd1;  // 雷达命令状态机“CMD / STOP / A5”的状态编码。
          localparam [3:0] CMD_STOP_25    = 4'd2;  // 雷达命令状态机“CMD / STOP / 25”的状态编码。
          localparam [3:0] CMD_STOP_WAIT  = 4'd3;  // 雷达命令状态机“CMD / STOP / WAIT”的状态编码。
          localparam [3:0] CMD_SCAN_A5    = 4'd4;  // 雷达命令状态机“CMD / SCAN / A5”的状态编码。
          localparam [3:0] CMD_SCAN_20    = 4'd5;  // 雷达命令状态机“CMD / SCAN / 20”的状态编码。
          localparam [3:0] CMD_WAIT_DESC  = 4'd6;  // 雷达命令状态机“CMD / WAIT / DESC”的状态编码。
          localparam [3:0] CMD_SCAN       = 4'd7;  // 雷达命令状态机“CMD / SCAN”的状态编码。
      localparam [3:0] CMD_RETRY_WAIT = 4'd8;  // 雷达命令状态机“CMD / RETRY / WAIT”的状态编码。
      localparam [3:0] CMD_DISABLED   = 4'd9;  // 雷达命令状态机“CMD / DISABLED”的状态编码。
      localparam [3:0] CMD_DISABLE_A5 = 4'd10;  // 雷达命令状态机“CMD / DISABLE / A5”的状态编码。
      localparam [3:0] CMD_DISABLE_25 = 4'd11;  // 雷达命令状态机“CMD / DISABLE / 25”的状态编码。

          // -----------------------------------------------------------------------------
          // 内部网络、寄存器和计数器
          // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
          // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
          // -----------------------------------------------------------------------------
          reg [1:0] rx_sync;  // 异步 RX 输入的两级同步寄存器，降低亚稳态进入 UART 状态机的概率。
          reg [1:0] rx_state;  // UART 接收状态，依次经历空闲、起始位确认、8 位数据采样和停止位检查。
          reg [15:0] rx_clk_count;  // UART 接收当前 bit 内的系统时钟计数；计满 CLKS_PER_BIT 后采样下一位。
          reg [2:0] rx_bit_index;  // UART 正在接收的数据位编号 0..7，数据按 LSB-first 顺序到达。
          reg [7:0] rx_shift;  // UART 接收移位缓存；8 个数据位收齐后整体复制到 rx_data。
          reg [7:0] rx_data;  // 最近一个通过停止位检查的完整 UART 字节。
          reg rx_valid;  // UART 字节接收完成脉冲；仅保持一个系统时钟周期。
          reg [1:0] tx_state;  // UART 发送状态，依次输出空闲高电平、起始位、8 位数据和停止位。
          reg [15:0] tx_clk_count;  // UART 当前发送 bit 已保持的系统时钟周期数。
          reg [2:0] tx_bit_index;  // UART 正在发送的数据位编号 0..7，发送顺序为 LSB-first。
          reg [7:0] tx_shift;  // UART 开始发送时锁存的数据字节，避免发送中途被新输入改变。
          reg [7:0] tx_data;  // 等待交给 UART 位发送状态机的下一个字节。
          reg tx_start;  // 启动一个 UART 字节发送的单周期脉冲。
          reg tx_line;  // UART 发送引脚的内部逻辑电平；空闲和停止位为 1。
          reg tx_busy;  // UART 正在发送一个字节；为 1 时上层不得覆盖 tx_data。
          reg tx_busy_d;  // tx_busy 的上一周期值，与当前值组合检测字节发送完成下降沿。
          reg [3:0] cmd_state;  // 雷达启动/停止/描述符等待/扫描/重试状态。
          reg [31:0] wait_count;  // 雷达命令状态机通用等待计数器；用于 500 ms 上电稳定、100 ms 停止间隔和重试等待。
          reg [31:0] byte_timeout_count;  // 扫描链路无字节超时计数，达到 2 s 后判离线。
          reg [2:0] desc_index;  // 已经连续匹配正确的 7 字节扫描描述符位置。
          reg [2:0] node_index;  // 当前 5 字节标准扫描节点的字节位置。
          reg [7:0] node0;  // 扫描节点第 0 字节：起始/反起始位和 6 bit 质量。
          reg [7:0] node1;  // 扫描节点第 1 字节：角度低位及固定校验位。
          reg [7:0] node2;  // 扫描节点第 2 字节：角度高 8 bit。
          reg [7:0] node3;  // 扫描节点第 3 字节：距离低 8 bit。
          reg [15:0] front_min_q2;  // 当前一圈正前方有效点的最小 Q2 距离。
          reg [15:0] sector_min_0_q2;  // 当前一圈前方 45 度扇区最小 Q2 距离。
          reg [15:0] sector_min_1_q2;  // 当前一圈右前 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
          reg [15:0] sector_min_2_q2;  // 当前一圈右侧 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
          reg [15:0] sector_min_3_q2;  // 当前一圈右后 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
          reg [15:0] sector_min_4_q2;  // 当前一圈后方 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
        reg [15:0] sector_min_5_q2;  // 当前一圈左后 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
        reg [15:0] sector_min_6_q2;  // 当前一圈左侧 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
        reg [15:0] sector_min_7_q2;  // 当前一圈左前 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。
        reg [15:0] front_hist_1_q2;  // 正前方上一圈最小值。
        reg [15:0] front_hist_2_q2;  // 正前方上上圈最小值。
        reg [15:0] sector_hist_1_q2 [0:7];  // 八扇区上一圈结果。
        reg [15:0] sector_hist_2_q2 [0:7];  // 八扇区上上圈结果，与当前圈共同做 median3。

          wire node_start;  // 扫描节点 byte0 的整圈起始位 S。
          wire node_inverse_start;  // 扫描节点 byte0 的反起始位 not-S；必须与 node_start 互斥。
          wire node_good;  // 起始位互斥且 byte1 固定位为 1 的扫描节点结构校验结果。
          wire [5:0] node_quality;  // 扫描节点 byte0[7:2] 的 6 bit 回波质量；零质量测点会被丢弃。
          wire [14:0] node_angle_q6;  // 由 node2 和 node1[7:1] 拼出的 15 bit Q6 扫描角度，单位 1/64 度。
          wire [15:0] rx_node_distance_q2;  // 由当前 rx_data 高字节和 node3 低字节拼出的 16 bit Q2 扫描距离。
          wire node_in_front;  // 节点角度落入跨 0 度的正前方窄扇区时为 1，用于独立 front_min_q2。
          wire rx_node_distance_good;  // 质量非零且距离位于 50..12000 mm 有效范围内的测点判定。
        wire tx_done;  // 雷达命令 UART 的 tx_busy 下降沿脉冲，状态机据此发送命令第二字节或进入等待。
        wire [15:0] safe_distance_q2;  // 手机设置的正前方安全距离从 mm 左移两位得到的 Q2 阈值。
        wire [15:0] corner_distance_q2;  // 正前安全距离加前角余量后的 Q2 阈值；加法溢出时饱和到 FFFF。
        wire [2:0] node_sector;  // 按 Q6 角度映射得到的八方向扇区编号：0前、1右前、2右、3右后、4后、5左后、6左、7左前。

          // -----------------------------------------------------------------------------
          // 组合派生信号
          // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
          // -----------------------------------------------------------------------------
          // [ASSIGN] 组合定义“lidar_uart_tx”：FPGA 到 RPLIDAR C1 RXD 的命令发送线。 该组合结果直接由 `tx_line`（发送 / 输出线）的当前值决定，不引入时钟周期延迟。
          assign lidar_uart_tx = tx_line;
          // [ASSIGN] 组合定义“node_start”：扫描节点 byte0 的整圈起始位 S。 该组合结果直接由 `node0`（node0）的当前值决定，不引入时钟周期延迟。
          assign node_start = node0[0];
          // [ASSIGN] 组合定义“node_inverse_start”：扫描节点 byte0 的反起始位 not-S；必须与 node_start 互斥。 该组合结果直接由 `node0`（node0）的当前值决定，不引入时钟周期延迟。
          assign node_inverse_start = node0[1];
          // [ASSIGN] 组合定义“node_quality”：扫描节点 byte0[7:2] 的 6 bit 回波质量；零质量测点会被丢弃。 该组合结果直接由 `node0`（node0）的当前值决定，不引入时钟周期延迟。
          assign node_quality = node0[7:2];
          // [ASSIGN] 组合定义“node_angle_q6”：由 node2 和 node1[7:1] 拼出的 15 bit Q6 扫描角度，单位 1/64 度。 该组合结果直接由 `node2`（node2）、`node1`（node1）的当前值决定，不引入时钟周期延迟。
          assign node_angle_q6 = {node2, node1[7:1]};
          // [ASSIGN] 组合定义“rx_node_distance_q2”：由当前 rx_data 高字节和 node3 低字节拼出的 16 bit Q2 扫描距离。 该组合结果直接由 `rx_data`（接收 / 数据）、`node3`（node3）的当前值决定，不引入时钟周期延迟。
          assign rx_node_distance_q2 = {rx_data, node3};
          // [ASSIGN] 组合定义“node_good”：起始位互斥且 byte1 固定位为 1 的扫描节点结构校验结果。 该组合结果直接由 `node_start`（node / 启动）、`node_inverse_start`（node / inverse / 启动）、`node1`（node1）的当前值决定，不引入时钟周期延迟。
          assign node_good = (node_start ^ node_inverse_start) && node1[0];
          // [ASSIGN] 组合定义“node_in_front”：节点角度落入跨 0 度的正前方窄扇区时为 1，用于独立 front_min_q2。 该组合结果直接由 `node_angle_q6`（node / 角度 / Q6）、`FRONT_HALF_Q6`（FRONT / HALF / Q6）、`FRONT_WRAP_Q6`（FRONT / WRAP / Q6）的当前值决定，不引入时钟周期延迟。
          assign node_in_front = (node_angle_q6 <= FRONT_HALF_Q6) || (node_angle_q6 >= FRONT_WRAP_Q6);
        // [ASSIGN] 组合定义“rx_node_distance_good”：质量非零且距离位于 50..12000 mm 有效范围内的测点判定。 该组合结果直接由 `node_quality`（node / 质量）、`rx_node_distance_q2`（接收 / node / 距离 / Q2）、`MIN_DISTANCE_Q2`（MIN / DISTANCE / Q2）、`MAX_DISTANCE_Q2`（MAX / DISTANCE / Q2）的当前值决定，不引入时钟周期延迟。
        assign rx_node_distance_good = (node_quality != 6'd0) &&
                                       (rx_node_distance_q2 >= MIN_DISTANCE_Q2) &&
                                       (rx_node_distance_q2 <= MAX_DISTANCE_Q2);
        // [ASSIGN] 组合定义“tx_done”：雷达命令 UART 的 tx_busy 下降沿脉冲，状态机据此发送命令第二字节或进入等待。 该组合结果直接由 `tx_busy_d`（发送 / 忙标志 / D）、`tx_busy`（发送 / 忙标志）的当前值决定，不引入时钟周期延迟。
        assign tx_done = tx_busy_d && !tx_busy;
        // [ASSIGN] 组合定义“safe_distance_q2”：手机设置的正前方安全距离从 mm 左移两位得到的 Q2 阈值。 该组合结果直接由 `safe_distance_mm`（安全阈值 / 距离 / MM）的当前值决定，不引入时钟周期延迟。
        assign safe_distance_q2 = safe_distance_mm << 2;
        // [ASSIGN] 组合定义“corner_distance_q2”：正前安全距离加前角余量后的 Q2 阈值；加法溢出时饱和到 FFFF。 该组合结果直接由 `safe_distance_q2`（安全阈值 / 距离 / Q2）、`CORNER_MARGIN_Q2`（CORNER / MARGIN / Q2）的当前值决定，不引入时钟周期延迟。
        assign corner_distance_q2 = (safe_distance_q2 > 16'hffff - CORNER_MARGIN_Q2) ?
                                    16'hffff : safe_distance_q2 + CORNER_MARGIN_Q2;
          // [ASSIGN] 组合定义“node_sector”：按 Q6 角度映射得到的八方向扇区编号：0前、1右前、2右、3右后、4后、5左后、6左、7左前。 该组合结果直接由 `node_angle_q6`（node / 角度 / Q6）、`SECTOR_0_END_Q6`（SECTOR / 0 / END / Q6）、`SECTOR_7_END_Q6`（SECTOR / 7 / END / Q6）、`SECTOR_1_END_Q6`（SECTOR / 1 / END / Q6）、`SECTOR_2_END_Q6`（SECTOR / 2 / END / Q6）、`SECTOR_3_END_Q6`（SECTOR / 3 / END / Q6）等当前值决定，不引入时钟周期延迟。
          assign node_sector = ((node_angle_q6 < SECTOR_0_END_Q6) || (node_angle_q6 >= SECTOR_7_END_Q6)) ? 3'd0 :
                               (node_angle_q6 < SECTOR_1_END_Q6) ? 3'd1 :
                               (node_angle_q6 < SECTOR_2_END_Q6) ? 3'd2 :
                               (node_angle_q6 < SECTOR_3_END_Q6) ? 3'd3 :
                               (node_angle_q6 < SECTOR_4_END_Q6) ? 3'd4 :
                               (node_angle_q6 < SECTOR_5_END_Q6) ? 3'd5 :
                               (node_angle_q6 < SECTOR_6_END_Q6) ? 3'd6 : 3'd7;

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: q2_to_mm
        // Q2 距离除以 4，并对无效 FFFF 保持无效标记。
        // -----------------------------------------------------------------------------
        function [15:0] q2_to_mm;
            input [15:0] value;  // 16 bit 函数 `q2_to_mm` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            begin
              q2_to_mm = (value == 16'hffff) ? 16'hffff : {2'b00, value[15:2]};
            end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: median3_q2
        // 三输入无乘除中值滤波，抑制单圈跳点。
        // -----------------------------------------------------------------------------
        function [15:0] median3_q2;
          input [15:0] a;  // 16 bit 函数 `median3_q2` 的“A”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          input [15:0] b;  // 16 bit 函数 `median3_q2` 的“B”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          input [15:0] c;  // 16 bit 函数 `median3_q2` 的“C”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            if ((b == 16'hffff) && (c == 16'hffff)) begin
              median3_q2 = a;
            end else if (c == 16'hffff) begin
              median3_q2 = (a < b) ? a : b;
            end else if (a > b) begin
              if (b > c)
                median3_q2 = b;
              else if (a > c)
                median3_q2 = c;
              else
                median3_q2 = a;
            end else begin
              if (a > c)
                median3_q2 = a;
              else if (b > c)
                median3_q2 = c;
              else
                median3_q2 = b;
            end
          end
        endfunction

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 1
          // 雷达 UART 接收器：以 460800 波特采样并输出 rx_data/rx_valid。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              rx_sync <= 2'b11;
              rx_state <= RX_IDLE;
              rx_clk_count <= 16'd0;
              rx_bit_index <= 3'd0;
              rx_shift <= 8'd0;
              rx_data <= 8'd0;
              rx_valid <= 1'b0;
            end else begin
              rx_sync <= {rx_sync[0], lidar_uart_rx};
              rx_valid <= 1'b0;
              case (rx_state)
                RX_IDLE: begin
                  rx_clk_count <= 16'd0;
                  rx_bit_index <= 3'd0;
                  if (!rx_sync[1]) rx_state <= RX_START;
                end
                RX_START: begin
                  if (rx_clk_count == (CLKS_PER_BIT / 2)) begin
                    rx_clk_count <= 16'd0;
                    rx_state <= !rx_sync[1] ? RX_DATA : RX_IDLE;
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
                RX_DATA: begin
                  if (rx_clk_count == CLKS_PER_BIT - 1) begin
                    rx_clk_count <= 16'd0;
                    rx_shift[rx_bit_index] <= rx_sync[1];
                    if (rx_bit_index == 3'd7) begin
                      rx_bit_index <= 3'd0;
                      rx_state <= RX_STOP;
                    end else begin
                      rx_bit_index <= rx_bit_index + 3'd1;
                    end
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
                default: begin
                  if (rx_clk_count == CLKS_PER_BIT - 1) begin
                    rx_clk_count <= 16'd0;
                    rx_data <= rx_shift;
                    rx_valid <= rx_sync[1];
                    rx_state <= RX_IDLE;
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
              endcase
            end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 2
          // 雷达 UART 发送器：发送 A5 25、A5 20 等两字节命令。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              tx_state <= TX_IDLE;
              tx_clk_count <= 16'd0;
              tx_bit_index <= 3'd0;
              tx_shift <= 8'd0;
              tx_line <= 1'b1;
              tx_busy <= 1'b0;
            end else begin
              case (tx_state)
                TX_IDLE: begin
                  tx_line <= 1'b1;
                  tx_busy <= 1'b0;
                  tx_clk_count <= 16'd0;
                  tx_bit_index <= 3'd0;
                  if (tx_start) begin
                    tx_busy <= 1'b1;
                    tx_shift <= tx_data;
                    tx_state <= TX_START;
                    tx_line <= 1'b0;
                  end
                end
                TX_START: begin
                  tx_busy <= 1'b1;
                  tx_line <= 1'b0;
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    tx_state <= TX_DATA;
                    tx_line <= tx_shift[0];
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
                TX_DATA: begin
                  tx_busy <= 1'b1;
                  tx_line <= tx_shift[tx_bit_index];
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    if (tx_bit_index == 3'd7) begin
                      tx_bit_index <= 3'd0;
                      tx_state <= TX_STOP;
                      tx_line <= 1'b1;
                    end else begin
                      tx_bit_index <= tx_bit_index + 3'd1;
                      tx_line <= tx_shift[tx_bit_index + 3'd1];
                    end
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
                default: begin
                  tx_busy <= 1'b1;
                  tx_line <= 1'b1;
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    tx_busy <= 1'b0;
                    tx_state <= TX_IDLE;
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
              endcase
            end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 3
          // 雷达启停状态机：控制启动延时、描述符匹配、在线超时和重新启动。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              cmd_state <= CMD_BOOT;
              wait_count <= 32'd0;
              byte_timeout_count <= 32'd0;
              desc_index <= 3'd0;
              scan_active <= 1'b0;
              lidar_online <= 1'b0;
              tx_data <= 8'd0;
              tx_start <= 1'b0;
              tx_busy_d <= 1'b0;
        end else begin
          tx_start <= 1'b0;
          tx_busy_d <= tx_busy;
          if (!scan_enable) begin
            scan_active <= 1'b0;
            lidar_online <= 1'b0;
            byte_timeout_count <= 32'd0;
            if (cmd_state == CMD_DISABLE_A5) begin
              if (!tx_busy) begin
                tx_data <= 8'hA5;
                tx_start <= 1'b1;
                cmd_state <= CMD_DISABLE_25;
              end
            end else if (cmd_state == CMD_DISABLE_25) begin
              if (tx_done) begin
                tx_data <= 8'h25;
                tx_start <= 1'b1;
                cmd_state <= CMD_DISABLED;
              end
            end else if (cmd_state != CMD_DISABLED) begin
              cmd_state <= CMD_DISABLE_A5;
            end
          end else if ((cmd_state == CMD_DISABLED) ||
                       (cmd_state == CMD_DISABLE_A5) ||
                       (cmd_state == CMD_DISABLE_25)) begin
            wait_count <= 32'd0;
            byte_timeout_count <= 32'd0;
            desc_index <= 3'd0;
            cmd_state <= CMD_BOOT;
          end else case (cmd_state)
                // -----------------------------------------------------------------------------
                // 关键分支
                // 等待雷达电源和电机稳定 0.5 s。
                // -----------------------------------------------------------------------------
                CMD_BOOT: begin
                  scan_active <= 1'b0;
                  lidar_online <= 1'b0;
                  if (wait_count >= BOOT_DELAY_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    cmd_state <= CMD_STOP_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
                CMD_STOP_A5: if (!tx_busy) begin tx_data <= 8'hA5; tx_start <= 1'b1; cmd_state <= CMD_STOP_25; end
                CMD_STOP_25: if (tx_done) begin tx_data <= 8'h25; tx_start <= 1'b1; cmd_state <= CMD_STOP_WAIT; end
                // -----------------------------------------------------------------------------
                // 关键分支
                // A5 25 停止命令后等待 0.1 s，清除雷达旧扫描状态。
                // -----------------------------------------------------------------------------
                CMD_STOP_WAIT: begin
                  if (wait_count >= STOP_WAIT_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    desc_index <= 3'd0;
                    cmd_state <= CMD_SCAN_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
                CMD_SCAN_A5: if (!tx_busy) begin tx_data <= 8'hA5; tx_start <= 1'b1; cmd_state <= CMD_SCAN_20; end
                CMD_SCAN_20: if (tx_done) begin tx_data <= 8'h20; tx_start <= 1'b1; byte_timeout_count <= 32'd0; cmd_state <= CMD_WAIT_DESC; end
                // -----------------------------------------------------------------------------
                // 关键分支
                // 逐字节匹配标准扫描响应描述符 A5 5A 05 00 00 40 81。
                // -----------------------------------------------------------------------------
                CMD_WAIT_DESC: begin
                  if (rx_valid) begin
                    byte_timeout_count <= 32'd0;
                    case (desc_index)
                      3'd0: desc_index <= (rx_data == 8'hA5) ? 3'd1 : 3'd0;
                      3'd1: desc_index <= (rx_data == 8'h5A) ? 3'd2 : ((rx_data == 8'hA5) ? 3'd1 : 3'd0);
                      3'd2: desc_index <= (rx_data == 8'h05) ? 3'd3 : 3'd0;
                      3'd3: desc_index <= (rx_data == 8'h00) ? 3'd4 : 3'd0;
                      3'd4: desc_index <= (rx_data == 8'h00) ? 3'd5 : 3'd0;
                      3'd5: desc_index <= (rx_data == 8'h40) ? 3'd6 : 3'd0;
                      default: begin
                        if (rx_data == 8'h81) begin
                          scan_active <= 1'b1;
                          lidar_online <= 1'b0;
                          cmd_state <= CMD_SCAN;
                        end else desc_index <= 3'd0;
                      end
                    endcase
                  end else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                    byte_timeout_count <= byte_timeout_count;
                  end else byte_timeout_count <= byte_timeout_count + 32'd1;
                end
                // -----------------------------------------------------------------------------
                // 关键分支
                // 连续扫描态：接收字节维持在线计时，有效节点令 lidar_online 置位。
                // -----------------------------------------------------------------------------
                CMD_SCAN: begin
                  if (rx_valid) byte_timeout_count <= 32'd0;
                  else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                    byte_timeout_count <= byte_timeout_count;
                    lidar_online <= 1'b0;
                  end else byte_timeout_count <= byte_timeout_count + 32'd1;
                  if (sample_valid) lidar_online <= 1'b1;
                end
                default: begin
                  scan_active <= 1'b0;
                  if (wait_count >= RETRY_WAIT_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    desc_index <= 3'd0;
                    cmd_state <= CMD_STOP_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
          endcase
        end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 4
          // 扫描数据面：组装 5 字节节点、按圈统计八扇区、三样本中值并判断轮廓障碍。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              sample_valid <= 1'b0;
              new_scan <= 1'b0;
              quality <= 6'd0;
              angle_q6 <= 15'd0;
              distance_q2 <= 16'd0;
              front_distance_mm <= 16'hffff;
              front_valid <= 1'b0;
              obstacle_stop <= 1'b0;
              node_index <= 3'd0;
              node0 <= 8'd0;
              node1 <= 8'd0;
              node2 <= 8'd0;
              node3 <= 8'd0;
              front_min_q2 <= FRONT_MIN_EMPTY;
              sector_min_0_q2 <= 16'hffff;
              sector_min_1_q2 <= 16'hffff;
              sector_min_2_q2 <= 16'hffff;
              sector_min_3_q2 <= 16'hffff;
              sector_min_4_q2 <= 16'hffff;
              sector_min_5_q2 <= 16'hffff;
              sector_min_6_q2 <= 16'hffff;
              sector_min_7_q2 <= 16'hffff;
              front_hist_1_q2 <= 16'hffff;
              front_hist_2_q2 <= 16'hffff;
              sector_hist_1_q2[0] <= 16'hffff;
              sector_hist_1_q2[1] <= 16'hffff;
              sector_hist_1_q2[2] <= 16'hffff;
              sector_hist_1_q2[3] <= 16'hffff;
              sector_hist_1_q2[4] <= 16'hffff;
              sector_hist_1_q2[5] <= 16'hffff;
              sector_hist_1_q2[6] <= 16'hffff;
              sector_hist_1_q2[7] <= 16'hffff;
              sector_hist_2_q2[0] <= 16'hffff;
              sector_hist_2_q2[1] <= 16'hffff;
              sector_hist_2_q2[2] <= 16'hffff;
              sector_hist_2_q2[3] <= 16'hffff;
              sector_hist_2_q2[4] <= 16'hffff;
              sector_hist_2_q2[5] <= 16'hffff;
              sector_hist_2_q2[6] <= 16'hffff;
              sector_hist_2_q2[7] <= 16'hffff;
              sector_front_mm <= 16'hffff;
              sector_front_right_mm <= 16'hffff;
              sector_right_mm <= 16'hffff;
              sector_rear_right_mm <= 16'hffff;
              sector_rear_mm <= 16'hffff;
              sector_rear_left_mm <= 16'hffff;
              sector_left_mm <= 16'hffff;
              sector_front_left_mm <= 16'hffff;
            end else begin
              sample_valid <= 1'b0;
              new_scan <= 1'b0;
              if (!scan_active) begin
                node_index <= 3'd0;
                front_min_q2 <= FRONT_MIN_EMPTY;
                sector_min_0_q2 <= 16'hffff;
                sector_min_1_q2 <= 16'hffff;
                sector_min_2_q2 <= 16'hffff;
                sector_min_3_q2 <= 16'hffff;
                sector_min_4_q2 <= 16'hffff;
                sector_min_5_q2 <= 16'hffff;
                sector_min_6_q2 <= 16'hffff;
                sector_min_7_q2 <= 16'hffff;
                front_hist_1_q2 <= 16'hffff;
                front_hist_2_q2 <= 16'hffff;
                sector_hist_1_q2[0] <= 16'hffff;
                sector_hist_1_q2[1] <= 16'hffff;
                sector_hist_1_q2[2] <= 16'hffff;
                sector_hist_1_q2[3] <= 16'hffff;
                sector_hist_1_q2[4] <= 16'hffff;
                sector_hist_1_q2[5] <= 16'hffff;
                sector_hist_1_q2[6] <= 16'hffff;
                sector_hist_1_q2[7] <= 16'hffff;
                sector_hist_2_q2[0] <= 16'hffff;
                sector_hist_2_q2[1] <= 16'hffff;
                sector_hist_2_q2[2] <= 16'hffff;
                sector_hist_2_q2[3] <= 16'hffff;
                sector_hist_2_q2[4] <= 16'hffff;
                sector_hist_2_q2[5] <= 16'hffff;
                sector_hist_2_q2[6] <= 16'hffff;
                sector_hist_2_q2[7] <= 16'hffff;
                front_distance_mm <= 16'hffff;
                front_valid <= 1'b0;
                obstacle_stop <= 1'b0;
                sector_front_mm <= 16'hffff;
                sector_front_right_mm <= 16'hffff;
                sector_right_mm <= 16'hffff;
                sector_rear_right_mm <= 16'hffff;
                sector_rear_mm <= 16'hffff;
                sector_rear_left_mm <= 16'hffff;
                sector_left_mm <= 16'hffff;
                sector_front_left_mm <= 16'hffff;
              end else if (rx_valid) begin
                case (node_index)
                  3'd0: begin node0 <= rx_data; node_index <= 3'd1; end
                  3'd1: begin node1 <= rx_data; node_index <= 3'd2; end
                  3'd2: begin node2 <= rx_data; node_index <= 3'd3; end
                  3'd3: begin node3 <= rx_data; node_index <= 3'd4; end
                  default: begin
                    node_index <= 3'd0;
                    if (node_good) begin
                      sample_valid <= 1'b1;
                      quality <= node_quality;
                      angle_q6 <= node_angle_q6;
                      distance_q2 <= rx_node_distance_q2;
                    if (node_start) begin
                      new_scan <= 1'b1;
                      obstacle_stop <= (median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2) <= safe_distance_q2) ||
                                       (median3_q2(sector_min_0_q2, sector_hist_1_q2[0], sector_hist_2_q2[0]) <= safe_distance_q2) ||
                                       (median3_q2(sector_min_1_q2, sector_hist_1_q2[1], sector_hist_2_q2[1]) <= corner_distance_q2) ||
                                       (median3_q2(sector_min_7_q2, sector_hist_1_q2[7], sector_hist_2_q2[7]) <= corner_distance_q2) ||
                                       (median3_q2(sector_min_2_q2, sector_hist_1_q2[2], sector_hist_2_q2[2]) <= SIDE_GUARD_Q2) ||
                                       (median3_q2(sector_min_6_q2, sector_hist_1_q2[6], sector_hist_2_q2[6]) <= SIDE_GUARD_Q2);
                      if (median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2) == FRONT_MIN_EMPTY) begin
                        front_distance_mm <= 16'hffff;
                        front_valid <= 1'b0;
                      end else begin
                        front_distance_mm <= q2_to_mm(median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2));
                        front_valid <= 1'b1;
                      end
                        front_hist_2_q2 <= front_hist_1_q2;
                        front_hist_1_q2 <= front_min_q2;
                        front_min_q2 <= (node_in_front && rx_node_distance_good) ? rx_node_distance_q2 : FRONT_MIN_EMPTY;
                        sector_front_mm <= q2_to_mm(median3_q2(sector_min_0_q2, sector_hist_1_q2[0], sector_hist_2_q2[0]));
                        sector_front_right_mm <= q2_to_mm(median3_q2(sector_min_1_q2, sector_hist_1_q2[1], sector_hist_2_q2[1]));
                        sector_right_mm <= q2_to_mm(median3_q2(sector_min_2_q2, sector_hist_1_q2[2], sector_hist_2_q2[2]));
                        sector_rear_right_mm <= q2_to_mm(median3_q2(sector_min_3_q2, sector_hist_1_q2[3], sector_hist_2_q2[3]));
                        sector_rear_mm <= q2_to_mm(median3_q2(sector_min_4_q2, sector_hist_1_q2[4], sector_hist_2_q2[4]));
                        sector_rear_left_mm <= q2_to_mm(median3_q2(sector_min_5_q2, sector_hist_1_q2[5], sector_hist_2_q2[5]));
                        sector_left_mm <= q2_to_mm(median3_q2(sector_min_6_q2, sector_hist_1_q2[6], sector_hist_2_q2[6]));
                        sector_front_left_mm <= q2_to_mm(median3_q2(sector_min_7_q2, sector_hist_1_q2[7], sector_hist_2_q2[7]));
                        sector_hist_2_q2[0] <= sector_hist_1_q2[0];
                        sector_hist_2_q2[1] <= sector_hist_1_q2[1];
                        sector_hist_2_q2[2] <= sector_hist_1_q2[2];
                        sector_hist_2_q2[3] <= sector_hist_1_q2[3];
                        sector_hist_2_q2[4] <= sector_hist_1_q2[4];
                        sector_hist_2_q2[5] <= sector_hist_1_q2[5];
                        sector_hist_2_q2[6] <= sector_hist_1_q2[6];
                        sector_hist_2_q2[7] <= sector_hist_1_q2[7];
                        sector_hist_1_q2[0] <= sector_min_0_q2;
                        sector_hist_1_q2[1] <= sector_min_1_q2;
                        sector_hist_1_q2[2] <= sector_min_2_q2;
                        sector_hist_1_q2[3] <= sector_min_3_q2;
                        sector_hist_1_q2[4] <= sector_min_4_q2;
                        sector_hist_1_q2[5] <= sector_min_5_q2;
                        sector_hist_1_q2[6] <= sector_min_6_q2;
                        sector_hist_1_q2[7] <= sector_min_7_q2;
                        sector_min_0_q2 <= (rx_node_distance_good && (node_sector == 3'd0)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_1_q2 <= (rx_node_distance_good && (node_sector == 3'd1)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_2_q2 <= (rx_node_distance_good && (node_sector == 3'd2)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_3_q2 <= (rx_node_distance_good && (node_sector == 3'd3)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_4_q2 <= (rx_node_distance_good && (node_sector == 3'd4)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_5_q2 <= (rx_node_distance_good && (node_sector == 3'd5)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_6_q2 <= (rx_node_distance_good && (node_sector == 3'd6)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_7_q2 <= (rx_node_distance_good && (node_sector == 3'd7)) ? rx_node_distance_q2 : 16'hffff;
                      end else if (node_in_front && rx_node_distance_good && (rx_node_distance_q2 < front_min_q2)) begin
                        front_min_q2 <= rx_node_distance_q2;
                      end
                      if (!node_start && rx_node_distance_good) begin
                        case (node_sector)
                          3'd0: if (rx_node_distance_q2 < sector_min_0_q2) sector_min_0_q2 <= rx_node_distance_q2;
                          3'd1: if (rx_node_distance_q2 < sector_min_1_q2) sector_min_1_q2 <= rx_node_distance_q2;
                          3'd2: if (rx_node_distance_q2 < sector_min_2_q2) sector_min_2_q2 <= rx_node_distance_q2;
                          3'd3: if (rx_node_distance_q2 < sector_min_3_q2) sector_min_3_q2 <= rx_node_distance_q2;
                          3'd4: if (rx_node_distance_q2 < sector_min_4_q2) sector_min_4_q2 <= rx_node_distance_q2;
                          3'd5: if (rx_node_distance_q2 < sector_min_5_q2) sector_min_5_q2 <= rx_node_distance_q2;
                          3'd6: if (rx_node_distance_q2 < sector_min_6_q2) sector_min_6_q2 <= rx_node_distance_q2;
                          default: if (rx_node_distance_q2 < sector_min_7_q2) sector_min_7_q2 <= rx_node_distance_q2;
                        endcase
                      end
                    end
                  end
                endcase
              end
            end
          end

endmodule
