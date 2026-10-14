// =============================================================================
// 模块: motion_sensor_subsystem
// 作用: 共享 SPI 总线驱动一块 PMW3901 与两块 BMI088，并输出原始光流、双 IMU 融合和标定遥测。
// 说明: SPI mode 3 约 1.923 MHz，五个独立片选共享 SCLK/MOSI/MISO，不使用厂商 IP 核。
// 说明: PMW3901 完成复位、49/B6 ID 检查、寄存器初始化和约 100 Hz DX/DY/SQUAL 轮询。
// 说明: 两块 BMI088 分别检查 1E/0F ID，配置加速度 +/-6 g 与陀螺 +/-250 dps。
// 说明: 启动静止 512 组样本估计两块陀螺偏置；一致时平均，单块异常时降级。
// 说明: 每 100 ms 输出固定 99 字节 @M ASCII 帧，供轴向、比例、噪声和一致性标定。
//
// Robei 源: little_car/model/motion_sensor_subsystem.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module motion_sensor_subsystem (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire sensor_spi_miso,  // PMW3901 与两块 BMI088 共享 SPI MISO；只有当前片选器件允许驱动。
  output reg sensor_spi_sclk,  // 约 1.923 MHz 的共享 SPI mode-3 时钟输出，空闲电平为高。
  output reg sensor_spi_mosi,  // 发送给 PMW3901/BMI088 的共享 SPI MOSI 数据。
  output wire pmw3901_cs_n,  // PMW3901 低有效片选；初始化特殊阶段可被强制保持为低。
  output wire bmi0_acc_cs_n,  // 第一块 BMI088 加速度计低有效 SPI 片选。
  output wire bmi0_gyro_cs_n,  // 第一块 BMI088 陀螺仪低有效 SPI 片选。
  output wire bmi1_acc_cs_n,  // 第二块 BMI088 加速度计低有效 SPI 片选。
  output wire bmi1_gyro_cs_n,  // 第二块 BMI088 陀螺仪低有效 SPI 片选。
  output reg sensor_debug_uart_tx,  // 115200 bit/s @M 标定遥测输出，独立固件连接板载 USB-UART RX。
  output reg pmw3901_online,  // PMW3901 产品 ID=49、反 ID=B6 且初始化完成后的在线锁存标志。
  output reg bmi0_online,  // 第一块 BMI088 加速度/陀螺 ID 与连续样本均有效时的在线标志。
  output reg bmi1_online,  // 第二块 BMI088 加速度/陀螺 ID 与连续样本均有效时的在线标志。
  output reg calibration_done,  // 两块 BMI088 已累计 512 个静止样本并锁存陀螺零偏。
  output reg fused_sample_valid,  // 当前周期产生一组双 IMU 平均或单 IMU 降级融合结果。
  output reg imu_degraded,  // 只有一块 BMI088 可用时置 1；融合输出退化到在线的单块传感器。
  output reg imu_disagreement,  // 两块有效 BMI088 的加速度或角速度差持续超过门限。
  output reg [1:0] imu_source,  // 融合来源编码：0 无有效源、1 BMI0、2 BMI1、3 两块平均。
  output reg [7:0] pmw3901_product_id,  // PMW3901 寄存器 0x00 产品 ID，正常值为 0x49。
  output reg [7:0] pmw3901_inverse_id,  // PMW3901 寄存器 0x5F 反产品 ID，正常值为 0xB6。
  output reg [7:0] bmi0_accel_id,  // 第一块 BMI088 加速度芯片 ID，正常值为 0x1E。
  output reg [7:0] bmi0_gyro_id,  // 第一块 BMI088 陀螺芯片 ID，正常值为 0x0F。
  output reg [7:0] bmi1_accel_id,  // 第二块 BMI088 加速度芯片 ID，正常值为 0x1E。
  output reg [7:0] bmi1_gyro_id,  // 第二块 BMI088 陀螺芯片 ID，正常值为 0x0F。
  output reg flow_sample_valid,  // PMW3901 一组 DX/DY/SQUAL 读取完成的单周期脉冲。
  output reg [7:0] flow_squal,  // PMW3901 当前表面质量；标定工具默认要求至少 90% 帧不低于 20。
  output reg [15:0] flow_delta_x,  // PMW3901 当前采样间隔的有符号 X 增量，轴向和比例尚待实物标定。
  output reg [15:0] flow_delta_y,  // PMW3901 当前采样间隔的有符号 Y 增量，轴向和比例尚待实物标定。
  output reg [31:0] flow_accum_x,  // 自复位以来累计的有符号光流 X 计数，32 bit 回绕。
  output reg [31:0] flow_accum_y,  // 自复位以来累计的有符号光流 Y 计数，32 bit 回绕。
  output reg [15:0] fused_accel_x,  // 双 BMI088 一致性检查后的 X 轴加速度原始计数。
  output reg [15:0] fused_accel_y,  // 双 BMI088 一致性检查后的 Y 轴加速度原始计数。
  output reg [15:0] fused_accel_z,  // 双 BMI088 一致性检查后的 Z 轴加速度原始计数。
  output reg [15:0] fused_gyro_x,  // 扣除启动零偏后的融合 X 轴陀螺原始计数。
  output reg [15:0] fused_gyro_y,  // 扣除启动零偏后的融合 Y 轴陀螺原始计数。
  output reg [15:0] fused_gyro_z,  // 扣除启动零偏后的融合 Z 轴陀螺原始计数。
  output reg [15:0] sample_sequence  // 每次形成融合采样时递增的 16 bit 序号，用于检测采集缺口。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
`ifdef SENSOR_SIM_FAST
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam integer SPI_HALF_CYCLES = 2;  // “SPI / HALF / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer ONE_MS_CYCLES = 10;  // “ONE / MS / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BOOT_WAIT_CYCLES = 40;  // “BOOT / WAIT / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer RUN_WAIT_CYCLES = 100;  // “RUN / WAIT / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_SETUP_CYCLES = 2;  // “PMW / SETUP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_READ_GAP_CYCLES = 2;  // “PMW / READ / GAP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_WRITE_HOLD_CYCLES = 2;  // “PMW / WRITE / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_READ_HOLD_CYCLES = 2;  // “PMW / READ / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_WRITE_POST_CYCLES = 3;  // “PMW / WRITE / POST / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_SETUP_CYCLES = 2;  // “BMI / SETUP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_HOLD_CYCLES = 2;  // “BMI / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_POST_CYCLES = 2;  // “BMI / POST / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer CALIBRATION_SAMPLES = 8;  // 本模块固定参数“CALIBRATION / SAMPLES”，RTL 取值为 8；用于统一相关比较和计算。
  localparam integer CALIBRATION_SHIFT = 3;  // 本模块固定参数“CALIBRATION / SHIFT”，RTL 取值为 3；用于统一相关比较和计算。
  localparam integer DEBUG_PERIOD_CYCLES = 500;  // “DEBUG / PERIOD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer UART_CLKS_PER_BIT = 4;  // UART 发送状态“UART / CLKS / PER / BIT”的编码。
`else
  localparam integer SPI_HALF_CYCLES = 13;  // “SPI / HALF / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer ONE_MS_CYCLES = 50000;  // “ONE / MS / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BOOT_WAIT_CYCLES = 5000000;  // “BOOT / WAIT / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer RUN_WAIT_CYCLES = 250000;  // “RUN / WAIT / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_SETUP_CYCLES = 2500;  // “PMW / SETUP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_READ_GAP_CYCLES = 2500;  // “PMW / READ / GAP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_WRITE_HOLD_CYCLES = 2500;  // “PMW / WRITE / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_READ_HOLD_CYCLES = 5000;  // “PMW / READ / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer PMW_WRITE_POST_CYCLES = 10000;  // “PMW / WRITE / POST / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_SETUP_CYCLES = 4;  // “BMI / SETUP / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_HOLD_CYCLES = 4;  // “BMI / HOLD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer BMI_POST_CYCLES = 100;  // “BMI / POST / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer CALIBRATION_SAMPLES = 512;  // 本模块固定参数“CALIBRATION / SAMPLES”，RTL 取值为 512；用于统一相关比较和计算。
  localparam integer CALIBRATION_SHIFT = 9;  // 本模块固定参数“CALIBRATION / SHIFT”，RTL 取值为 9；用于统一相关比较和计算。
  localparam integer DEBUG_PERIOD_CYCLES = 5000000;  // “DEBUG / PERIOD / CYCLES”对应的系统时钟周期数；达到该值后触发状态转换或超时。
  localparam integer UART_CLKS_PER_BIT = 434;  // UART 发送状态“UART / CLKS / PER / BIT”的编码。
`endif

  localparam [2:0] DEV_PMW       = 3'd0;  // 本模块固定参数“DEV / PMW”，RTL 取值为 3'd0；用于统一相关比较和计算。
  localparam [2:0] DEV_BMI0_ACC  = 3'd1;  // 本模块固定参数“DEV / BMI0 / ACC”，RTL 取值为 3'd1；用于统一相关比较和计算。
  localparam [2:0] DEV_BMI0_GYRO = 3'd2;  // 本模块固定参数“DEV / BMI0 / GYRO”，RTL 取值为 3'd2；用于统一相关比较和计算。
  localparam [2:0] DEV_BMI1_ACC  = 3'd3;  // 本模块固定参数“DEV / BMI1 / ACC”，RTL 取值为 3'd3；用于统一相关比较和计算。
  localparam [2:0] DEV_BMI1_GYRO = 3'd4;  // 本模块固定参数“DEV / BMI1 / GYRO”，RTL 取值为 3'd4；用于统一相关比较和计算。

  localparam [3:0] SPI_IDLE   = 4'd0;  // 本模块固定参数“SPI / IDLE”，RTL 取值为 4'd0；用于统一相关比较和计算。
  localparam [3:0] SPI_SETUP  = 4'd1;  // 本模块固定参数“SPI / SETUP”，RTL 取值为 4'd1；用于统一相关比较和计算。
  localparam [3:0] SPI_DRIVE  = 4'd2;  // 本模块固定参数“SPI / DRIVE”，RTL 取值为 4'd2；用于统一相关比较和计算。
  localparam [3:0] SPI_SAMPLE = 4'd3;  // 本模块固定参数“SPI / SAMPLE”，RTL 取值为 4'd3；用于统一相关比较和计算。
  localparam [3:0] SPI_GAP    = 4'd4;  // 本模块固定参数“SPI / GAP”，RTL 取值为 4'd4；用于统一相关比较和计算。
  localparam [3:0] SPI_HOLD   = 4'd5;  // 本模块固定参数“SPI / HOLD”，RTL 取值为 4'd5；用于统一相关比较和计算。
  localparam [3:0] SPI_POST   = 4'd6;  // 本模块固定参数“SPI / POST”，RTL 取值为 4'd6；用于统一相关比较和计算。

  localparam [7:0] ST_BOOT               = 8'd0;  // 超声/控制状态机“ST / BOOT”的状态编码。
  localparam [7:0] ST_PMW_CS_LOW         = 8'd1;  // 超声/控制状态机“ST / PMW / CS / LOW”的状态编码。
  localparam [7:0] ST_PMW_CS_HIGH        = 8'd2;  // 超声/控制状态机“ST / PMW / CS / HIGH”的状态编码。
  localparam [7:0] ST_PMW_RESET          = 8'd3;  // 超声/控制状态机“ST / PMW / RESET”的状态编码。
  localparam [7:0] ST_PMW_RESET_WAIT     = 8'd4;  // 超声/控制状态机“ST / PMW / RESET / WAIT”的状态编码。
  localparam [7:0] ST_PMW_ID             = 8'd5;  // 超声/控制状态机“ST / PMW / ID”的状态编码。
  localparam [7:0] ST_PMW_INVERSE_ID     = 8'd6;  // 超声/控制状态机“ST / PMW / INVERSE / ID”的状态编码。
  localparam [7:0] ST_PMW_RETRY_WAIT     = 8'd7;  // 超声/控制状态机“ST / PMW / RETRY / WAIT”的状态编码。
  localparam [7:0] ST_PMW_CLEAR          = 8'd8;  // 超声/控制状态机“ST / PMW / CLEAR”的状态编码。
  localparam [7:0] ST_PMW_INIT           = 8'd9;  // 超声/控制状态机“ST / PMW / INIT”的状态编码。
  localparam [7:0] ST_PMW_INIT_LONG_WAIT = 8'd10;  // 超声/控制状态机“ST / PMW / INIT / LONG / WAIT”的状态编码。
  localparam [7:0] ST_BMI_INIT           = 8'd11;  // 超声/控制状态机“ST / BMI / INIT”的状态编码。
  localparam [7:0] ST_RUN_WAIT           = 8'd12;  // 超声/控制状态机“ST / RUN / WAIT”的状态编码。
  localparam [7:0] ST_IMU0_ACCEL         = 8'd13;  // 超声/控制状态机“ST / IMU0 / ACCEL”的状态编码。
  localparam [7:0] ST_IMU0_GYRO          = 8'd14;  // 超声/控制状态机“ST / IMU0 / GYRO”的状态编码。
  localparam [7:0] ST_IMU1_ACCEL         = 8'd15;  // 超声/控制状态机“ST / IMU1 / ACCEL”的状态编码。
  localparam [7:0] ST_IMU1_GYRO          = 8'd16;  // 超声/控制状态机“ST / IMU1 / GYRO”的状态编码。
  localparam [7:0] ST_FUSION_STROBE      = 8'd17;  // 超声/控制状态机“ST / FUSION / STROBE”的状态编码。
  localparam [7:0] ST_PMW_MOTION         = 8'd18;  // 超声/控制状态机“ST / PMW / MOTION”的状态编码。
  localparam [7:0] ST_PMW_DX_LOW         = 8'd19;  // 超声/控制状态机“ST / PMW / DX / LOW”的状态编码。
  localparam [7:0] ST_PMW_DX_HIGH        = 8'd20;  // 超声/控制状态机“ST / PMW / DX / HIGH”的状态编码。
  localparam [7:0] ST_PMW_DY_LOW         = 8'd21;  // 超声/控制状态机“ST / PMW / DY / LOW”的状态编码。
  localparam [7:0] ST_PMW_DY_HIGH        = 8'd22;  // 超声/控制状态机“ST / PMW / DY / HIGH”的状态编码。
  localparam [7:0] ST_PMW_SQUAL          = 8'd23;  // 超声/控制状态机“ST / PMW / SQUAL”的状态编码。

  localparam [1:0] UART_IDLE  = 2'd0;  // UART 发送状态“UART / IDLE”的编码。
  localparam [1:0] UART_START = 2'd1;  // UART 发送状态“UART / START”的编码。
  localparam [1:0] UART_DATA  = 2'd2;  // UART 发送状态“UART / DATA”的编码。
  localparam [1:0] UART_STOP  = 2'd3;  // UART 发送状态“UART / STOP”的编码。

  localparam [16:0] ACCEL_DISAGREE_LIMIT = 17'd1500;  // 本模块固定参数“ACCEL / DISAGREE / LIMIT”，RTL 取值为 17'd1500；用于统一相关比较和计算。
  localparam [17:0] GYRO_DISAGREE_LIMIT  = 18'd2000;  // 本模块固定参数“GYRO / DISAGREE / LIMIT”，RTL 取值为 18'd2000；用于统一相关比较和计算。
  localparam [15:0] CALIBRATION_GYRO_LIMIT = 16'd2000;  // 本模块固定参数“CALIBRATION / GYRO / LIMIT”，RTL 取值为 16'd2000；用于统一相关比较和计算。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [4:0] spi_cs_n_raw;  // 五个 SPI 从设备的低有效片选寄存器，bit0..4 依次对应 PMW、BMI0 ACC/GYRO、BMI1 ACC/GYRO。
  reg pmw_force_cs_low;  // PMW3901 上电初始化要求片选保持低的特殊阶段控制。
  reg spi_start;  // 寄存器，保存“SPI / 启动”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg spi_busy;  // “SPI / 忙标志”信号；为 1 时当前事务尚未结束，上层必须等待而不能启动冲突操作。
  reg spi_done;  // 寄存器，保存“SPI / 完成标志”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [3:0] spi_state;  // 共享 SPI mode-3 引擎的空闲、建立、驱动、采样、字节间隔、保持和结束状态。
  reg [3:0] spi_length_latched;  // 一次 SPI 命令锁存的总字节数，传输开始后不再受主状态机修改。
  reg [3:0] spi_byte_count;  // 4 bit 用于“SPI / byte / 计数”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。
  reg [2:0] spi_bit_index;  // 3 bit 用于“SPI / BIT / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  reg [15:0] spi_half_count;  // 16 bit 用于“SPI / half / 计数”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。
  reg [15:0] spi_delay_count;  // 16 bit 用于“SPI / delay / 计数”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。
  reg [15:0] spi_setup_latched;  // 16 bit 寄存器，保存“SPI / setup / latched”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] spi_gap_latched;  // 16 bit 寄存器，保存“SPI / GAP / latched”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] spi_hold_latched;  // 16 bit 寄存器，保存“SPI / 保持时间 / latched”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] spi_post_latched;  // 16 bit 寄存器，保存“SPI / post / latched”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [71:0] spi_tx_shift;  // 最多 9 字节的 SPI 发送移位寄存器，最高有效字节先发送。
  reg [71:0] spi_rx_shift;  // SPI 时钟采样期间逐位移入的临时 MISO 数据。
  reg [71:0] spi_rx_data;  // 一条 SPI 命令完成时锁存的最多 9 字节返回数据。

  reg [7:0] main_state;  // 传感器初始化与轮询总状态，覆盖 PMW 初始化、BMI 配置、双 IMU 和光流采样。
  reg command_inflight;  // 主状态机已经提交一条 SPI 命令，等待 spi_done 后才允许推进。
  reg [31:0] state_wait_count;  // 32 bit 用于“状态 / 等待 / 计数”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。
  reg [6:0] pmw_init_index;  // PMW3901 参考初始化表当前寄存器项下标。
  reg [2:0] pmw_clear_index;  // 3 bit 用于“PMW / clear / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  reg [1:0] pmw_retry_count;  // PMW3901 ID 或初始化失败后的有限重试次数。
  reg bmi_index;  // 用于“BMI / 索引”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。
  reg [4:0] bmi_init_step;  // 两块 BMI088 加速度计和陀螺仪配置序列的当前步骤。
  reg [7:0] current_bmi_accel_id;  // 8 bit 寄存器，保存“当前 / BMI / accel / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] current_bmi_gyro_id;  // 8 bit 寄存器，保存“当前 / BMI / gyro / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg pmw_poll_toggle;  // 寄存器，保存“PMW / poll / toggle”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg pair_sample_strobe;  // 两块 BMI088 一轮采样均完成后产生的融合触发脉冲。
  reg flow_data_seen;  // 至少接收过一次 PMW3901 光流采样；进入 @M 状态 bit7。
  reg fused_data_ready;  // “fused / 数据 / 就绪”控制标志；为 1 表示名称描述的状态当前成立。

  reg signed [15:0] bmi0_accel_x_raw;  // 16 bit 寄存器，保存“bmi0 / accel / X 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_accel_y_raw;  // 16 bit 寄存器，保存“bmi0 / accel / Y 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_accel_z_raw;  // 16 bit 寄存器，保存“bmi0 / accel / Z / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_gyro_x_raw;  // 16 bit 寄存器，保存“bmi0 / gyro / X 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_gyro_y_raw;  // 16 bit 寄存器，保存“bmi0 / gyro / Y 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_gyro_z_raw;  // 16 bit 寄存器，保存“bmi0 / gyro / Z / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_accel_x_raw;  // 16 bit 寄存器，保存“bmi1 / accel / X 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_accel_y_raw;  // 16 bit 寄存器，保存“bmi1 / accel / Y 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_accel_z_raw;  // 16 bit 寄存器，保存“bmi1 / accel / Z / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_gyro_x_raw;  // 16 bit 寄存器，保存“bmi1 / gyro / X 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_gyro_y_raw;  // 16 bit 寄存器，保存“bmi1 / gyro / Y 坐标 / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_gyro_z_raw;  // 16 bit 寄存器，保存“bmi1 / gyro / Z / 原始值”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [3:0] bmi0_bad_sample_count;  // 第一块 BMI088 连续全零/全一坏样本计数，达到门限后判离线。
  reg [3:0] bmi1_bad_sample_count;  // 第二块 BMI088 连续全零/全一坏样本计数，达到门限后判离线。

  reg [7:0] pmw_dx_low;  // 8 bit 寄存器，保存“PMW / DX / LOW”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] pmw_dx_high;  // 8 bit 寄存器，保存“PMW / DX / high”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] pmw_dy_low;  // 8 bit 寄存器，保存“PMW / DY / LOW”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] pmw_dy_high;  // 8 bit 寄存器，保存“PMW / DY / high”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  reg [9:0] calibration_count;  // 启动静止陀螺偏置已经累计的样本数，目标为 512。
  reg signed [31:0] bmi0_bias_sum_x;  // 第一块 BMI088 X 轴启动偏置的 512 样本有符号累加和。
  reg signed [31:0] bmi0_bias_sum_y;  // 32 bit “bmi0 / bias / SUM / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg signed [31:0] bmi0_bias_sum_z;  // 32 bit 寄存器，保存“bmi0 / bias / SUM / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [31:0] bmi1_bias_sum_x;  // 32 bit “bmi1 / bias / SUM / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg signed [31:0] bmi1_bias_sum_y;  // 32 bit “bmi1 / bias / SUM / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg signed [31:0] bmi1_bias_sum_z;  // 32 bit 寄存器，保存“bmi1 / bias / SUM / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi0_gyro_bias_x;  // 第一块 BMI088 X 轴由启动样本均值得到的陀螺零偏。
  reg signed [15:0] bmi0_gyro_bias_y;  // 16 bit “bmi0 / gyro / bias / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg signed [15:0] bmi0_gyro_bias_z;  // 16 bit 寄存器，保存“bmi0 / gyro / bias / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg signed [15:0] bmi1_gyro_bias_x;  // 第二块 BMI088 X 轴由启动样本均值得到的陀螺零偏。
  reg signed [15:0] bmi1_gyro_bias_y;  // 16 bit “bmi1 / gyro / bias / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg signed [15:0] bmi1_gyro_bias_z;  // 16 bit 寄存器，保存“bmi1 / gyro / bias / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  reg cmd_valid;  // “命令 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  reg [2:0] cmd_device;  // 3 bit 组合过程变量“命令 / device”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。
  reg [3:0] cmd_length;  // 4 bit “命令”长度结果；由步数和单步长度组合换算。
  reg [71:0] cmd_tx_data;  // 72 bit 组合过程变量“命令 / 发送 / 数据”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。
  reg [15:0] cmd_setup_cycles;  // 16 bit 组合过程变量“命令 / setup / cycles”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。
  reg [15:0] cmd_gap_cycles;  // 16 bit 组合过程变量“命令 / GAP / cycles”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。
  reg [15:0] cmd_hold_cycles;  // 16 bit 组合过程变量“命令 / 保持时间 / cycles”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。
  reg [15:0] cmd_post_cycles;  // 16 bit 组合过程变量“命令 / post / cycles”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。

  reg [31:0] debug_period_count;  // @M 遥测 100 ms 周期计数器。
  reg debug_sending;  // 固定 99 字节 @M 帧正在原子发送，期间快照不会被新采样覆盖。
  reg [6:0] debug_char_index;  // 当前 @M 帧正在发送的字符下标。
  reg [15:0] debug_sequence;  // 16 bit 寄存器，保存“debug / sequence”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] uart_data_byte;  // 8 bit 寄存器，保存“UART / 数据 / byte”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg uart_start_pulse;  // “UART / 启动”事件脉冲；条件成立时只保持一个 clk 周期，供下游状态机触发一次动作。
  reg uart_busy;  // 底层 UART 正在发送一个字节。
  reg uart_busy_d;  // uart_busy 的一拍延迟，用于生成 uart_done。
  reg [1:0] uart_state;  // UART 字节发送状态，依次为 IDLE、START、DATA 和 STOP。
  reg [15:0] uart_clock_count;  // 16 bit 用于“UART / clock / 计数”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。
  reg [2:0] uart_bit_index;  // UART 当前发送的数据位编号 0..7。
  reg [7:0] uart_shift;  // 8 bit 寄存器，保存“UART / 移位缓存”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  reg [15:0] snap_sequence;  // 16 bit 寄存器，保存“snap / sequence”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_status;  // 开始一帧 @M 时锁存的在线、校准、融合、降级、分歧和光流状态位。
  reg [7:0] snap_squal;  // 8 bit 寄存器，保存“snap / squal”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_pmw_id;  // 8 bit 寄存器，保存“snap / PMW / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_pmw_inverse_id;  // 8 bit 寄存器，保存“snap / PMW / inverse / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_bmi0_accel_id;  // 8 bit 寄存器，保存“snap / bmi0 / accel / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_bmi0_gyro_id;  // 8 bit 寄存器，保存“snap / bmi0 / gyro / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_bmi1_accel_id;  // 8 bit 寄存器，保存“snap / bmi1 / accel / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [7:0] snap_bmi1_gyro_id;  // 8 bit 寄存器，保存“snap / bmi1 / gyro / ID”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] snap_flow_dx;  // 16 bit 寄存器，保存“snap / flow / DX”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] snap_flow_dy;  // 16 bit 寄存器，保存“snap / flow / DY”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [31:0] snap_flow_accum_x;  // 开始一帧 @M 时锁存的 32 bit 累计光流 X，保证整行字段一致。
  reg [31:0] snap_flow_accum_y;  // 32 bit “snap / flow / accum / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg [15:0] snap_accel_x;  // 16 bit “snap / accel / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg [15:0] snap_accel_y;  // 16 bit “snap / accel / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg [15:0] snap_accel_z;  // 16 bit 寄存器，保存“snap / accel / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] snap_gyro_x;  // 16 bit “snap / gyro / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg [15:0] snap_gyro_y;  // 16 bit “snap / gyro / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  reg [15:0] snap_gyro_z;  // 开始一帧 @M 时锁存的融合 Z 轴陀螺值。
  reg [15:0] snap_bmi0_gyro_z;  // 16 bit 寄存器，保存“snap / bmi0 / gyro / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。
  reg [15:0] snap_bmi1_gyro_z;  // 16 bit 寄存器，保存“snap / bmi1 / gyro / Z”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。

  wire uart_done;  // 检测到 UART busy 从 1 变 0，表示一个完整字节已经发送。
  wire bmi0_bad_sample;  // 组合网络，传递“bmi0 / BAD / 采样”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire bmi1_bad_sample;  // 组合网络，传递“bmi1 / BAD / 采样”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire calibration_stationary;  // 两块陀螺三轴原始值均未超过启动静止门限时为 1。
  wire signed [16:0] bmi0_gyro_x_corrected;  // 17 bit 组合网络，传递“bmi0 / gyro / X 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [16:0] bmi0_gyro_y_corrected;  // 17 bit 组合网络，传递“bmi0 / gyro / Y 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [16:0] bmi0_gyro_z_corrected;  // 17 bit 组合网络，传递“bmi0 / gyro / Z / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [16:0] bmi1_gyro_x_corrected;  // 17 bit 组合网络，传递“bmi1 / gyro / X 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [16:0] bmi1_gyro_y_corrected;  // 17 bit 组合网络，传递“bmi1 / gyro / Y 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [16:0] bmi1_gyro_z_corrected;  // 17 bit 组合网络，传递“bmi1 / gyro / Z / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [16:0] accel_diff_x;  // 17 bit “accel / 有符号差值 / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire [16:0] accel_diff_y;  // 17 bit “accel / 有符号差值 / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire [16:0] accel_diff_z;  // 17 bit 组合网络，传递“accel / 有符号差值 / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [17:0] gyro_diff_x;  // 两块已扣偏置 X 轴陀螺的绝对差，用于一致性判定。
  wire [17:0] gyro_diff_y;  // 18 bit “gyro / 有符号差值 / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire [17:0] gyro_diff_z;  // 18 bit 组合网络，传递“gyro / 有符号差值 / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [15:0] pmw_init_entry;  // pmw_init_word 当前返回的寄存器地址/数值组合。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“pmw3901_cs_n”：PMW3901 低有效片选；初始化特殊阶段可被强制保持为低。 该组合结果直接由 `pmw_force_cs_low`（PMW / 强制请求 / CS / LOW）、`spi_cs_n_raw`（SPI / CS / N / 原始值）的当前值决定，不引入时钟周期延迟。
  assign pmw3901_cs_n = pmw_force_cs_low ? 1'b0 : spi_cs_n_raw[0];
  // [ASSIGN] 组合定义“bmi0_acc_cs_n”：第一块 BMI088 加速度计低有效 SPI 片选。 该组合结果直接由 `spi_cs_n_raw`（SPI / CS / N / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi0_acc_cs_n = spi_cs_n_raw[1];
  // [ASSIGN] 组合定义“bmi0_gyro_cs_n”：第一块 BMI088 陀螺仪低有效 SPI 片选。 该组合结果直接由 `spi_cs_n_raw`（SPI / CS / N / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi0_gyro_cs_n = spi_cs_n_raw[2];
  // [ASSIGN] 组合定义“bmi1_acc_cs_n”：第二块 BMI088 加速度计低有效 SPI 片选。 该组合结果直接由 `spi_cs_n_raw`（SPI / CS / N / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi1_acc_cs_n = spi_cs_n_raw[3];
  // [ASSIGN] 组合定义“bmi1_gyro_cs_n”：第二块 BMI088 陀螺仪低有效 SPI 片选。 该组合结果直接由 `spi_cs_n_raw`（SPI / CS / N / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi1_gyro_cs_n = spi_cs_n_raw[4];
  // [ASSIGN] 组合定义“uart_done”：检测到 UART busy 从 1 变 0，表示一个完整字节已经发送。 该组合结果直接由 `uart_busy_d`（UART / 忙标志 / D）、`uart_busy`（UART / 忙标志）的当前值决定，不引入时钟周期延迟。
  assign uart_done = uart_busy_d && !uart_busy;

  // [ASSIGN] 组合定义“bmi0_bad_sample”：组合网络，传递“bmi0 / BAD / 采样”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi0_accel_x_raw`（bmi0 / accel / X 坐标 / 原始值）、`bmi0_accel_y_raw`（bmi0 / accel / Y 坐标 / 原始值）、`bmi0_accel_z_raw`（bmi0 / accel / Z / 原始值）、`bmi0_gyro_x_raw`（bmi0 / gyro / X 坐标 / 原始值）、`bmi0_gyro_y_raw`（bmi0 / gyro / Y 坐标 / 原始值）、`bmi0_gyro_z_raw`（bmi0 / gyro / Z / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi0_bad_sample = ((bmi0_accel_x_raw == 16'hffff) &&
                            (bmi0_accel_y_raw == 16'hffff) &&
                            (bmi0_accel_z_raw == 16'hffff) &&
                            (bmi0_gyro_x_raw == 16'hffff) &&
                            (bmi0_gyro_y_raw == 16'hffff) &&
                            (bmi0_gyro_z_raw == 16'hffff)) ||
                           ((bmi0_accel_x_raw == 16'h0000) &&
                            (bmi0_accel_y_raw == 16'h0000) &&
                            (bmi0_accel_z_raw == 16'h0000) &&
                            (bmi0_gyro_x_raw == 16'h0000) &&
                            (bmi0_gyro_y_raw == 16'h0000) &&
                            (bmi0_gyro_z_raw == 16'h0000));

  // [ASSIGN] 组合定义“bmi1_bad_sample”：组合网络，传递“bmi1 / BAD / 采样”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi1_accel_x_raw`（bmi1 / accel / X 坐标 / 原始值）、`bmi1_accel_y_raw`（bmi1 / accel / Y 坐标 / 原始值）、`bmi1_accel_z_raw`（bmi1 / accel / Z / 原始值）、`bmi1_gyro_x_raw`（bmi1 / gyro / X 坐标 / 原始值）、`bmi1_gyro_y_raw`（bmi1 / gyro / Y 坐标 / 原始值）、`bmi1_gyro_z_raw`（bmi1 / gyro / Z / 原始值）的当前值决定，不引入时钟周期延迟。
  assign bmi1_bad_sample = ((bmi1_accel_x_raw == 16'hffff) &&
                            (bmi1_accel_y_raw == 16'hffff) &&
                            (bmi1_accel_z_raw == 16'hffff) &&
                            (bmi1_gyro_x_raw == 16'hffff) &&
                            (bmi1_gyro_y_raw == 16'hffff) &&
                            (bmi1_gyro_z_raw == 16'hffff)) ||
                           ((bmi1_accel_x_raw == 16'h0000) &&
                            (bmi1_accel_y_raw == 16'h0000) &&
                            (bmi1_accel_z_raw == 16'h0000) &&
                            (bmi1_gyro_x_raw == 16'h0000) &&
                            (bmi1_gyro_y_raw == 16'h0000) &&
                            (bmi1_gyro_z_raw == 16'h0000));

  // [ASSIGN] 组合定义“calibration_stationary”：两块陀螺三轴原始值均未超过启动静止门限时为 1。 该组合结果直接由 `bmi0_online`（bmi0 / 在线状态）、`abs16`（abs16）、`bmi0_gyro_x_raw`（bmi0 / gyro / X 坐标 / 原始值）、`CALIBRATION_GYRO_LIMIT`（CALIBRATION / GYRO / LIMIT）、`bmi0_gyro_y_raw`（bmi0 / gyro / Y 坐标 / 原始值）、`bmi0_gyro_z_raw`（bmi0 / gyro / Z / 原始值）等当前值决定，不引入时钟周期延迟。
  assign calibration_stationary =
      (!bmi0_online || ((abs16(bmi0_gyro_x_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi0_gyro_y_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi0_gyro_z_raw) <= CALIBRATION_GYRO_LIMIT))) &&
      (!bmi1_online || ((abs16(bmi1_gyro_x_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi1_gyro_y_raw) <= CALIBRATION_GYRO_LIMIT) &&
                        (abs16(bmi1_gyro_z_raw) <= CALIBRATION_GYRO_LIMIT)));

  // [ASSIGN] 组合定义“bmi0_gyro_x_corrected”：组合网络，传递“bmi0 / gyro / X 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi0_gyro_x_raw`（bmi0 / gyro / X 坐标 / 原始值）、`bmi0_gyro_bias_x`（bmi0 / gyro / bias / X 坐标）的当前值决定，不引入时钟周期延迟。
  assign bmi0_gyro_x_corrected = $signed(bmi0_gyro_x_raw) - $signed(bmi0_gyro_bias_x);
  // [ASSIGN] 组合定义“bmi0_gyro_y_corrected”：组合网络，传递“bmi0 / gyro / Y 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi0_gyro_y_raw`（bmi0 / gyro / Y 坐标 / 原始值）、`bmi0_gyro_bias_y`（bmi0 / gyro / bias / Y 坐标）的当前值决定，不引入时钟周期延迟。
  assign bmi0_gyro_y_corrected = $signed(bmi0_gyro_y_raw) - $signed(bmi0_gyro_bias_y);
  // [ASSIGN] 组合定义“bmi0_gyro_z_corrected”：组合网络，传递“bmi0 / gyro / Z / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi0_gyro_z_raw`（bmi0 / gyro / Z / 原始值）、`bmi0_gyro_bias_z`（bmi0 / gyro / bias / Z）的当前值决定，不引入时钟周期延迟。
  assign bmi0_gyro_z_corrected = $signed(bmi0_gyro_z_raw) - $signed(bmi0_gyro_bias_z);
  // [ASSIGN] 组合定义“bmi1_gyro_x_corrected”：组合网络，传递“bmi1 / gyro / X 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi1_gyro_x_raw`（bmi1 / gyro / X 坐标 / 原始值）、`bmi1_gyro_bias_x`（bmi1 / gyro / bias / X 坐标）的当前值决定，不引入时钟周期延迟。
  assign bmi1_gyro_x_corrected = $signed(bmi1_gyro_x_raw) - $signed(bmi1_gyro_bias_x);
  // [ASSIGN] 组合定义“bmi1_gyro_y_corrected”：组合网络，传递“bmi1 / gyro / Y 坐标 / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi1_gyro_y_raw`（bmi1 / gyro / Y 坐标 / 原始值）、`bmi1_gyro_bias_y`（bmi1 / gyro / bias / Y 坐标）的当前值决定，不引入时钟周期延迟。
  assign bmi1_gyro_y_corrected = $signed(bmi1_gyro_y_raw) - $signed(bmi1_gyro_bias_y);
  // [ASSIGN] 组合定义“bmi1_gyro_z_corrected”：组合网络，传递“bmi1 / gyro / Z / corrected”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `bmi1_gyro_z_raw`（bmi1 / gyro / Z / 原始值）、`bmi1_gyro_bias_z`（bmi1 / gyro / bias / Z）的当前值决定，不引入时钟周期延迟。
  assign bmi1_gyro_z_corrected = $signed(bmi1_gyro_z_raw) - $signed(bmi1_gyro_bias_z);

  // [ASSIGN] 组合定义“accel_diff_x”：“accel / 有符号差值 / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。 该组合结果直接由 `abs_diff16`（ABS / diff16）、`bmi0_accel_x_raw`（bmi0 / accel / X 坐标 / 原始值）、`bmi1_accel_x_raw`（bmi1 / accel / X 坐标 / 原始值）的当前值决定，不引入时钟周期延迟。
  assign accel_diff_x = abs_diff16(bmi0_accel_x_raw, bmi1_accel_x_raw);
  // [ASSIGN] 组合定义“accel_diff_y”：“accel / 有符号差值 / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。 该组合结果直接由 `abs_diff16`（ABS / diff16）、`bmi0_accel_y_raw`（bmi0 / accel / Y 坐标 / 原始值）、`bmi1_accel_y_raw`（bmi1 / accel / Y 坐标 / 原始值）的当前值决定，不引入时钟周期延迟。
  assign accel_diff_y = abs_diff16(bmi0_accel_y_raw, bmi1_accel_y_raw);
  // [ASSIGN] 组合定义“accel_diff_z”：组合网络，传递“accel / 有符号差值 / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `abs_diff16`（ABS / diff16）、`bmi0_accel_z_raw`（bmi0 / accel / Z / 原始值）、`bmi1_accel_z_raw`（bmi1 / accel / Z / 原始值）的当前值决定，不引入时钟周期延迟。
  assign accel_diff_z = abs_diff16(bmi0_accel_z_raw, bmi1_accel_z_raw);
  // [ASSIGN] 组合定义“gyro_diff_x”：两块已扣偏置 X 轴陀螺的绝对差，用于一致性判定。 该组合结果直接由 `abs_diff17`（ABS / diff17）、`bmi0_gyro_x_corrected`（bmi0 / gyro / X 坐标 / corrected）、`bmi1_gyro_x_corrected`（bmi1 / gyro / X 坐标 / corrected）的当前值决定，不引入时钟周期延迟。
  assign gyro_diff_x = abs_diff17(bmi0_gyro_x_corrected, bmi1_gyro_x_corrected);
  // [ASSIGN] 组合定义“gyro_diff_y”：“gyro / 有符号差值 / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。 该组合结果直接由 `abs_diff17`（ABS / diff17）、`bmi0_gyro_y_corrected`（bmi0 / gyro / Y 坐标 / corrected）、`bmi1_gyro_y_corrected`（bmi1 / gyro / Y 坐标 / corrected）的当前值决定，不引入时钟周期延迟。
  assign gyro_diff_y = abs_diff17(bmi0_gyro_y_corrected, bmi1_gyro_y_corrected);
  // [ASSIGN] 组合定义“gyro_diff_z”：组合网络，传递“gyro / 有符号差值 / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。 该组合结果直接由 `abs_diff17`（ABS / diff17）、`bmi0_gyro_z_corrected`（bmi0 / gyro / Z / corrected）、`bmi1_gyro_z_corrected`（bmi1 / gyro / Z / corrected）的当前值决定，不引入时钟周期延迟。
  assign gyro_diff_z = abs_diff17(bmi0_gyro_z_corrected, bmi1_gyro_z_corrected);
  // [ASSIGN] 组合定义“pmw_init_entry”：pmw_init_word 当前返回的寄存器地址/数值组合。 该组合结果直接由 `pmw_init_word`（PMW / init / 字表项）、`pmw_init_index`（PMW / init / 索引）的当前值决定，不引入时钟周期延迟。
  assign pmw_init_entry = pmw_init_word(pmw_init_index);

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: abs16
  // 返回 16 bit 二补码输入的无符号绝对值，用于静止和异常门限。
  // -----------------------------------------------------------------------------
  function [15:0] abs16;
    input [15:0] value;  // 16 bit 函数 `abs16` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      abs16 = value[15] ? ((~value) + 16'd1) : value;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: abs_diff16
  // 计算两个 16 bit 有符号样本差的 17 bit 绝对值。
  // -----------------------------------------------------------------------------
  function [16:0] abs_diff16;
    input signed [15:0] a;  // 16 bit 函数 `abs_diff16` 的“A”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input signed [15:0] b;  // 16 bit 函数 `abs_diff16` 的“B”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    reg signed [16:0] difference;  // 17 bit 函数 `abs_diff16` 内部的组合临时变量“difference”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      difference = a - b;
      abs_diff16 = difference[16] ? -difference : difference;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: abs_diff17
  // 计算两个 17 bit 校正陀螺值差的 18 bit 绝对值。
  // -----------------------------------------------------------------------------
  function [17:0] abs_diff17;
    input signed [16:0] a;  // 17 bit 函数 `abs_diff17` 的“A”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input signed [16:0] b;  // 17 bit 函数 `abs_diff17` 的“B”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    reg signed [17:0] difference;  // 18 bit 函数 `abs_diff17` 内部的组合临时变量“difference”；仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。
    begin
      difference = a - b;
      abs_diff17 = difference[17] ? -difference : difference;
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: saturate16
  // 把 17 bit 融合中间值饱和到 16 bit 有符号输出范围。
  // -----------------------------------------------------------------------------
  function [15:0] saturate16;
    input signed [17:0] value;  // 18 bit 函数 `saturate16` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if (value > 18'sd32767)
        saturate16 = 16'h7fff;
      else if (value < -18'sd32768)
        saturate16 = 16'h8000;
      else
        saturate16 = value[15:0];
    end
  endfunction



  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: pmw_init_word
  // 按初始化索引返回 PMW3901 寄存器地址和值。
  // -----------------------------------------------------------------------------
  function [15:0] pmw_init_word;
    input [6:0] index;  // 7 bit 函数 `pmw_init_word` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (index)
        7'd0:  pmw_init_word = 16'h7f00;
        7'd1:  pmw_init_word = 16'h61ad;
        7'd2:  pmw_init_word = 16'h7f03;
        7'd3:  pmw_init_word = 16'h4000;
        7'd4:  pmw_init_word = 16'h7f05;
        7'd5:  pmw_init_word = 16'h41b3;
        7'd6:  pmw_init_word = 16'h43f1;
        7'd7:  pmw_init_word = 16'h4514;
        7'd8:  pmw_init_word = 16'h5b32;
        7'd9:  pmw_init_word = 16'h5f34;
        7'd10: pmw_init_word = 16'h7b08;
        7'd11: pmw_init_word = 16'h7f06;
        7'd12: pmw_init_word = 16'h441b;
        7'd13: pmw_init_word = 16'h40bf;
        7'd14: pmw_init_word = 16'h4e3f;
        7'd15: pmw_init_word = 16'h7f08;
        7'd16: pmw_init_word = 16'h6520;
        7'd17: pmw_init_word = 16'h6a18;
        7'd18: pmw_init_word = 16'h7f09;
        7'd19: pmw_init_word = 16'h4faf;
        7'd20: pmw_init_word = 16'h5f40;
        7'd21: pmw_init_word = 16'h4880;
        7'd22: pmw_init_word = 16'h4980;
        7'd23: pmw_init_word = 16'h5777;
        7'd24: pmw_init_word = 16'h6078;
        7'd25: pmw_init_word = 16'h6178;
        7'd26: pmw_init_word = 16'h6208;
        7'd27: pmw_init_word = 16'h6350;
        7'd28: pmw_init_word = 16'h7f0a;
        7'd29: pmw_init_word = 16'h4560;
        7'd30: pmw_init_word = 16'h7f00;
        7'd31: pmw_init_word = 16'h4d11;
        7'd32: pmw_init_word = 16'h5580;
        7'd33: pmw_init_word = 16'h741f;
        7'd34: pmw_init_word = 16'h751f;
        7'd35: pmw_init_word = 16'h4a78;
        7'd36: pmw_init_word = 16'h4b78;
        7'd37: pmw_init_word = 16'h4408;
        7'd38: pmw_init_word = 16'h4550;
        7'd39: pmw_init_word = 16'h64ff;
        7'd40: pmw_init_word = 16'h651f;
        7'd41: pmw_init_word = 16'h7f14;
        7'd42: pmw_init_word = 16'h6560;
        7'd43: pmw_init_word = 16'h6608;
        7'd44: pmw_init_word = 16'h6378;
        7'd45: pmw_init_word = 16'h7f15;
        7'd46: pmw_init_word = 16'h4858;
        7'd47: pmw_init_word = 16'h7f07;
        7'd48: pmw_init_word = 16'h410d;
        7'd49: pmw_init_word = 16'h4314;
        7'd50: pmw_init_word = 16'h4b0e;
        7'd51: pmw_init_word = 16'h450f;
        7'd52: pmw_init_word = 16'h4442;
        7'd53: pmw_init_word = 16'h4c80;
        7'd54: pmw_init_word = 16'h7f10;
        7'd55: pmw_init_word = 16'h5b02;
        7'd56: pmw_init_word = 16'h7f07;
        7'd57: pmw_init_word = 16'h4041;
        7'd58: pmw_init_word = 16'h7000;
        7'd59: pmw_init_word = 16'h3244;
        7'd60: pmw_init_word = 16'h7f07;
        7'd61: pmw_init_word = 16'h4040;
        7'd62: pmw_init_word = 16'h7f06;
        7'd63: pmw_init_word = 16'h62f0;
        7'd64: pmw_init_word = 16'h6300;
        7'd65: pmw_init_word = 16'h7f0d;
        7'd66: pmw_init_word = 16'h48c0;
        7'd67: pmw_init_word = 16'h6fd5;
        7'd68: pmw_init_word = 16'h7f00;
        7'd69: pmw_init_word = 16'h5ba0;
        7'd70: pmw_init_word = 16'h4ea8;
        7'd71: pmw_init_word = 16'h5a50;
        default: pmw_init_word = 16'h4080;
      endcase
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex_ascii
  // 把 4 bit 半字节转换为 @M 使用的大写十六进制 ASCII。
  // -----------------------------------------------------------------------------
  function [7:0] hex_ascii;
    input [3:0] nibble;  // 4 bit 函数 `hex_ascii` 的“十六进制半字节”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      hex_ascii = (nibble < 4'd10) ? (8'h30 + nibble) : (8'h41 + nibble - 4'd10);
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex8_char
  // 按字符位置输出 8 bit 数值的两个十六进制字符。
  // -----------------------------------------------------------------------------
  function [7:0] hex8_char;
    input [7:0] value;  // 8 bit 函数 `hex8_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input index;  // 函数 `hex8_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      hex8_char = index ? hex_ascii(value[3:0]) : hex_ascii(value[7:4]);
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex16_char
  // 按字符位置输出 16 bit 数值的四个十六进制字符。
  // -----------------------------------------------------------------------------
  function [7:0] hex16_char;
    input [15:0] value;  // 16 bit 函数 `hex16_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [1:0] index;  // 2 bit 函数 `hex16_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (index)
        2'd0: hex16_char = hex_ascii(value[15:12]);
        2'd1: hex16_char = hex_ascii(value[11:8]);
        2'd2: hex16_char = hex_ascii(value[7:4]);
        default: hex16_char = hex_ascii(value[3:0]);
      endcase
    end
  endfunction

  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: hex32_char
  // 按字符位置输出 32 bit 数值的八个十六进制字符。
  // -----------------------------------------------------------------------------
  function [7:0] hex32_char;
    input [31:0] value;  // 32 bit 函数 `hex32_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    input [2:0] index;  // 3 bit 函数 `hex32_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      case (index)
        3'd0: hex32_char = hex_ascii(value[31:28]);
        3'd1: hex32_char = hex_ascii(value[27:24]);
        3'd2: hex32_char = hex_ascii(value[23:20]);
        3'd3: hex32_char = hex_ascii(value[19:16]);
        3'd4: hex32_char = hex_ascii(value[15:12]);
        3'd5: hex32_char = hex_ascii(value[11:8]);
        3'd6: hex32_char = hex_ascii(value[7:4]);
        default: hex32_char = hex_ascii(value[3:0]);
      endcase
    end
  endfunction




  // -----------------------------------------------------------------------------
  // [FUNCTION] 辅助函数: debug_frame_char
  // 按固定索引生成 99 字节 @M 标定遥测帧中的当前 ASCII 字符。
  // -----------------------------------------------------------------------------
  function [7:0] debug_frame_char;
    input [6:0] index;  // 7 bit 函数 `debug_frame_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if (index == 7'd0) debug_frame_char = "@";
      else if (index == 7'd1) debug_frame_char = "M";
      else if ((index >= 7'd2) && (index <= 7'd5)) debug_frame_char = hex16_char(snap_sequence, index - 7'd2);
      else if (index == 7'd6) debug_frame_char = ",";
      else if ((index >= 7'd7) && (index <= 7'd8)) debug_frame_char = hex8_char(snap_status, index - 7'd7);
      else if (index == 7'd9) debug_frame_char = ",";
      else if ((index >= 7'd10) && (index <= 7'd11)) debug_frame_char = hex8_char(snap_squal, index - 7'd10);
      else if (index == 7'd12) debug_frame_char = ",";
      else if ((index >= 7'd13) && (index <= 7'd14)) debug_frame_char = hex8_char(snap_pmw_id, index - 7'd13);
      else if (index == 7'd15) debug_frame_char = ",";
      else if ((index >= 7'd16) && (index <= 7'd17)) debug_frame_char = hex8_char(snap_pmw_inverse_id, index - 7'd16);
      else if (index == 7'd18) debug_frame_char = ",";
      else if ((index >= 7'd19) && (index <= 7'd20)) debug_frame_char = hex8_char(snap_bmi0_accel_id, index - 7'd19);
      else if (index == 7'd21) debug_frame_char = ",";
      else if ((index >= 7'd22) && (index <= 7'd23)) debug_frame_char = hex8_char(snap_bmi0_gyro_id, index - 7'd22);
      else if (index == 7'd24) debug_frame_char = ",";
      else if ((index >= 7'd25) && (index <= 7'd26)) debug_frame_char = hex8_char(snap_bmi1_accel_id, index - 7'd25);
      else if (index == 7'd27) debug_frame_char = ",";
      else if ((index >= 7'd28) && (index <= 7'd29)) debug_frame_char = hex8_char(snap_bmi1_gyro_id, index - 7'd28);
      else if (index == 7'd30) debug_frame_char = ",";
      else if ((index >= 7'd31) && (index <= 7'd34)) debug_frame_char = hex16_char(snap_flow_dx, index - 7'd31);
      else if (index == 7'd35) debug_frame_char = ",";
      else if ((index >= 7'd36) && (index <= 7'd39)) debug_frame_char = hex16_char(snap_flow_dy, index - 7'd36);
      else if (index == 7'd40) debug_frame_char = ",";
      else if ((index >= 7'd41) && (index <= 7'd48)) debug_frame_char = hex32_char(snap_flow_accum_x, index - 7'd41);
      else if (index == 7'd49) debug_frame_char = ",";
      else if ((index >= 7'd50) && (index <= 7'd57)) debug_frame_char = hex32_char(snap_flow_accum_y, index - 7'd50);
      else if (index == 7'd58) debug_frame_char = ",";
      else if ((index >= 7'd59) && (index <= 7'd62)) debug_frame_char = hex16_char(snap_accel_x, index - 7'd59);
      else if (index == 7'd63) debug_frame_char = ",";
      else if ((index >= 7'd64) && (index <= 7'd67)) debug_frame_char = hex16_char(snap_accel_y, index - 7'd64);
      else if (index == 7'd68) debug_frame_char = ",";
      else if ((index >= 7'd69) && (index <= 7'd72)) debug_frame_char = hex16_char(snap_accel_z, index - 7'd69);
      else if (index == 7'd73) debug_frame_char = ",";
      else if ((index >= 7'd74) && (index <= 7'd77)) debug_frame_char = hex16_char(snap_gyro_x, index - 7'd74);
      else if (index == 7'd78) debug_frame_char = ",";
      else if ((index >= 7'd79) && (index <= 7'd82)) debug_frame_char = hex16_char(snap_gyro_y, index - 7'd79);
      else if (index == 7'd83) debug_frame_char = ",";
      else if ((index >= 7'd84) && (index <= 7'd87)) debug_frame_char = hex16_char(snap_gyro_z, index - 7'd84);
      else if (index == 7'd88) debug_frame_char = ",";
      else if ((index >= 7'd89) && (index <= 7'd92)) debug_frame_char = hex16_char(snap_bmi0_gyro_z, index - 7'd89);
      else if (index == 7'd93) debug_frame_char = ",";
      else if ((index >= 7'd94) && (index <= 7'd97)) debug_frame_char = hex16_char(snap_bmi1_gyro_z, index - 7'd94);
      else debug_frame_char = 8'h0a;
    end
  endfunction




  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 组合命令生成器：根据主状态选择 SPI 器件、寄存器序列、时序间隔和传输长度。
  // -----------------------------------------------------------------------------
  always @(*) begin
    cmd_valid = 1'b0;
    cmd_device = DEV_PMW;
    cmd_length = 4'd2;
    cmd_tx_data = 72'd0;
    cmd_setup_cycles = PMW_SETUP_CYCLES;
    cmd_gap_cycles = 16'd0;
    cmd_hold_cycles = PMW_WRITE_HOLD_CYCLES;
    cmd_post_cycles = PMW_WRITE_POST_CYCLES;

    case (main_state)
      ST_PMW_RESET: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'hba, 8'h5a, 56'd0};
      end
      ST_PMW_ID: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'h00, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_INVERSE_ID: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {8'h5f, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_CLEAR: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {{5'd0, pmw_clear_index + 3'd2}, 64'd0};
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      ST_PMW_INIT: begin
        cmd_valid = 1'b1;
        cmd_tx_data = {pmw_init_entry[15:8] | 8'h80,
                       pmw_init_entry[7:0], 56'd0};
      end
      ST_BMI_INIT: begin
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
        case (bmi_init_step)
          5'd0, 5'd3, 5'd4: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_length = 4'd3;
            cmd_tx_data = {8'h80, 64'd0};
          end
          5'd1: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7e, 8'hb6, 56'd0};
          end
          5'd5: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h14, 8'hb6, 56'd0};
          end
          5'd7: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h80, 64'd0};
          end
          5'd8: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7c, 8'h00, 56'd0};
          end
          5'd10: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h7d, 8'h04, 56'd0};
          end
          5'd12: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h40, 8'h89, 56'd0};
          end
          5'd14: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_ACC : DEV_BMI0_ACC;
            cmd_tx_data = {8'h41, 8'h01, 56'd0};
          end
          5'd15: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h11, 8'h00, 56'd0};
          end
          5'd17: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h10, 8'h04, 56'd0};
          end
          5'd18: begin
            cmd_valid = 1'b1;
            cmd_device = bmi_index ? DEV_BMI1_GYRO : DEV_BMI0_GYRO;
            cmd_tx_data = {8'h0f, 8'h03, 56'd0};
          end
          default: begin end
        endcase
      end
      ST_IMU0_ACCEL, ST_IMU1_ACCEL: begin
        cmd_valid = 1'b1;
        cmd_device = (main_state == ST_IMU0_ACCEL) ? DEV_BMI0_ACC : DEV_BMI1_ACC;
        cmd_length = 4'd8;
        cmd_tx_data = {8'h92, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_IMU0_GYRO, ST_IMU1_GYRO: begin
        cmd_valid = 1'b1;
        cmd_device = (main_state == ST_IMU0_GYRO) ? DEV_BMI0_GYRO : DEV_BMI1_GYRO;
        cmd_length = 4'd7;
        cmd_tx_data = {8'h82, 64'd0};
        cmd_setup_cycles = BMI_SETUP_CYCLES;
        cmd_hold_cycles = BMI_HOLD_CYCLES;
        cmd_post_cycles = BMI_POST_CYCLES;
      end
      ST_PMW_MOTION, ST_PMW_DX_LOW, ST_PMW_DX_HIGH,
      ST_PMW_DY_LOW, ST_PMW_DY_HIGH, ST_PMW_SQUAL: begin
        cmd_valid = 1'b1;
        case (main_state)
          ST_PMW_MOTION:  cmd_tx_data = {8'h02, 64'd0};
          ST_PMW_DX_LOW:  cmd_tx_data = {8'h03, 64'd0};
          ST_PMW_DX_HIGH: cmd_tx_data = {8'h04, 64'd0};
          ST_PMW_DY_LOW:  cmd_tx_data = {8'h05, 64'd0};
          ST_PMW_DY_HIGH: cmd_tx_data = {8'h06, 64'd0};
          default:        cmd_tx_data = {8'h07, 64'd0};
        endcase
        cmd_gap_cycles = PMW_READ_GAP_CYCLES;
        cmd_hold_cycles = PMW_READ_HOLD_CYCLES;
        cmd_post_cycles = 16'd0;
      end
      default: begin end
    endcase
  end



  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 共享 SPI mode-3 引擎：锁存一次命令后驱动五片选、SCLK/MOSI，并收集最多 9 字节 MISO。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sensor_spi_sclk <= 1'b1;
      sensor_spi_mosi <= 1'b0;
      spi_cs_n_raw <= 5'b11111;
      spi_busy <= 1'b0;
      spi_done <= 1'b0;
      spi_state <= SPI_IDLE;
      spi_length_latched <= 4'd0;
      spi_byte_count <= 4'd0;
      spi_bit_index <= 3'd7;
      spi_half_count <= 16'd0;
      spi_delay_count <= 16'd0;
      spi_setup_latched <= 16'd0;
      spi_gap_latched <= 16'd0;
      spi_hold_latched <= 16'd0;
      spi_post_latched <= 16'd0;
      spi_tx_shift <= 72'd0;
      spi_rx_shift <= 72'd0;
      spi_rx_data <= 72'd0;
    end else begin
      spi_done <= 1'b0;
      case (spi_state)
        SPI_IDLE: begin
          sensor_spi_sclk <= 1'b1;
          sensor_spi_mosi <= 1'b0;
          spi_cs_n_raw <= 5'b11111;
          spi_busy <= 1'b0;
          spi_half_count <= 16'd0;
          spi_delay_count <= 16'd0;
          if (spi_start) begin
            spi_busy <= 1'b1;
            spi_length_latched <= cmd_length;
            spi_byte_count <= 4'd0;
            spi_bit_index <= 3'd7;
            spi_setup_latched <= cmd_setup_cycles;
            spi_gap_latched <= cmd_gap_cycles;
            spi_hold_latched <= cmd_hold_cycles;
            spi_post_latched <= cmd_post_cycles;
            spi_tx_shift <= cmd_tx_data;
            spi_rx_shift <= 72'd0;
            case (cmd_device)
              DEV_PMW:       spi_cs_n_raw <= 5'b11110;
              DEV_BMI0_ACC:  spi_cs_n_raw <= 5'b11101;
              DEV_BMI0_GYRO: spi_cs_n_raw <= 5'b11011;
              DEV_BMI1_ACC:  spi_cs_n_raw <= 5'b10111;
              default:       spi_cs_n_raw <= 5'b01111;
            endcase
            spi_state <= SPI_SETUP;
          end
        end
        SPI_SETUP: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_setup_latched) begin
            spi_delay_count <= 16'd0;
            spi_half_count <= 16'd0;
            spi_state <= SPI_DRIVE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        SPI_DRIVE: begin
          spi_busy <= 1'b1;
          if (spi_half_count >= SPI_HALF_CYCLES - 1) begin
            spi_half_count <= 16'd0;
            sensor_spi_sclk <= 1'b0;
            sensor_spi_mosi <= spi_tx_shift[71];
            spi_state <= SPI_SAMPLE;
          end else begin
            spi_half_count <= spi_half_count + 16'd1;
          end
        end
        SPI_SAMPLE: begin
          spi_busy <= 1'b1;
          if (spi_half_count >= SPI_HALF_CYCLES - 1) begin
            spi_half_count <= 16'd0;
            sensor_spi_sclk <= 1'b1;
            spi_rx_shift <= {spi_rx_shift[70:0], sensor_spi_miso};
            spi_tx_shift <= {spi_tx_shift[70:0], 1'b0};
            if (spi_bit_index == 3'd0) begin
              spi_bit_index <= 3'd7;
              if (spi_byte_count + 4'd1 >= spi_length_latched) begin
                spi_rx_data <= {spi_rx_shift[70:0], sensor_spi_miso};
                spi_delay_count <= 16'd0;
                spi_state <= SPI_HOLD;
              end else begin
                spi_byte_count <= spi_byte_count + 4'd1;
                spi_delay_count <= 16'd0;
                spi_state <= SPI_GAP;
              end
            end else begin
              spi_bit_index <= spi_bit_index - 3'd1;
              spi_state <= SPI_DRIVE;
            end
          end else begin
            spi_half_count <= spi_half_count + 16'd1;
          end
        end
        SPI_GAP: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_gap_latched) begin
            spi_delay_count <= 16'd0;
            spi_state <= SPI_DRIVE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        SPI_HOLD: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_hold_latched) begin
            spi_delay_count <= 16'd0;
            spi_cs_n_raw <= 5'b11111;
            if (spi_post_latched == 16'd0) begin
              spi_busy <= 1'b0;
              spi_done <= 1'b1;
              spi_state <= SPI_IDLE;
            end else begin
              spi_state <= SPI_POST;
            end
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
        default: begin
          spi_busy <= 1'b1;
          if (spi_delay_count >= spi_post_latched) begin
            spi_delay_count <= 16'd0;
            spi_busy <= 1'b0;
            spi_done <= 1'b1;
            spi_state <= SPI_IDLE;
          end else begin
            spi_delay_count <= spi_delay_count + 16'd1;
          end
        end
      endcase
    end
  end


  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 3
  // 传感器主状态机：依次初始化 PMW3901/BMI088、采样双 IMU 和光流、校准偏置并融合。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      main_state <= ST_BOOT;
      spi_start <= 1'b0;
      command_inflight <= 1'b0;
      state_wait_count <= 32'd0;
      pmw_force_cs_low <= 1'b0;
      pmw_init_index <= 7'd0;
      pmw_clear_index <= 3'd0;
      pmw_retry_count <= 2'd0;
      bmi_index <= 1'b0;
      bmi_init_step <= 5'd0;
      current_bmi_accel_id <= 8'd0;
      current_bmi_gyro_id <= 8'd0;
      pmw_poll_toggle <= 1'b0;
      pair_sample_strobe <= 1'b0;
      flow_data_seen <= 1'b0;
      pmw3901_online <= 1'b0;
      bmi0_online <= 1'b0;
      bmi1_online <= 1'b0;
      pmw3901_product_id <= 8'd0;
      pmw3901_inverse_id <= 8'd0;
      bmi0_accel_id <= 8'd0;
      bmi0_gyro_id <= 8'd0;
      bmi1_accel_id <= 8'd0;
      bmi1_gyro_id <= 8'd0;
      bmi0_accel_x_raw <= 16'sd0;
      bmi0_accel_y_raw <= 16'sd0;
      bmi0_accel_z_raw <= 16'sd0;
      bmi0_gyro_x_raw <= 16'sd0;
      bmi0_gyro_y_raw <= 16'sd0;
      bmi0_gyro_z_raw <= 16'sd0;
      bmi1_accel_x_raw <= 16'sd0;
      bmi1_accel_y_raw <= 16'sd0;
      bmi1_accel_z_raw <= 16'sd0;
      bmi1_gyro_x_raw <= 16'sd0;
      bmi1_gyro_y_raw <= 16'sd0;
      bmi1_gyro_z_raw <= 16'sd0;
      bmi0_bad_sample_count <= 4'd0;
      bmi1_bad_sample_count <= 4'd0;
      pmw_dx_low <= 8'd0;
      pmw_dx_high <= 8'd0;
      pmw_dy_low <= 8'd0;
      pmw_dy_high <= 8'd0;
      flow_sample_valid <= 1'b0;
      flow_squal <= 8'd0;
      flow_delta_x <= 16'sd0;
      flow_delta_y <= 16'sd0;
      flow_accum_x <= 32'sd0;
      flow_accum_y <= 32'sd0;
      sample_sequence <= 16'd0;
    end else begin
      spi_start <= 1'b0;
      pair_sample_strobe <= 1'b0;
      flow_sample_valid <= 1'b0;

      if (cmd_valid && !command_inflight && !spi_busy) begin
        spi_start <= 1'b1;
        command_inflight <= 1'b1;
      end

      if (cmd_valid && command_inflight && spi_done) begin
        command_inflight <= 1'b0;
        state_wait_count <= 32'd0;
        case (main_state)
          ST_PMW_RESET: main_state <= ST_PMW_RESET_WAIT;
          ST_PMW_ID: begin
            pmw3901_product_id <= spi_rx_data[7:0];
            main_state <= ST_PMW_INVERSE_ID;
          end
          ST_PMW_INVERSE_ID: begin
            pmw3901_inverse_id <= spi_rx_data[7:0];
            if ((pmw3901_product_id == 8'h49) && (spi_rx_data[7:0] == 8'hb6)) begin
              pmw_clear_index <= 3'd0;
              main_state <= ST_PMW_CLEAR;
            end else if (pmw_retry_count < 2'd2) begin
              pmw_retry_count <= pmw_retry_count + 2'd1;
              main_state <= ST_PMW_RETRY_WAIT;
            end else begin
              pmw3901_online <= 1'b0;
              bmi_index <= 1'b0;
              bmi_init_step <= 5'd0;
              current_bmi_accel_id <= 8'd0;
              current_bmi_gyro_id <= 8'd0;
              main_state <= ST_BMI_INIT;
            end
          end
          ST_PMW_CLEAR: begin
            if (pmw_clear_index == 3'd4) begin
              pmw_init_index <= 7'd0;
              main_state <= ST_PMW_INIT;
            end else begin
              pmw_clear_index <= pmw_clear_index + 3'd1;
            end
          end
          ST_PMW_INIT: begin
            if (pmw_init_index == 7'd58) begin
              main_state <= ST_PMW_INIT_LONG_WAIT;
            end else if (pmw_init_index == 7'd72) begin
              pmw3901_online <= 1'b1;
              bmi_index <= 1'b0;
              bmi_init_step <= 5'd0;
              current_bmi_accel_id <= 8'd0;
              current_bmi_gyro_id <= 8'd0;
              main_state <= ST_BMI_INIT;
            end else begin
              pmw_init_index <= pmw_init_index + 7'd1;
            end
          end
          ST_BMI_INIT: begin
            if (bmi_init_step == 5'd4)
              current_bmi_accel_id <= spi_rx_data[7:0];
            if (bmi_init_step == 5'd7)
              current_bmi_gyro_id <= spi_rx_data[7:0];
            bmi_init_step <= bmi_init_step + 5'd1;
          end
          ST_IMU0_ACCEL: begin
            bmi0_accel_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi0_accel_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi0_accel_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU0_GYRO;
          end
          ST_IMU0_GYRO: begin
            bmi0_gyro_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi0_gyro_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi0_gyro_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU1_ACCEL;
          end
          ST_IMU1_ACCEL: begin
            bmi1_accel_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi1_accel_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi1_accel_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_IMU1_GYRO;
          end
          ST_IMU1_GYRO: begin
            bmi1_gyro_x_raw <= {spi_rx_data[39:32], spi_rx_data[47:40]};
            bmi1_gyro_y_raw <= {spi_rx_data[23:16], spi_rx_data[31:24]};
            bmi1_gyro_z_raw <= {spi_rx_data[7:0], spi_rx_data[15:8]};
            main_state <= ST_FUSION_STROBE;
          end
          ST_PMW_MOTION: main_state <= ST_PMW_DX_LOW;
          ST_PMW_DX_LOW: begin
            pmw_dx_low <= spi_rx_data[7:0];
            main_state <= ST_PMW_DX_HIGH;
          end
          ST_PMW_DX_HIGH: begin
            pmw_dx_high <= spi_rx_data[7:0];
            main_state <= ST_PMW_DY_LOW;
          end
          ST_PMW_DY_LOW: begin
            pmw_dy_low <= spi_rx_data[7:0];
            main_state <= ST_PMW_DY_HIGH;
          end
          ST_PMW_DY_HIGH: begin
            pmw_dy_high <= spi_rx_data[7:0];
            main_state <= ST_PMW_SQUAL;
          end
          default: begin
            flow_squal <= spi_rx_data[7:0];
            flow_delta_x <= {pmw_dx_high, pmw_dx_low};
            flow_delta_y <= {pmw_dy_high, pmw_dy_low};
            flow_accum_x <= $signed(flow_accum_x) + $signed({pmw_dx_high, pmw_dx_low});
            flow_accum_y <= $signed(flow_accum_y) + $signed({pmw_dy_high, pmw_dy_low});
            flow_sample_valid <= pmw3901_online;
            flow_data_seen <= pmw3901_online;
            main_state <= ST_RUN_WAIT;
          end
        endcase
      end else if (!cmd_valid) begin
        case (main_state)
          ST_BOOT: begin
            pmw_force_cs_low <= 1'b0;
            if (state_wait_count >= BOOT_WAIT_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b1;
              main_state <= ST_PMW_CS_LOW;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_CS_LOW: begin
            if (state_wait_count >= ONE_MS_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b0;
              main_state <= ST_PMW_CS_HIGH;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_CS_HIGH: begin
            if (state_wait_count >= ONE_MS_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_PMW_RESET;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_RESET_WAIT: begin
            if (state_wait_count >= (5 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_PMW_ID;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_RETRY_WAIT: begin
            if (state_wait_count >= (100 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              pmw_force_cs_low <= 1'b1;
              main_state <= ST_PMW_CS_LOW;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_PMW_INIT_LONG_WAIT: begin
            if (state_wait_count >= (100 * ONE_MS_CYCLES) - 1) begin
              state_wait_count <= 32'd0;
              pmw_init_index <= 7'd59;
              main_state <= ST_PMW_INIT;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_BMI_INIT: begin
            case (bmi_init_step)
              5'd2: begin
                if (state_wait_count >= (2 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= 5'd3;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd6, 5'd16: begin
                if (state_wait_count >= (30 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= bmi_init_step + 5'd1;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd9, 5'd11: begin
                if (state_wait_count >= (5 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= bmi_init_step + 5'd1;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd13: begin
                if (state_wait_count >= (40 * ONE_MS_CYCLES) - 1) begin
                  state_wait_count <= 32'd0;
                  bmi_init_step <= 5'd14;
                end else state_wait_count <= state_wait_count + 32'd1;
              end
              5'd19: begin
                if (!bmi_index) begin
                  bmi0_accel_id <= current_bmi_accel_id;
                  bmi0_gyro_id <= current_bmi_gyro_id;
                  bmi0_online <= (current_bmi_accel_id == 8'h1e) &&
                                 (current_bmi_gyro_id == 8'h0f);
                  bmi_index <= 1'b1;
                  bmi_init_step <= 5'd0;
                  current_bmi_accel_id <= 8'd0;
                  current_bmi_gyro_id <= 8'd0;
                end else begin
                  bmi1_accel_id <= current_bmi_accel_id;
                  bmi1_gyro_id <= current_bmi_gyro_id;
                  bmi1_online <= (current_bmi_accel_id == 8'h1e) &&
                                 (current_bmi_gyro_id == 8'h0f);
                  state_wait_count <= 32'd0;
                  main_state <= ST_RUN_WAIT;
                end
              end
              default: begin end
            endcase
          end
          ST_RUN_WAIT: begin
            if (state_wait_count >= RUN_WAIT_CYCLES - 1) begin
              state_wait_count <= 32'd0;
              main_state <= ST_IMU0_ACCEL;
            end else state_wait_count <= state_wait_count + 32'd1;
          end
          ST_FUSION_STROBE: begin
            pair_sample_strobe <= 1'b1;
            sample_sequence <= sample_sequence + 16'd1;

            if (bmi0_online) begin
              if (bmi0_bad_sample) begin
                if (bmi0_bad_sample_count >= 4'd7) begin
                  bmi0_bad_sample_count <= 4'd8;
                  bmi0_online <= 1'b0;
                end else bmi0_bad_sample_count <= bmi0_bad_sample_count + 4'd1;
              end else bmi0_bad_sample_count <= 4'd0;
            end
            if (bmi1_online) begin
              if (bmi1_bad_sample) begin
                if (bmi1_bad_sample_count >= 4'd7) begin
                  bmi1_bad_sample_count <= 4'd8;
                  bmi1_online <= 1'b0;
                end else bmi1_bad_sample_count <= bmi1_bad_sample_count + 4'd1;
              end else bmi1_bad_sample_count <= 4'd0;
            end

            if (pmw3901_online && !pmw_poll_toggle) begin
              pmw_poll_toggle <= 1'b1;
              main_state <= ST_PMW_MOTION;
            end else begin
              pmw_poll_toggle <= 1'b0;
              main_state <= ST_RUN_WAIT;
            end
          end
          default: begin end
        endcase
      end
    end
  end




  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 4
  // @M 遥测调度器：每 100 ms 锁存一次一致快照，再按固定字符索引启动 UART 字节发送。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      calibration_count <= 10'd0;
      bmi0_bias_sum_x <= 32'sd0;
      bmi0_bias_sum_y <= 32'sd0;
      bmi0_bias_sum_z <= 32'sd0;
      bmi1_bias_sum_x <= 32'sd0;
      bmi1_bias_sum_y <= 32'sd0;
      bmi1_bias_sum_z <= 32'sd0;
      bmi0_gyro_bias_x <= 16'sd0;
      bmi0_gyro_bias_y <= 16'sd0;
      bmi0_gyro_bias_z <= 16'sd0;
      bmi1_gyro_bias_x <= 16'sd0;
      bmi1_gyro_bias_y <= 16'sd0;
      bmi1_gyro_bias_z <= 16'sd0;
      calibration_done <= 1'b0;
      fused_sample_valid <= 1'b0;
      fused_data_ready <= 1'b0;
      imu_degraded <= 1'b1;
      imu_disagreement <= 1'b0;
      imu_source <= 2'd3;
      fused_accel_x <= 16'sd0;
      fused_accel_y <= 16'sd0;
      fused_accel_z <= 16'sd0;
      fused_gyro_x <= 16'sd0;
      fused_gyro_y <= 16'sd0;
      fused_gyro_z <= 16'sd0;
    end else begin
      fused_sample_valid <= 1'b0;
      if (pair_sample_strobe) begin
        if (!calibration_done) begin
          fused_data_ready <= 1'b0;
          imu_degraded <= 1'b1;
          imu_source <= 2'd3;
          if ((bmi0_online || bmi1_online) && calibration_stationary) begin
            if (bmi0_online) begin
              bmi0_bias_sum_x <= bmi0_bias_sum_x + $signed(bmi0_gyro_x_raw);
              bmi0_bias_sum_y <= bmi0_bias_sum_y + $signed(bmi0_gyro_y_raw);
              bmi0_bias_sum_z <= bmi0_bias_sum_z + $signed(bmi0_gyro_z_raw);
            end
            if (bmi1_online) begin
              bmi1_bias_sum_x <= bmi1_bias_sum_x + $signed(bmi1_gyro_x_raw);
              bmi1_bias_sum_y <= bmi1_bias_sum_y + $signed(bmi1_gyro_y_raw);
              bmi1_bias_sum_z <= bmi1_bias_sum_z + $signed(bmi1_gyro_z_raw);
            end
            if (calibration_count >= CALIBRATION_SAMPLES - 1) begin
              calibration_done <= 1'b1;
              if (bmi0_online) begin
                bmi0_gyro_bias_x <= $signed(bmi0_bias_sum_x + $signed(bmi0_gyro_x_raw)) >>> CALIBRATION_SHIFT;
                bmi0_gyro_bias_y <= $signed(bmi0_bias_sum_y + $signed(bmi0_gyro_y_raw)) >>> CALIBRATION_SHIFT;
                bmi0_gyro_bias_z <= $signed(bmi0_bias_sum_z + $signed(bmi0_gyro_z_raw)) >>> CALIBRATION_SHIFT;
              end
              if (bmi1_online) begin
                bmi1_gyro_bias_x <= $signed(bmi1_bias_sum_x + $signed(bmi1_gyro_x_raw)) >>> CALIBRATION_SHIFT;
                bmi1_gyro_bias_y <= $signed(bmi1_bias_sum_y + $signed(bmi1_gyro_y_raw)) >>> CALIBRATION_SHIFT;
                bmi1_gyro_bias_z <= $signed(bmi1_bias_sum_z + $signed(bmi1_gyro_z_raw)) >>> CALIBRATION_SHIFT;
              end
            end else calibration_count <= calibration_count + 10'd1;
          end
        end else if (bmi0_online && bmi1_online) begin
          imu_degraded <= 1'b0;
          imu_source <= 2'd0;
          if ((accel_diff_x <= ACCEL_DISAGREE_LIMIT) &&
              (accel_diff_y <= ACCEL_DISAGREE_LIMIT) &&
              (accel_diff_z <= ACCEL_DISAGREE_LIMIT) &&
              (gyro_diff_x <= GYRO_DISAGREE_LIMIT) &&
              (gyro_diff_y <= GYRO_DISAGREE_LIMIT) &&
              (gyro_diff_z <= GYRO_DISAGREE_LIMIT)) begin
            imu_disagreement <= 1'b0;
            fused_sample_valid <= 1'b1;
            fused_data_ready <= 1'b1;
            fused_accel_x <= ($signed(bmi0_accel_x_raw) + $signed(bmi1_accel_x_raw)) >>> 1;
            fused_accel_y <= ($signed(bmi0_accel_y_raw) + $signed(bmi1_accel_y_raw)) >>> 1;
            fused_accel_z <= ($signed(bmi0_accel_z_raw) + $signed(bmi1_accel_z_raw)) >>> 1;
            fused_gyro_x <= saturate16(($signed(bmi0_gyro_x_corrected) + $signed(bmi1_gyro_x_corrected)) >>> 1);
            fused_gyro_y <= saturate16(($signed(bmi0_gyro_y_corrected) + $signed(bmi1_gyro_y_corrected)) >>> 1);
            fused_gyro_z <= saturate16(($signed(bmi0_gyro_z_corrected) + $signed(bmi1_gyro_z_corrected)) >>> 1);
          end else begin
            imu_disagreement <= 1'b1;
            fused_sample_valid <= 1'b0;
            fused_data_ready <= 1'b0;
            imu_source <= 2'd3;
          end
        end else if (bmi0_online) begin
          imu_degraded <= 1'b1;
          imu_disagreement <= 1'b0;
          imu_source <= 2'd1;
          fused_sample_valid <= 1'b1;
          fused_data_ready <= 1'b1;
          fused_accel_x <= bmi0_accel_x_raw;
          fused_accel_y <= bmi0_accel_y_raw;
          fused_accel_z <= bmi0_accel_z_raw;
          fused_gyro_x <= saturate16(bmi0_gyro_x_corrected);
          fused_gyro_y <= saturate16(bmi0_gyro_y_corrected);
          fused_gyro_z <= saturate16(bmi0_gyro_z_corrected);
        end else if (bmi1_online) begin
          imu_degraded <= 1'b1;
          imu_disagreement <= 1'b0;
          imu_source <= 2'd2;
          fused_sample_valid <= 1'b1;
          fused_data_ready <= 1'b1;
          fused_accel_x <= bmi1_accel_x_raw;
          fused_accel_y <= bmi1_accel_y_raw;
          fused_accel_z <= bmi1_accel_z_raw;
          fused_gyro_x <= saturate16(bmi1_gyro_x_corrected);
          fused_gyro_y <= saturate16(bmi1_gyro_y_corrected);
          fused_gyro_z <= saturate16(bmi1_gyro_z_corrected);
        end else begin
          imu_degraded <= 1'b1;
          imu_disagreement <= 1'b0;
          imu_source <= 2'd3;
          fused_data_ready <= 1'b0;
        end
      end
    end
  end


  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 5
  // 调试 UART 位发送器：按 115200 8N1 LSB-first 输出一个 ASCII 字节。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sensor_debug_uart_tx <= 1'b1;
      uart_busy <= 1'b0;
      uart_busy_d <= 1'b0;
      uart_state <= UART_IDLE;
      uart_clock_count <= 16'd0;
      uart_bit_index <= 3'd0;
      uart_shift <= 8'd0;
    end else begin
      uart_busy_d <= uart_busy;
      case (uart_state)
        UART_IDLE: begin
          sensor_debug_uart_tx <= 1'b1;
          uart_busy <= 1'b0;
          uart_clock_count <= 16'd0;
          uart_bit_index <= 3'd0;
          if (uart_start_pulse) begin
            uart_shift <= uart_data_byte;
            sensor_debug_uart_tx <= 1'b0;
            uart_busy <= 1'b1;
            uart_state <= UART_START;
          end
        end
        UART_START: begin
          sensor_debug_uart_tx <= 1'b0;
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            sensor_debug_uart_tx <= uart_shift[0];
            uart_state <= UART_DATA;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        UART_DATA: begin
          sensor_debug_uart_tx <= uart_shift[uart_bit_index];
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            if (uart_bit_index == 3'd7) begin
              uart_bit_index <= 3'd0;
              sensor_debug_uart_tx <= 1'b1;
              uart_state <= UART_STOP;
            end else begin
              uart_bit_index <= uart_bit_index + 3'd1;
              sensor_debug_uart_tx <= uart_shift[uart_bit_index + 3'd1];
            end
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
        default: begin
          sensor_debug_uart_tx <= 1'b1;
          uart_busy <= 1'b1;
          if (uart_clock_count >= UART_CLKS_PER_BIT - 1) begin
            uart_clock_count <= 16'd0;
            uart_busy <= 1'b0;
            uart_state <= UART_IDLE;
          end else uart_clock_count <= uart_clock_count + 16'd1;
        end
      endcase
    end
  end


  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 6
  // 时序过程：在有效时钟边沿更新 debug_period_count（debug / period / 计数）、debug_sending（debug / sending）、debug_char_index（debug / 字符 / 索引）、debug_sequence（debug / sequence）、uart_data_byte（UART / 数据 / byte）、uart_start_pulse（UART / 启动 / 脉冲）等信号；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      debug_period_count <= 32'd0;
      debug_sending <= 1'b0;
      debug_char_index <= 7'd0;
      debug_sequence <= 16'd0;
      uart_data_byte <= 8'd0;
      uart_start_pulse <= 1'b0;
      snap_sequence <= 16'd0;
      snap_status <= 8'd0;
      snap_squal <= 8'd0;
      snap_pmw_id <= 8'd0;
      snap_pmw_inverse_id <= 8'd0;
      snap_bmi0_accel_id <= 8'd0;
      snap_bmi0_gyro_id <= 8'd0;
      snap_bmi1_accel_id <= 8'd0;
      snap_bmi1_gyro_id <= 8'd0;
      snap_flow_dx <= 16'd0;
      snap_flow_dy <= 16'd0;
      snap_flow_accum_x <= 32'd0;
      snap_flow_accum_y <= 32'd0;
      snap_accel_x <= 16'd0;
      snap_accel_y <= 16'd0;
      snap_accel_z <= 16'd0;
      snap_gyro_x <= 16'd0;
      snap_gyro_y <= 16'd0;
      snap_gyro_z <= 16'd0;
      snap_bmi0_gyro_z <= 16'd0;
      snap_bmi1_gyro_z <= 16'd0;
    end else begin
      uart_start_pulse <= 1'b0;
      if (!debug_sending) begin
        if (debug_period_count >= DEBUG_PERIOD_CYCLES - 1) begin
          debug_period_count <= 32'd0;
          debug_sending <= 1'b1;
          debug_char_index <= 7'd0;
          uart_data_byte <= "@";
          uart_start_pulse <= 1'b1;
          snap_sequence <= debug_sequence;
          debug_sequence <= debug_sequence + 16'd1;
          snap_status <= {flow_data_seen, imu_disagreement, imu_degraded,
                          fused_data_ready, calibration_done, bmi1_online,
                          bmi0_online, pmw3901_online};
          snap_squal <= flow_squal;
          snap_pmw_id <= pmw3901_product_id;
          snap_pmw_inverse_id <= pmw3901_inverse_id;
          snap_bmi0_accel_id <= bmi0_accel_id;
          snap_bmi0_gyro_id <= bmi0_gyro_id;
          snap_bmi1_accel_id <= bmi1_accel_id;
          snap_bmi1_gyro_id <= bmi1_gyro_id;
          snap_flow_dx <= flow_delta_x;
          snap_flow_dy <= flow_delta_y;
          snap_flow_accum_x <= flow_accum_x;
          snap_flow_accum_y <= flow_accum_y;
          snap_accel_x <= fused_accel_x;
          snap_accel_y <= fused_accel_y;
          snap_accel_z <= fused_accel_z;
          snap_gyro_x <= fused_gyro_x;
          snap_gyro_y <= fused_gyro_y;
          snap_gyro_z <= fused_gyro_z;
          snap_bmi0_gyro_z <= bmi0_gyro_z_raw;
          snap_bmi1_gyro_z <= bmi1_gyro_z_raw;
        end else debug_period_count <= debug_period_count + 32'd1;
      end else if (uart_done) begin
        if (debug_char_index >= 7'd98) begin
          debug_sending <= 1'b0;
          debug_char_index <= 7'd0;
        end else begin
          debug_char_index <= debug_char_index + 7'd1;
          uart_data_byte <= debug_frame_char(debug_char_index + 7'd1);
          uart_start_pulse <= 1'b1;
        end
      end
    end
  end

  // -----------------------------------------------------------------------------
  // [INITIAL] 初始化块 1
  // 初始化过程：在仿真开始或 FPGA 配置完成时建立声明变量的确定初值。
  // 该块不重复执行；后续状态变化由 always 过程负责。
  // -----------------------------------------------------------------------------
  initial begin
    sensor_spi_sclk = 1'b1;
    sensor_spi_mosi = 1'b0;
    spi_cs_n_raw = 5'b11111;
    pmw_force_cs_low = 1'b0;
    sensor_debug_uart_tx = 1'b1;
  end

endmodule
