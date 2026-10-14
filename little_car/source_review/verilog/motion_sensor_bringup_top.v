// =============================================================================
// 模块: motion_sensor_bringup_top
// 作用: PMW3901 与双 BMI088 独立上板验证顶层，不连接 HDMI、UWB、蓝牙或 C5 电机。
// 说明: 复用 power_on_reset 的 20 ms 内部复位，并把三块器件在线状态直接映射到 LED。
// 说明: 当前独立 bit 只用于采集标定数据；未完成轴向和比例标定前不能进入电机闭环。
//
// Robei 源: little_car/model/motion_sensor_bringup_top.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module motion_sensor_bringup_top (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire sensor_spi_miso,  // PMW3901 与两块 BMI088 共享 SPI MISO；只有当前片选器件允许驱动。
  output wire sensor_spi_sclk,  // 约 1.923 MHz 的共享 SPI mode-3 时钟输出，空闲电平为高。
  output wire sensor_spi_mosi,  // 发送给 PMW3901/BMI088 的共享 SPI MOSI 数据。
  output wire pmw3901_cs_n,  // PMW3901 低有效片选；初始化特殊阶段可被强制保持为低。
  output wire bmi0_acc_cs_n,  // 第一块 BMI088 加速度计低有效 SPI 片选。
  output wire bmi0_gyro_cs_n,  // 第一块 BMI088 陀螺仪低有效 SPI 片选。
  output wire bmi1_acc_cs_n,  // 第二块 BMI088 加速度计低有效 SPI 片选。
  output wire bmi1_gyro_cs_n,  // 第二块 BMI088 陀螺仪低有效 SPI 片选。
  output wire sensor_debug_uart_tx,  // 115200 bit/s @M 标定遥测输出，独立固件连接板载 USB-UART RX。
  output wire pmw_status_led,  // 独立验证顶层的 PMW3901 在线 LED。
  output wire bmi0_status_led,  // 独立验证顶层的第一块 BMI088 在线 LED。
  output wire bmi1_status_led  // 独立验证顶层的第二块 BMI088 在线 LED。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  wire system_rst_n;  // 组合网络，传递“system / RST / N”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire calibration_done;  // 组合网络，传递“calibration / 完成标志”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire fused_sample_valid;  // “fused / 采样 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  wire imu_degraded;  // 组合网络，传递“IMU / degraded”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire imu_disagreement;  // 组合网络，传递“IMU / disagreement”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [1:0] imu_source;  // 2 bit 组合网络，传递“IMU / 来源”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] pmw3901_product_id;  // 8 bit 组合网络，传递“pmw3901 / product / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] pmw3901_inverse_id;  // 8 bit 组合网络，传递“pmw3901 / inverse / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] bmi0_accel_id;  // 8 bit 组合网络，传递“bmi0 / accel / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] bmi0_gyro_id;  // 8 bit 组合网络，传递“bmi0 / gyro / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] bmi1_accel_id;  // 8 bit 组合网络，传递“bmi1 / accel / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [7:0] bmi1_gyro_id;  // 8 bit 组合网络，传递“bmi1 / gyro / ID”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire flow_sample_valid;  // “flow / 采样 / 有效标志”信号；为 1 的周期内，与它配套的数据可以被下游锁存。
  wire [7:0] flow_squal;  // 8 bit 组合网络，传递“flow / squal”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [15:0] flow_delta_x;  // 16 bit “flow / delta / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] flow_delta_y;  // 16 bit “flow / delta / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [31:0] flow_accum_x;  // 32 bit “flow / accum / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [31:0] flow_accum_y;  // 32 bit “flow / accum / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] fused_accel_x;  // 16 bit “fused / accel / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] fused_accel_y;  // 16 bit “fused / accel / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] fused_accel_z;  // 16 bit 组合网络，传递“fused / accel / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire signed [15:0] fused_gyro_x;  // 16 bit “fused / gyro / X 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] fused_gyro_y;  // 16 bit “fused / gyro / Y 坐标”数值；表示路线栅格或显示空间中的对应坐标。
  wire signed [15:0] fused_gyro_z;  // 16 bit 组合网络，传递“fused / gyro / Z”；由连续赋值或上游模块驱动，本身不保存跨周期状态。
  wire [15:0] sample_sequence;  // 16 bit 组合网络，传递“采样 / sequence”；由连续赋值或上游模块驱动，本身不保存跨周期状态。

  power_on_reset power_on_reset_sensor(
    .clk(clk),
    .ext_rst_n(rst_n),
    .system_rst_n(system_rst_n)
  );

  motion_sensor_subsystem motion_sensor_subsystem0(
    .clk(clk),
    .rst_n(system_rst_n),
    .vehicle_motion_expected(1'b0),
    .sensor_spi_miso(sensor_spi_miso),
    .sensor_spi_sclk(sensor_spi_sclk),
    .sensor_spi_mosi(sensor_spi_mosi),
    .pmw3901_cs_n(pmw3901_cs_n),
    .bmi0_acc_cs_n(bmi0_acc_cs_n),
    .bmi0_gyro_cs_n(bmi0_gyro_cs_n),
    .bmi1_acc_cs_n(bmi1_acc_cs_n),
    .bmi1_gyro_cs_n(bmi1_gyro_cs_n),
    .sensor_debug_uart_tx(sensor_debug_uart_tx),
    .pmw3901_online(pmw_status_led),
    .bmi0_online(bmi0_status_led),
    .bmi1_online(bmi1_status_led),
    .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement),
    .imu_source(imu_source),
    .pmw3901_product_id(pmw3901_product_id),
    .pmw3901_inverse_id(pmw3901_inverse_id),
    .bmi0_accel_id(bmi0_accel_id),
    .bmi0_gyro_id(bmi0_gyro_id),
    .bmi1_accel_id(bmi1_accel_id),
    .bmi1_gyro_id(bmi1_gyro_id),
    .flow_sample_valid(flow_sample_valid),
    .flow_squal(flow_squal),
    .flow_delta_x(flow_delta_x),
    .flow_delta_y(flow_delta_y),
    .flow_accum_x(flow_accum_x),
    .flow_accum_y(flow_accum_y),
    .fused_accel_x(fused_accel_x),
    .fused_accel_y(fused_accel_y),
    .fused_accel_z(fused_accel_z),
    .fused_gyro_x(fused_gyro_x),
    .fused_gyro_y(fused_gyro_y),
    .fused_gyro_z(fused_gyro_z),
    .sample_sequence(sample_sequence)
  );

endmodule
