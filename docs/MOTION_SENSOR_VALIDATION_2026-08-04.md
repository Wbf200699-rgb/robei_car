# PMW3901 与双 BMI088 实物验证记录

## 验证边界

本次使用独立 `motion_sensor_bringup_top` 固件，只包含 50 MHz 时钟、复位、
H3 SPI、COM20 调试串口和三路状态灯。固件没有 C5 电机、蓝牙、HDMI、
RPLIDAR、超声波或 UWB 控制端口，因此本次烧录不能证明主工程已经完成
运动传感器融合，也不会向底盘发送电机帧。

## 构建与烧录

- Robei 源：`motion_sensor_subsystem.model`、`motion_sensor_bringup_top.model`；
- FPGA：XC7A50T-FGG484-1；
- LUT：2196，6.74%；
- 寄存器：1398，2.14%；
- BRAM/DSP：0/0；
- 路由失败：0；
- 最差建立时间裕量：+7.447 ns；
- bit SHA-256：
  `2275590B9E9936F1CC86A7C31814D20C6B9858ADF1C55B576F167A1AFAB0F831`；
- Vivado Hardware Manager：`program_complete=1`，startup status HIGH。

## 串口证据

采样时间为 2026-08-04 17:38，COM20，115200 8N1，共 185 帧：

| 项目 | 结果 |
| --- | --- |
| 序号 | 238 至 422，连续，0 丢帧 |
| 状态 | 全部为 `0x9F` |
| PMW3901 | ID `49/B6`，在线 |
| BMI088 #0 | 加速度/陀螺 ID `1E/0F`，在线 |
| BMI088 #1 | 加速度/陀螺 ID `1E/0F`，在线 |
| PMW3901 SQUAL | 最小 64，平均 72.405，最大 80 |
| 光流增量 | X/Y 都为 0，测试期间没有移动底盘 |
| 后 50 帧融合陀螺均值 | X=-1.42，Y=3.44，Z=2.36 raw counts |
| 后 50 帧融合陀螺标准差 | X=4.754，Y=9.267，Z=4.902 raw counts |
| 后 50 帧融合加速度均值 | X=189.06，Y=56.36，Z=-5415.10 raw counts |
| 后 50 帧双 IMU 原始 Z 差 | 均值 6.18，标准差 8.662，范围 -14 至 29 |

原始日志和解码 CSV 保存在：

- `docs/motion_sensor_evidence/motion_sensor_20260804_173818.log`；
- `docs/motion_sensor_evidence/motion_sensor_20260804_173818.csv`。

对应 SHA-256：

- log：`3FA98F37F86B7009294714A064CB7963EEE796AE93DDC76B61F2FF8CCEB6B91D`；
- CSV：`81D824964ADBE8BBE6D3C79804854F5C48687744DA4D2B31CEDDBF41AAF119A1`。

## 判定

三个 SPI 器件的芯片选择、共享 SCLK/MOSI/MISO、初始化寄存器和串口输出均
已通过实物验证。静止阶段双 IMU 没有触发分歧或降级，PMW3901 的 SQUAL
有足够余量，说明当前镜头和地面纹理能够形成有效图像质量。

尚未证明的参数包括：PMW3901 每毫米计数、X/Y 方向、安装偏航角、离地高度
变化系数；BMI088 的车体轴映射、正负号、动态振动和温漂；双 IMU 在电机
运行时的一致性。以上必须通过后续移动实验标定，不能从本次静止数据推断。
