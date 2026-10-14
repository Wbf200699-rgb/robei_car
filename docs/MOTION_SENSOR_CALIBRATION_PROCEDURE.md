# PMW3901 与双 BMI088 标定流程

## 1. 当前结论

独立传感器固件已经识别 PMW3901 和两块 BMI088，UART 连续帧、设备 ID、
启动偏置校准和双 IMU 融合链路均已打通。当前仍不能把光流或 IMU 用于小车
闭环控制，因为以下参数尚未由实物动作确定：

- PMW3901 X/Y 与车体前/右方向的对应关系及正负号；
- 光流每毫米计数和交叉轴误差；
- 两块 BMI088 的安装轴方向；
- 车体原地旋转时对应的陀螺轴、正负号和积分比例；
- 电机振动、地面纹理和安装高度对数据质量的影响。

INA226 和 VL53L1X 不属于本轮标定，不接线、不分配顶层端口，也不加入
Robei 主工程。

## 2. 自动采集与判定工具

采集工具会同时保存原始串口、解码 CSV 和元数据 JSON：

```powershell
.\tools\read_motion_sensor_uart.ps1 `
  -Experiment stationary `
  -Seconds 60 `
  -Quiet
```

文件写入 `little_car/build/motion_sensor_capture/`。命令结束时会打印本次
CSV 的完整路径。分析静止实验：

```powershell
python .\tools\analyze_motion_calibration.py `
  <本次CSV路径> `
  --mode stationary `
  --output .\little_car\build\motion_sensor_capture\stationary_analysis.md `
  --json-output .\little_car\build\motion_sensor_capture\stationary_analysis.json
```

分析器检查连续序号、状态位、SQUAL、陀螺噪声和静止光流峰值。出现
`FAIL` 时保留数据用于故障定位，但不能把它固化为标定参数。

## 3. 实验前准备

1. 车轮架空或断开 C5 电机供电，独立固件本身不会发送电机命令。
2. 两块 BMI088 刚性固定，不能用松软胶带悬挂，且实验中不能移动安装方向。
3. PMW3901 镜头垂直朝向有纹理、无反光的平面，视野中不能出现车体、线束
   或安装板边缘。
4. 放好小车后按一次 FPGA `RESET`，至少 5 秒内不要触碰，让 512 个静止样本
   完成陀螺偏置估计。
5. 先运行 10 秒预检。`SQUAL` 长期为 0 时先处理镜头遮挡、焦距、地面纹理和
   照明，不继续位移标定。

## 4. 必做实验

### 4.1 静止 60 秒

```powershell
.\tools\read_motion_sensor_uart.ps1 -Experiment stationary -Seconds 60 -Quiet
```

整个采集期间不要碰车。合格样本要求三块器件始终在线、无 IMU 降级或分歧、
无序号缺口、至少 90% 帧 `SQUAL >= 20`，且静止陀螺噪声不过门限。

### 4.2 沿车头方向直移 500 mm

```powershell
.\tools\read_motion_sensor_uart.ps1 -Experiment forward_500mm -Seconds 20 -Quiet
```

开始后静止 2 秒，手推小车沿直尺方向匀速前移 500 mm，再静止至少 2 秒。
不要让车体旋转。分析命令：

```powershell
python .\tools\analyze_motion_calibration.py <CSV> --mode linear --distance-mm 500
```

### 4.3 向车体右侧平移 500 mm

```powershell
.\tools\read_motion_sensor_uart.ps1 -Experiment right_500mm -Seconds 20 -Quiet
python .\tools\analyze_motion_calibration.py <CSV> --mode linear --distance-mm 500
```

这两次直移共同确定光流轴交换、正负号、每毫米计数和交叉轴比例。每个方向
至少重复三次，使用中位数固化，不能只采用单次结果。

### 4.4 顺/逆时针原地旋转 360 度

```powershell
.\tools\read_motion_sensor_uart.ps1 -Experiment rotate_cw_360 -Seconds 20 -Quiet
python .\tools\analyze_motion_calibration.py <CSV> --mode rotation --angle-deg 360

.\tools\read_motion_sensor_uart.ps1 -Experiment rotate_ccw_360 -Seconds 20 -Quiet
python .\tools\analyze_motion_calibration.py <CSV> --mode rotation --angle-deg 360
```

围绕车体中心缓慢旋转，结束位置尽量回到起点。分析器会分别积分三根陀螺轴，
报告主旋转轴、符号和角度误差；顺逆方向结果必须近似互为相反数。

## 5. 第二轮稳定性实验

第一轮参数通过后再做以下实验：

- 静止但四轮架空低速转动 30 秒，测机械和电源噪声；
- 落地直行 1000 mm 三次，比较光流里程与实测距离；
- 原地 90/180/360 度各三次，检查角度误差是否随角度线性增长；
- 在浅色、深色、低纹理和正常赛道地面分别记录 SQUAL；
- 遮挡镜头和断开任一 BMI088，验证质量降级与安全状态，不验证自动控制。

## 6. 参数固化条件

只有以下材料齐全后才生成主工程参数：

- 合格静止 CSV 一份；
- 前移和右移各三份 500 mm CSV；
- 顺时针和逆时针各三份 360 度 CSV；
- 每份对应的分析 JSON；
- 两块 BMI088 和 PMW3901 在车体上的最终安装方向照片或文字记录。

届时从多次实验中取中位数，生成定点轴变换和比例常量，先加入只读
`motion_nav_fusion` Robei 模块并上报诊断，不直接改动 C5 电机速度。

## 7. 2026-08-04 即时预检

当前 COM20 仍能连续输出 `@M`。第一次 47 帧即时样本序号无缺口且状态均为
`0x9F`，但 `SQUAL=0` 且采样时车体存在明显扰动；随后车体静止时的 28 帧
样本中，融合陀螺 X/Y/Z 标准差已经降到约 `4.0/7.6/4.5` 原始计数，说明
双 IMU 可以稳定，但 `SQUAL` 仍只有 4 到 5。当前主要问题是 PMW3901 镜头
高度、遮挡、焦点、地面纹理或照明，不应继续光流位移标定。此前保留的
185 帧样本最后 50 帧较稳定，但完整样本中也存在明显扰动，不能代替新的
60 秒规范静止实验。
