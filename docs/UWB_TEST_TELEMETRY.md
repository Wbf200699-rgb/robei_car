# UWB 测试观测通道

## 用途边界

这条链路只用于研发阶段观测小车的位置、航向、速度趋势和 UWB 数据质量，便于定位问题、标定参数和验证后续算法。

- 它不是参赛方案的一部分，不作为竞赛功能、计算能力或通信架构的申报依据。
- 它不参与电机控制、避障、急停或任何实时安全闭环。
- S3、服务器或公网失联时，小车现有的蓝牙控制、本地路线执行和本地安全逻辑不受影响。
- 原始采集文件只作为工程测试证据；从数据中得到的参数或算法仍需在 FPGA 本地独立验证后才能使用。

## 数据链路

```text
4 个车载 UWB 标签
        |
        v
基站 A/B/C/D 定位网络
        |
        v
主基站 A TTL UART -> ESP32-S3 -> HTTPS/WSS 服务器
                                      |
                                      v
                         /api/v1/state 状态快照
                                      |
                                      v
                    本机采集器 -> JSONL + CSV
                                      |
                                      v
                    离线分析器 -> JSON + Markdown
```

现有服务器按四角标签计算车体中心和航向。当前标签角色为：

| 车体位置 | 标签 ID |
|---|---:|
| 左前 | 1 |
| 右前 | 2 |
| 左后 | 3 |
| 右后 | 0 |

车长和车宽当前仍为 `0`，因此四标签齐全时姿态质量通常为 `unconfigured`。安装位置最终确定后，应测量四个标签天线相位中心之间的实际长、宽，再配置服务器并重新采集基线。

## 采集

采集器默认从环境变量 `UWB_MOBILE_TOKEN` 或本项目 `server/.env` 读取令牌，不会打印令牌。

静止采集 60 秒：

```powershell
python .\tools\capture_uwb_test_telemetry.py --label stationary --duration 60
```

低速直线采集 30 秒：

```powershell
python .\tools\capture_uwb_test_telemetry.py --label straight_low_speed --duration 30
```

不设时长，按 `Ctrl+C` 停止：

```powershell
python .\tools\capture_uwb_test_telemetry.py --label free_run
```

默认输出目录：

```text
little_car/test_results/uwb_test_telemetry/
```

每次采集生成：

- `*.jsonl`：完整、逐条带时间戳的测试快照，是后续复查的主记录。
- `*.csv`：扁平字段，适合 Excel、Python 或绘图软件直接读取。

JSONL 元数据和每条采样均带有 `test_only=true`；元数据还带有 `competition_design_input=false`。

## 离线分析

把命令中的文件名替换为实际采集文件：

```powershell
python .\tools\analyze_uwb_test_telemetry.py .\little_car\test_results\uwb_test_telemetry\20260805_120000_stationary.jsonl
```

分析器在同一目录生成：

- `*.summary.json`：供脚本继续处理的结构化统计结果。
- `*.report.md`：便于人工阅读和留档的报告。

报告包含：

- 网关在线率、数据新鲜率、四标签同时有效率和有效采样频率。
- 每个标签的位置中位数、数据年龄、基站距离和相对中位位置的 RMS/P95 半径。
- 车体中心轨迹长度、首尾位移、速度和角速度统计。
- 四边测量值与四标签时间偏差。
- S3 的有效帧、CRC 错误、组帧错误、队列丢弃等计数器增量。

标签半径统计只有在小车全程静止时才能解释为定位抖动；运动采集中的该数值包含真实运动，不能当成定位精度。

## 建议实验顺序

1. **静止基线**：小车不动，连续采集 120 秒，确认四标签有效率、时间偏差和静止抖动。
2. **低速直线**：固定速度前进和后退各做 3 次，记录起点、终点实测距离，评估速度比例和直线偏航。
3. **原地旋转**：左右各转一圈，观察航向连续性、角度跨越 `0/360` 时是否正常，以及左右旋转差异。
4. **矩形路线**：相同路线重复 5 次，对比闭环误差和每条边的重复性。
5. **暂时遮挡**：短时遮挡一个标签，确认观测工具把姿态标记为无效，不跨越失效区间计算虚假速度。

每次只改变一个实验变量，并把速度、轮速修正、地面材质、载荷和电池电压写进 `--label` 或单独实验记录。这样才能把机械偏差、电机差异和 UWB 抖动区分开。

## 验证工具本身

```powershell
python -m unittest .\tools\test_uwb_test_telemetry.py -v
```

单元测试覆盖角度回绕、窗口速度估计、失效后窗口清空、字段隔离、轨迹统计和网关计数器增量。

## 给 Codex 分析

采集完成后保留对应的 `jsonl`、`csv`、`summary.json` 和 `report.md`，直接告诉 Codex 文件路径和该次实验的真实动作、实测距离、速度设置及异常现象。Codex 可以读取这些文件，给出定位质量、运动误差和参数调整建议；不需要把令牌、服务器配置或手机日志贴出来。
