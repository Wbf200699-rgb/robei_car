# little_car 源码通读包

本目录从当前 Robei 工程导出，用于脱离图形界面通读 Verilog。它不参与 Robei
工程编译，也不会覆盖 `little_car/model`、`little_car/build` 或 `little_car/test`。

## 目录内容

- `verilog/`：45 个可读 Verilog 模块。包括当前项目登记模块，以及顶层实际依赖但名称拼写不同的 `route_contol_core`。
- `testbenches/`：4 个已经用于实验的独立 Verilog testbench。
- `module_manifest.csv`：每个模块的源 `.model`、端口数、实例数、连线数和代码行数。
- `all_sources.vf`：全部导出模块的文件清单。
- `ARCHITECTURE.md`：系统数据流、模块分层和主要算法。
- `ARCHITECTURE_DIAGRAM.md`：按当前顶层真实连接绘制的详细可缩放架构图。
- `KEY_VARIABLES.md`：状态编码、参数、协议字段和关键变量。
- `READING_GUIDE.md`：建议阅读顺序和调试定位方法。
- `export_sources.py`：可重复执行的导出脚本。
- `source_annotations.py`：45 个模块的中文职责、端口、内部变量、过程和关键分支注释元数据。
- `verify_annotations.ps1`：检查所有声明、常量、连续赋值、过程、函数、任务和初始化块是否都有注释。
- `verify_sources.bat`：调用 Vivado 对全部导出模块和当前系统顶层做 RTL 展开验证。

重新导出：

```powershell
python little_car\source_review\export_sources.py
```

## 导出范围说明

Robei 的 `.model` 有两种源码形态：叶子模块把行为代码放在根节点 `Code` 属性；
层次模块把子模块和连线分别保存在 `Module`、`Port`、`Wire` XML 节点中。原有
`tools/robei_converter.py model2v` 只导出根端口和根 `Code`，不能还原顶层图形连线。
本目录的导出器额外恢复实例和网络，因此 `sys_route_gui_fsm_top.v`、`hdmi_top.v`
和 `sys_route_control_fsm.v` 都包含完整层次。

所有 `verilog/*.v` 还会自动加入以下阅读注释：模块职责和算法、逐端口含义、层次网络
驱动端/接收端、子模块职责、每一条内部声明和常量、每一条连续赋值、函数参数和局部量、
任务、时序/组合过程、初始化块以及关键状态分支。变量注释会尽量给出单位、编码、有效条件、
更新时机和下游用途；函数内部的组合临时量不会误写成时钟寄存器。这些注释由
`source_annotations.py` 生成，重新执行导出脚本不会丢失。

注释块中的 `[ASSIGN]`、`[PROCESS]`、`[FUNCTION]`、`[TASK]` 和 `[INITIAL]`
是供自动覆盖检查使用的 ASCII 锚点，标签后的中文才是对应代码的完整语义说明。
校验脚本会按 UTF-8 显式读取文件，因此 Windows PowerShell 5 也不会把中文行错误合并。

重新导出后可执行完整检查：

```powershell
powershell -ExecutionPolicy Bypass -File little_car\source_review\verify_annotations.ps1
little_car\source_review\verify_sources.bat
```

第一条命令要求所有受检项目的 `missing=0`；第二条命令要求 Vivado
`synth_design -rtl` 成功且没有 Critical Warning 或 Error。

## 当前源码需要留意的历史问题

1. `sys_route_control_fsm` 实际实例化名称拼写为 `route_contol_core`，不要误读成项目清单里的 `sys_route_control_core`。
2. 实际 `route_contol_core` 使用 16 项数组，但部分边界判断允许计数达到 32。阅读路线容量时以该模块的真实数组边界为准，这里存在需要单独验证的越界风险。
3. `clk_wiz_0` 不是 Vivado IP 输出，而是直接使用允许的 `MMCME2_ADV`、`BUFG` 原语；HDMI 串化也直接使用 `ODDR`、`OBUFDS`。
4. `temp_*`、`gui_route_test_top`、旧 TB6612 电机模块属于配置源、演示或旧底盘链路，并非当前 C5 小车主执行链。
