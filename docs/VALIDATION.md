# 本次整理验证记录

日期：2026-10-14。测试对象为本展示仓库，不继承此前其他工作区的测试通过结论。

## 已在本机执行

| 检查 | 结果 |
| --- | --- |
| `python tools/generate_vehicle_geometry.py --check` | 通过，几何派生代码一致 |
| `python -m unittest tools.test_robei_converter tools.test_uwb_test_telemetry` | 12 项通过 |
| `python -m pytest server/tests -q` | 31 项通过 |
| `python tools/prepare_fpga_sources.py` | 生成 58 个 RTL 模块、相对路径 filelist 和 top.xdc |
| `python tools/run_hardware_parameter_tests.py` | 九组通过；VCD 总计 26,951 字节，最大 11,494 字节 |
| `python tools/check_repository.py` | 九份波形大小/SHA-256/日志、两个 Robei 工程引用、发布文件检查通过 |
| `git diff --cached --check` | 通过，源快照中的行尾空格已整理 |

Python 3.12；Icarus 使用本机 RobeiEDA 附带的工具，通过 `ROBEI_BIN` 指定。服务器测试依赖安装在隔离虚拟环境；实际 RF/蓝牙设备不参与这些测试。

## Android 本地检查

已尝试：

```powershell
.\gradlew.bat testDebugUnitTest :app:assembleDebug :app:lintDebug
```

使用本机 Android SDK 和 Android Studio JBR。Gradle 在下载 Maven Central 依赖时出现 `Remote host terminated the handshake`，离线模式缺少部分已登记缓存。已尝试本机配置的代理与官方备用域名，仍被网络握手阻断，因此当前**不声称 Android 本地构建、测试或 Lint 通过**，也没有生成本次 APK。

## GitHub 独立环境验证：已通过

已实际运行 [Actions #38502945903](https://github.com/Wbf200699-rgb/robei_car/actions/runs/38502945903)，验证提交 `1950e015fe968a8e5c409f2a2bce4655a99f42aa`：

| 任务 | 实际结果 |
| --- | --- |
| `python-and-rtl` | 成功：发布检查、几何一致性、12 项工具测试、31 项服务器测试、源码生成和九组 RTL 测试 |
| `android` | 成功：`testDebugUnitTest :app:assembleDebug :app:lintDebug`，构建日志 `BUILD SUCCESSFUL` |

已上传两个 Actions 产物：`compact-rtl-waveforms` 和 `debug-apk-and-reports`，可在该运行页面下载。Android 构建耗时约 2 分 29 秒；Lint 任务成功不等于不存在警告。

首次 CI 暴露了 VCD 在 Git 暂存时的行尾转换和 Android 初始化请求已下架 `tools` 包的问题。已修复并重跑成功；干净检出副本也验证了九份 VCD 原始字节与清单一致。上节保留本机网络失败的记录，与此处远端成功结果分别报告。

后续文档记录提交不改变已验证的代码；最新运行状态以 [Actions](https://github.com/Wbf200699-rgb/robei_car/actions) 为准。工作流仍会保存测试与构建产物，不把 CI 通过外推成实车验证。

## 本次未执行

- Vivado 综合、布局布线、时序签核和烧录。
- 手机安装、BLE 连接、运动急停和后台/断线实车行为。
- UWB RF 精度、传感器动态标定、真实地图/导航误差。
- 实车视频、照片或界面截图采集。

历史数据请查阅原实验文档并保留测试条件。新构建入口修复了可复现性，不等于硬件时序、管脚、电气或整车性能已重新验收。
