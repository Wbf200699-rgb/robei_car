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

仓库 `.github/workflows/verify.yml` 会在 GitHub 独立环境执行 Python、RTL 和 Android 构建/测试/Lint，并保存波形、APK 与报告。实际远端结果以 [Actions](https://github.com/Wbf200699-rgb/robei_car/actions) 为准；不能只凭工作流文件存在就标记通过。

## 本次未执行

- Vivado 综合、布局布线、时序签核和烧录。
- 手机安装、BLE 连接、运动急停和后台/断线实车行为。
- UWB RF 精度、传感器动态标定、真实地图/导航误差。
- 实车视频、照片或界面截图采集。

历史数据请查阅原实验文档并保留测试条件。新构建入口修复了可复现性，不等于硬件时序、管脚、电气或整车性能已重新验收。
