# 环境与复现步骤

## 1. 下载

```powershell
git clone https://github.com/Wbf200699-rgb/robei_car.git
cd robei_car
```

路径可以自行选择；新构建入口以仓库位置为基准，不依赖原开发者 `D:/Projects` 目录。

## 2. 最快查看 FPGA 波形

已归档小波形在 `little_car/test/results/hardware_parameters`。用 RobeiEDA 打开 `robei_model_project/robei_model_project.proj`，选 `_parameter.test`，仿真结束检查 `RESULT TEST_PASS`。完整截图步骤见 [DEMO_GUIDE](DEMO_GUIDE.md)。

使用 Python 3.10+ 和 Icarus Verilog 重现：

```powershell
# 若工具不在 PATH，设为自己的 Robei/Icarus bin 目录。
$env:ROBEI_BIN='D:\robei\Robei\bin'
python tools/run_hardware_parameter_tests.py
python tools/check_repository.py
```

脚本运行九组确定性测试，单项有超时，VCD 上限 8 MiB；清单包含文件大小和 SHA-256。UART 测试使用加速时钟参数，时间轴不能当作真实波特率的实测。

## 3. 生成 FPGA 构建输入

```powershell
python tools/prepare_fpga_sources.py
```

从当前 `.model` 导出完整结构 Verilog、`little_car/build/filelist.vf` 和 `top.xdc`。生成目录被 Git 忽略；不需要将 Xilinx 随工具提供的几百个仿真库文件复制进仓库。

安装合法的 Vivado 并使 `vivado` 可从 PATH 访问，或设置 `VIVADO` 为自己的 `vivado.bat`，然后：

```powershell
tools/vivado_build_main.bat
```

目标 `xc7a50tfgg484-1`。本次整理仅验证源文件生成和 RTL 测试，未重跑 Vivado 综合/布线。构建前核对自己的板卡型号、管脚、电压和时钟约束；原始工程的约束完整性见历史报告。

Robei 全芯片原语仿真需要工具自带 Xilinx 库。精简 Robei 工程只包含项目源；实际芯片原语由安装环境提供。

## 4. Android App

使用 Android Studio、Android SDK Platform 36、JDK 17，打开 `android/car_app`。本地 SDK 路径保存在未跟踪的 `local.properties`，可从 `local.properties.example` 复制并修改。Android Studio 自带 JBR 也可作为满足 AGP 要求的运行 JDK。

```powershell
cd android/car_app
.\gradlew.bat testDebugUnitTest :app:assembleDebug :app:lintDebug
```

APK：`app/build/outputs/apk/debug/app-debug.apk`。Debug 包名为 `car.car_app.debug`。有同包不同签名时不能直接覆盖；保留标定数据后再处理签名冲突。

BLE 使用 CH9141K：FFF0 服务、FFF1 通知、FFF2 写入。需要手机授予附近设备权限。初次测试先验证连接、状态更新、松开停止和紧急停止，再运行自动动作。

## 5. 服务器与 ESP32-S3

```powershell
python -m venv server/.venv
server/.venv/Scripts/python.exe -m pip install -r server/requirements-dev.txt
server/.venv/Scripts/python.exe -m pytest server/tests -q
Copy-Item server/.env.example server/.env
```

编辑 `server/.env`，设置自己生成的两个 Token；不要使用示例占位符。Android 用 `uwb.serverUrl` / `uwb.mobileToken` 本地构建属性或环境变量注入。没有 UWB 环境时可以只验证 BLE 功能。

在仓库根目录启动本机服务：

```powershell
./server/run_server.ps1 -ListenAddress 127.0.0.1 -Port 18765
```

局域网演示参考 [tools/uwb/README](../tools/uwb/README.md)。ESP32 配置模板为 `esp32/uwb_gateway/gateway_secrets.example.h`；复制到被忽略的 `gateway_secrets.h` 并填写 Wi-Fi、服务地址和 Token。固件构建、接线详见 [网关说明](../esp32/uwb_gateway/README.md)。

## 6. 修改几何后

```powershell
python tools/generate_vehicle_geometry.py
python tools/generate_vehicle_geometry.py --check
python -m unittest tools.test_robei_converter tools.test_uwb_test_telemetry
```

不要用配置文件里的场地尺寸和标签位置代替实物测量，也不要用算法回放数值代替实车运动标定。
