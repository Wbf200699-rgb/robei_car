# 代码来源与第三方依赖

本项目整理自 `meguminawa-vrc/little_car_new` 的工程快照。各模块的原始署名、版权与注释应保留。当前未确认全部源码的授权归属，因此本次不添加 MIT/Apache 等全仓库开源许可，也不将全部模块声明为 Wbf200699-rgb 独立原创。公开展示不等于授予无限制再分发权。

使用的开发工具/依赖包括：

| 内容 | 提供方 / 来源 | 本仓库处理 |
| --- | --- | --- |
| Artix-7 原语与仿真库 | AMD/Xilinx Vivado | 不复制完整工具库，使用安装环境 |
| RobeiEDA | RobeiEDA | 保存项目 `.model/.test/.const/.proj`，不分发软件 |
| Gradle Wrapper | Gradle | 保留 Wrapper 和原始许可注释 |
| AndroidX、Material、OkHttp、JUnit | 各依赖项目 | 通过 Gradle 获取，不复制第三方二进制依赖目录 |
| FastAPI、Uvicorn、pytest | 各依赖项目 | 通过 requirements 获取 |
| ESP32 Arduino Core、WebSockets | Espressif / Links2004 等 | 通过 Arduino CLI 获取 |
| Icarus Verilog | Icarus Verilog 项目 | 通过 PATH / 本地工具安装调用 |

新增展示文档中的个人分工、比赛和奖项须由本人确认；未提供的内容不推断。
