# UWB 服务与移动端交接

## 当前部署入口

- 健康检查：`https://relay.example.invalid/health`
- 状态查询：`GET https://relay.example.invalid/api/v1/state`
- 控制命令：`POST https://relay.example.invalid/api/v1/command`
- 雷达定位启停：`POST https://relay.example.invalid/api/v1/uwb/run`
- 移动端 WebSocket：`wss://relay.example.invalid/ws/mobile?token=<mobile-token>`

健康检查无需鉴权，其余移动端接口使用 `server/.env` 中的
`UWB_MOBILE_TOKEN`。密钥文件已被 `.gitignore` 排除，不应提交、截图或
写入 APK 源码；移动端应通过本地构建配置或安全配置页注入。

## REST 示例

所有受保护的 REST 请求都带请求头：

```text
X-API-Token: <mobile-token>
Content-Type: application/json
```

读取全量状态：

```http
GET /api/v1/state
```

向车载标签 1 发送现有 FPGA 单字节命令：

```http
POST /api/v1/command

{"command":"W","tag_id":1}
```

启动或停止 LinkPG 的临时连续定位输出：

```http
POST /api/v1/uwb/run

{"running":true}
```

`running=false` 停止输出。该接口只改 LinkPG 寄存器 59 的临时值，不会
改写基站坐标、设备角色、标签列表、信道或掉电保存配置。

## WebSocket 命令

```json
{"type":"command","command":"W","tag_id":1}
{"type":"uwb_run","running":true}
{"type":"ping","nonce":"mobile-check-1"}
```

手动运动按键按住时约每 100 ms 重发一次方向命令，松开立即发送 `C`。
这与 FPGA 现有约 300 ms 手动控制看门狗配合，不能只在按下瞬间发送
一次运动命令。

USB 直读确认四块标签的协议 ID 为 0/1/2/3。物理标签 1 负责 FPGA
数据透传，因此移动端控制命令默认 `tag_id` 使用 1。四块标签当前随机
摆放，移动端不得解释车体姿态；最终安装后再配置四角 ID 映射。

## 移动端应消费的数据

- `gateway`：网关在线状态、固件版本、RSSI、UWB 好帧数和 CRC 错误数；
- `tags`：各标签最近一次坐标、距离和定位有效标志；
- `pose`：四标签融合得到的车体中心、航向和矩形一致性结果；
- `vehicle_lines`：由 FPGA 原协议还原的 `@S`、`@L`、`@K`、`@PONG`；
- 命令 ACK：只证明命令已送入 FPGA 桥接器，车辆实际状态仍以新
  `@S` 帧为准。

在实测并填写车长、车宽前，服务器会把四标签姿态标为
`unconfigured`。不要把零坐标或过期标签数据画成有效轨迹。移动端应
同时检查定位成功位、数据时间和 `pose.quality`。

## 联调判定

1. `gateway_connected=true`：ESP32 到服务器网络链路正常。
2. `good_frames` 递增：主基站到 ESP32 串口及 LinkPG 定位输出正常。
3. 命令获得 `A1...0...` ACK：服务器、ESP32、UWB 透传和 FPGA 接收正常。
4. 收到新 `@S`：FPGA 回传、UWB 透传、服务器重组正常。
5. 四个标签均新鲜且 `pose.quality=full`：基站坐标、标签角色和车体尺寸
   均通过一致性检查，可以开始记录轨迹和建图。

完整字节协议见 `docs/UWB_GATEWAY_PROTOCOL.md`，物理接线见
`docs/UWB_INTEGRATION_WIRING.md`。
