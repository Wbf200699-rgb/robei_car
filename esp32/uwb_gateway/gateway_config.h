#pragma once

// GPIO13/GPIO14 are the dedicated LinkPG UART pins on the current ESP32-S3
// gateway. They remain separate from UART0 used by the board's COM connector.
constexpr int UWB_UART_RX_PIN = 13;  // ESP32 RX <- LinkPG A TXD
constexpr int UWB_UART_TX_PIN = 14;  // ESP32 TX -> LinkPG A RXD
constexpr uint32_t UWB_UART_BAUD = 115200;
constexpr uint8_t LINKPG_MODBUS_ID = 1;

constexpr bool AUTO_START_POSITIONING = true;
// PG3.9 can take longer than the ESP32-S3 to finish its own power-on sequence.
// Do not send the first Modbus request while the anchor is still booting.
constexpr uint32_t AUTO_START_DELAY_MS = 3000;
constexpr uint32_t POSITION_COMMAND_ACK_TIMEOUT_MS = 700;
constexpr uint32_t POSITION_COMMAND_SETTLE_MS = 150;
constexpr uint32_t POSITION_START_BURST_GAP_MS = 500;
constexpr uint8_t POSITION_START_BURST_ATTEMPTS = 3;
constexpr uint32_t POSITION_DATA_START_TIMEOUT_MS = 8000;
constexpr uint32_t POSITION_RECOVERY_INTERVAL_MS = 10000;
constexpr uint32_t POSITION_STALE_MS = 8000;
constexpr uint32_t UWB_TX_GAP_MS = 25;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;
constexpr uint32_t WIFI_RECOVERY_INTERVAL_MS = 10000;

constexpr char GATEWAY_ID[] = "anchor-a";
constexpr char FIRMWARE_VERSION[] = "0.4.0-s3";

#if __has_include("gateway_secrets.h")
#include "gateway_secrets.h"
#else
constexpr char WIFI_SSID[] = "CHANGE_ME";
constexpr char WIFI_PASSWORD[] = "CHANGE_ME";
constexpr char SERVER_HOST[] = "192.168.1.2";
constexpr uint16_t SERVER_PORT = 8765;
constexpr bool SERVER_USE_TLS = false;
constexpr char SERVER_GATEWAY_TOKEN[] = "change-me-gateway";
#endif
