#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <esp_system.h>

#include "gateway_config.h"
#include "linkpg_protocol.h"


namespace {

HardwareSerial UwbSerial(2);
WebSocketsClient ServerSocket;
LinkPgParser UwbParser(LINKPG_MODBUS_ID);

constexpr size_t TX_QUEUE_DEPTH = 8;
constexpr size_t TX_ITEM_CAPACITY = 128;
constexpr uint32_t SERIAL_DIAGNOSTIC_INTERVAL_MS = 5000;
constexpr size_t UWB_BOOT_SAMPLE_BYTES = 96;

enum class PositionRecoveryState : uint8_t {
  WAIT_BOOT,
  QUEUE_STOP,
  WAIT_STOP_ACK,
  STOP_SETTLE,
  QUEUE_START,
  WAIT_START_ACK,
  START_RETRY_GAP,
  WAIT_POSITION,
  RUNNING,
  RECOVERY_DELAY,
  INACTIVE,
};

enum class AnchorCalibrationState : uint8_t {
  IDLE,
  WAIT_STOP,
  WAIT_ENTER_ACK,
  QUEUE_PAIR,
  WAIT_PAIR_RESULT,
  WAIT_EXIT_ACK,
};

struct AnchorPair {
  uint8_t active;
  uint8_t passive;
};

constexpr AnchorPair ANCHOR_PAIRS[] = {
    {0xFF, 0xF0},  // A-B
    {0xFF, 0xF1},  // A-C
    {0xFF, 0xF2},  // A-D
    {0xF0, 0xF1},  // B-C
    {0xF0, 0xF2},  // B-D
    {0xF1, 0xF2},  // C-D
};
// Use the vendor manual's exact 0x01FF example while bringing the feature up.
// This removes sample-count compatibility as a variable during hardware tests.
constexpr uint16_t CALIBRATION_SAMPLE_COUNT = 255;
constexpr uint32_t CALIBRATION_COMMAND_TIMEOUT_MS = 2500;
constexpr uint32_t CALIBRATION_PAIR_TIMEOUT_MS = 20000;

struct TxItem {
  uint8_t data[TX_ITEM_CAPACITY] = {};
  size_t length = 0;
};

TxItem txQueue[TX_QUEUE_DEPTH];
size_t txHead = 0;
size_t txTail = 0;
size_t txCount = 0;
uint32_t txQueueDrops = 0;
uint32_t uwbTxCount = 0;
uint32_t gatewaySequence = 0;
uint32_t lastUwbTxMs = 0;
uint32_t lastStatusMs = 0;
uint32_t uwbRxBytes = 0;
uint32_t positionFrameCount = 0;
uint32_t passFrameCount = 0;
uint32_t rawFrameCount = 0;
uint32_t dataSendTxCount = 0;
uint32_t dataSendShortWriteCount = 0;
uint32_t atOkCount = 0;
uint32_t atErrorCount = 0;
uint8_t atOkMatch = 0;
uint8_t atErrorMatch = 0;
uint32_t lastUwbRxMs = 0;
uint32_t lastPositionFrameMs = 0;
uint32_t lastPositionStartRequestMs = 0;
uint32_t positionStartRequestCount = 0;
uint32_t positionStopRequestCount = 0;
uint32_t positionRunModeAckCount = 0;
uint32_t positionStartAckCount = 0;
uint32_t positionStateSinceMs = 0;
uint32_t lastSerialDiagnosticMs = 0;
uint32_t lastWifiRecoveryMs = 0;
uint32_t wifiRecoveryCount = 0;
bool positioningWanted = AUTO_START_POSITIONING;
uint8_t positionStopAttempts = 0;
uint8_t positionStartAttempts = 0;
PositionRecoveryState positionState =
    AUTO_START_POSITIONING ? PositionRecoveryState::WAIT_BOOT
                           : PositionRecoveryState::INACTIVE;
bool socketStarted = false;
bool socketConnected = false;
wl_status_t lastWifiStatus = WL_NO_SHIELD;
AnchorCalibrationState calibrationState = AnchorCalibrationState::IDLE;
uint32_t calibrationStateSinceMs = 0;
size_t calibrationPairIndex = 0;
uint8_t calibrationPairRetries = 0;
uint8_t calibrationStopRetries = 0;
uint8_t uwbBootSample[UWB_BOOT_SAMPLE_BYTES] = {};
size_t uwbBootSampleCount = 0;
bool uwbBootSamplePrinted = false;


void captureUwbBootSample(uint8_t value) {
  if (uwbBootSamplePrinted) return;
  if (uwbBootSampleCount < UWB_BOOT_SAMPLE_BYTES) {
    uwbBootSample[uwbBootSampleCount++] = value;
  }
  if (uwbBootSampleCount != UWB_BOOT_SAMPLE_BYTES) return;

  Serial.print("[UWB-RAW] ");
  for (size_t index = 0; index < UWB_BOOT_SAMPLE_BYTES; ++index) {
    if (uwbBootSample[index] < 0x10) Serial.print('0');
    Serial.print(uwbBootSample[index], HEX);
    if (index + 1U < UWB_BOOT_SAMPLE_BYTES) Serial.print(' ');
  }
  Serial.println();
  uwbBootSamplePrinted = true;
}


bool queueBytes(const uint8_t *data, size_t length) {
  if (length == 0 || length > TX_ITEM_CAPACITY || txCount >= TX_QUEUE_DEPTH) {
    ++txQueueDrops;
    return false;
  }
  memcpy(txQueue[txTail].data, data, length);
  txQueue[txTail].length = length;
  txTail = (txTail + 1U) % TX_QUEUE_DEPTH;
  ++txCount;
  return true;
}


bool queueRunMode(uint16_t mode) {
  uint8_t frame[11];
  const size_t length = linkPgBuildWriteRegister(LINKPG_MODBUS_ID, 59, mode, frame, sizeof(frame));
  return queueBytes(frame, length);
}

bool queueRegisterWrite(uint16_t address, uint16_t value) {
  uint8_t frame[11];
  const size_t length = linkPgBuildWriteRegister(
      LINKPG_MODBUS_ID, address, value, frame, sizeof(frame));
  const bool queued = queueBytes(frame, length);
  Serial.printf("[UWB-REG] addr=%04X value=%04X queued=%u\n",
                static_cast<unsigned>(address), static_cast<unsigned>(value),
                queued ? 1U : 0U);
  return queued;
}


bool queuePositioningStart() {
  if (!queueRunMode(4)) return false;
  lastPositionStartRequestMs = millis();
  ++positionStartRequestCount;
  return true;
}


void setPositionState(PositionRecoveryState state) {
  positionState = state;
  positionStateSinceMs = millis();
}


void beginPositioningRecovery() {
  positionStopAttempts = 0;
  positionStartAttempts = 0;
  setPositionState(PositionRecoveryState::QUEUE_STOP);
}


int hexNibble(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}


bool decodeHex(const String &text, uint8_t *output, size_t capacity, size_t &length) {
  if ((text.length() & 1U) != 0 || text.length() / 2U > capacity) return false;
  length = text.length() / 2U;
  for (size_t index = 0; index < length; ++index) {
    const int high = hexNibble(text[index * 2]);
    const int low = hexNibble(text[index * 2 + 1]);
    if (high < 0 || low < 0) return false;
    output[index] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}


String encodeHex(const uint8_t *data, size_t length) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(length * 2U);
  for (size_t index = 0; index < length; ++index) {
    encoded += kHex[data[index] >> 4];
    encoded += kHex[data[index] & 0x0F];
  }
  return encoded;
}


bool queueDataSend(uint16_t targetId, const uint8_t *payload, size_t payloadLength) {
  if (payloadLength == 0 || payloadLength > LINKPG_MAX_PASS_BYTES || targetId > 255) return false;
  TxItem item;
  const char prefix[] = "AT+DataSend=\"";
  const size_t prefixLength = sizeof(prefix) - 1;
  if (prefixLength + payloadLength + 10U > sizeof(item.data)) return false;

  memcpy(item.data, prefix, prefixLength);
  size_t offset = prefixLength;
  for (size_t index = 0; index < payloadLength; ++index) {
    const uint8_t value = payload[index];
    if (value < 0x20 || value > 0x7E || value == '"' || value == '\\') return false;
    item.data[offset++] = value;
  }
  const int written = snprintf(reinterpret_cast<char *>(&item.data[offset]),
                               sizeof(item.data) - offset, "\",\"%u\"\r\n", targetId);
  if (written <= 0 || offset + static_cast<size_t>(written) > sizeof(item.data)) return false;
  item.length = offset + static_cast<size_t>(written);
  return queueBytes(item.data, item.length);
}


void updateAtResponseDiagnostic(uint8_t value) {
  static constexpr uint8_t kOk[] = {'O', 'K', '\r', '\n'};
  static constexpr uint8_t kError[] = {'E', 'R', 'R', 'O', 'R', '\r', '\n'};

  if (value == kOk[atOkMatch]) {
    ++atOkMatch;
    if (atOkMatch == sizeof(kOk)) {
      ++atOkCount;
      atOkMatch = 0;
      Serial.printf("[UWB-AT] OK count=%lu\n", static_cast<unsigned long>(atOkCount));
    }
  } else {
    atOkMatch = value == kOk[0] ? 1U : 0U;
  }

  if (value == kError[atErrorMatch]) {
    ++atErrorMatch;
    if (atErrorMatch == sizeof(kError)) {
      ++atErrorCount;
      atErrorMatch = 0;
      Serial.printf("[UWB-AT] ERROR count=%lu\n", static_cast<unsigned long>(atErrorCount));
    }
  } else {
    atErrorMatch = value == kError[0] ? 1U : 0U;
  }
}


void processUwbTx() {
  if (txCount == 0 || millis() - lastUwbTxMs < UWB_TX_GAP_MS) return;
  TxItem &item = txQueue[txHead];
  const bool isDataSend =
      item.length >= 12U && memcmp(item.data, "AT+DataSend=", 12U) == 0;
  const size_t written = UwbSerial.write(item.data, item.length);
  // Wait only for TX completion. Passing false also flushes the RX side in
  // ESP32 Arduino and can discard a concurrent CA DA/ED DA report.
  UwbSerial.flush();
  if (isDataSend) {
    ++dataSendTxCount;
    if (written != item.length) ++dataSendShortWriteCount;
    Serial.printf("[UWB-TX] DataSend count=%lu bytes=%u/%u\n",
                  static_cast<unsigned long>(dataSendTxCount),
                  static_cast<unsigned>(written), static_cast<unsigned>(item.length));
  }
  txHead = (txHead + 1U) % TX_QUEUE_DEPTH;
  --txCount;
  ++uwbTxCount;
  lastUwbTxMs = millis();
}


void sendServerText(const String &message) {
  if (socketConnected) ServerSocket.sendTXT(message.c_str(), message.length());
}


void setCalibrationState(AnchorCalibrationState state) {
  calibrationState = state;
  calibrationStateSinceMs = millis();
}


void finishAnchorCalibration(bool success, const char *reason) {
  sendServerText(String("CALSTATE,") + (success ? "COMPLETE," : "FAILED,") + reason);
  calibrationPairIndex = 0;
  calibrationPairRetries = 0;
  calibrationStopRetries = 0;
  setCalibrationState(AnchorCalibrationState::IDLE);
  // Register 59 values 5..8 are persistent auto-start modes. The vendor
  // manual requires value 9 to leave such a mode before calibration. Restore
  // value 8 here so normal automatic positioning survives the next power-up.
  queueRunMode(8);
  positioningWanted = true;
  setPositionState(PositionRecoveryState::WAIT_POSITION);
}


bool startAnchorCalibration() {
  if (calibrationState != AnchorCalibrationState::IDLE) return false;
  positioningWanted = false;
  setPositionState(PositionRecoveryState::INACTIVE);
  calibrationPairIndex = 0;
  calibrationPairRetries = 0;
  calibrationStopRetries = 0;
  // The main anchor currently uses power-on automatic output (mode 8). The
  // official V6.6 manual says mode 9, not mode 0, is required to leave any
  // power-on automatic mode and stop ranging before anchor calibration.
  if (!queueRunMode(9)) {
    positioningWanted = true;
    beginPositioningRecovery();
    return false;
  }
  setCalibrationState(AnchorCalibrationState::WAIT_STOP);
  sendServerText("CALSTATE,STARTED,stopping_positioning");
  return true;
}


void cancelAnchorCalibration(const char *reason) {
  if (calibrationState == AnchorCalibrationState::IDLE) return;
  queueRegisterWrite(0x0073, 0x0000);
  finishAnchorCalibration(false, reason);
}


void processAnchorCalibration() {
  if (calibrationState == AnchorCalibrationState::IDLE) return;
  const uint32_t age = millis() - calibrationStateSinceMs;
  if (calibrationState == AnchorCalibrationState::WAIT_STOP &&
      age >= CALIBRATION_COMMAND_TIMEOUT_MS) {
    if (++calibrationStopRetries < 3U && queueRunMode(9)) {
      setCalibrationState(AnchorCalibrationState::WAIT_STOP);
      sendServerText(String("CALSTATE,STOPPING,attempt_") +
                     static_cast<unsigned>(calibrationStopRetries + 1U));
    } else {
      cancelAnchorCalibration("stop_timeout_after_3_attempts");
    }
  } else if (calibrationState == AnchorCalibrationState::WAIT_ENTER_ACK &&
             age >= CALIBRATION_COMMAND_TIMEOUT_MS) {
    if (++calibrationPairRetries < 3U &&
        queueRegisterWrite(0x0073,
                           static_cast<uint16_t>(0x0100U | CALIBRATION_SAMPLE_COUNT))) {
      setCalibrationState(AnchorCalibrationState::WAIT_ENTER_ACK);
      sendServerText(String("CALSTATE,ENTERING,attempt_") +
                     static_cast<unsigned>(calibrationPairRetries + 1U));
    } else {
      cancelAnchorCalibration("enter_timeout_after_3_attempts");
    }
  } else if (calibrationState == AnchorCalibrationState::QUEUE_PAIR) {
    const AnchorPair pair = ANCHOR_PAIRS[calibrationPairIndex];
    const uint16_t value = (static_cast<uint16_t>(pair.active) << 8) | pair.passive;
    if (queueRegisterWrite(0x0074, value)) {
      setCalibrationState(AnchorCalibrationState::WAIT_PAIR_RESULT);
    }
  } else if (calibrationState == AnchorCalibrationState::WAIT_PAIR_RESULT &&
             age >= CALIBRATION_PAIR_TIMEOUT_MS) {
    if (calibrationPairRetries++ < 1) {
      setCalibrationState(AnchorCalibrationState::QUEUE_PAIR);
    } else {
      cancelAnchorCalibration("pair_timeout");
    }
  } else if (calibrationState == AnchorCalibrationState::WAIT_EXIT_ACK &&
             age >= CALIBRATION_COMMAND_TIMEOUT_MS) {
    finishAnchorCalibration(true, "exit_ack_timeout_but_measurements_complete");
  }
}


void reportPosition(const LinkPgPosition &position) {
  ++positionFrameCount;
  lastPositionFrameMs = millis();
  setPositionState(PositionRecoveryState::RUNNING);
  char message[224];
  snprintf(message, sizeof(message),
           "POS,%lu,%u,%08lX,%d,%d,%d,%u,%u,%u,%u",
           static_cast<unsigned long>(gatewaySequence++), position.tagId,
           static_cast<unsigned long>(position.statusFlags), position.xCm, position.yCm,
           position.zCm, position.distancesCm[0], position.distancesCm[1],
           position.distancesCm[2], position.distancesCm[3]);
  sendServerText(message);
}


void reportPassData(const LinkPgPassData &passData) {
  ++passFrameCount;
  String message = "PASS," + String(gatewaySequence++) + "," + String(passData.senderId) + ",";
  message += encodeHex(passData.payload, passData.payloadLength);
  sendServerText(message);
}


void reportRawFrame(const LinkPgFrame &frame) {
  ++rawFrameCount;
  String message = "RAW," + String(gatewaySequence++) + ",";
  message += encodeHex(frame.data, frame.length);
  sendServerText(message);
}


void processUwbRx() {
  LinkPgFrame frame;
  while (UwbSerial.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(UwbSerial.read());
    ++uwbRxBytes;
    lastUwbRxMs = millis();
    captureUwbBootSample(value);
    updateAtResponseDiagnostic(value);
    if (!UwbParser.feed(value, frame)) continue;

    if (linkPgIsWriteRegisterAck(frame, LINKPG_MODBUS_ID, 0x0073)) {
      reportRawFrame(frame);
      if (calibrationState == AnchorCalibrationState::WAIT_ENTER_ACK) {
        calibrationPairRetries = 0;
        setCalibrationState(AnchorCalibrationState::QUEUE_PAIR);
        sendServerText("CALSTATE,MEASURING,0");
      } else if (calibrationState == AnchorCalibrationState::WAIT_EXIT_ACK) {
        finishAnchorCalibration(true, "ok");
      }
      continue;
    }

    if (linkPgIsWriteRegisterAck(frame, LINKPG_MODBUS_ID, 0x0074)) {
      reportRawFrame(frame);
      continue;
    }

    LinkPgAnchorDistance anchorDistance;
    if (linkPgParseAnchorDistance(frame, anchorDistance)) {
      if (calibrationState == AnchorCalibrationState::WAIT_PAIR_RESULT) {
        const AnchorPair expected = ANCHOR_PAIRS[calibrationPairIndex];
        if (anchorDistance.activeAnchor == expected.active &&
            anchorDistance.passiveAnchor == expected.passive) {
          char report[96];
          snprintf(report, sizeof(report), "CALPAIR,%u,%u,%u,%u,%u",
                   static_cast<unsigned>(calibrationPairIndex),
                   static_cast<unsigned>(anchorDistance.activeAnchor),
                   static_cast<unsigned>(anchorDistance.passiveAnchor),
                   anchorDistance.success ? 1U : 0U,
                   static_cast<unsigned>(anchorDistance.distanceCm));
          sendServerText(report);
          if (!anchorDistance.success || anchorDistance.distanceCm == 0) {
            cancelAnchorCalibration("pair_measurement_failed");
          } else if (++calibrationPairIndex >=
                     sizeof(ANCHOR_PAIRS) / sizeof(ANCHOR_PAIRS[0])) {
            if (queueRegisterWrite(0x0073, 0x0000)) {
              setCalibrationState(AnchorCalibrationState::WAIT_EXIT_ACK);
              sendServerText("CALSTATE,EXITING,measurements_complete");
            }
          } else {
            calibrationPairRetries = 0;
            setCalibrationState(AnchorCalibrationState::QUEUE_PAIR);
            sendServerText(String("CALSTATE,MEASURING,") + calibrationPairIndex);
          }
        }
      }
      reportRawFrame(frame);
      continue;
    }

    if (linkPgIsWriteRegisterAck(frame, LINKPG_MODBUS_ID, 59)) {
      ++positionRunModeAckCount;
      if (calibrationState == AnchorCalibrationState::WAIT_STOP) {
        calibrationStopRetries = 0;
        if (queueRegisterWrite(
                0x0073, static_cast<uint16_t>(0x0100U | CALIBRATION_SAMPLE_COUNT))) {
          setCalibrationState(AnchorCalibrationState::WAIT_ENTER_ACK);
          sendServerText("CALSTATE,ENTERING,attempt_1");
        } else {
          cancelAnchorCalibration("enter_queue_full");
        }
      } else if (positionState == PositionRecoveryState::WAIT_STOP_ACK) {
        setPositionState(PositionRecoveryState::STOP_SETTLE);
      } else if (positionState == PositionRecoveryState::WAIT_START_ACK) {
        ++positionStartAckCount;
        setPositionState(PositionRecoveryState::WAIT_POSITION);
      }
      reportRawFrame(frame);
      continue;
    }

    LinkPgPosition position;
    if (linkPgParsePosition(frame, position)) {
      reportPosition(position);
      continue;
    }

    LinkPgPassData passData;
    if (linkPgParsePassData(frame, passData)) {
      reportPassData(passData);
      continue;
    }

    reportRawFrame(frame);
  }
}


void handleReadRegisters(const String &message) {
  const int comma1 = message.indexOf(',');
  const int comma2 = message.indexOf(',', comma1 + 1);
  const int comma3 = message.indexOf(',', comma2 + 1);
  if (comma1 < 0 || comma2 < 0 || comma3 < 0) {
    sendServerText("SENDQ,UWB_READ,FORMAT");
    return;
  }

  const String requestId = message.substring(comma1 + 1, comma2);
  const long address = message.substring(comma2 + 1, comma3).toInt();
  const long count = message.substring(comma3 + 1).toInt();
  uint8_t frame[8];
  const size_t length = (address >= 0 && address <= 0xFFFF && count >= 1 && count <= 60)
      ? linkPgBuildReadRegisters(LINKPG_MODBUS_ID, static_cast<uint16_t>(address),
                                 static_cast<uint16_t>(count), frame, sizeof(frame))
      : 0;
  if (length == 0 || !queueBytes(frame, length)) {
    sendServerText("SENDQ," + requestId + ",REJECTED");
    return;
  }
  sendServerText("SENDQ," + requestId + ",0");
}


void handleSendCommand(const String &message) {
  const int comma1 = message.indexOf(',');
  const int comma2 = message.indexOf(',', comma1 + 1);
  const int comma3 = message.indexOf(',', comma2 + 1);
  if (comma1 < 0 || comma2 < 0 || comma3 < 0) {
    sendServerText("SENDQ,UNKNOWN,FORMAT");
    return;
  }

  const String requestId = message.substring(comma1 + 1, comma2);
  const int target = message.substring(comma2 + 1, comma3).toInt();
  const String payloadHex = message.substring(comma3 + 1);
  uint8_t payload[LINKPG_MAX_PASS_BYTES];
  size_t payloadLength = 0;
  if (target < 0 || target > 255 ||
      !decodeHex(payloadHex, payload, sizeof(payload), payloadLength) ||
      !queueDataSend(static_cast<uint16_t>(target), payload, payloadLength)) {
    sendServerText("SENDQ," + requestId + ",REJECTED");
    return;
  }
  sendServerText("SENDQ," + requestId + ",0");
}


void handleServerText(const String &message) {
  if (message.startsWith("SEND,")) {
    handleSendCommand(message);
  } else if (message.startsWith("UWB_READ,")) {
    handleReadRegisters(message);
  } else if (message == "UWB_START") {
    positioningWanted = true;
    beginPositioningRecovery();
    sendServerText("SENDQ,UWB_START,0");
  } else if (message == "UWB_STOP") {
    positioningWanted = false;
    setPositionState(PositionRecoveryState::INACTIVE);
    sendServerText(queueRunMode(0) ? "SENDQ,UWB_STOP,0" : "SENDQ,UWB_STOP,QUEUE_FULL");
  } else if (message == "UWB_CALIBRATE") {
    sendServerText(startAnchorCalibration()
                       ? "CALSTATE,ACCEPTED,0"
                       : "CALSTATE,REJECTED,busy_or_queue_full");
  } else if (message == "UWB_CALIBRATE_CANCEL") {
    cancelAnchorCalibration("cancelled");
  } else if (message.startsWith("PING,")) {
    sendServerText("PONG," + message.substring(5));
  } else {
    sendServerText("ERROR,UNKNOWN_COMMAND");
  }
}


void socketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      socketConnected = true;
      Serial.println("[WS] connected");
      sendServerText(String("HELLO,1,") + GATEWAY_ID + "," + FIRMWARE_VERSION);
      break;
    case WStype_DISCONNECTED:
      socketConnected = false;
      Serial.println("[WS] disconnected");
      break;
    case WStype_ERROR:
      Serial.printf("[WS] error length=%u\n", static_cast<unsigned>(length));
      break;
    case WStype_TEXT:
      handleServerText(String(reinterpret_cast<char *>(payload), length));
      break;
    default:
      break;
  }
}


void startServerSocket() {
  String path = String("/ws/gateway/") + GATEWAY_ID + "?token=" + SERVER_GATEWAY_TOKEN;
  if (SERVER_USE_TLS) {
    ServerSocket.beginSSL(SERVER_HOST, SERVER_PORT, path);
  } else {
    ServerSocket.begin(SERVER_HOST, SERVER_PORT, path);
  }
  ServerSocket.onEvent(socketEvent);
  ServerSocket.setReconnectInterval(3000);
  ServerSocket.enableHeartbeat(15000, 3000, 2);
  socketStarted = true;
}


void processNetwork() {
  const wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    const bool wasConnected = lastWifiStatus == WL_CONNECTED;
    lastWifiStatus = wifiStatus;
    if (wifiStatus == WL_CONNECTED) {
      Serial.printf("[WIFI] connected ip=%s rssi=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
      Serial.printf("[WIFI] status=%d\n", static_cast<int>(wifiStatus));
      // A WebSocketsClient instance retains the old TCP endpoint across a
      // Wi-Fi outage. Tear it down so the next association creates a fresh
      // socket instead of waiting indefinitely on the stale connection.
      if (wasConnected && socketStarted) {
        ServerSocket.disconnect();
        socketStarted = false;
        socketConnected = false;
      }
    }
  }

  if (wifiStatus != WL_CONNECTED && strcmp(WIFI_SSID, "CHANGE_ME") != 0 &&
      millis() - lastWifiRecoveryMs >= WIFI_RECOVERY_INTERVAL_MS) {
    lastWifiRecoveryMs = millis();
    ++wifiRecoveryCount;
    Serial.printf("[WIFI] recovery attempt=%lu\n",
                  static_cast<unsigned long>(wifiRecoveryCount));
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  if (wifiStatus == WL_CONNECTED && !socketStarted) startServerSocket();
  if (socketStarted) ServerSocket.loop();

  if (millis() - lastSerialDiagnosticMs >= SERIAL_DIAGNOSTIC_INTERVAL_MS) {
    lastSerialDiagnosticMs = millis();
    Serial.printf(
        "[STAT] up=%lu wifi=%d ws=%u heap=%u psram_free=%u rx=%lu good=%lu pos=%lu pass=%lu "
        "tx=%lu data_tx=%lu short=%lu at_ok=%lu at_err=%lu starts=%lu stops=%lu "
        "run_ack=%lu start_ack=%lu pos_state=%u drops=%lu wifi_recovery=%lu\n",
        static_cast<unsigned long>(millis()), static_cast<int>(wifiStatus),
        socketConnected ? 1U : 0U, static_cast<unsigned>(ESP.getFreeHeap()),
        static_cast<unsigned>(ESP.getFreePsram()), static_cast<unsigned long>(uwbRxBytes),
        static_cast<unsigned long>(UwbParser.goodFrames()),
        static_cast<unsigned long>(positionFrameCount), static_cast<unsigned long>(passFrameCount),
        static_cast<unsigned long>(uwbTxCount),
        static_cast<unsigned long>(dataSendTxCount),
        static_cast<unsigned long>(dataSendShortWriteCount),
        static_cast<unsigned long>(atOkCount), static_cast<unsigned long>(atErrorCount),
        static_cast<unsigned long>(positionStartRequestCount),
        static_cast<unsigned long>(positionStopRequestCount),
        static_cast<unsigned long>(positionRunModeAckCount),
        static_cast<unsigned long>(positionStartAckCount),
        static_cast<unsigned>(positionState),
        static_cast<unsigned long>(txQueueDrops),
        static_cast<unsigned long>(wifiRecoveryCount));
  }

  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    const int rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127;
    const uint32_t rxIdleMs = lastUwbRxMs == 0 ? 0xFFFFFFFFUL : millis() - lastUwbRxMs;
    const uint32_t positionIdleMs =
        lastPositionFrameMs == 0 ? 0xFFFFFFFFUL : millis() - lastPositionFrameMs;
    char status[280];
    snprintf(status, sizeof(status),
             "STATUS,%lu,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu",
             static_cast<unsigned long>(millis()), rssi,
             static_cast<unsigned long>(UwbParser.goodFrames()),
             static_cast<unsigned long>(UwbParser.crcErrors()),
             static_cast<unsigned long>(uwbTxCount),
             static_cast<unsigned long>(txQueueDrops),
             static_cast<unsigned long>(uwbRxBytes),
             static_cast<unsigned long>(UwbParser.framingErrors()),
             static_cast<unsigned long>(positionFrameCount),
             static_cast<unsigned long>(passFrameCount),
             static_cast<unsigned long>(rawFrameCount),
             static_cast<unsigned long>(rxIdleMs), positioningWanted ? 1U : 0U,
             static_cast<unsigned long>(positionIdleMs),
             static_cast<unsigned long>(positionStartRequestCount));
    sendServerText(status);
  }
}


void processPositioningWatchdog() {
  if (!positioningWanted) {
    if (positionState != PositionRecoveryState::INACTIVE) {
      setPositionState(PositionRecoveryState::INACTIVE);
    }
    return;
  }
  const uint32_t now = millis();
  const uint32_t stateAge = now - positionStateSinceMs;

  switch (positionState) {
    case PositionRecoveryState::WAIT_BOOT:
      if (now >= AUTO_START_DELAY_MS) beginPositioningRecovery();
      break;

    case PositionRecoveryState::QUEUE_STOP:
      // Modes 5..8 are persistent power-on modes. The LinkPG V6.6 manual
      // explicitly requires value 9, rather than 0, to leave one of them.
      if (queueRunMode(9)) {
        ++positionStopAttempts;
        ++positionStopRequestCount;
        setPositionState(PositionRecoveryState::WAIT_STOP_ACK);
      }
      break;

    case PositionRecoveryState::WAIT_STOP_ACK:
      if (stateAge >= POSITION_COMMAND_ACK_TIMEOUT_MS) {
        if (positionStopAttempts < POSITION_START_BURST_ATTEMPTS) {
          setPositionState(PositionRecoveryState::QUEUE_STOP);
        } else {
          // Some PG3.9 firmware executes mode 9 but drops or truncates its
          // Modbus ACK on USART1. Continue after the bounded retry burst; the
          // following mode-4 command is accepted only when positioning data
          // resumes, so a missing ACK cannot create a false RUNNING state.
          setPositionState(PositionRecoveryState::STOP_SETTLE);
        }
      }
      break;

    case PositionRecoveryState::STOP_SETTLE:
      if (stateAge >= POSITION_COMMAND_SETTLE_MS) {
        setPositionState(PositionRecoveryState::QUEUE_START);
      }
      break;

    case PositionRecoveryState::QUEUE_START:
      if (queuePositioningStart()) {
        ++positionStartAttempts;
        setPositionState(PositionRecoveryState::WAIT_START_ACK);
      }
      break;

    case PositionRecoveryState::WAIT_START_ACK:
      // A fresh, CRC-valid position frame is stronger evidence than the
      // optional write ACK and works with PG3.9 revisions that omit the ACK.
      if (lastPositionFrameMs >= lastPositionStartRequestMs &&
          lastPositionFrameMs != 0 && now - lastPositionFrameMs < POSITION_STALE_MS) {
        setPositionState(PositionRecoveryState::RUNNING);
      } else if (stateAge >= POSITION_COMMAND_ACK_TIMEOUT_MS) {
        if (positionStartAttempts < POSITION_START_BURST_ATTEMPTS) {
          setPositionState(PositionRecoveryState::START_RETRY_GAP);
        } else {
          setPositionState(PositionRecoveryState::RECOVERY_DELAY);
        }
      }
      break;

    case PositionRecoveryState::START_RETRY_GAP:
      if (stateAge >= POSITION_START_BURST_GAP_MS) {
        setPositionState(PositionRecoveryState::QUEUE_START);
      }
      break;

    case PositionRecoveryState::WAIT_POSITION:
      if (lastPositionFrameMs != 0 && now - lastPositionFrameMs < POSITION_STALE_MS) {
        setPositionState(PositionRecoveryState::RUNNING);
      } else if (stateAge >= POSITION_DATA_START_TIMEOUT_MS) {
        setPositionState(PositionRecoveryState::RECOVERY_DELAY);
      }
      break;

    case PositionRecoveryState::RUNNING:
      if (lastPositionFrameMs == 0 || now - lastPositionFrameMs >= POSITION_STALE_MS) {
        setPositionState(PositionRecoveryState::RECOVERY_DELAY);
      }
      break;

    case PositionRecoveryState::RECOVERY_DELAY:
      if (stateAge >= POSITION_RECOVERY_INTERVAL_MS) beginPositioningRecovery();
      break;

    case PositionRecoveryState::INACTIVE:
      beginPositioningRecovery();
      break;
  }
}

}  // namespace


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[BOOT] firmware=%s reset_reason=%d\n", FIRMWARE_VERSION,
                static_cast<int>(esp_reset_reason()));
  Serial.printf("[BOOT] flash=%u psram=%u free_heap=%u uart_rx=%d uart_tx=%d\n",
                static_cast<unsigned>(ESP.getFlashChipSize()),
                static_cast<unsigned>(ESP.getPsramSize()), static_cast<unsigned>(ESP.getFreeHeap()),
                UWB_UART_RX_PIN, UWB_UART_TX_PIN);
  UwbSerial.begin(UWB_UART_BAUD, SERIAL_8N1, UWB_UART_RX_PIN, UWB_UART_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (strcmp(WIFI_SSID, "CHANGE_ME") != 0) {
    Serial.printf("[WIFI] connecting ssid=%s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else {
    Serial.println("Wi-Fi credentials are placeholders; UWB UART parsing remains active.");
  }
}


void loop() {
  processAnchorCalibration();
  processPositioningWatchdog();
  processUwbRx();
  processUwbTx();
  processNetwork();
  delay(1);
}
