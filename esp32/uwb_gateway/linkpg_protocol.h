#pragma once

#include <Arduino.h>


constexpr size_t LINKPG_MAX_FRAME_BYTES = 128;
constexpr size_t LINKPG_MAX_PASS_BYTES = 80;
constexpr uint8_t LINKPG_ANY_MODBUS_ID = 0xFF;


struct LinkPgFrame {
  uint8_t data[LINKPG_MAX_FRAME_BYTES];
  size_t length = 0;
};


struct LinkPgPosition {
  uint16_t outputFlags = 0;
  uint16_t tagId = 0;
  uint32_t statusFlags = 0;
  int16_t xCm = 0;
  int16_t yCm = 0;
  int16_t zCm = 0;
  uint16_t distancesCm[16] = {};
};


struct LinkPgPassData {
  uint16_t senderId = 0;
  uint8_t payload[LINKPG_MAX_PASS_BYTES] = {};
  size_t payloadLength = 0;
};

struct LinkPgAnchorDistance {
  bool success = false;
  uint8_t activeAnchor = 0;
  uint8_t passiveAnchor = 0;
  uint16_t distanceCm = 0;
};


class LinkPgParser {
 public:
  explicit LinkPgParser(uint8_t modbusId = 1) : modbusId_(modbusId) {}

  bool feed(uint8_t value, LinkPgFrame &frame);
  uint32_t goodFrames() const { return goodFrames_; }
  uint32_t crcErrors() const { return crcErrors_; }
  uint32_t framingErrors() const { return framingErrors_; }

 private:
  void reset();
  void discardPrefix(size_t count);
  uint8_t modbusId_;
  uint8_t buffer_[LINKPG_MAX_FRAME_BYTES] = {};
  size_t index_ = 0;
  uint32_t goodFrames_ = 0;
  uint32_t crcErrors_ = 0;
  uint32_t framingErrors_ = 0;
};


uint16_t linkPgCrc16Update(uint16_t crc, uint8_t value);
uint16_t linkPgCrc16(const uint8_t *data, size_t length);
size_t linkPgBuildWriteRegister(uint8_t modbusId, uint16_t address, uint16_t value,
                                uint8_t *output, size_t capacity);
size_t linkPgBuildReadRegisters(uint8_t modbusId, uint16_t address, uint16_t count,
                               uint8_t *output, size_t capacity);
bool linkPgParsePosition(const LinkPgFrame &frame, LinkPgPosition &position);
bool linkPgParsePassData(const LinkPgFrame &frame, LinkPgPassData &passData);
bool linkPgParseAnchorDistance(const LinkPgFrame &frame,
                               LinkPgAnchorDistance &measurement);
bool linkPgIsWriteRegisterAck(const LinkPgFrame &frame, uint8_t modbusId,
                              uint16_t address);
bool linkPgIsSingleRegisterReadResponse(const LinkPgFrame &frame);
size_t linkPgEnvelopeLength(const uint8_t *payload, size_t available);
