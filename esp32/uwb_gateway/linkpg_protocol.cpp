#include "linkpg_protocol.h"

#include <cstring>


namespace {

uint16_t readBe16(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

uint32_t readBe32(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

int hexNibble(uint8_t value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

bool isSupportedFunction(uint8_t function) {
  return function == 0x03 || function == 0x06 || function == 0x10;
}

}  // namespace


uint16_t linkPgCrc16Update(uint16_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U)
                     : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}


uint16_t linkPgCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc = linkPgCrc16Update(crc, data[index]);
  }
  return crc;
}


void LinkPgParser::reset() {
  index_ = 0;
}


void LinkPgParser::discardPrefix(size_t count) {
  if (count >= index_) {
    reset();
    return;
  }
  memmove(buffer_, buffer_ + count, index_ - count);
  index_ -= count;
}


bool LinkPgParser::feed(uint8_t value, LinkPgFrame &frame) {
  if (index_ >= LINKPG_MAX_FRAME_BYTES) {
    ++framingErrors_;
    discardPrefix(1);
  }
  buffer_[index_++] = value;

  // Search with a sliding window instead of committing to the first possible
  // address byte. A partial frame at ESP32 boot or one damaged UART byte must
  // not consume the beginning of the next valid CA DA/ACK frame.
  while (index_ > 0) {
    size_t addressOffset = 0;
    if (modbusId_ == LINKPG_ANY_MODBUS_ID) {
      while (addressOffset + 1U < index_ &&
             !isSupportedFunction(buffer_[addressOffset + 1U])) {
        ++addressOffset;
      }
      if (addressOffset + 1U >= index_) {
        // Keep the final byte because it may be the address byte of a frame
        // whose function byte has not arrived yet.
        if (index_ > 1) discardPrefix(index_ - 1U);
        return false;
      }
    } else {
      while (addressOffset < index_ && buffer_[addressOffset] != modbusId_) {
        ++addressOffset;
      }
    }
    if (addressOffset > 0) discardPrefix(addressOffset);
    if (index_ < 2) return false;

    const uint8_t function = buffer_[1];
    if (!isSupportedFunction(function)) {
      ++framingErrors_;
      discardPrefix(1);
      continue;
    }

    size_t expectedLength = 8;
    if (function == 0x03) {
      if (index_ < 3) return false;
      expectedLength = static_cast<size_t>(buffer_[2]) + 5U;
      if (buffer_[2] < 2 || expectedLength > LINKPG_MAX_FRAME_BYTES) {
        ++framingErrors_;
        discardPrefix(1);
        continue;
      }
    }
    if (index_ < expectedLength) return false;

    if (linkPgCrc16(buffer_, expectedLength) != 0) {
      ++crcErrors_;
      discardPrefix(1);
      continue;
    }

    frame.length = expectedLength;
    memcpy(frame.data, buffer_, expectedLength);
    discardPrefix(expectedLength);
    ++goodFrames_;
    return true;
  }
  return false;
}


size_t linkPgBuildWriteRegister(uint8_t modbusId, uint16_t address, uint16_t value,
                                uint8_t *output, size_t capacity) {
  // Use function 0x10 with one register, matching the vendor start/stop example.
  if (capacity < 11) return 0;
  output[0] = modbusId;
  output[1] = 0x10;
  output[2] = static_cast<uint8_t>(address >> 8);
  output[3] = static_cast<uint8_t>(address);
  output[4] = 0x00;
  output[5] = 0x01;
  output[6] = 0x02;
  output[7] = static_cast<uint8_t>(value >> 8);
  output[8] = static_cast<uint8_t>(value);
  const uint16_t crc = linkPgCrc16(output, 9);
  output[9] = static_cast<uint8_t>(crc);
  output[10] = static_cast<uint8_t>(crc >> 8);
  return 11;
}


size_t linkPgBuildReadRegisters(uint8_t modbusId, uint16_t address, uint16_t count,
                               uint8_t *output, size_t capacity) {
  if (capacity < 8 || count == 0) return 0;
  output[0] = modbusId;
  output[1] = 0x03;
  output[2] = static_cast<uint8_t>(address >> 8);
  output[3] = static_cast<uint8_t>(address);
  output[4] = static_cast<uint8_t>(count >> 8);
  output[5] = static_cast<uint8_t>(count);
  const uint16_t crc = linkPgCrc16(output, 6);
  output[6] = static_cast<uint8_t>(crc);
  output[7] = static_cast<uint8_t>(crc >> 8);
  return 8;
}


bool linkPgParsePosition(const LinkPgFrame &frame, LinkPgPosition &position) {
  if (frame.length < 15 || frame.data[1] != 0x03 || frame.data[3] != 0xCA ||
      frame.data[4] != 0xDA) {
    return false;
  }

  const size_t dataEnd = 3U + frame.data[2];
  if (dataEnd + 2U != frame.length || dataEnd < 13U) return false;

  position = LinkPgPosition{};
  position.outputFlags = readBe16(&frame.data[5]);
  position.tagId = readBe16(&frame.data[7]);
  position.statusFlags = readBe32(&frame.data[9]);
  size_t offset = 13;

  if (position.outputFlags & 0x0001U) {
    if (offset + 6U > dataEnd) return false;
    position.xCm = static_cast<int16_t>(readBe16(&frame.data[offset]));
    position.yCm = static_cast<int16_t>(readBe16(&frame.data[offset + 2]));
    position.zCm = static_cast<int16_t>(readBe16(&frame.data[offset + 4]));
    offset += 6;
  }

  if (position.outputFlags & 0x0002U) {
    if (offset + 32U > dataEnd) return false;
    for (size_t anchor = 0; anchor < 16; ++anchor) {
      position.distancesCm[anchor] = readBe16(&frame.data[offset + anchor * 2]);
    }
  }
  return true;
}

bool linkPgParseAnchorDistance(const LinkPgFrame &frame,
                               LinkPgAnchorDistance &measurement) {
  if (frame.length != 13 || frame.data[1] != 0x03 || frame.data[2] != 0x08 ||
      frame.data[3] != 0xDA || frame.data[4] != 0xDA) {
    return false;
  }
  measurement = LinkPgAnchorDistance{};
  measurement.success = readBe16(&frame.data[5]) == 0x0001;
  measurement.activeAnchor = frame.data[7];
  measurement.passiveAnchor = frame.data[8];
  measurement.distanceCm = readBe16(&frame.data[9]);
  return true;
}


size_t linkPgEnvelopeLength(const uint8_t *payload, size_t available) {
  if (available < 2 || payload[1] != '1') return 0;
  if (payload[0] == 'C') return available >= 7 ? 7 : 0;
  if (payload[0] == 'A') return available >= 8 ? 8 : 0;
  if (payload[0] == 'F') {
    if (available < 10) return 0;
    const int high = hexNibble(payload[6]);
    const int low = hexNibble(payload[7]);
    if (high < 0 || low < 0) return 0;
    const size_t dataLength = static_cast<size_t>((high << 4) | low);
    const size_t totalLength = 10U + dataLength;
    return dataLength <= 64 && totalLength <= available ? totalLength : 0;
  }
  return 0;
}


bool linkPgParsePassData(const LinkPgFrame &frame, LinkPgPassData &passData) {
  if (frame.length < 11 || frame.data[1] != 0x03 || frame.data[3] != 0xED ||
      frame.data[4] != 0xDA || frame.data[2] < 4) {
    return false;
  }

  const size_t available = static_cast<size_t>(frame.data[2]) - 4U;
  if (available > LINKPG_MAX_PASS_BYTES || 7U + available + 2U != frame.length) return false;

  passData = LinkPgPassData{};
  passData.senderId = readBe16(&frame.data[5]);
  size_t payloadLength = linkPgEnvelopeLength(&frame.data[7], available);
  if (payloadLength == 0) {
    payloadLength = available;
    while (payloadLength > 0 && frame.data[7 + payloadLength - 1] == 0) --payloadLength;
  }
  passData.payloadLength = payloadLength;
  memcpy(passData.payload, &frame.data[7], payloadLength);
  return true;
}


bool linkPgIsWriteRegisterAck(const LinkPgFrame &frame, uint8_t modbusId,
                              uint16_t address) {
  return frame.length == 8 && frame.data[0] == modbusId && frame.data[1] == 0x10 &&
         readBe16(&frame.data[2]) == address && readBe16(&frame.data[4]) == 1;
}


bool linkPgIsSingleRegisterReadResponse(const LinkPgFrame &frame) {
  return frame.length == 7 && frame.data[1] == 0x03 && frame.data[2] == 0x02;
}
