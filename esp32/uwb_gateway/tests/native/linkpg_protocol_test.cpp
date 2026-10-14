#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "linkpg_protocol.h"

namespace {

bool feedBytes(LinkPgParser &parser, const uint8_t *data, size_t length,
               LinkPgFrame &lastFrame) {
  bool found = false;
  for (size_t index = 0; index < length; ++index) {
    if (parser.feed(data[index], lastFrame)) found = true;
  }
  return found;
}

}  // namespace

int main() {
  constexpr std::array<uint8_t, 11> kOfficialCancelPowerOnMode = {
      0x01, 0x10, 0x00, 0x3B, 0x00, 0x01, 0x02, 0x00, 0x09, 0x62, 0xDD};
  uint8_t generated[11] = {};
  assert(linkPgBuildWriteRegister(1, 59, 9, generated, sizeof(generated)) ==
         kOfficialCancelPowerOnMode.size());
  for (size_t index = 0; index < kOfficialCancelPowerOnMode.size(); ++index) {
    assert(generated[index] == kOfficialCancelPowerOnMode[index]);
  }

  constexpr std::array<uint8_t, 11> kOfficialStart = {
      0x01, 0x10, 0x00, 0x3B, 0x00, 0x01, 0x02, 0x00, 0x04, 0xA3, 0x18};
  assert(linkPgBuildWriteRegister(1, 59, 4, generated, sizeof(generated)) ==
         kOfficialStart.size());
  for (size_t index = 0; index < kOfficialStart.size(); ++index) {
    assert(generated[index] == kOfficialStart[index]);
  }

  constexpr std::array<uint8_t, 8> kOfficialAck = {
      0x01, 0x10, 0x00, 0x3B, 0x00, 0x01, 0x70, 0x04};
  LinkPgParser ackParser(1);
  LinkPgFrame frame;
  assert(feedBytes(ackParser, kOfficialAck.data(), kOfficialAck.size(), frame));
  assert(linkPgIsWriteRegisterAck(frame, 1, 59));

  // Reproduce a boot-time half-frame followed by unrelated bytes and then the
  // official ACK. The parser must slide to the later valid address/function.
  constexpr std::array<uint8_t, 15> kNoisyAck = {
      0x55, 0xAA, 0x01, 0x10, 0x00, 0x3B, 0x01,
      0x10, 0x00, 0x3B, 0x00, 0x01, 0x70, 0x04, 0x7E};
  LinkPgParser recoveryParser(1);
  assert(feedBytes(recoveryParser, kNoisyAck.data(), kNoisyAck.size(), frame));
  assert(linkPgIsWriteRegisterAck(frame, 1, 59));
  assert(recoveryParser.crcErrors() >= 1);

  // Vendor manual V6.6 continuous-auto-output example (CA DA position frame).
  constexpr std::array<uint8_t, 53> kOfficialPosition = {
      0x01, 0x03, 0x30, 0xCA, 0xDA, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01,
      0xFF, 0xFF, 0x01, 0x02, 0xFF, 0xC4, 0x00, 0x00, 0x00, 0x96, 0x00,
      0x82, 0x00, 0xC8, 0x01, 0xF0, 0x01, 0xFB, 0x02, 0x28, 0x01, 0xA8,
      0x01, 0x32, 0x01, 0x27, 0x01, 0x4A, 0x01, 0x46, 0x01, 0x4D, 0x01,
      0x50, 0x01, 0x49, 0x01, 0x4A, 0x01, 0x52, 0x5A, 0x9C};
  LinkPgParser positionParser(1);
  assert(feedBytes(positionParser, kOfficialPosition.data(), kOfficialPosition.size(), frame));
  LinkPgPosition position;
  assert(linkPgParsePosition(frame, position));
  assert(position.outputFlags == 0x0003);
  assert(position.tagId == 0);
  assert(position.xCm == 258);
  assert(position.yCm == -60);
  assert(position.distancesCm[0] == 150);

  constexpr std::array<uint8_t, 13> kOfficialAnchorDistance = {
      0x01, 0x03, 0x08, 0xDA, 0xDA, 0x00, 0x01,
      0xFF, 0xF0, 0x01, 0x8F, 0x2E, 0xEA};
  LinkPgParser calibrationParser(1);
  assert(feedBytes(calibrationParser, kOfficialAnchorDistance.data(),
                   kOfficialAnchorDistance.size(), frame));
  LinkPgAnchorDistance measurement;
  assert(linkPgParseAnchorDistance(frame, measurement));
  assert(measurement.success);
  assert(measurement.activeAnchor == 0xFF);
  assert(measurement.passiveAnchor == 0xF0);
  assert(measurement.distanceCm == 399);

  std::puts("TEST_PASS linkpg_protocol");
  return 0;
}
