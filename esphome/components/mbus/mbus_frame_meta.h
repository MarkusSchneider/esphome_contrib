#pragma once
#include <cstdint>

namespace esphome {
namespace mbus {

/// @brief M-Bus special addresses
class MBusAddresses {
 public:
  static const uint8_t NETWORK_LAYER = 0xFD;
  static const uint8_t TEST_ADDRESS = 0xFE;
  static const uint8_t BROADCAST = 0xFF;
};

/// @brief M-Bus control codes for different command types
class MBusControlCodes {
 public:
  static const uint8_t SND_NKE = 0x40;
  static const uint8_t SND_UD_DEVICE = 0x53;
  static const uint8_t SND_UD_CONTROLLER = 0x73;
  static const uint8_t REQ_UD1 = 0x5A;
  static const uint8_t REQ_UD2 = 0x5B;
  static const uint8_t RSP_UD = 0x08;
};

/// @brief M-Bus control information codes
class MBusControlInformationCodes {
 public:
  static const uint8_t SELECTION_OF_DEVICES_MODE1 = 0x52;
  static const uint8_t VARIABLE_DATA_RESPONSE_MODE1 = 0x72;
};

/// @brief Metadata for different M-Bus frame types
class MBusFrameMeta {
 public:
  const uint8_t start_bit{0};
  const uint8_t stop_bit{0};
  const uint8_t length{0};
  const uint8_t base_frame_size{0};

  MBusFrameMeta(uint8_t start_bit, uint8_t stop_bit, uint8_t length, uint8_t base_frame_size)
      : start_bit(start_bit), stop_bit(stop_bit), length(length), base_frame_size(base_frame_size) {}
};

class MBusFrameDefinition {
 public:
  static const uint8_t MAX_DATA_LENGTH{252};
  static const MBusFrameMeta ACK_FRAME;
  static const MBusFrameMeta SHORT_FRAME;
  static const MBusFrameMeta CONTROL_FRAME;
  static const MBusFrameMeta LONG_FRAME;
};

}  // namespace mbus
}  // namespace esphome
