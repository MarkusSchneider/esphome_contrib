#include "esphome/core/log.h"
#include "mbus_frame_meta.h"
#include "network_adapter.h"

namespace esphome {
namespace mbus {
static const char *const TAG = "mbus_serial_adapter";

int8_t SerialAdapter::send(std::vector<uint8_t> &payload) {
  this->uart_->write_array(payload);
  this->uart_->flush();
  return 0;
}

int8_t SerialAdapter::receive(std::vector<uint8_t> &payload) {
  // Append all available UART bytes to the accumulation buffer
  uint8_t byte = 0;
  while (this->uart_->available()) {
    this->uart_->read_byte(&byte);
    payload.push_back(byte);
    ESP_LOGV(TAG, "  <- 0x%02X", byte);
  }

  if (payload.empty()) {
    return -1;  // no data at all
  }

  // ACK frame: single byte 0xE5
  if (payload[0] == MBusFrameDefinition::ACK_FRAME.start_bit) {
    return 1;  // complete
  }

  // Short frame: start byte 0x10, exactly 5 bytes total
  if (payload[0] == MBusFrameDefinition::SHORT_FRAME.start_bit) {
    return (payload.size() >= MBusFrameDefinition::SHORT_FRAME.base_frame_size) ? 1 : 0;
  }

  // Long or Control frame: start byte 0x68
  // Frame structure: start(1) + L(1) + L(1) + start(1) + [C+A+CI+data](L) + checksum(1) + stop(1)
  // Total expected bytes = L + 6
  if (payload[0] == MBusFrameDefinition::LONG_FRAME.start_bit) {
    if (payload.size() < 2) {
      return 0;  // need at least 2 bytes to read the L field
    }
    const uint8_t l_field = payload[1];
    const size_t expected_size = static_cast<size_t>(l_field) + 6;
    return (payload.size() >= expected_size) ? 1 : 0;
  }

  // Unknown start byte — discard and signal error
  ESP_LOGW(TAG, "receive(): unknown start byte 0x%02X, discarding %zu bytes", payload[0], payload.size());
  payload.clear();
  return -1;
}

void SerialAdapter::flush_rx() {
  uint8_t byte = 0;
  while (this->uart_->available()) {
    this->uart_->read_byte(&byte);
  }
}

}  // namespace mbus
}  // namespace esphome
