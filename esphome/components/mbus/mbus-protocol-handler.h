#pragma once

#include <deque>
#include <memory>
#include <vector>

#include "stdint.h"

#include "mbus-frame.h"
#include "network-adapter.h"

namespace esphome {
namespace mbus {

class MBusCommand;
class MBusProtocolHandler {
 public:
  static const uint32_t rx_timeout{1000};

  void loop();
  void register_command(MBusFrame &command, void (*response_handler)(MBusCommand *command, const MBusFrame &response),
                        uint8_t data = 0, uint32_t delay = 0, bool wait_for_response = true);

  MBusProtocolHandler(INetworkAdapter *networkAdapter) : _networkAdapter(networkAdapter) {}

 protected:
  // Communication
  int8_t receive();
  int8_t send(MBusFrame &frame);

  // Parsing
  std::unique_ptr<MBusFrame> parse_response();
  std::unique_ptr<MBusDataVariable> parse_variable_data_response(std::vector<uint8_t> data);
  uint8_t get_dif_datalength(const uint8_t dif);

  // Decoder
  uint64_t decode_bcd_hex(std::vector<uint8_t> &bcd_data);
  std::string decode_manufacturer(uint8_t byte1, uint8_t byte2);
  uint8_t decode_int(std::vector<uint8_t> &data, int16_t *value);

  // Helper
  void delete_first_command();

  INetworkAdapter *_networkAdapter;
  std::vector<uint8_t> _rx_buffer;
  std::deque<MBusCommand *> _commands;

  uint32_t _timestamp{0};
  bool _waiting_for_response{false};
};

class MBusCommand {
 public:
  MBusFrame *command{nullptr};
  MBusProtocolHandler *protocol_handler{nullptr};
  uint8_t data{0};
  uint32_t created;
  uint32_t delay;
  bool wait_for_response;

  void (*response_handler)(MBusCommand *command, const MBusFrame &response){nullptr};

  MBusCommand(MBusFrame &command, void (*response_handler)(MBusCommand *command, const MBusFrame &response),
              uint8_t data, MBusProtocolHandler *protocol_handler, uint32_t delay, bool wait_for_response) {
    this->command = new MBusFrame(command);
    this->created = millis();
    this->data = data;
    this->delay = delay;
    this->protocol_handler = protocol_handler;
    this->response_handler = response_handler;
    this->wait_for_response = wait_for_response;
  }

  ~MBusCommand() {
    if (this->command != nullptr) {
      delete this->command;
      this->command = nullptr;
    }
  }
};

}  // namespace mbus
}  // namespace esphome
