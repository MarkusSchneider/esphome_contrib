#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "mbus_frame.h"
#include "mbus_frame_factory.h"
#include "mbus_frame_meta.h"
#include "mbus_protocol_handler.h"

namespace esphome {
namespace mbus {

static const char *const TAG = "mbus_protocol";

void MBusProtocolHandler::register_command(MBusFrame &command,
                                           void (*response_handler)(const MBusCommand &command,
                                                                    const MBusFrame &response),
                                           uint8_t step, uint32_t delay, bool wait_for_response) {
  auto cmd = std::make_shared<MBusCommand>(command, response_handler, step, this->mbus_, delay, wait_for_response);
  this->commands_.push_back(cmd);
}

void MBusProtocolHandler::loop() {
  auto now = millis();
  ESP_LOGVV(TAG, "loop. waiting_for_response_ = %d, timestamp_ = %d, now = %d, timed out: %d",
            this->waiting_for_response_, this->timestamp_, now,
            (now - this->timestamp_) > MBusProtocolHandler::RX_TIMEOUT);

  if (!this->waiting_for_response_ && !this->commands_.empty()) {
    auto cmd = this->commands_.front();
    if ((now - cmd->created) < cmd->delay) {
      return;
    }

    auto frame = std::move(cmd->command);
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
    frame->dump();
#endif
    this->send_(*frame);

    this->waiting_for_response_ = true;
    this->timestamp_ = now;

    delay(10);
    return;
  }

  if (this->waiting_for_response_) {
    auto command = this->commands_.front();

    if (!command->wait_for_response) {
      delay(25);

      if (command->response_handler != nullptr) {
        auto frame = MBusFrameFactory::create_empty_frame();
        command->response_handler(*command, *frame);
      }

      delete_first_command_();
      return;
    }

    if ((now - this->timestamp_) > MBusProtocolHandler::RX_TIMEOUT) {
      ESP_LOGW(TAG, "M-Bus data: Timeout");

      if (command->response_handler != nullptr) {
        auto frame = MBusFrameFactory::create_empty_frame();
        command->response_handler(*command, *frame);
      }

      delete_first_command_();
      return;  // rx timeout
    }

    auto rx_status = this->receive_();

    // Stop Bit received
    if (rx_status == 1) {
      if (command->response_handler != nullptr) {
        auto frame = this->parse_response_();
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
        frame->dump();
#endif
        command->response_handler(*command, *frame);
      }

      delete_first_command_();
      return;
    }

    // Partial Data received
    if (rx_status == 0) {
      this->timestamp_ = now;
      delay(10);
      return;
    }

    delay(25);
  }
}

void MBusProtocolHandler::delete_first_command_() {
  // Remove the first command
  if (!this->commands_.empty()) {
    this->commands_.erase(this->commands_.begin());
  }

  this->waiting_for_response_ = false;
  this->timestamp_ = 0;

  // Clear and release memory
  this->rx_buffer_.clear();
  this->rx_buffer_.shrink_to_fit();
}

int8_t MBusProtocolHandler::send_(MBusFrame &frame) {
  // Clear software receive buffer and drain any leftover bytes from the UART hardware FIFO.
  // Without this, stale bytes from a previous truncated frame would corrupt the next response.
  this->rx_buffer_.clear();
  this->rx_buffer_.shrink_to_fit();
  this->network_adapter_->flush_rx();

  uint8_t payload_size = 0;
  switch (frame.frame_type) {
    case MBUS_FRAME_TYPE_EMPTY:
      payload_size = 0;
      break;
    case MBUS_FRAME_TYPE_ACK:
      payload_size = MBusFrameDefinition::ACK_FRAME.base_frame_size;
      break;
    case MBUS_FRAME_TYPE_SHORT:
      payload_size = MBusFrameDefinition::SHORT_FRAME.base_frame_size;
      break;
    case MBUS_FRAME_TYPE_CONTROL:
      payload_size = MBusFrameDefinition::CONTROL_FRAME.base_frame_size;
      break;
    case MBUS_FRAME_TYPE_LONG:
      payload_size = MBusFrameDefinition::LONG_FRAME.base_frame_size + frame.data.size();
      break;
  }

  std::vector<uint8_t> payload(payload_size, 0);
  MBusFrame::serialize(frame, payload);

  char hex_buf_send[format_hex_pretty_size(MBUS_FRAME_DATA_LENGTH + 9)];
  ESP_LOGV(TAG, "Send mbus data: %s", format_hex_pretty_to(hex_buf_send, payload));
  this->network_adapter_->send(payload);
  this->timestamp_ = millis();

  // payload will be automatically destroyed when going out of scope
  return 0;
}

/// @return 0 if more data needed, 1 if frame is completed, -1 if response timed out
int8_t MBusProtocolHandler::receive_() {
  auto rx_status = this->network_adapter_->receive(this->rx_buffer_);

  if (rx_status == 0) {
    return 0;
  }

  if (rx_status == -1) {
    // no data received. Try next loop.
    return -1;
  }

  if (rx_status == 1) {
    // End of Frame received.
    char hex_buf_recv[format_hex_pretty_size(MBUS_FRAME_DATA_LENGTH + 9)];
    ESP_LOGV(TAG, "Received mbus data: %s", format_hex_pretty_to(hex_buf_recv, this->rx_buffer_));
    return 1;
  }

  return 0;
}

std::unique_ptr<MBusFrame> MBusProtocolHandler::parse_response_() {
  if (this->rx_buffer_.empty()) {
    return MBusFrameFactory::create_empty_frame();
  }

  //     Single Character (ACK)
  //    ------------------
  // 0  |      E5h       |
  //    ------------------
  if (this->rx_buffer_.at(0) == MBusFrameDefinition::ACK_FRAME.start_bit) {
    return MBusFrameFactory::create_ack_frame();
  }

  //      Short Frame
  //   ------------------
  // 0 |   Start 10h    |
  //   ------------------
  // 1 |    C Field     |
  //   ------------------
  // 2 |    A Field     |
  //   ------------------
  // 3 |   Check Sum    |
  //   ------------------
  // 4 |    Stop 16h    |
  //   ------------------
  if (this->rx_buffer_.at(0) == MBusFrameDefinition::SHORT_FRAME.start_bit &&
      this->rx_buffer_.size() == MBusFrameDefinition::SHORT_FRAME.base_frame_size) {
    auto frame =
        MBusFrameFactory::create_short_frame(this->rx_buffer_.at(1), this->rx_buffer_.at(2), this->rx_buffer_.at(3));
    const uint8_t expected_checksum = MBusFrame::calc_checksum(*frame);
    if (frame->checksum != expected_checksum) {
      ESP_LOGE(TAG, "parse_response_(): Short frame checksum mismatch: got 0x%02X expected 0x%02X", frame->checksum,
               expected_checksum);
      return MBusFrameFactory::create_empty_frame();
    }
    return frame;
  }

  //     Control Frame
  //   ------------------
  // 0 |   Start 68h    |
  //   ------------------
  // 1 |  L Field = 3   |
  //   ------------------
  // 2 |  L Field = 3   |
  //   ------------------
  // 3 |   Start 68h    |
  //   ------------------
  // 4 |    C Field     |
  //   ------------------
  // 5 |    A Field     |
  //   ------------------
  // 6 |    CI Field    |
  //   ------------------
  // 7 |   Check Sum    |
  //   ------------------
  // 8 |    Stop 16h    |
  //   ------------------
  if (this->rx_buffer_.at(0) == MBusFrameDefinition::CONTROL_FRAME.start_bit &&
      this->rx_buffer_.size() == MBusFrameDefinition::CONTROL_FRAME.base_frame_size) {
    auto frame = MBusFrameFactory::create_control_frame(this->rx_buffer_.at(4), this->rx_buffer_.at(5),
                                                        this->rx_buffer_.at(6), this->rx_buffer_.at(7));
    const uint8_t expected_checksum = MBusFrame::calc_checksum(*frame);
    if (frame->checksum != expected_checksum) {
      ESP_LOGE(TAG, "parse_response_(): Control frame checksum mismatch: got 0x%02X expected 0x%02X", frame->checksum,
               expected_checksum);
      return MBusFrameFactory::create_empty_frame();
    }
    return frame;
  }

  //     Long Frame
  //   ------------------
  // 0 |   Start 68h    |
  //   ------------------
  // 1 |    L Field     |
  //   ------------------
  // 2 |    L Field     |
  //   ------------------
  // 3 |   Start 68h    |
  //   ------------------
  // 4 |    C Field     |
  //   ------------------
  // 5 |    A Field     |
  //   ------------------
  // 6 |    CI Field    |
  //   ------------------
  // 7 |    User Data   |
  //   |  (0-252 Byte)  |
  //   ------------------
  // ..|   Check Sum    |
  //   ------------------
  // ..|    Stop 16h    |
  //   ------------------
  if (this->rx_buffer_.at(0) == MBusFrameDefinition::LONG_FRAME.start_bit) {
    // Minimum size: start(1) + L(1) + L(1) + start(1) + C(1) + A(1) + CI(1) + checksum(1) + stop(1) = 9 bytes
    if (this->rx_buffer_.size() < MBusFrameDefinition::LONG_FRAME.base_frame_size) {
      ESP_LOGE(TAG, "parse_response_(): Long frame too short: %zu bytes", this->rx_buffer_.size());
      return MBusFrameFactory::create_empty_frame();
    }

    const uint8_t l_field = this->rx_buffer_.at(1);
    const size_t expected_size = static_cast<size_t>(l_field) + 6;

    // Validate repeated L field, repeated start byte, total size and stop byte
    if (this->rx_buffer_.at(2) != l_field || this->rx_buffer_.at(3) != MBusFrameDefinition::LONG_FRAME.start_bit ||
        this->rx_buffer_.size() != expected_size ||
        this->rx_buffer_.back() != MBusFrameDefinition::LONG_FRAME.stop_bit) {
      ESP_LOGE(TAG, "parse_response_(): Long frame structure invalid: size=%zu expected=%zu L=0x%02X",
               this->rx_buffer_.size(), expected_size, l_field);
      return MBusFrameFactory::create_empty_frame();
    }

    std::vector<uint8_t> data(this->rx_buffer_.begin() + 7, this->rx_buffer_.end() - 2);
    auto frame =
        MBusFrameFactory::create_long_frame(this->rx_buffer_.at(4), this->rx_buffer_.at(5), this->rx_buffer_.at(6),
                                            data, this->rx_buffer_.at(this->rx_buffer_.size() - 2));

    // Validate checksum over C + A + CI + user data
    const uint8_t expected_checksum = MBusFrame::calc_checksum(*frame);
    if (frame->checksum != expected_checksum) {
      ESP_LOGE(TAG, "parse_response_(): Long frame checksum mismatch: got 0x%02X expected 0x%02X", frame->checksum,
               expected_checksum);
      return MBusFrameFactory::create_empty_frame();
    }

    if (frame->control_information == MBusControlInformationCodes::VARIABLE_DATA_RESPONSE_MODE1) {
      frame->variable_data = parse_variable_data_response_(frame->data);
    }

    return frame;
  }

  char hex_buf_err[format_hex_pretty_size(MBUS_FRAME_DATA_LENGTH + 9)];
  ESP_LOGE(TAG, "parse_response_(): ERROR 'invalid frame' %s", format_hex_pretty_to(hex_buf_err, this->rx_buffer_));
  return MBusFrameFactory::create_empty_frame();
}

// variable data response
// ------------------------------------------------------------------------------------
// | Fixed Data Header | Variable Data Blocks (Records) |  MDH   |  Mfg.specific data |
// |     12 Byte       |        variable number         | 1 Byte |    variable number |
// ------------------------------------------------------------------------------------
std::unique_ptr<MBusDataVariable> MBusProtocolHandler::parse_variable_data_response_(std::vector<uint8_t> data) {
  auto response = make_unique<MBusDataVariable>();
  if (data.size() < 12) {
    ESP_LOGE(TAG, "Variable Data Header less than 12 byte: %d", data.size());
    return nullptr;
  }

  // parse header
  // -----------------------------------------------------------------------------
  // | Ident.Nr.  | Manufr. | Version | Medium | Access No. | Status | Signature |
  // | 4 Byte BCD | 2 Byte  | 1 Byte  | 1 Byte | 1 Byte     | 1 Byte | 2 Byte    |
  // |    0..3    |  4..5   |    6    |   7    |     8      |    9   |  10.. 11  |
  // -----------------------------------------------------------------------------
  auto *header = &response->header;
  std::vector<uint8_t> id(data.begin(), data.begin() + 4);
  header->id[0] = data[0];
  header->id[1] = data[1];
  header->id[2] = data[2];
  header->id[3] = data[3];
  header->manufacturer[0] = data[4];
  header->manufacturer[1] = data[5];
  header->version = data[6];
  header->medium = data[7];
  header->access_no = data[8];
  header->status = data[9];
  header->signature[0] = data[10];
  header->signature[1] = data[11];

  // parse records
  // -------------------------------------------------------------------------
  // |   DIF   |        DIFE        |   VIF  |        VIFE        |   Data   |
  // | 1 Byte  | 0-10 (1 Byte each) | 1 Byte | 0-10 (1 Byte each) | 0-N Byte |
  // | Data Information Block DIB   | Value Information Block VIB |
  // |                 Data Record Header DRH                     |
  // --------------------------------------------------------------

  auto it = data.begin() + 12;
  // Reserve space to avoid reallocations during parsing
  response->records.reserve(16);  // Typical M-Bus frames have <16 records

  while (it < data.end()) {
    if ((*it & 0xFF) == MBusDataDifMask::IDLE_FILLER) {
      it++;
      continue;
    }

    // The manufacturer data header (MDH) is made up by the character 0Fh or 1Fh
    // and indicates the beginning of the manufacturer specific part of the user data
    // and should be omitted, if there is no manufacturer specific data
    if ((*it & 0xFF) == MBusDataDifMask::MANUFACTURER_SPECIFIC ||
        (*it & 0xFF) == MBusDataDifMask::MORE_RECORDS_FOLLOW) {
      it++;
      continue;
    }

    MBusDataRecord record;
    bool truncated = false;

    // DIF
    //    Bit 7         6         5          4       3     2      1     0
    // ----------------------------------------------------------------------
    // | Extension | LSB of  |   Function Field  |   Data Field:            |
    // |    Bit    | storage |                   | Lengh and coding of data |
    // |           | Number  |                   |                          |
    // ----------------------------------------------------------------------

    record.drh.dib.dif = *it;
    // Extension Bit of DIF / DIFE Frame set => next Frame is DIFE
    uint8_t dife_count = 0;
    while (it < data.end() && (*it & MBusDataDifMask::EXTENSION_BIT) && dife_count < MBUS_MAX_DIFE_COUNT) {
      ++it;
      if (it >= data.end()) {
        ESP_LOGW(TAG, "Truncated frame: expected DIFE byte %d but frame ended", dife_count + 1);
        truncated = true;
        break;
      }
      record.drh.dib.dife.push_back(*it);
      dife_count++;
    }
    if (truncated) {
      break;
    }
    if (dife_count >= MBUS_MAX_DIFE_COUNT) {
      ESP_LOGW(TAG, "Too many DIFE extensions (>%d), possible malformed frame", MBUS_MAX_DIFE_COUNT);
    }
    ++it;
    if (it >= data.end()) {
      ESP_LOGW(TAG, "Truncated frame: expected VIF byte but frame ended");
      break;
    }

    // VIB
    record.drh.vib.vif = *it;

    // Extension Bit of VIF / VIFE Frame set => next Frame is VIFE
    uint8_t vife_count = 0;
    while (it < data.end() && (*it & MBusDataVifMask::EXTENSION_BIT) && vife_count < MBUS_MAX_VIFE_COUNT) {
      ++it;
      if (it >= data.end()) {
        ESP_LOGW(TAG, "Truncated frame: expected VIFE byte %d but frame ended", vife_count + 1);
        truncated = true;
        break;
      }
      record.drh.vib.vife.push_back(*it);
      vife_count++;
    }
    if (truncated) {
      break;
    }
    if (vife_count >= MBUS_MAX_VIFE_COUNT) {
      ESP_LOGW(TAG, "Too many VIFE extensions (>%d), possible malformed frame", MBUS_MAX_VIFE_COUNT);
    }
    ++it;

    auto data_len = get_dif_datalength_(record.drh.dib.dif, it);
    if (data_len < 0) {
      ESP_LOGW(TAG, "Invalid record data length %d, stopping record parse", data_len);
      break;
    }
    const auto remaining = static_cast<size_t>(data.end() - it);
    if (static_cast<size_t>(data_len) > remaining) {
      ESP_LOGW(TAG, "Truncated frame: record needs %d bytes, only %zu remaining", data_len, remaining);
      break;
    }
    record.data.insert(record.data.begin(), it, it + data_len);
    it += data_len;

    response->records.push_back(record);
  }

  // Release any excess capacity allocated during parsing
  response->records.shrink_to_fit();

  return response;
}

int8_t MBusProtocolHandler::get_dif_datalength_(uint8_t dif, std::vector<uint8_t>::iterator &it) {
  static const uint8_t DIF_DATA_LENGTH_MASK = 0x0F;
  switch (dif & DIF_DATA_LENGTH_MASK) {
    case 0x0:
      return 0;
    case 0x1:
      return 1;
    case 0x2:
      return 2;
    case 0x3:
      return 3;
    case 0x4:
      return 4;
    case 0x5:
      return 4;  // 32-bit IEEE 754 real = 4 bytes (EN 13757-3 Table 5)
    case 0x6:
      return 6;
    case 0x7:
      return 8;
    case 0x8:
      return 0;
    case 0x9:
      return 1;
    case 0xA:
      return 2;
    case 0xB:
      return 3;
    case 0xC:
      return 4;
    case 0xD: {
      // variable data length,
      // data length stored in data field
      uint8_t data_1 = *it;
      it++;

      if (data_1 <= 0xBF) {
        // ASCII string with LVAR characters
        return data_1;
      } else if (data_1 >= 0xC0 && data_1 <= 0xCF) {
        // positive BCD number with (LVAR - C0h) • 2 digits
        return (data_1 - 0xC0) * 2;
      } else if (data_1 >= 0xD0 && data_1 <= 0xDF) {
        // negative BCD number with (LVAR - D0h) • 2 digits
        return (data_1 - 0xD0) * 2;
      } else if (data_1 >= 0xE0 && data_1 <= 0xEF) {
        // binary number with (LVAR - E0h) bytes
        return data_1 - 0xE0;
      } else if (data_1 >= 0xF0 && data_1 <= 0xFA) {
        // floating point number with (LVAR - F0h) bytes [to be defined]
        return data_1 - 0xF0;
      }

      ESP_LOGE(TAG, "get_dif_datalength(): invalid mask = %d", data_1);
      return 0;
    }
    case 0xE:
      return 6;
    case 0xF:
      return 0;  // Special functions = 0 data bytes (EN 13757-3 Table 5)
    default:     // never reached
      ESP_LOGE(TAG, "Invalid value for diff data length = %d", dif & DIF_DATA_LENGTH_MASK);
      return 0x0;
  }
}

}  // namespace mbus
}  // namespace esphome
