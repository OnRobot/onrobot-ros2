#pragma once

// ROS integration-test peer. This file is test-only, is not installed or
// bundled, and depends on POSIX/C++ rather than private Tool API headers.
// It uses a PTY so libmodbus exercises the production RTU framing, CRC,
// address and timeout path rather than a transport mock.

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#ifdef B115200
#  undef B115200
#endif
#ifdef B1000000
#  undef B1000000
#endif

class ModbusRtuTestServer {
public:
  explicit ModbusRtuTestServer(uint16_t product_code = 0x00c0) {
    termios terminal{};
    ::cfmakeraw(&terminal);
    terminal.c_cflag |= CLOCAL | CREAD;
    char slave_name[128]{};
    if (::openpty(&master_, &slave_, slave_name, &terminal, nullptr) != 0) {
      throw std::system_error(errno, std::generic_category(),
                              "RTU test PTY creation failed");
    }
    device_ = slave_name;
    ::close(slave_);
    slave_ = -1;
    const int flags = ::fcntl(master_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(master_, F_SETFL, flags | O_NONBLOCK) != 0) {
      const auto error = std::system_error(
          errno, std::generic_category(), "RTU test PTY setup failed");
      ::close(master_);
      master_ = -1;
      throw error;
    }
    seed(product_code);
    worker_ = std::thread([this] { serve(); });
  }

  ~ModbusRtuTestServer() {
    shutdown_.store(true);
    const int master = master_;
    master_ = -1;
    if (master >= 0) ::close(master);
    if (worker_.joinable()) worker_.join();
  }

  ModbusRtuTestServer(const ModbusRtuTestServer &) = delete;
  ModbusRtuTestServer &operator=(const ModbusRtuTestServer &) = delete;

  const std::string &device() const noexcept { return device_; }

  void set_register(uint16_t address, uint16_t value) {
    std::lock_guard<std::mutex> lock(register_mutex_);
    registers_[address] = value;
  }

  uint16_t register_value(uint16_t address) const {
    std::lock_guard<std::mutex> lock(register_mutex_);
    const auto found = registers_.find(address);
    return found == registers_.end() ? 0 : found->second;
  }

  uint32_t function_count(uint8_t function) const {
    std::lock_guard<std::mutex> lock(request_mutex_);
    return function_counts_.at(function);
  }

private:
  static uint16_t read_word(const std::vector<uint8_t> &bytes, std::size_t offset) {
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(bytes.at(offset)) << 8U) |
        bytes.at(offset + 1));
  }

  static void append_word(std::vector<uint8_t> &bytes, uint16_t value) {
    bytes.push_back(static_cast<uint8_t>(value >> 8U));
    bytes.push_back(static_cast<uint8_t>(value));
  }

  static uint16_t crc16(const std::vector<uint8_t> &bytes) {
    uint16_t crc = 0xffffU;
    for (const auto byte : bytes) {
      crc ^= byte;
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 1U) != 0U
                  ? static_cast<uint16_t>((crc >> 1U) ^ 0xa001U)
                  : static_cast<uint16_t>(crc >> 1U);
      }
    }
    return crc;
  }

  void seed(uint16_t product_code) {
    registers_[0x0600] = product_code;
    if (product_code == 0x00c1) {
      registers_[0x0604] = 0x0101;
      registers_[0x0605] = 21;
      const std::array<uint16_t, 10> hash{
          0x0e03, 0x7259, 0x7cc6, 0x6ef5, 0xf5ec,
          0x4174, 0xa143, 0x73e2, 0x3215, 0xd071};
      for (std::size_t index = 0; index < hash.size(); ++index) {
        registers_[0x0220 + static_cast<uint16_t>(index)] = hash[index];
      }
    } else {
      registers_[0x0604] = 0x0100;
      registers_[0x0605] = 33;
      const std::array<uint16_t, 10> hash{
          0xb422, 0x2b25, 0x1054, 0xdec1, 0x1291,
          0x3425, 0xa623, 0x86d2, 0x8939, 0xc375};
      for (std::size_t index = 0; index < hash.size(); ++index) {
        registers_[0x0220 + static_cast<uint16_t>(index)] = hash[index];
      }
    }
    registers_[0x0103] = 0;
    registers_[0x0104] = 730;
    registers_[0x0105] = 0;
    registers_[0x0106] = 730;
    registers_[0x0404] = 14;
    registers_[0x0405] = 100;
    registers_[0x0407] = product_code == 0x00c1 ? 196 : 95;
    registers_[0x0101] = 200;
    registers_[0x0102] = 100;
    registers_[0x0107] = static_cast<uint16_t>(static_cast<int16_t>(-40));
  }

  bool read_byte(uint8_t &byte) {
    while (!shutdown_.load()) {
      pollfd descriptor{master_, POLLIN | POLLERR | POLLHUP, 0};
      const int result = ::poll(&descriptor, 1, 50);
      if (result < 0) {
        if (errno == EINTR) continue;
        return false;
      }
      if (result == 0) continue;
      if ((descriptor.revents & (POLLERR | POLLNVAL)) != 0) return false;
      const ssize_t count = ::read(master_, &byte, 1);
      if (count == 1) return true;
      if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EIO)) {
        if (errno == EIO) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      return false;
    }
    return false;
  }

  bool read_bytes(std::vector<uint8_t> &bytes, std::size_t count) {
    const auto size = bytes.size();
    bytes.resize(size + count);
    for (std::size_t index = 0; index < count; ++index) {
      if (!read_byte(bytes[size + index])) {
        bytes.resize(size);
        return false;
      }
    }
    return true;
  }

  bool read_request(std::vector<uint8_t> &request) {
    request.clear();
    if (!read_bytes(request, 2)) return false;
    std::size_t total = 8;
    switch (request.at(1)) {
      case 3:
      case 6:
        break;
      case 16:
        if (!read_bytes(request, 5)) return false;
        total = 9U + request.at(6);
        break;
      case 23:
        if (!read_bytes(request, 9)) return false;
        total = 13U + request.at(10);
        break;
      default:
        return false;
    }
    return total >= request.size() && total <= 262U &&
           read_bytes(request, total - request.size());
  }

  uint16_t value(uint16_t address) const { return register_value(address); }

  std::vector<uint8_t> response(const std::vector<uint8_t> &request) {
    const uint8_t function = request.at(1);
    std::vector<uint8_t> pdu;
    if (function == 3) {
      const auto address = read_word(request, 2);
      const auto count = read_word(request, 4);
      pdu = {3, static_cast<uint8_t>(count * 2U)};
      for (uint16_t index = 0; index < count; ++index) append_word(pdu, value(address + index));
    } else if (function == 6) {
      pdu.insert(pdu.end(), request.begin() + 1, request.begin() + 6);
      set_register(read_word(request, 2), read_word(request, 4));
    } else if (function == 16) {
      const auto address = read_word(request, 2);
      const auto count = read_word(request, 4);
      for (uint16_t index = 0; index < count; ++index) {
        set_register(address + index, read_word(request, 7U + index * 2U));
      }
      pdu = {16};
      append_word(pdu, address);
      append_word(pdu, count);
    } else if (function == 23) {
      const auto read_address = read_word(request, 2);
      const auto read_count = read_word(request, 4);
      const auto write_address = read_word(request, 6);
      const auto write_count = read_word(request, 8);
      for (uint16_t index = 0; index < write_count; ++index) {
        set_register(write_address + index, read_word(request, 11U + index * 2U));
      }
      pdu = {23, static_cast<uint8_t>(read_count * 2U)};
      for (uint16_t index = 0; index < read_count; ++index) {
        append_word(pdu, value(read_address + index));
      }
    }
    std::vector<uint8_t> result{request.at(0)};
    result.insert(result.end(), pdu.begin(), pdu.end());
    const auto crc = crc16(result);
    result.push_back(static_cast<uint8_t>(crc));
    result.push_back(static_cast<uint8_t>(crc >> 8U));
    return result;
  }

  bool write_response(const std::vector<uint8_t> &bytes) {
    std::size_t written = 0;
    while (written < bytes.size() && !shutdown_.load()) {
      const auto count = ::write(master_, bytes.data() + written, bytes.size() - written);
      if (count > 0) {
        written += static_cast<std::size_t>(count);
      } else if (count < 0 && (errno == EINTR || errno == EAGAIN)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        return false;
      }
    }
    return written == bytes.size();
  }

  void serve() {
    while (!shutdown_.load()) {
      std::vector<uint8_t> request;
      if (!read_request(request)) return;
      {
        std::lock_guard<std::mutex> lock(request_mutex_);
        ++function_counts_.at(request.at(1));
      }
      if (!write_response(response(request))) return;
    }
  }

  int master_{-1};
  int slave_{-1};
  std::string device_;
  std::atomic_bool shutdown_{false};
  std::thread worker_;
  std::map<uint16_t, uint16_t> registers_;
  mutable std::mutex register_mutex_;
  std::array<uint32_t, 256> function_counts_{};
  mutable std::mutex request_mutex_;
};
