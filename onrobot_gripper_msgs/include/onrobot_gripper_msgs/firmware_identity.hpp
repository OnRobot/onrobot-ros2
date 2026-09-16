#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>

#include "diagnostic_interfaces.hpp"

namespace onrobot_gripper_msgs {

/// Bounded, allocation-free firmware text decoded from numeric state interfaces.
struct FirmwareIdentityText {
  /// Version and optional source hash; only the first size bytes are text.
  std::array<char, 64> data{};
  /// Text length, or zero when the identity is unavailable or malformed.
  std::size_t size{0};
};

/// Format observed identity; reject invalid flags, ranges and source-hash words.
inline FirmwareIdentityText firmwareIdentityText(
    const std::array<double, kDiagnosticInterfaceCount> &values) noexcept {
  FirmwareIdentityText result;
  const auto number = [&](DiagnosticInterface index, uint32_t maximum,
                          uint32_t &output) {
    const auto value = values[diagnosticIndex(index)];
    if (!std::isfinite(value) || value < 0.0 || value > maximum ||
        std::floor(value) != value) {
      return false;
    }
    output = static_cast<uint32_t>(value);
    return true;
  };
  if (values[diagnosticIndex(DiagnosticInterface::IdentityValid)] != 1.0) {
    return result;
  }
  uint32_t major, minor, build;
  if (!number(DiagnosticInterface::FirmwareMajor, 255, major) ||
      !number(DiagnosticInterface::FirmwareMinor, 255, minor) ||
      !number(DiagnosticInterface::FirmwareBuild, 65535, build)) {
    return result;
  }
  const auto append = [&](uint32_t value) {
    const auto encoded = std::to_chars(result.data.data() + result.size,
                                       result.data.data() + result.data.size(), value);
    result.size = static_cast<std::size_t>(encoded.ptr - result.data.data());
  };
  // 255.255.65535+g followed by 40 hex digits fits in the fixed buffer.
  append(major);
  result.data[result.size++] = '.';
  append(minor);
  result.data[result.size++] = '.';
  append(build);
  const auto sourceValid = values[diagnosticIndex(DiagnosticInterface::FirmwareSourceValid)];
  if (sourceValid == 0.0) {
    return result;
  }
  if (sourceValid != 1.0) {
    return {};
  }
  result.data[result.size++] = '+';
  result.data[result.size++] = 'g';
  constexpr char hex[] = "0123456789abcdef";
  for (std::size_t index = 0; index < 5; ++index) {
    uint32_t word;
    const auto field = static_cast<DiagnosticInterface>(
        diagnosticIndex(DiagnosticInterface::FirmwareGitWord0) + index);
    if (!number(field, UINT32_MAX, word)) {
      return {};
    }
    for (int shift = 28; shift >= 0; shift -= 4) {
      result.data[result.size++] = hex[(word >> shift) & 0xf];
    }
  }
  return result;
}

} // namespace onrobot_gripper_msgs
