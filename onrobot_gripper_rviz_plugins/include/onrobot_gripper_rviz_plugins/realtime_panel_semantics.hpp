#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace onrobot_gripper_rviz_plugins {

/// @brief Convert a centered joystick axis into a safe realtime velocity.
///
/// The UI uses -1 for full left/open and +1 for full right/close.  The
/// device's positive raw velocity is the opening direction, so the sign is
/// inverted at this boundary.  A deadband keeps a centered joystick still.
class RealtimeJoystickMapper {
public:
  static constexpr double kDefaultDeadband = 0.05;
  static constexpr double kDefaultMaximumVelocityMps = 0.03;

  static int pointerToSliderValue(int i_pointerPosition, int i_span,
                                  int i_minimum = -100, int i_maximum = 100,
                                  bool i_inverted = false) {
    if (i_span <= 0 || i_maximum < i_minimum) {
      return i_minimum;
    }
    const int position = std::clamp(i_pointerPosition, 0, i_span);
    const double fraction = static_cast<double>(position) / i_span;
    const double directedFraction = i_inverted ? 1.0 - fraction : fraction;
    return static_cast<int>(
        std::lround(i_minimum + directedFraction * (i_maximum - i_minimum)));
  }

  static double
  axisToVelocity(double i_axis,
                 double i_maximumVelocityMps = kDefaultMaximumVelocityMps,
                 double i_deadband = kDefaultDeadband) {
    if (!std::isfinite(i_axis) || !std::isfinite(i_maximumVelocityMps) ||
        !std::isfinite(i_deadband) || i_maximumVelocityMps < 0.0 ||
        i_deadband < 0.0 || i_deadband >= 1.0) {
      return 0.0;
    }
    const double axis = std::clamp(i_axis, -1.0, 1.0);
    const double magnitude = std::abs(axis);
    if (magnitude <= i_deadband) {
      return 0.0;
    }
    const double scaled = (magnitude - i_deadband) / (1.0 - i_deadband);
    // Left/negative axis opens (positive device velocity); right/positive
    // axis closes (negative device velocity).
    return (axis < 0.0 ? 1.0 : -1.0) * scaled * i_maximumVelocityMps;
  }
};

/// @brief Detect measured task motion that contradicts a held velocity command.
class RealtimeDirectionGuard {
public:
  explicit RealtimeDirectionGuard(double i_tolerance = 0.001)
      : tolerance_(i_tolerance) {}

  void reset() {
    direction_ = 0;
    extremum_ = 0.0;
    has_extremum_ = false;
    commanded_direction_observed_ = false;
  }

  bool observe(double i_commandedVelocity, double i_taskPosition) {
    if (!std::isfinite(i_commandedVelocity) || !std::isfinite(i_taskPosition) ||
        !std::isfinite(tolerance_) || tolerance_ < 0.0) {
      reset();
      return false;
    }
    const int direction =
        (i_commandedVelocity > 0.0) - (i_commandedVelocity < 0.0);
    if (direction == 0) {
      reset();
      return false;
    }
    if (!has_extremum_ || direction != direction_) {
      direction_ = direction;
      extremum_ = i_taskPosition;
      has_extremum_ = true;
      commanded_direction_observed_ = false;
      return false;
    }
    if (direction > 0) {
      if (!commanded_direction_observed_) {
        if (i_taskPosition > extremum_ + tolerance_) {
          commanded_direction_observed_ = true;
          extremum_ = i_taskPosition;
        } else {
          extremum_ = std::min(extremum_, i_taskPosition);
        }
        return false;
      }
      extremum_ = std::max(extremum_, i_taskPosition);
      return i_taskPosition < extremum_ - tolerance_;
    }
    if (!commanded_direction_observed_) {
      if (i_taskPosition < extremum_ - tolerance_) {
        commanded_direction_observed_ = true;
        extremum_ = i_taskPosition;
      } else {
        extremum_ = std::max(extremum_, i_taskPosition);
      }
      return false;
    }
    extremum_ = std::min(extremum_, i_taskPosition);
    return i_taskPosition > extremum_ + tolerance_;
  }

private:
  double tolerance_;
  int direction_{0};
  double extremum_{0.0};
  bool has_extremum_{false};
  bool commanded_direction_observed_{false};
};

/// @brief Require stable, finite task feedback before declaring a target met.
class RealtimePositionTargetTracker {
public:
  explicit RealtimePositionTargetTracker(double i_toleranceM = 0.0005,
                                         std::size_t i_requiredSamples = 2)
      : tolerance_m_(i_toleranceM), required_samples_(i_requiredSamples) {}

  void reset(double i_targetM) {
    target_m_ = i_targetM;
    matching_samples_ = 0;
  }

  bool observe(bool i_positionValid, double i_positionM) {
    if (!i_positionValid || !std::isfinite(i_positionM) ||
        !std::isfinite(target_m_) || !std::isfinite(tolerance_m_) ||
        tolerance_m_ < 0.0 || required_samples_ == 0) {
      matching_samples_ = 0;
      return false;
    }
    if (std::abs(i_positionM - target_m_) > tolerance_m_) {
      matching_samples_ = 0;
      return false;
    }
    ++matching_samples_;
    return matching_samples_ >= required_samples_;
  }

  double target() const { return target_m_; }

private:
  double tolerance_m_;
  std::size_t required_samples_;
  double target_m_{0.0};
  std::size_t matching_samples_{0};
};

} // namespace onrobot_gripper_rviz_plugins
