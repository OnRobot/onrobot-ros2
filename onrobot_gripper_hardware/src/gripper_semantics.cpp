#include "onrobot_gripper_hardware/gripper_semantics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace onrobot_gripper_hardware {

GripperKinematics::GripperKinematics(double i_rawMinimumMm,
                                     double i_rawMaximumMm,
                                     double i_jointUpperM)
    : m_rawMinimumMm(i_rawMinimumMm), m_rawMaximumMm(i_rawMaximumMm),
      m_jointUpperM(i_jointUpperM) {
  if (!std::isfinite(m_rawMinimumMm) || !std::isfinite(m_rawMaximumMm) ||
      m_rawMaximumMm <= m_rawMinimumMm) {
    throw std::invalid_argument(
        "raw maximum must be finite and greater than raw minimum");
  }
  if (!std::isfinite(m_jointUpperM) || m_jointUpperM <= 0.0) {
    throw std::invalid_argument(
        "joint upper limit must be finite and positive");
  }
}

double GripperKinematics::rawPositionToJoint(double i_rawPositionMm) const {
  const double ratio =
      (i_rawPositionMm - m_rawMinimumMm) / (m_rawMaximumMm - m_rawMinimumMm);
  return std::clamp(ratio, 0.0, 1.0) * m_jointUpperM;
}

double GripperKinematics::jointPositionToRaw(double i_jointPositionM) const {
  const double ratio = std::clamp(i_jointPositionM / m_jointUpperM, 0.0, 1.0);
  return m_rawMinimumMm + ratio * (m_rawMaximumMm - m_rawMinimumMm);
}

double GripperKinematics::rawVelocityToJoint(double i_rawVelocityMmS) const {
  return i_rawVelocityMmS * m_jointUpperM / (m_rawMaximumMm - m_rawMinimumMm);
}

double GripperKinematics::jointVelocityToRaw(double i_jointVelocityMS) const {
  return i_jointVelocityMS * (m_rawMaximumMm - m_rawMinimumMm) / m_jointUpperM;
}

RgVisualKinematics::RgVisualKinematics(double i_minimumWidthM,
                                       double i_maximumWidthM,
                                       double i_fingerJointUpperRad)
    : m_minimumWidthM(i_minimumWidthM), m_maximumWidthM(i_maximumWidthM),
      m_fingerJointUpperRad(i_fingerJointUpperRad) {
  if (!std::isfinite(m_minimumWidthM) || !std::isfinite(m_maximumWidthM) ||
      m_minimumWidthM < 0.0 || m_maximumWidthM <= m_minimumWidthM) {
    throw std::invalid_argument("RG visual width limits are invalid");
  }
  if (!std::isfinite(m_fingerJointUpperRad) || m_fingerJointUpperRad <= 0.0 ||
      m_fingerJointUpperRad >= 1.5707963267948966) {
    throw std::invalid_argument("RG visual finger angle limit is invalid");
  }
}

double RgVisualKinematics::widthToFingerAngle(double i_widthM) const {
  if (!std::isfinite(i_widthM)) {
    throw std::invalid_argument("RG measured width must be finite");
  }
  const double openRatio = std::clamp((i_widthM - m_minimumWidthM) /
                                          (m_maximumWidthM - m_minimumWidthM),
                                      0.0, 1.0);
  return std::asin(openRatio * std::sin(m_fingerJointUpperRad));
}

double RgVisualKinematics::fingerAngleToWidth(double i_angleRad) const {
  if (!std::isfinite(i_angleRad)) {
    throw std::invalid_argument("RG finger angle must be finite");
  }
  const double angle = std::clamp(i_angleRad, 0.0, m_fingerJointUpperRad);
  const double openRatio = std::sin(angle) / std::sin(m_fingerJointUpperRad);
  return m_minimumWidthM + openRatio * (m_maximumWidthM - m_minimumWidthM);
}

RgCadKinematics::RgCadKinematics(bool i_rg6)
    : m_linkLengthM(i_rg6 ? 0.080 : 0.055),
      m_lateralOffsetM(i_rg6 ? 0.0042 : 0.0025),
      m_gapOffsetM(i_rg6 ? -0.0016 : -0.0039) {}

double RgCadKinematics::cadZeroPhase() const {
  return std::asin(m_lateralOffsetM / m_linkLengthM);
}

double RgCadKinematics::widthToFingerAngle(double i_widthM) const {
  if (!std::isfinite(i_widthM)) {
    throw std::invalid_argument("RG aperture must be finite");
  }
  // Negative compensated feedback can occur during rubber compression. The
  // rigid CAD surfaces show contact, not an impossible mesh interpenetration.
  const double sine =
      (std::max(0.0, i_widthM) - m_gapOffsetM) / (2.0 * m_linkLengthM);
  return cadZeroPhase() + std::asin(std::clamp(sine, -1.0, 1.0));
}

double RgCadKinematics::fingerAngleToWidth(double i_angleRad) const {
  if (!std::isfinite(i_angleRad)) {
    throw std::invalid_argument("RG CAD angle must be finite");
  }
  return 2.0 * m_linkLengthM * std::sin(i_angleRad - cadZeroPhase()) +
         m_gapOffsetM;
}

bool clearUnavailableJointStateField(std::vector<double> &io_values) {
  if (io_values.empty() ||
      std::all_of(io_values.begin(), io_values.end(),
                  [](double i_value) { return std::isfinite(i_value); })) {
    return false;
  }
  io_values.clear();
  return true;
}

} // namespace onrobot_gripper_hardware
