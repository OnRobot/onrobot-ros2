#pragma once

#include <vector>

namespace onrobot_gripper_hardware {

/// @brief Product geometry needed to translate device and ROS coordinates.
///
/// The firmware reports a linear mechanism coordinate in millimetres, while
/// the URDF joint describes one finger's travel in metres. Keeping that
/// mapping in a small ROS-independent type lets physical, fake, and simulation
/// backends use the same coordinate contract.
class GripperKinematics {
public:
  GripperKinematics(double i_rawMinimumMm, double i_rawMaximumMm,
                    double i_jointUpperM);

  /// @brief Convert a device opening to the clamped URDF joint coordinate.
  double rawPositionToJoint(double i_rawPositionMm) const;
  /// @brief Convert a URDF joint position to the clamped device coordinate.
  double jointPositionToRaw(double i_jointPositionM) const;
  /// @brief Convert device opening velocity from mm/s to joint m/s.
  double rawVelocityToJoint(double i_rawVelocityMmS) const;
  /// @brief Convert joint velocity from m/s to device opening mm/s.
  double jointVelocityToRaw(double i_jointVelocityMS) const;

private:
  double m_rawMinimumMm;
  double m_rawMaximumMm;
  double m_jointUpperM;
};

/// @brief Map normalized RG task travel to the Isaac bridge's joint span.
///
/// This simulator mapping uses the configured task and asset endpoints. The
/// supplied physical CAD surfaces use RgCadKinematics instead.
class RgVisualKinematics {
public:
  RgVisualKinematics(double i_minimumWidthM, double i_maximumWidthM,
                     double i_fingerJointUpperRad);

  /// @brief Convert measured aperture in metres to the URDF finger angle.
  double widthToFingerAngle(double i_widthM) const;
  /// @brief Convert the URDF finger angle back to external aperture in metres.
  double fingerAngleToWidth(double i_angleRad) const;

private:
  double m_minimumWidthM;
  double m_maximumWidthM;
  double m_fingerJointUpperRad;
};

/// @brief Exact gap mapping for the supplied inward-facing standard RG CAD tips.
///
/// CAD zero is not fingertip contact. This mapping includes the lateral link
/// offset and the rubber surfaces. Firmware mechanism angle remains a separate
/// coordinate; custom fingers require their own aperture geometry.
class RgCadKinematics {
public:
  /// @param i_rg6 True for RG6 geometry; false for RG2 geometry.
  explicit RgCadKinematics(bool i_rg6);
  /// Convert measured gap in metres to CAD radians; compression shows contact.
  double widthToFingerAngle(double i_widthM) const;
  /// Return signed rubber-surface gap in metres at a CAD joint angle in radians.
  double fingerAngleToWidth(double i_angleRad) const;
  /// Offset in radians added to firmware angle to enter the CAD joint frame.
  double cadZeroPhase() const;

private:
  double m_linkLengthM;
  double m_lateralOffsetM;
  double m_gapOffsetM;
};

/// @brief Remove a JointState field that contains a non-finite placeholder.
///
/// JointState arrays cannot express per-joint validity. An empty array is the
/// truthful representation when any member of that field is unavailable.
/// @return true when the array was cleared.
bool clearUnavailableJointStateField(std::vector<double> &io_values);

} // namespace onrobot_gripper_hardware
