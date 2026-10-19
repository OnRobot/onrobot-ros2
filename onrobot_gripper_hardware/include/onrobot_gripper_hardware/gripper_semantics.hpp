#pragma once

#include <vector>

namespace onrobot_gripper_hardware {

/// @brief Product geometry needed to translate device and ROS coordinates.
///
/// The device reports linear mechanism position in millimetres. The URDF joint
/// describes one finger's travel in metres. This type converts between them
/// without depending on ROS, so real, fake and simulation backends can use the
/// same conversion.
class GripperKinematics {
public:
  /// @brief Configure the conversion from raw stroke to one finger joint.
  ///
  /// @param i_rawMinimumMm Raw mechanism coordinate at minimum travel, in mm.
  /// @param i_rawMaximumMm Raw mechanism coordinate at maximum travel, in mm.
  /// @param i_jointUpperM Maximum travel of one URDF finger joint, in metres.
  /// @throws std::invalid_argument If a value is not finite, raw limits are not
  /// increasing, or joint travel is not greater than zero.
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

/// @brief Convert RG aperture to the Isaac bridge's joint angle range.
///
/// This simulator mapping uses the configured task and asset endpoints. The
/// supplied CAD geometry uses RgCadKinematics instead.
class RgVisualKinematics {
public:
  /// @brief Configure the aperture-to-angle mapping used by the Isaac bridge.
  ///
  /// @param i_minimumWidthM Minimum task aperture, in metres; must be
  /// nonnegative.
  /// @param i_maximumWidthM Maximum task aperture, in metres; must exceed the
  /// minimum.
  /// @param i_fingerJointUpperRad Finger angle limit, in radians; must be
  /// greater than zero and less than pi/2.
  /// @throws std::invalid_argument If a value is not finite or a limit is
  /// invalid.
  /// @see RgCadKinematics for the supplied physical RG CAD surfaces.
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

/// @brief Convert between fingertip gap and joint angle for the standard RG CAD tips.
///
/// The supplied tips face inward. A CAD joint angle of zero does not mean the
/// fingertips touch. The calculation accounts for the sideways link offset and
/// rubber surfaces. Device mechanism angle is a separate coordinate; custom
/// fingers need a calculation based on their own geometry.
class RgCadKinematics {
public:
  /// @param i_rg6 True for RG6 geometry; false for RG2 geometry.
  explicit RgCadKinematics(bool i_rg6);
  /// Convert gap in metres to CAD joint angle in radians. A negative gap is
  /// displayed as fingertip contact.
  double widthToFingerAngle(double i_widthM) const;
  /// Return signed rubber-surface gap in metres at a CAD joint angle in radians.
  double fingerAngleToWidth(double i_angleRad) const;
  /// Offset added to the device angle to obtain the CAD joint angle, in radians.
  double cadZeroPhase() const;

private:
  double m_linkLengthM;
  double m_lateralOffsetM;
  double m_gapOffsetM;
};

/// @brief Remove a JointState field that contains a non-finite placeholder.
///
/// A JointState array cannot mark individual joint values as unavailable.
/// Clear the array if any value is NaN or infinity.
/// @return true when the array was cleared.
bool clearUnavailableJointStateField(std::vector<double> &io_values);

} // namespace onrobot_gripper_hardware
