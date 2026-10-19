#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace onrobot_gripper_description {

/// Allowed range for a coordinate, including both endpoints, in its own unit.
struct Domain {
  /// Minimum allowed value.
  double minimum{0.0};
  /// Maximum allowed value.
  double maximum{0.0};
};

/// Formula used to convert mechanism position to joint position.
enum class LawType {
  /// Linear stroke formula used by 2FG profiles.
  TwoFingerLinear,
  /// Sine formula used by RG profiles.
  RgSine
};

/// Validated conversion between mechanism position, joint position and aperture.
///
/// The mechanism coordinate is a raw linear stroke in metres for 2FG profiles
/// or a measured angle in radians for RG profiles. Joint position uses the
/// same units as mechanism position; aperture is in metres. The conversion
/// settings cannot be changed after construction.
class KinematicMap {
public:
  /// Construct a coordinate conversion and validate its ranges and coefficients.
  ///
  /// For the 2FG formula, `m0` is the mechanism intercept in metres and the RG
  /// geometry parameters are unused and must be zero. For the RG formula, `m0` is
  /// unused and must be zero; `length`, `thickness`, and `compensation` are
  /// metres, while `scale` is radians per joint-radian and `theta_reference`
  /// is radians. Offsets are metres.
  ///
  /// @throws ProfileError if a value is NaN or infinity, a range is empty, or
  /// the settings do not match the selected formula.
  KinematicMap(LawType law, Domain mechanism_domain, Domain joint_domain,
               Domain aperture_domain, double m0, double length,
               double thickness, double compensation, double scale,
               double theta_reference, double left_offset,
               double right_offset);

  /// Convert a mechanism coordinate to the physical joint coordinate.
  /// @throws ProfileError if `mechanism` is NaN, infinity or outside its allowed range.
  double mechanismToJoint(double mechanism) const;

  /// Convert a physical joint coordinate to the mechanism coordinate.
  /// @throws ProfileError if `joint` is NaN, infinity or outside its allowed range.
  double jointToMechanism(double joint) const;

  /// Convert a physical joint coordinate to task aperture in metres.
  /// @throws ProfileError if `joint` is NaN, infinity or outside its allowed range.
  double jointToAperture(double joint) const;

  /// Convert task aperture in metres to the physical joint coordinate.
  /// @throws ProfileError if `aperture` is NaN, infinity or outside its allowed range.
  double apertureToJoint(double aperture) const;

  /// Return d(aperture)/d(joint) at a valid physical joint coordinate.
  ///
  /// The result is metres per metre for 2FG and metres per radian for RG.
  /// @throws ProfileError if `joint` is outside its allowed range or the Jacobian is
  /// singular.
  double apertureJacobian(double joint) const;

  /// Return the selected conversion formula.
  LawType law() const noexcept { return law_; }

  /// Return the allowed mechanism range, including both endpoints.
  const Domain &mechanismDomain() const noexcept { return mechanism_domain_; }

  /// Return the allowed physical joint range, including both endpoints.
  const Domain &jointDomain() const noexcept { return joint_domain_; }

  /// Return the allowed aperture range in metres, including both endpoints.
  const Domain &apertureDomain() const noexcept { return aperture_domain_; }

  /// Return the 2FG mechanism intercept `m0`, in metres.
  double m0() const noexcept { return m0_; }

  /// Return the RG link length, in metres.
  double length() const noexcept { return length_; }

  /// Return the RG mechanism thickness, in metres.
  double thickness() const noexcept { return thickness_; }

  /// Return the RG aperture compensation, in metres.
  double compensation() const noexcept { return compensation_; }

  /// Return the RG mechanism-to-angle scale, in radians per joint radian.
  double scale() const noexcept { return scale_; }

  /// Return the RG reference angle, in radians.
  double thetaReference() const noexcept { return theta_reference_; }

  /// Return the left contact offset, in metres.
  double leftOffset() const noexcept { return left_offset_; }

  /// Return the right contact offset, in metres.
  double rightOffset() const noexcept { return right_offset_; }

private:
  void check(double value, const char *name) const;
  LawType law_;
  Domain mechanism_domain_;
  Domain joint_domain_;
  Domain aperture_domain_;
  double m0_;
  double length_;
  double thickness_;
  double compensation_;
  double scale_;
  double theta_reference_;
  double left_offset_;
  double right_offset_;
};

/// Error raised when a profile or coordinate map fails validation.
class ProfileError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// A validated schema-v2 profile with its calculated coordinate conversion.
class GripperProfile {
public:
  /// Construct a profile from metadata, a coordinate map and a safe aperture range.
  /// The safe range must fit within the map's aperture range, including its
  /// endpoints. Even a small amount outside that range is rejected.
  /// @throws ProfileError if the safe aperture is invalid or outside the map.
  GripperProfile(std::string name, std::string model, int revision,
                 std::string geometry_identity, std::string source,
                 KinematicMap map, Domain safe_aperture);

  /// Load and validate a profile YAML file, limiting the depth of inherited profiles.
  /// @throws ProfileError if the file cannot be read or fails schema checks.
  static GripperProfile load(const std::string &path);

  /// Parse and validate profile YAML held in memory.
  /// @param yaml YAML document containing a schema-v2 profile.
  /// @param source Label used in validation errors; defaults to `<memory>`.
  /// @throws ProfileError if the document fails schema checks.
  static GripperProfile loadText(const std::string &yaml,
                                 const std::string &source = "<memory>");

  /// Return the profile name from the source document.
  const std::string &name() const noexcept { return name_; }

  /// Return the product model identifier from the source document.
  const std::string &model() const noexcept { return model_; }

  /// Return the positive profile revision from the source document.
  int revision() const noexcept { return revision_; }

  /// Return the geometry identity associated with the profile.
  const std::string &geometryIdentity() const noexcept {
    return geometry_identity_;
  }

  /// Return the source label supplied when the profile was loaded.
  const std::string &source() const noexcept { return source_; }

  /// Return the SHA-256 hash of the resolved profile. The same data gives the same hash.
  const std::string &profileHash() const noexcept { return profile_hash_; }

  /// Return the calculated conversion between mechanism, joint and aperture coordinates.
  const KinematicMap &map() const noexcept { return map_; }

  /// Return the safe aperture range in metres, including both endpoints.
  const Domain &safeAperture() const noexcept { return safe_aperture_; }

  /// Return the physical joint range for the safe aperture range.
  Domain safeJoint() const;

  /// Convert a planning target, rejecting positions where fingertips would overlap.
  /// @throws ProfileError for NaN, infinity or a value outside the safe range.
  double planningJointToAperture(double joint) const;

  /// Convert a safe aperture target into its physical planning coordinate.
  /// @throws ProfileError for NaN, infinity or a value outside the safe range.
  double apertureToPlanningJoint(double aperture) const;

  /// Serialize resolved data as stable YAML with a fixed key order and hash.
  std::string resolvedYaml() const;

private:
  std::string name_;
  std::string model_;
  int revision_{0};
  std::string geometry_identity_;
  std::string source_;
  std::string profile_hash_;
  KinematicMap map_;
  Domain safe_aperture_;
};

} // namespace onrobot_gripper_description
