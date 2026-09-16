#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace onrobot_gripper_description {

/// Inclusive interval for one coordinate, expressed in that coordinate's unit.
struct Domain {
  /// Lower inclusive bound of the interval.
  double minimum{0.0};
  /// Upper inclusive bound of the interval.
  double maximum{0.0};
};

/// Mapping law used to relate a mechanism coordinate to a joint coordinate.
enum class LawType {
  /// Linear stroke law used by 2FG profiles.
  TwoFingerLinear,
  /// Sine geometry law used by RG profiles.
  RgSine
};

/// Validated, immutable map between mechanism, joint, and task coordinates.
///
/// The mechanism coordinate is a raw linear stroke in metres for 2FG profiles
/// or a measured angle in radians for RG profiles. The joint coordinate uses
/// the same units as the mechanism coordinate; the aperture is in metres.
class KinematicMap {
public:
  /// Construct a map and validate all domains and law coefficients.
  ///
  /// For a 2FG law, `m0` is the mechanism intercept in metres and the RG
  /// geometry parameters are unused and must be zero. For an RG law, `m0` is
  /// unused and must be zero; `length`, `thickness`, and `compensation` are
  /// metres, while `scale` is radians per joint-radian and `theta_reference`
  /// is radians. Offsets are metres.
  ///
  /// @throws ProfileError if a value is non-finite, a domain is empty, or the
  /// map is inconsistent with its law.
  KinematicMap(LawType law, Domain mechanism_domain, Domain joint_domain,
               Domain aperture_domain, double m0, double length,
               double thickness, double compensation, double scale,
               double theta_reference, double left_offset,
               double right_offset);

  /// Convert a mechanism coordinate to the physical joint coordinate.
  /// @throws ProfileError if `mechanism` is non-finite or outside its domain.
  double mechanismToJoint(double mechanism) const;

  /// Convert a physical joint coordinate to the mechanism coordinate.
  /// @throws ProfileError if `joint` is non-finite or outside its domain.
  double jointToMechanism(double joint) const;

  /// Convert a physical joint coordinate to task aperture in metres.
  /// @throws ProfileError if `joint` is non-finite or outside its domain.
  double jointToAperture(double joint) const;

  /// Convert task aperture in metres to the physical joint coordinate.
  /// @throws ProfileError if `aperture` is non-finite or outside its domain.
  double apertureToJoint(double aperture) const;

  /// Return d(aperture)/d(joint) at a valid physical joint coordinate.
  ///
  /// The result is metres per metre for 2FG and metres per radian for RG.
  /// @throws ProfileError if `joint` is outside its domain or the Jacobian is
  /// singular.
  double apertureJacobian(double joint) const;

  /// Return the mapping law.
  LawType law() const noexcept { return law_; }

  /// Return the inclusive mechanism-coordinate domain.
  const Domain &mechanismDomain() const noexcept { return mechanism_domain_; }

  /// Return the inclusive physical-joint-coordinate domain.
  const Domain &jointDomain() const noexcept { return joint_domain_; }

  /// Return the inclusive task-aperture domain in metres.
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

/// A validated schema-v2 profile and its resolved numeric map.
class GripperProfile {
public:
  /// Construct a profile from metadata, a map, and its safe aperture domain.
  /// The safe aperture must be within the map's inclusive aperture domain;
  /// this check does not admit an epsilon outside that domain.
  /// @throws ProfileError if the safe aperture is invalid or outside the map.
  GripperProfile(std::string name, std::string model, int revision,
                 std::string geometry_identity, std::string source,
                 KinematicMap map, Domain safe_aperture);

  /// Load and validate a profile YAML file, including bounded inheritance.
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

  /// Return the deterministic SHA-256 identity of the resolved profile.
  const std::string &profileHash() const noexcept { return profile_hash_; }

  /// Return the resolved mechanism/joint/aperture coordinate map.
  const KinematicMap &map() const noexcept { return map_; }

  /// Return the inclusive safe task-aperture domain in metres.
  const Domain &safeAperture() const noexcept { return safe_aperture_; }

  /// Return the physical joint interval corresponding to the safe aperture.
  Domain safeJoint() const;

  /// Convert a planning target without admitting overlapping fingertip poses.
  /// @throws ProfileError for non-finite or out-of-safe-domain input.
  double planningJointToAperture(double joint) const;

  /// Convert a safe aperture target into its physical planning coordinate.
  /// @throws ProfileError for non-finite or out-of-safe-domain input.
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
