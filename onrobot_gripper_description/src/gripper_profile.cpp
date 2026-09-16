#include "onrobot_gripper_description/gripper_profile.hpp"

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace onrobot_gripper_description {
namespace {

constexpr double kPiOverTwo = 1.57079632679489661923;
constexpr double kEpsilon = 1e-12;

[[noreturn]] void fail(const std::string &message) { throw ProfileError(message); }

void requireMap(const YAML::Node &node, const std::string &where,
                std::initializer_list<const char *> allowed) {
  if (!node || !node.IsMap()) fail(where + " must be a map");
  std::unordered_set<std::string> names;
  for (const auto *key : allowed) names.emplace(key);
  std::unordered_set<std::string> seen;
  for (const auto &entry : node) {
    if (!entry.first.IsScalar()) fail(where + " contains a non-scalar key");
    const auto key = entry.first.as<std::string>();
    if (!seen.emplace(key).second)
      fail("duplicate field '" + where + "." + key + "'");
    if (!names.count(key)) fail("unknown field '" + where + "." + key + "'");
  }
}

void requireKeys(const YAML::Node &node, const std::string &where,
                 std::initializer_list<const char *> required) {
  for (const auto *key : required) {
    if (!node[key]) fail("missing field '" + where + "." + key + "'");
  }
}

template <typename T>
T value(const YAML::Node &node, const std::string &name) {
  try {
    return node[name].as<T>();
  } catch (const YAML::Exception &e) {
    fail("invalid field '" + name + "': " + e.what());
  }
}

double finiteValue(const YAML::Node &node, const std::string &name) {
  const auto result = value<double>(node, name);
  if (!std::isfinite(result)) fail(name + " must be finite");
  return result;
}

Domain domain(const YAML::Node &node, const std::string &where) {
  // Coordinate maps are validated by unit(); domain() also accepts their
  // dimension/unit keys. Safe-aperture maps contain only the two bounds.
  requireMap(node, where, {"minimum", "maximum", "dimension", "unit"});
  requireKeys(node, where, {"minimum", "maximum"});
  Domain result{finiteValue(node, "minimum"), finiteValue(node, "maximum")};
  if (!(result.minimum < result.maximum)) fail(where + " must have a range");
  return result;
}

double boundedResult(double result, const Domain &target, const char *name) {
  if (!std::isfinite(result))
    fail(std::string(name) + " conversion produced a non-finite result");
  return std::clamp(result, target.minimum, target.maximum);
}

std::string unit(const YAML::Node &node, const std::string &where,
                 const char *expected_dimension, const char *expected_unit) {
  requireMap(node, where, {"dimension", "unit", "minimum", "maximum"});
  requireKeys(node, where, {"dimension", "unit", "minimum", "maximum"});
  if (value<std::string>(node, "dimension") != expected_dimension ||
      value<std::string>(node, "unit") != expected_unit) {
    fail(where + " has an incompatible dimension or unit");
  }
  return value<std::string>(node, "unit");
}

void setScalar(YAML::Node &node, const char *key, const YAML::Node &value_node) {
  node[key] = value_node;
}

void mergeCustom(YAML::Node &base, const YAML::Node &custom,
                 const std::string &source) {
  requireMap(custom, source,
             {"schema_version", "name", "revision",
              "base_profile", "contact_offsets",
              "safe_aperture", "geometry_identity"});
  requireKeys(custom, source, {"schema_version", "name", "revision",
                               "base_profile"});
  // The base artifact is resolved, so recover its guarded physical q interval
  // before deleting derived fields.  Contact offsets are then applied to the
  // same q interval; changing fingers must not silently change mechanism q.
  const auto base_profile = GripperProfile::loadText(YAML::Dump(base), source);
  const double q0 = base_profile.map().apertureToJoint(base_profile.safeAperture().minimum);
  const double q1 = base_profile.map().apertureToJoint(base_profile.safeAperture().maximum);
  double left = base_profile.map().leftOffset();
  double right = base_profile.map().rightOffset();
  if (custom["contact_offsets"]) {
    requireMap(custom["contact_offsets"], source + ".contact_offsets", {"left", "right"});
    requireKeys(custom["contact_offsets"], source + ".contact_offsets", {"left", "right"});
    left = finiteValue(custom["contact_offsets"], "left");
    right = finiteValue(custom["contact_offsets"], "right");
  }
  const auto aperture_at_q = [&](double q) {
    if (base_profile.map().law() == LawType::TwoFingerLinear)
      return base_profile.map().m0() + 2.0 * q + left + right;
    const double theta = base_profile.map().scale() * q + base_profile.map().thetaReference();
    return 2.0 * (base_profile.map().length() * std::sin(theta) -
                   base_profile.map().thickness() + base_profile.map().compensation()) -
           left - right;
  };
  Domain inherited_safe{std::min(aperture_at_q(q0), aperture_at_q(q1)),
                        std::max(aperture_at_q(q0), aperture_at_q(q1))};
  if (custom["safe_aperture"]) {
    requireMap(custom["safe_aperture"], source + ".safe_aperture",
               {"minimum", "maximum"});
    const auto requested = domain(custom["safe_aperture"], source + ".safe_aperture");
    inherited_safe.minimum = std::max(inherited_safe.minimum, requested.minimum);
    inherited_safe.maximum = std::min(inherited_safe.maximum, requested.maximum);
  }
  if (!(inherited_safe.minimum < inherited_safe.maximum))
    fail(source + " custom profile has no safe aperture after base guard");
  base["task_coordinate"] = YAML::Node(YAML::NodeType::Map);
  base["task_coordinate"]["dimension"] = "linear";
  base["task_coordinate"]["unit"] = "m";
  base["task_coordinate"]["minimum"] = inherited_safe.minimum;
  base["task_coordinate"]["maximum"] = inherited_safe.maximum;
  base["safe_aperture"] = YAML::Node(YAML::NodeType::Map);
  base["safe_aperture"]["minimum"] = inherited_safe.minimum;
  base["safe_aperture"]["maximum"] = inherited_safe.maximum;
  for (const char *key : {"schema_version", "name", "revision",
                          "contact_offsets", "geometry_identity"}) {
    if (custom[key]) setScalar(base, key, custom[key]);
  }
  for (const char *key : {"aperture_domain", "safe_q_domain", "physical_joint",
                          "task_joint", "profile_hash", "source_profile",
                          "resolved", "source_hash"}) {
    base.remove(key);
  }
  // A custom profile may not replace mechanism/task/law fields. The strict
  // custom map above intentionally has no such keys.
}

std::string canonical(const std::string &name, const std::string &model,
                      int revision, const std::string &geometry,
                      const KinematicMap &map, const Domain &safe) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  const auto string_field = [&out](const char *key, const std::string &value) {
    out << key << ".len=" << value.size() << ";" << key << "=" << value << "\n";
  };
  out << std::setprecision(17) << "schema_version=2\n";
  string_field("name", name);
  string_field("model", model);
  out << "revision=" << revision << "\n";
  string_field("geometry_identity", geometry);
  out << "law="
      << (map.law() == LawType::TwoFingerLinear ? "two_fg_linear" : "rg_sine")
      << "\nmechanism=" << map.mechanismDomain().minimum << ","
      << map.mechanismDomain().maximum << "\njoint="
      << map.jointDomain().minimum << "," << map.jointDomain().maximum
      << "\naperture=" << map.apertureDomain().minimum << ","
      << map.apertureDomain().maximum << "\nsafe=" << safe.minimum << ","
      << safe.maximum << "\nm0=" << map.m0() << "\nlength=" << map.length()
      << "\nthickness=" << map.thickness() << "\ncompensation="
      << map.compensation() << "\nscale=" << map.scale()
      << "\ntheta_reference=" << map.thetaReference() << "\nleft_offset="
      << map.leftOffset() << "\nright_offset=" << map.rightOffset() << "\n";
  return out.str();
}

std::string sha256(const std::string &input) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(),
         digest);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (const auto byte : digest) out << std::setw(2) << static_cast<int>(byte);
  return out.str();
}

GripperProfile parseNode(const YAML::Node &input, const std::string &source) {
  if (!input || !input.IsMap()) fail(source + " must contain a YAML map");
  requireMap(input, source,
             {"schema_version", "name", "revision", "model",
              "compatible_models", "mechanism_coordinate", "joint_coordinate",
              "task_coordinate", "law", "contact_offsets", "safe_aperture",
              "aperture_domain",
              "geometry_identity", "base_profile", "source_profile",
              "profile_hash", "resolved", "safe_q_domain",
              "physical_joint", "task_joint"});
  requireKeys(input, source,
              {"schema_version", "name", "revision", "model",
               "mechanism_coordinate", "joint_coordinate", "task_coordinate",
               "law"});
  const bool resolved = input["resolved"] && value<bool>(input, "resolved");
  if (!resolved && (input["aperture_domain"] || input["safe_q_domain"] ||
                    input["physical_joint"] || input["task_joint"] ||
                    input["profile_hash"] || input["source_profile"]))
    fail(source + " contains resolved-only fields without resolved: true");
  if (input["base_profile"])
    fail(source + " base_profile is only valid when loaded from a file");
  if (value<int>(input, "schema_version") != 2)
    fail(source + " has unsupported schema_version (expected 2)");

  const auto model = value<std::string>(input, "model");
  if (model != "2fg7" && model != "2fg14" && model != "rg2" && model != "rg6")
    fail(source + " has an unsupported model");
  const auto revision = value<int>(input, "revision");
  if (revision < 1) fail(source + ".revision must be positive");
  const auto name = value<std::string>(input, "name");
  if (name.empty()) fail(source + ".name must not be empty");
  if (input["compatible_models"]) {
    if (!input["compatible_models"].IsSequence() || input["compatible_models"].size() == 0)
      fail(source + ".compatible_models must be a non-empty sequence");
    bool compatible = false;
    for (const auto &entry : input["compatible_models"]) {
      const auto candidate = entry.as<std::string>();
      if (candidate == model) compatible = true;
    }
    if (!compatible) fail(source + ".compatible_models does not include model");
  }

  const auto &mechanism = input["mechanism_coordinate"];
  const bool is_two_fg = model.rfind("2fg", 0) == 0;
  unit(mechanism, source + ".mechanism_coordinate",
       is_two_fg ? "linear" : "angular", is_two_fg ? "m" : "rad");
  const auto mechanism_domain = domain(mechanism, source + ".mechanism_coordinate");
  const auto &joint = input["joint_coordinate"];
  unit(joint, source + ".joint_coordinate", is_two_fg ? "linear" : "angular",
       is_two_fg ? "m" : "rad");
  const auto joint_domain = domain(joint, source + ".joint_coordinate");

  const auto &law = input["law"];
  requireMap(law, source + ".law",
             {"type", "m0", "length", "thickness", "compensation", "scale",
              "theta_reference"});
  requireKeys(law, source + ".law", {"type"});
  const auto type = value<std::string>(law, "type");
  LawType law_type;
  double m0 = 0.0, length = 0.0, thickness = 0.0, compensation = 0.0,
         scale = 0.0, theta_reference = 0.0;
  if (is_two_fg) {
    if (type != "two_fg_linear") fail(source + " requires two_fg_linear law");
    if (law["length"] || law["thickness"] || law["compensation"] ||
        law["scale"] || law["theta_reference"])
      fail(source + ".law contains RG coefficients for a 2FG model");
    requireKeys(law, source + ".law", {"type", "m0"});
    m0 = finiteValue(law, "m0");
    law_type = LawType::TwoFingerLinear;
  } else {
    if (type != "rg_sine") fail(source + " requires rg_sine law");
    if (law["m0"]) fail(source + ".law contains a 2FG coefficient for an RG model");
    requireKeys(law, source + ".law",
                {"type", "length", "thickness", "compensation", "scale",
                 "theta_reference"});
    length = finiteValue(law, "length");
    thickness = finiteValue(law, "thickness");
    compensation = finiteValue(law, "compensation");
    scale = finiteValue(law, "scale");
    theta_reference = finiteValue(law, "theta_reference");
    if (length <= 0.0 || scale == 0.0 || std::abs(theta_reference) >= kPiOverTwo)
      fail(source + ".law contains invalid RG geometry");
    law_type = LawType::RgSine;
  }

  double left = 0.0, right = 0.0;
  if (input["contact_offsets"]) {
    const auto &offsets = input["contact_offsets"];
    requireMap(offsets, source + ".contact_offsets", {"left", "right"});
    requireKeys(offsets, source + ".contact_offsets", {"left", "right"});
    left = finiteValue(offsets, "left");
    right = finiteValue(offsets, "right");
  }
  if (input["safe_aperture"]) {
    const auto &safe = input["safe_aperture"];
    requireMap(safe, source + ".safe_aperture", {"minimum", "maximum"});
  }

  Domain full_aperture{};
  const double q0 = joint_domain.minimum;
  const double q1 = joint_domain.maximum;
  if (law_type == LawType::TwoFingerLinear) {
    const double m_at_q0 = m0 + 2.0 * q0;
    const double m_at_q1 = m0 + 2.0 * q1;
    if (std::abs(m_at_q0 - mechanism_domain.minimum) > kEpsilon ||
        std::abs(m_at_q1 - mechanism_domain.maximum) > kEpsilon)
      fail(source + " mechanism domain is inconsistent with m=m0+2q");
    full_aperture = {m_at_q0 + left + right, m_at_q1 + left + right};
  } else {
    const double theta0 = scale * q0 + theta_reference;
    const double theta1 = scale * q1 + theta_reference;
    if (std::abs(theta0) >= kPiOverTwo || std::abs(theta1) >= kPiOverTwo ||
        std::abs(theta1 - theta0) < kEpsilon ||
        (std::cos(theta0) * scale) * (std::cos(theta1) * scale) <= 0.0)
      fail(source + " RG domain is singular or non-monotonic");
    const double me0 = std::min(theta0, theta1);
    const double me1 = std::max(theta0, theta1);
    if (std::abs(me0 - mechanism_domain.minimum) > kEpsilon ||
        std::abs(me1 - mechanism_domain.maximum) > kEpsilon)
      fail(source + " mechanism domain is inconsistent with theta=s*q+ref");
    const auto aperture = [=](double q) {
      const double theta = scale * q + theta_reference;
      return 2.0 * (length * std::sin(theta) - thickness + compensation) -
             left - right;
    };
    const double a0 = aperture(q0), a1 = aperture(q1);
    full_aperture = {std::min(a0, a1), std::max(a0, a1)};
  }
  if (full_aperture.maximum <= 0.0)
    fail(source + " has no nonnegative usable aperture");
  if (!(full_aperture.minimum < full_aperture.maximum))
    fail(source + " has no usable aperture range");

  Domain safe = full_aperture;
  safe.minimum = std::max(0.0, safe.minimum);
  if (input["task_coordinate"]) {
    const auto &task = input["task_coordinate"];
    unit(task, source + ".task_coordinate", "linear", "m");
    const auto declared_task = domain(task, source + ".task_coordinate");
    safe.minimum = std::max(safe.minimum, declared_task.minimum);
    safe.maximum = std::min(safe.maximum, declared_task.maximum);
  }
  if (input["safe_aperture"]) {
    const auto declared = domain(input["safe_aperture"], source + ".safe_aperture");
    safe.minimum = std::max(safe.minimum, declared.minimum);
    safe.maximum = std::min(safe.maximum, declared.maximum);
  }
  if (!(safe.minimum < safe.maximum)) fail(source + " has no usable safe aperture range");

  const std::string geometry = input["geometry_identity"]
                                   ? value<std::string>(input, "geometry_identity")
                                   : "unspecified";
  if (geometry.empty()) fail(source + ".geometry_identity must not be empty");
  KinematicMap map(law_type, mechanism_domain, joint_domain, full_aperture, m0,
                   length, thickness, compensation, scale, theta_reference, left,
                   right);
  auto profile = GripperProfile(name, model, revision, geometry, source,
                                std::move(map), safe);
  if (resolved) {
    const auto expected_aperture = domain(input["aperture_domain"], source + ".aperture_domain");
    if (std::abs(expected_aperture.minimum - profile.map().apertureDomain().minimum) > kEpsilon ||
        std::abs(expected_aperture.maximum - profile.map().apertureDomain().maximum) > kEpsilon)
      fail(source + ".aperture_domain does not match the map");
    const auto expected_q = domain(input["safe_q_domain"], source + ".safe_q_domain");
    const double q0 = profile.map().apertureToJoint(profile.safeAperture().minimum);
    const double q1 = profile.map().apertureToJoint(profile.safeAperture().maximum);
    if (std::abs(expected_q.minimum - std::min(q0, q1)) > kEpsilon ||
        std::abs(expected_q.maximum - std::max(q0, q1)) > kEpsilon)
      fail(source + ".safe_q_domain does not match the map");
    if (value<std::string>(input, "physical_joint") !=
            (is_two_fg ? "finger_stroke" : "finger_joint") ||
        value<std::string>(input, "task_joint") != "grip_stroke")
      fail(source + " has invalid resolved resource names");
    if (value<std::string>(input, "source_profile") != profile.name())
      fail(source + ".source_profile does not match profile name");
    if (value<std::string>(input, "profile_hash") != profile.profileHash())
      fail(source + ".profile_hash does not match resolved contents");
  }
  return profile;
}

GripperProfile loadPath(const std::filesystem::path &path, int depth) {
  if (depth > 8) fail("profile inheritance exceeds eight levels");
  YAML::Node input;
  try {
    std::ifstream stream(path);
    if (!stream) fail("cannot open " + path.string());
    std::ostringstream text;
    text << stream.rdbuf();
    input = YAML::Load(text.str());
  } catch (const YAML::Exception &e) {
    fail("cannot parse " + path.string() + ": " + e.what());
  }
  if (input["base_profile"]) {
    const auto base_name = input["base_profile"].as<std::string>();
    const auto base_path = path.parent_path() / base_name;
    auto base = loadPath(base_path, depth + 1);
    YAML::Node merged = YAML::Load(base.resolvedYaml());
    mergeCustom(merged, input, path.string());
    merged.remove("base_profile");
    return parseNode(merged, path.string());
  }
  return parseNode(input, path.string());
}

} // namespace

KinematicMap::KinematicMap(LawType law, Domain mechanism_domain, Domain joint_domain,
                           Domain aperture_domain, double m0, double length,
                           double thickness, double compensation, double scale,
                           double theta_reference, double left_offset,
                           double right_offset)
    : law_(law), mechanism_domain_(mechanism_domain), joint_domain_(joint_domain),
      aperture_domain_(aperture_domain), m0_(m0), length_(length),
      thickness_(thickness), compensation_(compensation), scale_(scale),
      theta_reference_(theta_reference), left_offset_(left_offset),
      right_offset_(right_offset) {
  if (law_ != LawType::TwoFingerLinear && law_ != LawType::RgSine)
    fail("kinematic map has an invalid law type");
  for (const auto value : {mechanism_domain_.minimum, mechanism_domain_.maximum,
                           joint_domain_.minimum, joint_domain_.maximum,
                           aperture_domain_.minimum, aperture_domain_.maximum,
                           m0_, length_, thickness_, compensation_, scale_,
                           theta_reference_, left_offset_, right_offset_}) {
    if (!std::isfinite(value)) fail("kinematic map contains a non-finite value");
  }
  if (!(mechanism_domain_.minimum < mechanism_domain_.maximum) ||
      !(joint_domain_.minimum < joint_domain_.maximum) ||
      !(aperture_domain_.minimum < aperture_domain_.maximum))
    fail("kinematic map domains must be non-empty");
  if (law_ == LawType::TwoFingerLinear) {
    if (length_ != 0.0 || thickness_ != 0.0 || compensation_ != 0.0 ||
        scale_ != 0.0 || theta_reference_ != 0.0)
      fail("2FG map contains non-zero RG coefficients");
    // Keep accepted signed zero values canonical for stable profile hashes.
    length_ = 0.0;
    thickness_ = 0.0;
    compensation_ = 0.0;
    scale_ = 0.0;
    theta_reference_ = 0.0;
    if (std::abs((m0_ + 2.0 * joint_domain_.minimum) - mechanism_domain_.minimum) > kEpsilon ||
        std::abs((m0_ + 2.0 * joint_domain_.maximum) - mechanism_domain_.maximum) > kEpsilon ||
        std::abs((m0_ + 2.0 * joint_domain_.minimum + left_offset_ + right_offset_) -
                 aperture_domain_.minimum) > kEpsilon ||
        std::abs((m0_ + 2.0 * joint_domain_.maximum + left_offset_ + right_offset_) -
                 aperture_domain_.maximum) > kEpsilon)
      fail("2FG map domains are inconsistent with m=m0+2q");
  } else {
    if (m0_ != 0.0) fail("RG map contains non-zero 2FG coefficient");
    // Keep accepted signed zero values canonical for stable profile hashes.
    m0_ = 0.0;
    if (length_ <= 0.0 || scale_ == 0.0)
      fail("RG map geometry is invalid");
    const double theta0 = scale_ * joint_domain_.minimum + theta_reference_;
    const double theta1 = scale_ * joint_domain_.maximum + theta_reference_;
    if (std::abs(theta0) >= kPiOverTwo || std::abs(theta1) >= kPiOverTwo ||
        (std::cos(theta0) * scale_) * (std::cos(theta1) * scale_) <= 0.0 ||
        std::abs(std::min(theta0, theta1) - mechanism_domain_.minimum) > kEpsilon ||
        std::abs(std::max(theta0, theta1) - mechanism_domain_.maximum) > kEpsilon)
      fail("RG map domains are singular or inconsistent");
    const auto aperture = [=](double q) {
      return 2.0 * (length_ * std::sin(scale_ * q + theta_reference_) -
                    thickness_ + compensation_) - left_offset_ - right_offset_;
    };
    if (std::abs(std::min(aperture(joint_domain_.minimum), aperture(joint_domain_.maximum)) -
                 aperture_domain_.minimum) > kEpsilon ||
        std::abs(std::max(aperture(joint_domain_.minimum), aperture(joint_domain_.maximum)) -
                 aperture_domain_.maximum) > kEpsilon)
      fail("RG map aperture domain is inconsistent with its law");
  }
}

void KinematicMap::check(double value, const char *name) const {
  if (!std::isfinite(value)) fail(std::string(name) + " must be finite");
}

double KinematicMap::mechanismToJoint(double mechanism) const {
  check(mechanism, "mechanism");
  if (mechanism < mechanism_domain_.minimum || mechanism > mechanism_domain_.maximum)
    fail("mechanism is outside its domain");
  const double result = law_ == LawType::TwoFingerLinear
                            ? (mechanism - m0_) / 2.0
                            : (mechanism - theta_reference_) / scale_;
  return boundedResult(result, joint_domain_, "mechanism-to-joint");
}

double KinematicMap::jointToMechanism(double joint) const {
  check(joint, "joint");
  if (joint < joint_domain_.minimum || joint > joint_domain_.maximum)
    fail("joint is outside its domain");
  const double result = law_ == LawType::TwoFingerLinear
                            ? m0_ + 2.0 * joint
                            : scale_ * joint + theta_reference_;
  return boundedResult(result, mechanism_domain_, "joint-to-mechanism");
}

double KinematicMap::jointToAperture(double joint) const {
  const double mechanism = jointToMechanism(joint);
  double result;
  if (law_ == LawType::TwoFingerLinear) {
    result = mechanism + left_offset_ + right_offset_;
  } else {
    const double theta = mechanism;
    result = 2.0 * (length_ * std::sin(theta) - thickness_ + compensation_) -
             left_offset_ - right_offset_;
  }
  return boundedResult(result, aperture_domain_, "joint-to-aperture");
}

double KinematicMap::apertureToJoint(double aperture) const {
  check(aperture, "aperture");
  if (aperture < aperture_domain_.minimum || aperture > aperture_domain_.maximum)
    fail("aperture is outside its domain");
  if (law_ == LawType::TwoFingerLinear)
    return boundedResult((aperture - left_offset_ - right_offset_ - m0_) / 2.0,
                         joint_domain_, "aperture-to-joint");
  const double numerator = aperture + left_offset_ + right_offset_ +
                           2.0 * (thickness_ - compensation_);
  const double argument = numerator / (2.0 * length_);
  check(argument, "RG inverse argument");
  if (argument < -1.0 - kEpsilon || argument > 1.0 + kEpsilon)
    fail("aperture has no RG inverse");
  const double bounded_argument = std::clamp(argument, -1.0, 1.0);
  return boundedResult((std::asin(bounded_argument) - theta_reference_) / scale_,
                       joint_domain_, "aperture-to-joint");
}

double KinematicMap::apertureJacobian(double joint) const {
  const double theta = jointToMechanism(joint);
  if (law_ == LawType::TwoFingerLinear) return 2.0;
  const double jacobian = 2.0 * length_ * std::cos(theta) * scale_;
  if (!std::isfinite(jacobian) || std::abs(jacobian) <= kEpsilon)
    fail("aperture Jacobian is singular");
  return jacobian;
}

GripperProfile::GripperProfile(std::string name, std::string model, int revision,
                               std::string geometry_identity, std::string source,
                               KinematicMap map, Domain safe_aperture)
    : name_(std::move(name)), model_(std::move(model)), revision_(revision),
      geometry_identity_(std::move(geometry_identity)), source_(std::move(source)),
      map_(std::move(map)), safe_aperture_(safe_aperture) {
  if (name_.empty() || geometry_identity_.empty() || revision_ < 1 ||
      (model_ != "2fg7" && model_ != "2fg14" && model_ != "rg2" &&
       model_ != "rg6"))
    fail("profile identity is invalid");
  const bool is_two_fg = model_ == "2fg7" || model_ == "2fg14";
  if (is_two_fg != (map_.law() == LawType::TwoFingerLinear))
    fail("profile model and mechanism law are incompatible");
  if (!std::isfinite(safe_aperture_.minimum) ||
      !std::isfinite(safe_aperture_.maximum) ||
      safe_aperture_.minimum < 0.0 ||
      !(safe_aperture_.minimum < safe_aperture_.maximum) ||
      safe_aperture_.minimum < map_.apertureDomain().minimum ||
      safe_aperture_.maximum > map_.apertureDomain().maximum)
    fail("safe aperture is outside the map domain");
  profile_hash_ = sha256(canonical(name_, model_, revision_, geometry_identity_,
                                   map_, safe_aperture_));
}

GripperProfile GripperProfile::load(const std::string &path) {
  return loadPath(std::filesystem::path(path), 0);
}

Domain GripperProfile::safeJoint() const {
  const double a = map_.apertureToJoint(safe_aperture_.minimum);
  const double b = map_.apertureToJoint(safe_aperture_.maximum);
  return {std::min(a, b), std::max(a, b)};
}

double GripperProfile::planningJointToAperture(double joint) const {
  const auto domain = safeJoint();
  if (!std::isfinite(joint) || joint < domain.minimum || joint > domain.maximum)
    fail("planning joint is outside its safe domain");
  // Only round-off in a conversion of an already validated input is bounded.
  // In particular, nextafter outside the domain is rejected above.
  return boundedResult(map_.jointToAperture(joint), safe_aperture_,
                       "planning-joint-to-aperture");
}

double GripperProfile::apertureToPlanningJoint(double aperture) const {
  if (!std::isfinite(aperture) || aperture < safe_aperture_.minimum ||
      aperture > safe_aperture_.maximum)
    fail("planning aperture is outside its safe domain");
  return boundedResult(map_.apertureToJoint(aperture), safeJoint(),
                       "aperture-to-planning-joint");
}

GripperProfile GripperProfile::loadText(const std::string &yaml,
                                        const std::string &source) {
  try {
    return parseNode(YAML::Load(yaml), source);
  } catch (const YAML::Exception &e) {
    fail("cannot parse " + source + ": " + e.what());
  }
}

std::string GripperProfile::resolvedYaml() const {
  YAML::Emitter out;
  out.SetIndent(2);
  out << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << 2;
  out << YAML::Key << "name" << YAML::Value << name_;
  out << YAML::Key << "revision" << YAML::Value << revision_;
  out << YAML::Key << "model" << YAML::Value << model_;
  out << YAML::Key << "geometry_identity" << YAML::Value << geometry_identity_;
  out << YAML::Key << "law" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "type" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "two_fg_linear" : "rg_sine");
  if (map_.law() == LawType::TwoFingerLinear) {
    out << YAML::Key << "m0" << YAML::Value << map_.m0();
  } else {
    out << YAML::Key << "length" << YAML::Value << map_.length();
    out << YAML::Key << "thickness" << YAML::Value << map_.thickness();
    out << YAML::Key << "compensation" << YAML::Value << map_.compensation();
    out << YAML::Key << "scale" << YAML::Value << map_.scale();
    out << YAML::Key << "theta_reference" << YAML::Value << map_.thetaReference();
  }
  out << YAML::EndMap;
  out << YAML::Key << "mechanism_coordinate" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "dimension" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "linear" : "angular");
  out << YAML::Key << "unit" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "m" : "rad");
  out << YAML::Key << "minimum" << YAML::Value << map_.mechanismDomain().minimum;
  out << YAML::Key << "maximum" << YAML::Value << map_.mechanismDomain().maximum;
  out << YAML::EndMap;
  out << YAML::Key << "joint_coordinate" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "dimension" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "linear" : "angular");
  out << YAML::Key << "unit" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "m" : "rad");
  out << YAML::Key << "minimum" << YAML::Value << map_.jointDomain().minimum;
  out << YAML::Key << "maximum" << YAML::Value << map_.jointDomain().maximum;
  out << YAML::EndMap;
  out << YAML::Key << "task_coordinate" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "dimension" << YAML::Value << "linear";
  out << YAML::Key << "unit" << YAML::Value << "m";
  out << YAML::Key << "minimum" << YAML::Value << safe_aperture_.minimum;
  out << YAML::Key << "maximum" << YAML::Value << safe_aperture_.maximum;
  out << YAML::EndMap;
  out << YAML::Key << "aperture_domain" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "minimum" << YAML::Value << map_.apertureDomain().minimum;
  out << YAML::Key << "maximum" << YAML::Value << map_.apertureDomain().maximum;
  out << YAML::EndMap;
  out << YAML::Key << "safe_aperture" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "minimum" << YAML::Value << safe_aperture_.minimum;
  out << YAML::Key << "maximum" << YAML::Value << safe_aperture_.maximum;
  out << YAML::EndMap;
  out << YAML::Key << "safe_q_domain" << YAML::Value << YAML::BeginMap;
  const double safe_q0 = map_.apertureToJoint(safe_aperture_.minimum);
  const double safe_q1 = map_.apertureToJoint(safe_aperture_.maximum);
  out << YAML::Key << "minimum" << YAML::Value
      << std::min(safe_q0, safe_q1);
  out << YAML::Key << "maximum" << YAML::Value
      << std::max(safe_q0, safe_q1);
  out << YAML::EndMap;
  out << YAML::Key << "contact_offsets" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "left" << YAML::Value << map_.leftOffset();
  out << YAML::Key << "right" << YAML::Value << map_.rightOffset();
  out << YAML::EndMap;
  out << YAML::Key << "physical_joint" << YAML::Value
      << (map_.law() == LawType::TwoFingerLinear ? "finger_stroke"
                                                   : "finger_joint");
  out << YAML::Key << "task_joint" << YAML::Value << "grip_stroke";
  out << YAML::Key << "source_profile" << YAML::Value
      << name_;
  out << YAML::Key << "resolved" << YAML::Value << true;
  out << YAML::Key << "profile_hash" << YAML::Value << profile_hash_;
  out << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

} // namespace onrobot_gripper_description
