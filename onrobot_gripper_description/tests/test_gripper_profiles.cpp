#include <gtest/gtest.h>

#include "onrobot_gripper_description/gripper_profile.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <cstdlib>

namespace {
using onrobot_gripper_description::GripperProfile;
using onrobot_gripper_description::KinematicMap;

TEST(StockPlanningProfile, SafeEndpointsRoundTripWithoutChangingGeometry) {
  const std::vector<std::string> models{"2fg7", "2fg14", "rg2", "rg6"};
  const std::vector<double> minima{0.033, 0.055, 0.0, 0.0};
  const std::vector<double> maxima{0.071, 0.105, 0.1011, 0.150};
  for (std::size_t i = 0; i < models.size(); ++i) {
    const auto profile = GripperProfile::load(
        std::string(STOCK_PLANNING_PROFILE_DIR) + "/" + models[i] + ".yaml");
    EXPECT_NEAR(profile.safeAperture().minimum, minima[i], 1e-12);
    EXPECT_NEAR(profile.safeAperture().maximum, maxima[i], 1e-12);
    const auto a = profile.safeAperture();
    const auto q = profile.safeJoint();
    for (const double position : {a.minimum, (a.minimum + a.maximum) / 2.0, a.maximum}) {
      const double joint = profile.apertureToPlanningJoint(position);
      EXPECT_GE(joint, q.minimum);
      EXPECT_LE(joint, q.maximum);
      EXPECT_NEAR(profile.planningJointToAperture(joint), position, 1e-12);
    }
    for (const double invalid : {std::nextafter(a.minimum, -INFINITY),
                                 std::nextafter(a.maximum, INFINITY),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
      EXPECT_THROW(profile.apertureToPlanningJoint(invalid),
                   onrobot_gripper_description::ProfileError);
    for (const double invalid : {std::nextafter(q.minimum, -INFINITY),
                                 std::nextafter(q.maximum, INFINITY),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
      EXPECT_THROW(profile.planningJointToAperture(invalid),
                   onrobot_gripper_description::ProfileError);
    if (i >= 2) {
      // Mathematical CAD zero overlaps; planning zero is actual pad contact.
      EXPECT_LT(profile.map().jointToAperture(0.0), 0.0);
      EXPECT_GT(q.minimum, 0.0);
      EXPECT_THROW(profile.planningJointToAperture(0.0),
                   onrobot_gripper_description::ProfileError);
      EXPECT_THROW(profile.apertureToPlanningJoint(i == 2 ? 0.110 : 0.160),
                   onrobot_gripper_description::ProfileError);
    }
  }
}

std::string twoFg(const char *model, double m0, double b = 0.0) {
  const double mechanism_maximum = std::string(model) == "2fg14" ? 0.050 : 0.039;
  const double joint_maximum = std::string(model) == "2fg14" ? 0.025 : 0.019;
  return std::string("schema_version: 2\n") +
         "name: synthetic_" + model + "\nrevision: 1\nmodel: " + model +
         "\ngeometry_identity: fixture\nmechanism_coordinate:\n  dimension: linear\n  unit: m\n  minimum: " +
         std::to_string(m0) +
         "\n  maximum: " + std::to_string(mechanism_maximum) +
         "\njoint_coordinate:\n  dimension: linear\n  unit: m\n  minimum: 0.0\n  maximum: " + std::to_string(joint_maximum) +
         "\nlaw:\n  type: two_fg_linear\n  m0: " +
         std::to_string(m0) +
         "\ntask_coordinate:\n  dimension: linear\n  unit: m\n  minimum: 0.0\n  maximum: 0.08\ncontact_offsets:\n  left: " +
         std::to_string(b / 2.0) + "\n  right: " + std::to_string(b / 2.0) + "\n";
}

const char *rg() {
  return R"yaml(schema_version: 2
name: synthetic_rg2
revision: 7
model: rg2
geometry_identity: fixture
mechanism_coordinate:
  dimension: angular
  unit: rad
  minimum: -0.05
  maximum: 1.15
joint_coordinate:
  dimension: angular
  unit: rad
  minimum: 0.0
  maximum: 1.2
task_coordinate:
  dimension: linear
  unit: m
  minimum: 0.0
  maximum: 0.1
law:
  type: rg_sine
  length: 0.055
  thickness: 0.005
  compensation: 0.0075
  scale: 1.0
  theta_reference: -0.05
contact_offsets:
  left: 0.001
  right: -0.002
safe_aperture:
  minimum: 0.0
  maximum: 0.1
)yaml";
}

void rejectWithReason(const std::string &source, const std::string &reason) {
  try {
    (void)GripperProfile::loadText(source);
    FAIL() << "profile unexpectedly accepted";
  } catch (const std::exception &error) {
    EXPECT_NE(std::string(error.what()).find(reason), std::string::npos)
        << error.what();
  }
}

std::filesystem::path uniqueTestDirectory(const char *prefix) {
  auto template_path = (std::filesystem::current_path() /
                        (std::string(prefix) + "_XXXXXX")).string();
  std::vector<char> writable(template_path.begin(), template_path.end());
  writable.push_back('\0');
  const char *created = ::mkdtemp(writable.data());
  if (created == nullptr) throw std::runtime_error("mkdtemp failed");
  return created;
}

TEST(GripperProfile, TwoFingerLawRoundTripsAndKeepsModelSpecificM0) {
  const auto profile_2fg7 =
      GripperProfile::loadText(twoFg("2fg7", 0.001, 0.004));
  const auto profile_2fg14 =
      GripperProfile::loadText(twoFg("2fg14", 0.0, 0.004));
  EXPECT_DOUBLE_EQ(profile_2fg7.map().mechanismToJoint(0.001), 0.0);
  EXPECT_DOUBLE_EQ(profile_2fg14.map().mechanismToJoint(0.0), 0.0);
  EXPECT_NEAR(profile_2fg7.map().jointToAperture(0.0125), 0.030, 1e-12);
  EXPECT_NEAR(profile_2fg7.map().apertureToJoint(0.030), 0.0125, 1e-12);
  EXPECT_DOUBLE_EQ(profile_2fg7.map().apertureJacobian(0.0125), 2.0);
  EXPECT_NE(profile_2fg7.profileHash(), profile_2fg14.profileHash());
}

TEST(GripperProfile, RgLawRoundTripsAndJacobianMatchesFiniteDifference) {
  const auto profile = GripperProfile::loadText(rg());
  const auto &map = profile.map();
  for (const double q : {0.1, 0.4, 0.9, 1.1}) {
    const double aperture = map.jointToAperture(q);
    EXPECT_NEAR(map.apertureToJoint(aperture), q, 1e-11);
    const double h = 1e-7;
    const double finite_difference =
        (map.jointToAperture(q + h) - map.jointToAperture(q - h)) / (2.0 * h);
    EXPECT_NEAR(map.apertureJacobian(q), finite_difference, 1e-8);
  }
  EXPECT_GT(map.apertureJacobian(0.5), 0.0);
  EXPECT_DOUBLE_EQ(map.leftOffset() + map.rightOffset(), -0.001);
}

TEST(GripperProfile, RejectsDomainsUnitsModelsAndUnknownFields) {
  const auto reject = [](std::string source) {
    EXPECT_THROW(GripperProfile::loadText(source),
                 onrobot_gripper_description::ProfileError);
  };
  reject(twoFg("2fg7", 0.001) + "unexpected: true\n");
  reject(twoFg("2fg7", 0.001) + "name: duplicate\n");
  auto inverted_task = twoFg("2fg7", 0.001);
  inverted_task.replace(inverted_task.find("maximum: 0.08"),
                        std::string("maximum: 0.08").size(), "maximum: -0.1");
  reject(inverted_task);
  reject(twoFg("2fg7", 0.001) + "profile_hash: bogus\n");
  rejectWithReason(R"yaml(schema_version: 2
name: flow
revision: 1
model: 2fg7
mechanism_coordinate: {dimension: linear, unit: m, minimum: 0.001, maximum: 0.039}
joint_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.019}
task_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.039}
law: {type: two_fg_linear, m0: 0.001, m0: 0.001}
)yaml", "duplicate");
  reject(twoFg("2fg7", 0.001) + "'name': quoted\n");
  rejectWithReason(R"yaml(schema_version: 2
name: wrong_law
revision: 1
model: 2fg7
mechanism_coordinate: {dimension: linear, unit: m, minimum: 0.001, maximum: 0.039}
joint_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.019}
task_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.039}
law: {type: two_fg_linear, m0: 0.001, length: 0.05}
)yaml", "RG coefficients");
  reject(twoFg("2fg7", 0.001).replace(
      twoFg("2fg7", 0.001).find("unit: m"), 7, "unit: mm"));
  reject(twoFg("rg2", 0.0));
  rejectWithReason(R"yaml(schema_version: 2
name: no_range
revision: 1
model: 2fg7
mechanism_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.001}
joint_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.001}
task_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: 0.02}
law: {type: two_fg_linear, m0: 0.01}
)yaml", "mechanism domain is inconsistent");
}

TEST(GripperProfile, RejectsSingularRgAndInvalidSafeRange) {
  std::string singular = rg();
  singular.replace(singular.find("maximum: 1.15"), std::string("maximum: 1.15").size(),
                  "maximum: 1.6207");
  singular.replace(singular.find("maximum: 1.2"), std::string("maximum: 1.2").size(),
                  "maximum: 1.6707");
  EXPECT_THROW(GripperProfile::loadText(singular),
               onrobot_gripper_description::ProfileError);
  std::string no_range = rg();
  no_range.replace(no_range.find("maximum: 0.1"), 13, "maximum: 0.0");
  EXPECT_THROW(GripperProfile::loadText(no_range),
               onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, CustomProfileCanOnlyChangeContactAndSafeFields) {
  const std::string base = twoFg("2fg7", 0.001, 0.0);
  const std::string custom = R"yaml(schema_version: 2
name: custom
revision: 3
base_profile: base.yaml
contact_offsets: {left: 0.003, right: -0.001}
safe_aperture: {minimum: 0.01, maximum: 0.03}
geometry_identity: customer-fixture
)yaml";
  const auto dir = uniqueTestDirectory("r08_profile_test");
  std::ofstream(dir / "base.yaml") << base;
  std::ofstream(dir / "custom.yaml") << custom;
  const auto profile = GripperProfile::load((dir / "custom.yaml").string());
  EXPECT_DOUBLE_EQ(profile.map().m0(), 0.001);
  EXPECT_DOUBLE_EQ(profile.map().leftOffset(), 0.003);
  EXPECT_DOUBLE_EQ(profile.map().rightOffset(), -0.001);
  EXPECT_EQ(profile.geometryIdentity(), "customer-fixture");
  EXPECT_DOUBLE_EQ(profile.safeAperture().minimum, 0.01);
  EXPECT_EQ(profile.resolvedYaml(), GripperProfile::loadText(profile.resolvedYaml()).resolvedYaml());
  auto invalid_units = custom;
  invalid_units.replace(invalid_units.find("maximum: 0.03}"),
                        std::string("maximum: 0.03}").size(),
                        "maximum: 0.03, unit: mm}");
  std::ofstream(dir / "custom.yaml") << invalid_units;
  EXPECT_THROW(GripperProfile::load((dir / "custom.yaml").string()),
               onrobot_gripper_description::ProfileError);
  std::filesystem::remove_all(dir);
}

TEST(GripperProfile, RejectsOutOfDomainValuesRatherThanClamping) {
  const auto profile = GripperProfile::loadText(twoFg("2fg7", 0.001));
  EXPECT_THROW(profile.map().jointToAperture(-1.0),
               onrobot_gripper_description::ProfileError);
  EXPECT_THROW(profile.map().apertureToJoint(-1.0),
               onrobot_gripper_description::ProfileError);
  EXPECT_THROW(profile.map().mechanismToJoint(
                   std::numeric_limits<double>::quiet_NaN()),
               onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, VerifiedRgGeometryRetainsNegativeOverlapInFullDomain) {
  const auto make = [](const char *model, double length, double thickness,
                       double compensation, double theta_ref,
                       double q_max, double m_max) {
    std::ostringstream yaml;
    yaml << std::setprecision(17);
    yaml << "schema_version: 2\nname: " << model << "_geometry\nrevision: 1\nmodel: "
         << model << "\ngeometry_identity: stock-contact-geometry\n"
         << "mechanism_coordinate: {dimension: angular, unit: rad, minimum: "
         << theta_ref << ", maximum: " << theta_ref + q_max << "}\n"
         << "joint_coordinate: {dimension: angular, unit: rad, minimum: 0, maximum: "
         << q_max << "}\n"
         << "task_coordinate: {dimension: linear, unit: m, minimum: 0, maximum: "
         << m_max << "}\n"
         << "law: {type: rg_sine, length: " << length << ", thickness: "
         << thickness << ", compensation: " << compensation
         << ", scale: 1, theta_reference: " << theta_ref << "}\n";
    const double offset = std::string(model) == "rg6" ? 0.005 : 0.00445;
    yaml << "contact_offsets: {left: " << offset << ", right: " << offset
         << "}\n";
    return GripperProfile::loadText(yaml.str());
  };
  const auto rg2 = make("rg2", 0.055, 0.005, 0.0075,
                         -0.04547021241699715, 1.22277767395, 0.110);
  const auto rg6 = make("rg6", 0.080, 0.0063, 0.0105,
                         -0.052524147149517135, 1.1928, 0.160);
  EXPECT_LT(rg2.map().apertureDomain().minimum, 0.0);
  EXPECT_LT(rg6.map().apertureDomain().minimum, 0.0);
  EXPECT_DOUBLE_EQ(rg2.safeAperture().minimum, 0.0);
  EXPECT_DOUBLE_EQ(rg6.safeAperture().minimum, 0.0);
  const auto check_vectors = [](const GripperProfile &profile,
                                const std::vector<double> &q,
                                const std::vector<double> &a) {
    ASSERT_EQ(q.size(), a.size());
    for (std::size_t i = 0; i < q.size(); ++i) {
      EXPECT_NEAR(profile.map().jointToAperture(q[i]), a[i], 2e-12);
      EXPECT_NEAR(profile.map().apertureToJoint(a[i]), q[i], 2e-12);
      EXPECT_NEAR(profile.map().jointToAperture(
                      profile.map().apertureToJoint(a[i])),
                  a[i], 2e-12);
    }
  };
  check_vectors(rg2, {0.0, 0.3, 0.6, 1.22277767395},
                {-0.0089, 0.0237969410815, 0.0540197969047,
                 0.0976934708051});
  check_vectors(rg6, {0.0, 0.3, 0.6, 1.1928},
                {-0.010, 0.0375931993888, 0.0816853870033,
                 0.1437997849884});
}

TEST(GripperProfile, ExplicitTwoFingerInwardProfileClipsOnlySafeRange) {
  const auto profile = GripperProfile::loadText(twoFg("2fg7", 0.001, -0.002));
  EXPECT_LT(profile.map().apertureDomain().minimum, 0.0);
  EXPECT_DOUBLE_EQ(profile.safeAperture().minimum, 0.0);
  EXPECT_NEAR(profile.map().jointToAperture(0.0), -0.001, 1e-12);
  EXPECT_NEAR(profile.map().jointToAperture(0.019), 0.037, 1e-12);
}

TEST(GripperProfile, InheritedGuardRemapsWithLargeAndAsymmetricOffsets) {
  const std::string base = twoFg("2fg7", 0.001, 0.032);
  const std::string custom = R"yaml(schema_version: 2
name: wide_custom
revision: 3
base_profile: base.yaml
contact_offsets: {left: 0.100, right: 0.032}
)yaml";
  const auto dir = uniqueTestDirectory("r08_profile_guard");
  std::ofstream(dir / "base.yaml") << base;
  std::ofstream(dir / "custom.yaml") << custom;
  const auto profile = GripperProfile::load((dir / "custom.yaml").string());
  EXPECT_NEAR(profile.map().apertureToJoint(profile.safeAperture().minimum), 0.0, 1e-12);
  EXPECT_NEAR(profile.map().apertureToJoint(profile.safeAperture().maximum), 0.019, 1e-12);
  EXPECT_NEAR(profile.safeAperture().minimum, 0.133, 1e-12);
  EXPECT_NEAR(profile.safeAperture().maximum, 0.171, 1e-12);
  std::filesystem::remove_all(dir);
}

TEST(GripperProfile, DirectConstructorsEnforceIdentityAndLawInvariants) {
  EXPECT_THROW(
      KinematicMap(static_cast<onrobot_gripper_description::LawType>(99),
                   {0.001, 0.039}, {0.0, 0.019}, {0.001, 0.039}, 0.001,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      onrobot_gripper_description::ProfileError);
  const auto valid = GripperProfile::loadText(twoFg("2fg7", 0.001));
  EXPECT_THROW(
      GripperProfile("", "2fg7", 1, "fixture", "test", valid.map(),
                     valid.safeAperture()),
      onrobot_gripper_description::ProfileError);
  EXPECT_THROW(
      GripperProfile("name", "unknown", 1, "fixture", "test", valid.map(),
                     valid.safeAperture()),
      onrobot_gripper_description::ProfileError);
  EXPECT_THROW(
      GripperProfile("name", "rg2", 1, "fixture", "test", valid.map(),
                     valid.safeAperture()),
      onrobot_gripper_description::ProfileError);
  const auto inward = GripperProfile::loadText(twoFg("2fg7", 0.001, -0.002));
  EXPECT_THROW(
      GripperProfile("name", "2fg7", 1, "fixture", "test", inward.map(),
                     {-0.0005, 0.03}),
      onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, TwoFingerStockEndpointRoundTripsRemainBounded) {
  const auto profile = GripperProfile::loadText(twoFg("2fg7", 0.001, 0.032));
  const auto &map = profile.map();
  EXPECT_DOUBLE_EQ(map.mechanismToJoint(map.mechanismDomain().minimum),
                   map.jointDomain().minimum);
  EXPECT_DOUBLE_EQ(map.mechanismToJoint(map.mechanismDomain().maximum),
                   map.jointDomain().maximum);
  EXPECT_DOUBLE_EQ(map.jointToMechanism(map.jointDomain().minimum),
                   map.mechanismDomain().minimum);
  EXPECT_DOUBLE_EQ(map.jointToMechanism(map.jointDomain().maximum),
                   map.mechanismDomain().maximum);
  EXPECT_NEAR(map.apertureToJoint(map.apertureDomain().minimum),
              map.jointDomain().minimum, 1e-15);
  EXPECT_NEAR(map.apertureToJoint(map.apertureDomain().maximum),
              map.jointDomain().maximum, 1e-15);
}

TEST(GripperProfile, AllSupportedLawConversionsStayInDeclaredDomains) {
  std::string rg6_source = rg();
  rg6_source.replace(rg6_source.find("synthetic_rg2"), 13, "synthetic_rg6");
  rg6_source.replace(rg6_source.find("model: rg2"), 10, "model: rg6");
  const std::vector<GripperProfile> profiles{
      GripperProfile::loadText(twoFg("2fg7", 0.001, 0.032)),
      GripperProfile::loadText(twoFg("2fg14", 0.0, 0.032)),
      GripperProfile::loadText(rg()), GripperProfile::loadText(rg6_source)};
  for (const auto &profile : profiles) {
    const auto &map = profile.map();
    for (const double mechanism : {map.mechanismDomain().minimum,
                                   map.mechanismDomain().maximum}) {
      const double joint = map.mechanismToJoint(mechanism);
      EXPECT_GE(joint, map.jointDomain().minimum);
      EXPECT_LE(joint, map.jointDomain().maximum);
      const double recovered = map.jointToMechanism(joint);
      EXPECT_GE(recovered, map.mechanismDomain().minimum);
      EXPECT_LE(recovered, map.mechanismDomain().maximum);
    }
    for (const double joint : {map.jointDomain().minimum,
                               map.jointDomain().maximum}) {
      const double mechanism = map.jointToMechanism(joint);
      const double recovered = map.mechanismToJoint(mechanism);
      EXPECT_GE(recovered, map.jointDomain().minimum);
      EXPECT_LE(recovered, map.jointDomain().maximum);
      const double aperture = map.jointToAperture(joint);
      EXPECT_GE(aperture, map.apertureDomain().minimum);
      EXPECT_LE(aperture, map.apertureDomain().maximum);
      const double recovered_joint = map.apertureToJoint(aperture);
      EXPECT_GE(recovered_joint, map.jointDomain().minimum);
      EXPECT_LE(recovered_joint, map.jointDomain().maximum);
    }
    for (const double aperture : {map.apertureDomain().minimum,
                                  map.apertureDomain().maximum}) {
      const double joint = map.apertureToJoint(aperture);
      EXPECT_GE(joint, map.jointDomain().minimum);
      EXPECT_LE(joint, map.jointDomain().maximum);
      const double recovered = map.jointToAperture(joint);
      EXPECT_GE(recovered, map.apertureDomain().minimum);
      EXPECT_LE(recovered, map.apertureDomain().maximum);
    }
  }
}

TEST(GripperProfile, NextafterOutsideInputsAreRejected) {
  const auto map = GripperProfile::loadText(twoFg("2fg7", 0.001)).map();
  const double below = -std::numeric_limits<double>::infinity();
  const double above = std::numeric_limits<double>::infinity();
  EXPECT_THROW(map.mechanismToJoint(
                   std::nextafter(map.mechanismDomain().minimum, below)),
               onrobot_gripper_description::ProfileError);
  EXPECT_THROW(map.jointToMechanism(
                   std::nextafter(map.jointDomain().maximum, above)),
               onrobot_gripper_description::ProfileError);
  EXPECT_THROW(map.jointToAperture(
                   std::nextafter(map.jointDomain().minimum, below)),
               onrobot_gripper_description::ProfileError);
  EXPECT_THROW(map.apertureToJoint(
                   std::nextafter(map.apertureDomain().maximum, above)),
               onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, DirectMapRejectsNonzeroUnusedLawCoefficients) {
  const auto two_fg = GripperProfile::loadText(twoFg("2fg7", 0.001)).map();
  const auto expect_two_fg_rejected = [&](double length, double thickness,
                                          double compensation, double scale,
                                          double theta_reference) {
    EXPECT_THROW(
        KinematicMap(two_fg.law(), two_fg.mechanismDomain(),
                     two_fg.jointDomain(), two_fg.apertureDomain(), two_fg.m0(),
                     length, thickness, compensation, scale, theta_reference,
                     two_fg.leftOffset(), two_fg.rightOffset()),
        onrobot_gripper_description::ProfileError);
  };
  expect_two_fg_rejected(1e-6, 0.0, 0.0, 0.0, 0.0);
  expect_two_fg_rejected(0.0, 1e-6, 0.0, 0.0, 0.0);
  expect_two_fg_rejected(0.0, 0.0, 1e-6, 0.0, 0.0);
  expect_two_fg_rejected(0.0, 0.0, 0.0, 1e-6, 0.0);
  expect_two_fg_rejected(0.0, 0.0, 0.0, 0.0, 1e-6);

  const auto rg_map = GripperProfile::loadText(rg()).map();
  EXPECT_THROW(
      KinematicMap(rg_map.law(), rg_map.mechanismDomain(), rg_map.jointDomain(),
                   rg_map.apertureDomain(), 1e-6, rg_map.length(),
                   rg_map.thickness(), rg_map.compensation(), rg_map.scale(),
                   rg_map.thetaReference(), rg_map.leftOffset(),
                   rg_map.rightOffset()),
      onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, DirectMapCanonicalizesSignedZeroUnusedCoefficients) {
  const auto base = GripperProfile::loadText(twoFg("2fg7", 0.001));
  const auto &m = base.map();
  const KinematicMap map(m.law(), m.mechanismDomain(), m.jointDomain(),
                         m.apertureDomain(), m.m0(), -0.0, -0.0, -0.0, -0.0,
                         -0.0, m.leftOffset(), m.rightOffset());
  const GripperProfile profile(base.name(), base.model(), base.revision(),
                               base.geometryIdentity(), "signed-zero", map,
                               base.safeAperture());
  const auto restored = GripperProfile::loadText(profile.resolvedYaml());
  EXPECT_EQ(restored.profileHash(), profile.profileHash());
  EXPECT_EQ(restored.resolvedYaml(), profile.resolvedYaml());
}

TEST(GripperProfile, SafeApertureOutsideMapIsRejectedWithoutEpsilon) {
  const auto profile = GripperProfile::loadText(twoFg("2fg7", 0.001));
  const auto map = profile.map();
  EXPECT_THROW(
      GripperProfile(profile.name(), profile.model(), profile.revision(),
                     profile.geometryIdentity(), "strict-safe", map,
                     {map.apertureDomain().minimum - 5e-13,
                      map.apertureDomain().maximum}),
      onrobot_gripper_description::ProfileError);
}

TEST(GripperProfile, InheritedStockEndpointProfileRetainsGuard) {
  const auto dir = uniqueTestDirectory("r08_profile_endpoint");
  std::ofstream(dir / "base.yaml") << twoFg("2fg7", 0.001, 0.032);
  std::ofstream(dir / "custom.yaml") << R"yaml(schema_version: 2
name: endpoint_custom
revision: 1
base_profile: base.yaml
geometry_identity: endpoint-fixture
)yaml";
  const auto profile = GripperProfile::load((dir / "custom.yaml").string());
  EXPECT_DOUBLE_EQ(profile.safeAperture().minimum,
                   profile.map().apertureDomain().minimum);
  EXPECT_DOUBLE_EQ(profile.safeAperture().maximum,
                   profile.map().apertureDomain().maximum);
  EXPECT_NO_THROW(profile.resolvedYaml());
  std::filesystem::remove_all(dir);
}

} // namespace
