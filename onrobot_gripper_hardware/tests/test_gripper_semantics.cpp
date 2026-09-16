#include <limits>

#include <gtest/gtest.h>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace {

using onrobot_gripper_hardware::clearUnavailableJointStateField;
using onrobot_gripper_hardware::GripperKinematics;
using onrobot_gripper_hardware::RgCadKinematics;
using onrobot_gripper_hardware::RgVisualKinematics;

TEST(GripperKinematics, MapsRawMechanismMeasurementToPhysicalFingerTravel) {
  const GripperKinematics mapping(1.0, 39.0, 0.019);

  EXPECT_DOUBLE_EQ(mapping.rawPositionToJoint(1.0), 0.0);
  EXPECT_DOUBLE_EQ(mapping.rawPositionToJoint(39.0), 0.019);
  EXPECT_DOUBLE_EQ(mapping.rawPositionToJoint(20.0), 0.0095);
  EXPECT_DOUBLE_EQ(mapping.jointPositionToRaw(0.0), 1.0);
  EXPECT_DOUBLE_EQ(mapping.jointPositionToRaw(0.019), 39.0);
  EXPECT_DOUBLE_EQ(mapping.jointPositionToRaw(0.0095), 20.0);

  EXPECT_DOUBLE_EQ(mapping.rawVelocityToJoint(38.0), 0.019);
  EXPECT_DOUBLE_EQ(mapping.jointVelocityToRaw(-0.019), -38.0);
}

TEST(GripperKinematics, MapsTwoFG14RawTravelToTwentyFiveMillimetreJawTravel) {
  const GripperKinematics kinematics(1.0, 51.0, 0.025);
  EXPECT_DOUBLE_EQ(kinematics.rawPositionToJoint(1.0), 0.0);
  EXPECT_NEAR(kinematics.rawPositionToJoint(26.0), 0.0125, 1e-12);
  EXPECT_DOUBLE_EQ(kinematics.rawPositionToJoint(51.0), 0.025);
  EXPECT_NEAR(kinematics.jointPositionToRaw(0.0125), 26.0, 1e-12);
}

TEST(GripperKinematics, ClampsPositionsAtUrdfAndDeviceLimits) {
  const GripperKinematics mapping(1.0, 39.0, 0.019);

  EXPECT_DOUBLE_EQ(mapping.rawPositionToJoint(-100.0), 0.0);
  EXPECT_DOUBLE_EQ(mapping.rawPositionToJoint(100.0), 0.019);
  EXPECT_DOUBLE_EQ(mapping.jointPositionToRaw(-1.0), 1.0);
  EXPECT_DOUBLE_EQ(mapping.jointPositionToRaw(1.0), 39.0);
}

TEST(GripperKinematics, RejectsInvalidGeometry) {
  EXPECT_THROW((GripperKinematics(1.0, 1.0, 0.019)), std::invalid_argument);
  EXPECT_THROW((GripperKinematics(39.0, 1.0, 0.019)), std::invalid_argument);
  EXPECT_THROW((GripperKinematics(1.0, 39.0, 0.0)), std::invalid_argument);
  EXPECT_THROW((GripperKinematics(std::numeric_limits<double>::quiet_NaN(),
                                  39.0, 0.019)),
               std::invalid_argument);
}

TEST(RgVisualKinematics, MapsMeasuredApertureToLinkageAngle) {
  const RgVisualKinematics rg2(0.0, 0.110, 1.22277767395);
  EXPECT_DOUBLE_EQ(rg2.widthToFingerAngle(0.0), 0.0);
  EXPECT_NEAR(rg2.widthToFingerAngle(0.110), 1.22277767395, 1e-12);
  EXPECT_LT(rg2.widthToFingerAngle(0.050), rg2.widthToFingerAngle(0.100));
  EXPECT_NEAR(rg2.widthToFingerAngle(1.0), 1.22277767395, 1e-12);
  EXPECT_DOUBLE_EQ(rg2.widthToFingerAngle(-1.0), 0.0);
}

TEST(RgVisualKinematics, MapsLinkageAngleBackToAperture) {
  const RgVisualKinematics rg2(0.0, 0.110, 1.22277767395);
  EXPECT_DOUBLE_EQ(rg2.fingerAngleToWidth(0.0), 0.0);
  EXPECT_NEAR(rg2.fingerAngleToWidth(1.22277767395), 0.110, 1e-12);
  EXPECT_NEAR(rg2.fingerAngleToWidth(rg2.widthToFingerAngle(0.050)), 0.050,
              1e-12);
  EXPECT_DOUBLE_EQ(rg2.fingerAngleToWidth(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(rg2.fingerAngleToWidth(10.0), 0.110);
}

TEST(RgVisualKinematics, RejectsInvalidConfigurationAndSamples) {
  EXPECT_THROW((RgVisualKinematics(0.1, 0.1, 1.2)), std::invalid_argument);
  EXPECT_THROW((RgVisualKinematics(0.0, 0.1, 0.0)), std::invalid_argument);
  const RgVisualKinematics mapping(0.0, 0.1, 1.2);
  EXPECT_THROW(
      mapping.widthToFingerAngle(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(RgCadKinematics, InwardStandardRubberContactIsNotJointZero) {
  const RgCadKinematics rg2(false), rg6(true);
  // Independent FK of the supplied STL meshes gives these contact angles.
  EXPECT_NEAR(rg2.widthToFingerAngle(0.0), 0.08093218995, 1e-10);
  EXPECT_NEAR(rg6.widthToFingerAngle(0.0), 0.06252431382, 1e-10);
  EXPECT_NEAR(rg2.fingerAngleToWidth(0.0), -0.0089, 1e-10);
  EXPECT_NEAR(rg6.fingerAngleToWidth(0.0), -0.0100, 1e-10);
  EXPECT_NEAR(rg2.widthToFingerAngle(0.020), 0.26448979067, 1e-10);
  EXPECT_NEAR(rg6.widthToFingerAngle(0.020), 0.18793760962, 1e-10);
  for (const auto &mapping : {rg2, rg6}) {
    for (const double gap : {0.0, 0.005, 0.01, 0.02, 0.05, 0.09}) {
      EXPECT_NEAR(mapping.fingerAngleToWidth(mapping.widthToFingerAngle(gap)),
                  gap, 1e-12);
    }
    EXPECT_DOUBLE_EQ(mapping.widthToFingerAngle(-0.001),
                     mapping.widthToFingerAngle(0.0));
    EXPECT_THROW(
        mapping.widthToFingerAngle(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
    EXPECT_THROW(
        mapping.fingerAngleToWidth(std::numeric_limits<double>::infinity()),
        std::invalid_argument);
  }
}

TEST(JointStateSemantics, ClearsFieldsContainingUnavailableValues) {
  std::vector<double> unavailable{std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity()};
  EXPECT_TRUE(clearUnavailableJointStateField(unavailable));
  EXPECT_TRUE(unavailable.empty());

  std::vector<double> measured{std::numeric_limits<double>::quiet_NaN(), 12.0};
  EXPECT_TRUE(clearUnavailableJointStateField(measured));
  EXPECT_TRUE(measured.empty());

  std::vector<double> finite{0.0, 12.0};
  EXPECT_FALSE(clearUnavailableJointStateField(finite));
  ASSERT_EQ(finite.size(), 2u);

  std::vector<double> absent;
  EXPECT_FALSE(clearUnavailableJointStateField(absent));
}

} // namespace
