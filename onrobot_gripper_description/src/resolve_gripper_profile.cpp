#include "onrobot_gripper_description/gripper_profile.hpp"

#include <iostream>
#include <iomanip>
#include <optional>
#include <string>

int main(int argc, char **argv) {
  std::string profile;
  std::optional<double> aperture, joint;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--profile" || arg == "--input") && i + 1 < argc) {
      profile = argv[++i];
    } else if ((arg == "--aperture" || arg == "--joint") && i + 1 < argc) {
      try {
        const std::string number(argv[++i]);
        std::size_t used = 0;
        const double parsed = std::stod(number, &used);
        if (used != number.size() || aperture || joint)
          throw std::invalid_argument("provide exactly one coordinate");
        (arg == "--aperture" ? aperture : joint) = parsed;
      } catch (const std::exception &error) {
        std::cerr << "invalid coordinate: " << error.what() << "\n";
        return 2;
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: resolve_gripper_profile --profile PROFILE "
                   "[--aperture METRES | --joint POSITION]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }
  if (profile.empty()) {
    std::cerr << "--profile is required\n";
    return 2;
  }
  try {
    const auto resolved =
        onrobot_gripper_description::GripperProfile::load(profile);
    if (!aperture && !joint) {
      std::cout << resolved.resolvedYaml();
    } else {
      const auto &map = resolved.map();
      const double q = joint ? *joint : resolved.apertureToPlanningJoint(*aperture);
      const double a = resolved.planningJointToAperture(q);
      std::cout << std::setprecision(17) << "joint_position: " << q
                << "\naperture_m: " << a
                << "\naperture_jacobian: " << map.apertureJacobian(q)
                << "\nprofile_hash: " << resolved.profileHash() << "\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "profile rejected: " << error.what() << "\n";
    return 1;
  }
}
