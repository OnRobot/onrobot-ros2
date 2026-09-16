// Offline FCL probe: no ROS graph, action client, or device I/O.
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <geometric_shapes/shapes.h>
#include <onrobot_gripper_description/gripper_profile.hpp>
#include <yaml-cpp/yaml.h>

std::string read(const char* path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read test input");
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  try {
    const auto urdf = urdf::parseURDF(read(argv[1]));
    auto semantic = std::make_shared<srdf::Model>();
    if (!urdf || !semantic->initString(*urdf, read(argv[2])))
      throw std::runtime_error("invalid robot description");
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
    planning_scene::PlanningScene scene(model);
    auto& state = scene.getCurrentStateNonConst();
    state.setToDefaultValues();
    const auto profile = onrobot_gripper_description::GripperProfile::load(argv[3]);
    const bool linear = profile.model().rfind("2fg", 0) == 0;
    const std::string joint = linear ? "finger_stroke" : "finger_joint";
    const auto* tip = model->getLinkModel(linear ? "right_fingertip_link" : "right_finger_tip_link");
    if (!tip || tip->getShapes().empty()) throw std::runtime_error("tip collision geometry missing");
    const auto* mesh = dynamic_cast<const shapes::Mesh*>(tip->getShapes()[0].get());
    if (!mesh || !mesh->vertex_count) throw std::runtime_error("tip mesh missing");
    auto transform = [&]() {
      state.update();
      return Eigen::Isometry3d(state.getGlobalLinkTransform(tip) * tip->getCollisionOriginTransforms()[0]);
    };
    const auto safe = profile.safeJoint();
    YAML::Node output;
    output["model"] = profile.model();
    output["physical_joint"] = joint;
    output["profile_hash"] = profile.profileHash();
    const auto* group = state.getJointModelGroup("gripper");
    output["group_variable_count"] = group->getVariableCount();
    output["group_active_variable_count"] = group->getActiveVariableCount();
    std::vector<double> samples;
    for (int i = 0; i <= 128; ++i)
      samples.push_back(safe.minimum + (safe.maximum - safe.minimum) * i / 128.0);
    samples.back() = safe.maximum;
    if (!linear) {
      samples.push_back(0.3);
      samples.push_back(0.300044);
      samples.push_back(0.300015);
    }
    for (double q : samples) {
      state.setVariablePosition(joint, q);
      const auto pose = transform();
      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = 100;
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result);
      YAML::Node sample;
      sample["q"] = q;
      sample["self_collision"] = result.collision;
      for (const auto& contact : result.contacts)
        sample["self_pairs"].push_back(contact.first.first + " / " + contact.first.second);
      // Exercise the group API used by planners as well as direct RobotState
      // assignment. Both must update the full mimic linkage identically.
      std::vector<double> planned_group;
      state.copyJointGroupPositions(group, planned_group);
      state.setVariablePosition(joint, q == safe.minimum ? safe.maximum : safe.minimum);
      state.setJointGroupPositions(group, planned_group);
      state.enforceBounds(group);
      state.update();
      sample["mimics_follow_group"] = true;
      for (const auto* jm : model->getJointModels()) {
        if (jm->getMimic() == model->getJointModel(joint)) {
          const double expected = q * jm->getMimicFactor() + jm->getMimicOffset();
          const double actual = state.getVariablePosition(jm->getName());
          if (std::abs(actual - expected) > 1e-10) {
            sample["mimics_follow_group"] = false;
            sample["bad_mimics"].push_back(jm->getName());
          }
        }
      }
      collision_detection::CollisionResult group_check;
      scene.checkSelfCollision(request, group_check);
      sample["group_self_collision"] = group_check.collision;
      for (const auto& contact : group_check.contacts)
        sample["group_self_pairs"].push_back(contact.first.first + " / " + contact.first.second);
      // Surface-mounted 1 mm obstacle: choose the distal mesh vertex furthest
      // along Z. It intersects the physical tip, not its task-only leaf.
      Eigen::Vector3d point = pose * Eigen::Map<const Eigen::Vector3d>(mesh->vertices);
      for (unsigned i = 1; i < mesh->vertex_count; ++i) {
        const Eigen::Vector3d candidate = pose * Eigen::Map<const Eigen::Vector3d>(mesh->vertices + 3*i);
        if (candidate.z() > point.z()) point = candidate;
      }
      Eigen::Isometry3d obstacle = Eigen::Isometry3d::Identity();
      obstacle.translation() = point;
      scene.getWorldNonConst()->addToObject("test_obstacle", std::make_shared<shapes::Box>(0.001, 0.001, 0.001), obstacle);
      auto hits_obstacle = [&]() {
        state.update();
        collision_detection::CollisionResult check;
        scene.checkCollision(request, check);
        for (const auto& contact : check.contacts) {
          if (contact.first.first == "test_obstacle" || contact.first.second == "test_obstacle") return true;
        }
        return false;
      };
      sample["obstacle_blocks_at_q"] = hits_obstacle();
      // Changing only the former planning leaf must not move physical meshes.
      state.setVariablePosition("grip_stroke", profile.safeAperture().maximum);
      sample["task_leaf_keeps_collision"] = hits_obstacle();
      const double separated_q = q < (safe.minimum + safe.maximum)/2.0 ? safe.maximum : safe.minimum;
      state.setVariablePosition(joint, separated_q);
      const auto moved = transform();
      sample["physical_translation_m"] = (moved.translation() - pose.translation()).norm();
      sample["obstacle_clear_after_physical_move"] = !hits_obstacle();
      scene.getWorldNonConst()->removeObject("test_obstacle");
      output["samples"].push_back(sample);
    }
    std::cout << "---PROBE---\n" << output << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
