#pragma once

#include <QString>

namespace onrobot_gripper_rviz_plugins {

inline QString formatSafetyState(bool i_safety1Pushed, bool i_safety1Triggered,
                                 bool i_safety2Pushed, bool i_safety2Triggered,
                                 bool i_dcError) {
  if (i_dcError) {
    return "DC ERROR: power-cycle the gripper after making the area safe";
  }
  if (i_safety1Pushed || i_safety2Pushed) {
    return "STOPPED: safety switch pushed; release it before recovery";
  }
  if (i_safety1Triggered || i_safety2Triggered) {
    return "Triggered; clear the area, then request recovery";
  }
  return "Clear";
}

} // namespace onrobot_gripper_rviz_plugins
