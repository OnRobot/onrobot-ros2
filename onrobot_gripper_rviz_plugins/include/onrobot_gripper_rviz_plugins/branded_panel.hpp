#pragma once

#include <QString>

class QWidget;

namespace onrobot_gripper_rviz_plugins {

/// Apply the shared OnRobot typography and restrained control-panel palette.
void applyOnRobotPanelStyle(QWidget *panel);

/// Build a compact brand header using the approved, embedded OnRobot logo.
QWidget *createOnRobotPanelHeader(QWidget *parent, const QString &title,
                                  const QString &subtitle,
                                  int logo_width = 150);

} // namespace onrobot_gripper_rviz_plugins
