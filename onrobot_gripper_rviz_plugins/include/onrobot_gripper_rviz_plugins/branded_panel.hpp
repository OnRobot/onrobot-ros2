#pragma once

#include <QString>

class QWidget;

namespace onrobot_gripper_rviz_plugins {

/// Apply the shared OnRobot fonts and panel colors.
void applyOnRobotPanelStyle(QWidget *panel);

/// Create a panel header with the embedded OnRobot logo.
QWidget *createOnRobotPanelHeader(QWidget *parent, const QString &title,
                                  const QString &subtitle,
                                  int logo_width = 150);

} // namespace onrobot_gripper_rviz_plugins
