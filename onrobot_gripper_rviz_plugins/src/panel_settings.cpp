#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"

#include <QVariant>

namespace onrobot_gripper_rviz_plugins {

void PanelSettings::load(const rviz_common::Config &i_config) {
  // The nested map keeps panel settings separate from RViz's Name/Class keys
  // and leaves room for future panel-owned values without collisions.
  m_values = i_config.mapGetChild("OnRobot panel settings");
}

void PanelSettings::save(rviz_common::Config i_config) const {
  if (!m_values.isValid()) {
    return;
  }
  auto values = i_config.mapMakeChild("OnRobot panel settings");
  values.copy(m_values);
}

bool PanelSettings::readString(const QString &i_key,
                               std::string &o_value) const {
  QString value;
  if (!m_values.isValid() || !m_values.mapGetString(i_key, &value)) {
    return false;
  }
  o_value = value.toStdString();
  return true;
}

bool PanelSettings::readDouble(const QString &i_key, double &o_value) const {
  float value = 0.0F;
  if (!m_values.isValid() || !m_values.mapGetFloat(i_key, &value)) {
    return false;
  }
  o_value = static_cast<double>(value);
  return true;
}

void PanelSettings::writeString(const QString &i_key,
                                const std::string &i_value) {
  if (!m_values.isValid()) {
    m_values.setType(rviz_common::Config::Map);
  }
  m_values.mapSetValue(i_key, QString::fromStdString(i_value));
}

void PanelSettings::writeDouble(const QString &i_key, double i_value) {
  if (!m_values.isValid()) {
    m_values.setType(rviz_common::Config::Map);
  }
  m_values.mapSetValue(i_key, i_value);
}

} // namespace onrobot_gripper_rviz_plugins
