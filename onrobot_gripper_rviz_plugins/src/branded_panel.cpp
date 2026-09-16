#include "onrobot_gripper_rviz_plugins/branded_panel.hpp"

#include <QLabel>
#include <QPixmap>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>

namespace onrobot_gripper_rviz_plugins {

void applyOnRobotPanelStyle(QWidget *panel) {
  panel->setStyleSheet(R"(
    QWidget {
      font-family: "Ubuntu", "Arial", sans-serif;
    }
    QLabel#onrobotPanelTitle {
      color: palette(text);
      font-size: 14px;
      font-weight: 600;
    }
    QLabel#onrobotPanelSubtitle {
      color: #7e878e;
    }
    QGroupBox {
      border: 1px solid #dfe4e8;
      border-radius: 3px;
      margin-top: 0.7em;
      padding-top: 0.35em;
    }
    QGroupBox::title {
      color: palette(text);
      font-weight: 600;
      subcontrol-origin: margin;
      left: 8px;
      padding: 0 3px;
    }
    QPushButton {
      min-height: 22px;
    }
    QPushButton:hover, QPushButton:focus {
      border-color: #499dda;
    }
  )");
}

QWidget *createOnRobotPanelHeader(QWidget *parent, const QString &title,
                                  const QString &subtitle, int logo_width) {
  auto *header = new QWidget(parent);
  header->setObjectName("onrobotPanelHeader");
  auto *layout = new QVBoxLayout(header);
  layout->setContentsMargins(0, 0, 0, 4);
  layout->setSpacing(3);

  auto *logo = new QLabel(header);
  logo->setObjectName("onrobotPanelLogo");
  const QPixmap logo_pixmap(":/onrobot/branding/logo_onrobot_rgb.png");
  if (!logo_pixmap.isNull()) {
    logo->setPixmap(
        logo_pixmap.scaledToWidth(logo_width, Qt::SmoothTransformation));
  }
  logo->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  logo->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  layout->addWidget(logo);

  auto *title_label = new QLabel(title, header);
  title_label->setObjectName("onrobotPanelTitle");
  title_label->setWordWrap(true);
  layout->addWidget(title_label);

  auto *subtitle_label = new QLabel(subtitle, header);
  subtitle_label->setObjectName("onrobotPanelSubtitle");
  subtitle_label->setWordWrap(true);
  layout->addWidget(subtitle_label);
  return header;
}

} // namespace onrobot_gripper_rviz_plugins
