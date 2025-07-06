/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/longitudinal/dec_controller.h"

DecControllerSubpanel::DecControllerSubpanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);

  ListWidgetSP *list = new ListWidgetSP(this, false);
  ScrollViewSP *scroll = new ScrollViewSP(list, this);
  main_layout->addWidget(scroll);

  PanelBackButton *back_btn = new PanelBackButton();
  back_btn->setStyleSheet(R"(
    QPushButton#back_btn {
      border: 4px solid #393939 !important;
      border-radius: 30px;
      font-size: 50px;
      margin: 0px;
      padding: 10px;
      color: #dddddd;
      background-color: #393939;
    }
    QPushButton#back_btn:pressed {
      background-color: #4a4a4a;
    }
  )");
  connect(back_btn, &QPushButton::clicked, this, &DecControllerSubpanel::backPress);
  list->addItem(back_btn);

  // Reset to Defaults
  QPushButton *reset_btn = new QPushButton(tr("Reset to Defaults"));
  reset_btn->setStyleSheet(R"(
    QPushButton {
      border-radius: 20px;
      font-size: 45px;
      font-weight: 500;
      height: 120px;
      margin: 20px 40px 20px 40px;
      color: #FFFFFF;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
  )");
  list->addItem(reset_btn);

  connect(reset_btn, &QPushButton::clicked, [=]() {
    params.put("DynamicExperimentalStandstill", "1");
    params.put("DynamicExperimentalModelSlowDown", "1");
    params.put("DynamicExperimentalFCW", "1");
    params.put("DynamicExperimentalHasLead", "0");
    params.put("DynamicExperimentalDistanceBased", "0");
    params.put("DynamicExperimentalDistanceValue", "30");
    params.put("DynamicExperimentalSpeedBased", "0");
    params.put("DynamicExperimentalSpeedValue", "25");
    params.put("DynamicExperimentalSlowness", "0");

    std::vector<ParamControlSP*> toggles = {standstillControl, modelSlowDownControl, fcwControl, hasLeadControl, distanceBasedControl, speedBasedControl, slownessControl};
    for (auto toggle : toggles) {
      if (toggle) toggle->refresh();
    }
    emit distanceValueControl->updateLabels();
    emit speedValueControl->updateLabels();
    updateToggles();
  });

  auto add_toggle = [&](ParamControlSP *&ptr, const char *param, const QString &title, const QString &desc, const char *default_val) {
    ptr = new ParamControlSP(param, title, desc, "../assets/offroad/icon_blank.png");
    list->addItem(ptr);
  };

  auto add_value_control = [&](OptionControlSP *&ptr, const char *param, const QString &title, const QString &desc, int min, int max, int step, const char *unit, ParamControlSP *toggle, const char *parent_param, const char *default_val) {
    ptr = new OptionControlSP(param, title, desc, "../assets/offroad/icon_blank.png", {min, max}, step, false, nullptr);
    QWidget *container = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(container);
    layout->setContentsMargins(40, 0, 0, 0);
    layout->setAlignment(Qt::AlignLeft);
    layout->addWidget(ptr, 0, Qt::AlignLeft);
    list->addItem(container);

    connect(ptr, &OptionControlSP::updateLabels, [=]() {
      auto param_value = params.get(param);
      ptr->setLabel(QString::fromStdString(param_value) + " " + unit);
    });
    auto initial = params.get(param);
    ptr->setLabel(QString::fromStdString(initial) + " " + unit);

    connect(toggle, &ParamControlSP::toggleFlipped, [=](bool enabled) {
      container->setVisible(enabled);
    });
    container->setVisible(params.getBool(parent_param));
  };

  add_toggle(standstillControl, "DynamicExperimentalStandstill", tr("Enable at Standstill"), tr("Use blended mode when the vehicle is at a standstill."), "1");
  add_toggle(modelSlowDownControl, "DynamicExperimentalModelSlowDown", tr("Model Slow Down Detection"), tr("Use blended mode when the model detects a slow down scenario ahead."), "1");
  add_toggle(fcwControl, "DynamicExperimentalFCW", tr("FCW Detection"), tr("Use blended mode when FCW is detected in the road ahead."), "1");
  add_toggle(hasLeadControl, "DynamicExperimentalHasLead", tr("Lead Vehicle Detection"), tr("Use blended mode when a lead vehicle is detected and approaching."), "0");
  add_toggle(slownessControl, "DynamicExperimentalSlowness", tr("Slowness Detection"), tr("Use blended mode when driving significantly slower than the cruise speed."), "0");

  // Distance-based DEC
  add_toggle(distanceBasedControl, "DynamicExperimentalDistanceBased", tr("Distance-Based Switching"), tr("Use blended mode when the distance to the lead vehicle is below the specified threshold."), "0");
  add_value_control(distanceValueControl, "DynamicExperimentalDistanceValue", tr("Distance Threshold"), tr("Distance from lead vehicle in meters below which blended mode will be used."), 10, 100, 5, "m", distanceBasedControl, "DynamicExperimentalDistanceBased", "30.0");

  // Speed-based DEC
  add_toggle(speedBasedControl, "DynamicExperimentalSpeedBased", tr("Speed-Based Switching"), tr("Use blended mode when the vehicle speed is below the specified threshold."), "0");
  add_value_control(speedValueControl, "DynamicExperimentalSpeedValue", tr("Speed Threshold"), tr("Speed in km/h below which blended mode will be used."), 0, 80, 5, "km/h", speedBasedControl, "DynamicExperimentalSpeedBased", "26.0");

  connect(distanceBasedControl, &ParamControlSP::toggleFlipped, this, &DecControllerSubpanel::updateToggles);
  connect(speedBasedControl, &ParamControlSP::toggleFlipped, this, &DecControllerSubpanel::updateToggles);

  updateToggles();
}

void DecControllerSubpanel::updateToggles() {
  bool distanceBasedEnabled = params.getBool("DynamicExperimentalDistanceBased");
  bool speedBasedEnabled = params.getBool("DynamicExperimentalSpeedBased");

  distanceValueControl->setVisible(distanceBasedEnabled);
  speedValueControl->setVisible(speedBasedEnabled);
  distanceValueControl->showDescription();
  speedValueControl->showDescription();
}
