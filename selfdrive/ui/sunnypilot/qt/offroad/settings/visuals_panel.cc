/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/visuals_panel.h"

VisualsPanel::VisualsPanel(QWidget *parent) : QWidget(parent) {
  param_watcher = new ParamWatcher(this);
  connect(param_watcher, &ParamWatcher::paramChanged, [=](const QString &param_name, const QString &param_value) {
    paramsRefresh();
    if (is_metric_distance_toggle && param_name == "ChevronInfo") {
      int chevron_val = params.get("ChevronInfo").empty() ? 0 : std::stoi(params.get("ChevronInfo"));
      is_metric_distance_toggle->setVisible(chevron_val == 1 || chevron_val == 4);
    }
  });

  main_layout = new QStackedLayout(this);
  ListWidgetSP *list = new ListWidgetSP(this, false);

  sunnypilotScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(sunnypilotScreen);
  vlayout->setContentsMargins(50, 20, 50, 20);

  std::vector<std::tuple<QString, QString, QString, QString, bool> > toggle_defs{
    {
      "BlindSpot",
      tr("Show Blind Spot Warnings"),
      tr("Enabling this will display warnings when a vehicle is detected in your blind spot as long as your car has BSM supported."),
      "../assets/offroad/icon_monitoring.png",
      false,
    },
    {
      "RainbowMode",
      tr("Enable Tesla Rainbow Mode"),
      tr("Enable the rainbow mode, which displays a rainbow effect on the screen."),
      "../assets/offroad/icon_monitoring.png",
      false,
    },
    {
      "RainbowMode",
      tr("Enable Tesla Rainbow Mode"),
      RainbowizeWords(tr("A beautiful rainbow effect on the path the model wants to take.")) + "<br/><i>" + tr("It")+ " <b>" + tr("does not") + "</b> " + tr("affect driving in any way.") + "</i>",
      "../assets/offroad/icon_monitoring.png",
      false,
    },
  };

  for (auto &[param, title, desc, icon, needs_restart] : toggle_defs) {
    auto toggle = new ParamControlSP(param, title, desc, icon, this);

    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    if (needs_restart && !locked) {
      toggle->setDescription(toggle->getDescription() + tr(" Changing this setting will restart openpilot if the car is powered on."));

      QObject::connect(uiState(), &UIState::engagedChanged, [toggle](bool engaged) {
        toggle->setEnabled(!engaged);
      });

      QObject::connect(toggle, &ParamControlSP::toggleFlipped, [=](bool state) {
        params.putBool("OnroadCycleRequested", true);
      });
    }

    list->addItem(toggle);
    toggles[param.toStdString()] = toggle;
    param_watcher->addParam(param);
  }

  // Visuals: Display Metrics below Chevron
  std::vector<QString> chevron_info_settings_texts{tr("Off"), tr("Distance"), tr("Speed"), tr("Time"), tr("All")};
  chevron_info_settings = new ButtonParamControlSP(
    "ChevronInfo", tr("Display Metrics Below Chevron"), tr("Display useful metrics below the chevron that tracks the lead car (only applicable to cars with openpilot longitudinal control)."),
    "",
    chevron_info_settings_texts,
    200);
  chevron_info_settings->showDescription();
  list->addItem(chevron_info_settings);
  param_watcher->addParam("ChevronInfo");

  is_metric_distance_toggle = new ParamControlSP(
    "IsMetricDistance",
    tr("Use Metric Units for Distance"),
    tr("If enabled, distances below the chevron are shown in meters. If disabled, distances are shown in feet."),
    "",
    this
  );
  is_metric_distance_toggle->setVisible(false);
  list->addItem(is_metric_distance_toggle);
  param_watcher->addParam("IsMetricDistance");

  sunnypilotScroller = new ScrollViewSP(list, this);
  vlayout->addWidget(sunnypilotScroller);

  main_layout->addWidget(sunnypilotScreen);

  // Set initial visibility of is_metric_distance_toggle
  if (is_metric_distance_toggle) {
    int chevron_val = params.get("ChevronInfo").empty() ? 0 : std::stoi(params.get("ChevronInfo"));
    is_metric_distance_toggle->setVisible(chevron_val == 1 || chevron_val == 4);
  }
}

void VisualsPanel::paramsRefresh() {
  if (!isVisible()) {
    return;
  }

  for (auto toggle : toggles) {
    toggle.second->refresh();
  }

  if (chevron_info_settings) {
    chevron_info_settings->refresh();
  }

  if (is_metric_distance_toggle) {
    is_metric_distance_toggle->refresh();
  }
}
