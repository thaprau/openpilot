/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/longitudinal_panel.h"

LongitudinalPanel::LongitudinalPanel(QWidget *parent) : QWidget(parent) {
  setStyleSheet(R"(
    #back_btn {
      font-size: 50px;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #393939;
    }
    #back_btn:pressed {
      background-color:  #4a4a4a;
    }
  )");

  main_layout = new QStackedLayout(this);
  ListWidget *list = new ListWidget(this, false);

  cruisePanelScreen = new QWidget(this);
  QVBoxLayout *vlayout = new QVBoxLayout(cruisePanelScreen);
  vlayout->setContentsMargins(0, 0, 0, 0);

  cruisePanelScroller = new ScrollViewSP(list, this);
  vlayout->addWidget(cruisePanelScroller);

  customAccIncrement = new CustomAccIncrement("CustomAccIncrementsEnabled", tr("Custom ACC Speed Increments"), "", "", this);
  list->addItem(customAccIncrement);

  QObject::connect(uiState(), &UIState::offroadTransition, this, &LongitudinalPanel::refresh);

  dynamicExperimentalControl = new ParamControlSP("DynamicExperimentalControl",
    tr("Enable Dynamic Experimental Control"),
    tr("Enable toggle to allow the model to determine when to use sunnypilot ACC or sunnypilot End to End Longitudinal."),
    "../assets/offroad/icon_blank.png");
  list->addItem(dynamicExperimentalControl);
  PushButtonSP *decManageRectBtn = new PushButtonSP(tr("Customize DEC"), 800, this);
  list->addItem(decManageRectBtn);

  connect(decManageRectBtn, &QPushButton::clicked, [=]() {
    cruisePanelScroller->setLastScrollPosition();
    main_layout->setCurrentWidget(decScreen);
  });

  connect(dynamicExperimentalControl, &ParamControlSP::toggleFlipped, [=](bool enabled) {
    decManageRectBtn->setVisible(enabled);
  });

  bool decEnabled = params.getBool("DynamicExperimentalControl");
  decManageRectBtn->setVisible(decEnabled);

  decScreen = new DecControllerSubpanel(this);
  connect(decScreen, &DecControllerSubpanel::backPress, [=]() {
    cruisePanelScroller->restoreScrollPosition();
    main_layout->setCurrentWidget(cruisePanelScreen);
  });

  main_layout->addWidget(cruisePanelScreen);
  main_layout->addWidget(decScreen);
  main_layout->setCurrentWidget(cruisePanelScreen);
  refresh(offroad);
}

void LongitudinalPanel::showEvent(QShowEvent *event) {
  main_layout->setCurrentWidget(cruisePanelScreen);
  refresh(offroad);
}

void LongitudinalPanel::refresh(bool _offroad) {
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    has_longitudinal_control = hasLongitudinalControl(CP);
    is_pcm_cruise = CP.getPcmCruise();
  } else {
    has_longitudinal_control = false;
    is_pcm_cruise = false;
  }

  QString accEnabledDescription = tr("Enable custom Short & Long press increments for cruise speed increase/decrease.");
  QString accNoLongDescription = tr("This feature can only be used with openpilot longitudinal control enabled.");
  QString accPcmCruiseDisabledDescription = tr("This feature is not supported on this platform due to vehicle limitations.");
  QString onroadOnlyDescription = tr("Start the vehicle to check vehicle compatibility.");

  if (offroad) {
    customAccIncrement->setDescription(onroadOnlyDescription);
    customAccIncrement->showDescription();
  } else {
    if (has_longitudinal_control) {
      if (is_pcm_cruise) {
        customAccIncrement->setDescription(accPcmCruiseDisabledDescription);
        customAccIncrement->showDescription();
      } else {
        customAccIncrement->setDescription(accEnabledDescription);
      }
    } else {
      params.remove("CustomAccIncrementsEnabled");
      customAccIncrement->toggleFlipped(false);
      customAccIncrement->setDescription(accNoLongDescription);
      customAccIncrement->showDescription();
    }
  }

  // enable toggle when long is available and is not PCM cruise
  customAccIncrement->setEnabled(has_longitudinal_control && !is_pcm_cruise && !offroad);
  customAccIncrement->refresh();

  // Refresh DEC manage button
  if (decManageBtn) {
    bool decEnabled = params.getBool("DynamicExperimentalControl");
    decManageBtn->setVisible(decEnabled);
  }

  offroad = _offroad;
}
