/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/longitudinal_panel.h"

LongitudinalPanel::LongitudinalPanel(QWidget *parent) : QWidget(parent) {

    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->setMargin(0);

    listWidget = new ListWidgetSP(this);
    listWidget->setContentsMargins(0, 0, 0, 0);
    listWidget->setSpacing(0);
    main_layout->addWidget(listWidget);

    experimentalLongitudinalToggle = new ParamControl(
        "AlphaLongitudinalEnabled",
        tr("openpilot Longitudinal Control (Alpha)"),
        QString("<b>%1</b><br><br>%2")
        .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
        .arg(tr("On this car, sunnypilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
                "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
        ""
        );
    experimentalLongitudinalToggle->setConfirmation(true, false);
    QObject::connect(experimentalLongitudinalToggle, &ParamControl::toggleFlipped, [=]() {
      updateToggles(offroad);
    });
    listWidget->addItem(experimentalLongitudinalToggle);

    main_layout->addStretch();

    // Update toggles that should be not available to change in onroad state
    QObject::connect(uiState(), &UIState::offroadTransition, this, &LongitudinalPanel::updateToggles);
}

void LongitudinalPanel::updateToggles(bool _offroad) {

    // experimentalLongitudinalToggle should not be toggleable if the car does not have longitudinal control
    auto cp_bytes = params.get("CarParamsPersistent");
    if (!cp_bytes.empty()) {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
        cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

        if (!CP.getAlphaLongitudinalAvailable()) {
            params.remove("AlphaLongitudinalEnabled");
            experimentalLongitudinalToggle->setEnabled(false);
        }

        // experimentalLongitudinalToggle should be visible when the car supports experimental longitudinal control (alpha)
        experimentalLongitudinalToggle->setVisible(CP.getAlphaLongitudinalAvailable());

    } else {
        experimentalLongitudinalToggle->setVisible(false);
    }
    experimentalLongitudinalToggle->refresh();
    offroad = _offroad;
}

void LongitudinalPanel::showEvent(QShowEvent *event) {
    updateToggles(offroad);
}
