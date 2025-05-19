/**
* Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/visuals_panel.h"

VisualsPanel::VisualsPanel(QWidget *parent) : QWidget(parent) {
 QVBoxLayout* main_layout = new QVBoxLayout(this);
 main_layout->setMargin(0);

 listWidget = new ListWidgetSP(this);
 listWidget->setContentsMargins(0, 0, 0, 0);
 listWidget->setSpacing(0);
 main_layout->addWidget(listWidget);

 minimalChevronToggle = new ParamControlSP(
  "ChevronMinimal",
  "Minimalistic Lead Indicator",
  "Use a minimalistic style for lead chevron",
  "", this);
 listWidget->addItem(minimalChevronToggle);

 smoothChevronToggle = new ParamControlSP(
  "ChevronHysteresis",
  "Smoother Lead Indicator Movement",
  "Reduces jerky lead chevron movement and smoothens it out. This is purely a UI change, and has no impact on lead tracking.",
  "", this);
 listWidget->addItem(smoothChevronToggle);

 main_layout->addStretch();
}
