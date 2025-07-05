
/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/sunnypilot/ui.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/controls.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"

class DecControllerSubpanel : public QWidget {
  Q_OBJECT

public:
  explicit DecControllerSubpanel(QWidget *parent = nullptr);

signals:
  void backPress();

private slots:
  void updateToggles();

private:
  Params params;

  ParamControlSP *standstillControl = nullptr;
  ParamControlSP *modelSlowDownControl = nullptr;
  ParamControlSP *curvatureControl = nullptr;
  ParamControlSP *hasLeadControl = nullptr;
  ParamControlSP *distanceBasedControl = nullptr;
  ParamControlSP *speedBasedControl = nullptr;
  ParamControlSP *slownessControl = nullptr;
  OptionControlSP *distanceValueControl = nullptr;
  OptionControlSP *speedValueControl = nullptr;
};
