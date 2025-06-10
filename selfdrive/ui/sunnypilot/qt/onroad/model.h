/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/qt/onroad/model.h"

class ModelRendererSP : public ModelRenderer {
public:
  ModelRendererSP() = default;

private:
  void update_model(const cereal::ModelDataV2::Reader &model, const cereal::RadarState::LeadData::Reader &lead) override;
  void drawPath(QPainter &painter, const cereal::ModelDataV2::Reader &model, const QRect &rect) override;
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, const QRect &surface_rect);

  float hysteretic_x = 0.0f;
  const float hysteresis_factor = 0.4f;
  Params params;
  QPolygonF left_blindspot_vertices;
  QPolygonF right_blindspot_vertices;
};
