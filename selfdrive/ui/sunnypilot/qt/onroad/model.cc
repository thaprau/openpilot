/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/onroad/model.h"


void ModelRendererSP::update_model(const cereal::ModelDataV2::Reader &model, const cereal::RadarState::LeadData::Reader &lead) {
  ModelRenderer::update_model(model, lead);
  const auto &model_position = model.getPosition();
  const auto &lane_lines = model.getLaneLines();
  float max_distance = std::clamp(*(model_position.getX().end() - 1), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  // update blindspot vertices
  float max_distance_barrier = 100;
  int max_idx_barrier = std::min(max_idx, get_path_length_idx(lane_lines[0], max_distance_barrier));
  mapLineToPolygon(model.getLaneLines()[1], 0.2, -0.05, &left_blindspot_vertices, max_idx_barrier);
  mapLineToPolygon(model.getLaneLines()[2], 0.2, -0.05, &right_blindspot_vertices, max_idx_barrier);
}

void ModelRendererSP::drawPath(QPainter &painter, const cereal::ModelDataV2::Reader &model, const QRect &surface_rect) {
  auto *s = uiState();
  auto &sm = *(s->sm);
  bool blindspot = Params().getBool("BlindSpot");

  if (blindspot) {
    bool left_blindspot = sm["carState"].getCarState().getLeftBlindspot();
    bool right_blindspot = sm["carState"].getCarState().getRightBlindspot();

    //painter.setBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.4));  // Red with alpha for blind spot

    if (left_blindspot && !left_blindspot_vertices.isEmpty()) {
      QLinearGradient gradient(0, 0, surface_rect.width(), 0); // Horizontal gradient from left to right
      gradient.setColorAt(0.0, QColor(255, 165, 0, 102)); // Orange with alpha
      gradient.setColorAt(1.0, QColor(255, 255, 0, 102)); // Yellow with alpha
      painter.setBrush(gradient);
      painter.drawPolygon(left_blindspot_vertices);
    }

    if (right_blindspot && !right_blindspot_vertices.isEmpty()) {
      QLinearGradient gradient(surface_rect.width(), 0, 0, 0); // Horizontal gradient from right to left
      gradient.setColorAt(0.0, QColor(255, 165, 0, 102)); // Orange with alpha
      gradient.setColorAt(1.0, QColor(255, 255, 0, 102)); // Yellow with alpha
      painter.setBrush(gradient);
      painter.drawPolygon(right_blindspot_vertices);
    }
  }
  ModelRenderer::drawPath(painter, model, surface_rect.height());
}

void ModelRendererSP::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data,
                             const QPointF &vd, const QRect &surface_rect) {
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float raw_x = std::clamp<float>(vd.x(), 0.f, surface_rect.width() - sz / 2);
  float y = std::min<float>(vd.y(), surface_rect.height() - sz * 0.6);

  float x_delta = std::abs(raw_x - hysteretic_x);
  float threshold = 100.0f;

  if (x_delta > threshold) {
    hysteretic_x = raw_x; // For large changes, use raw value
  } else {
    hysteretic_x = (hysteresis_factor * raw_x) + ((1.0f - hysteresis_factor) * hysteretic_x); // For small changes, apply hysteresis
  }

  float x = raw_x;
  if (params.getBool("ChevronHysteresis")) {
    x = hysteretic_x;
  }

  if (params.getBool("ChevronMinimal")) {
    // Set up the pen for drawing
    QPen pen;
    pen.setCapStyle(Qt::RoundCap);  // Round ends of the line
    pen.setJoinStyle(Qt::RoundJoin);  // Round corners

    // Disable fill
    painter.setBrush(Qt::NoBrush);


    // Draw the outer glow effect
    pen.setColor(QColor(218, 202, 37, 255));  // Yellow glow color
    pen.setWidth(10);  // Thicker width for glow
    painter.setPen(pen);

    // Create path for the line
    QPainterPath path;
    path.moveTo(x + (sz * 1.35), y + sz);   // right point
    path.lineTo(x, y); // top point
    path.lineTo(x - (sz * 1.35), y + sz);  // left point

    painter.drawPath(path);  // Draw the glow

    // Draw the main line
    pen.setColor(QColor(201, 34, 49, fillAlpha));  // Red color with calculated opacity
    pen.setWidth(7);  // Slightly thinner than the glow
    painter.setPen(pen);
    painter.drawPath(path);  // Draw the main line
  } else {
    float g_xo = sz / 5;
    float g_yo = sz / 10;

    QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
    painter.setBrush(QColor(218, 202, 37, 255));
    painter.drawPolygon(glow, std::size(glow));

    // chevron
    QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
    painter.setBrush(QColor(201, 34, 49, fillAlpha));
    painter.drawPolygon(chevron, std::size(chevron));
  }
}
