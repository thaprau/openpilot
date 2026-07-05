"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import log
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.sunnypilot.lib.utils import AlertFadeAnimator


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.lead_approach_active = False
    self._lead_approach_fade = AlertFadeAnimator(gui_app.target_fps)

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    if ui_state.sm.updated["longitudinalPlanSP"]:
      self.lead_approach_active = ui_state.sm["longitudinalPlanSP"].leadApproachActive
    self._lead_approach_fade.update(self.lead_approach_active)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_lead_approach_badge(rect)

  def _draw_lead_approach_badge(self, rect: rl.Rectangle) -> None:
    if not self.lead_approach_active or ui_state.sm["selfdriveState"].alertSize != log.SelfdriveState.AlertSize.none:
      return

    alpha = self._lead_approach_fade.alpha
    text = "LEAD"
    font_size = 34
    padding_v = 5
    box_width = 130

    sz = measure_text_cached(self._font_bold, text, font_size)
    box_height = int(sz.y + padding_v * 2)
    box_x = rect.x + rect.width / 2 - box_width / 2
    box_y = rect.y + rect.height / 4 - 110

    box_color = rl.Color(255, 175, 3, int(alpha * 255))
    text_color = rl.Color(0, 0, 0, int(alpha * 255))

    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_width, box_height), 0.2, 10, box_color)
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(box_x + (box_width - sz.x) / 2, box_y + (box_height - sz.y) / 2), font_size, 0, text_color)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
