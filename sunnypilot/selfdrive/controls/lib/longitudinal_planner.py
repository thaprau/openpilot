"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.models.helpers import get_active_bundle

from openpilot.sunnypilot.selfdrive.controls.lib.vibe_personality.vibe_personality import VibePersonalityController
DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, mpc):
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl(CP)
    self.vibe_controller = VibePersonalityController()
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.transition_init()

  @property
  def mlsim(self) -> bool:
    # If we don't have a generation set, we assume it's default model. Which as of today are mlsim.
    return bool(self.generation is None or self.generation >= 11)

  def get_mpc_mode(self) -> str | None:
    if not self.dec.active():
      return None

    return self.dec.mode()

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    scc_output_v_target, scc_output_a_target = self.scc.update(sm, v_ego, a_ego, v_cruise)

    targets = {
      'cruise': (v_cruise, a_ego),
      'scc': (scc_output_v_target, scc_output_a_target),
    }

    src = min(targets, key=lambda k: targets[k][0])
    v_target, a_target = targets[src]

    return v_target, a_target

  def transition_init(self) -> None:
    self._transition_counter = 0
    self._transition_steps = 20
    self._last_mode = 'acc'

  def handle_mode_transition(self, mode: str) -> None:
    if self._last_mode != mode:
      if mode == 'blended':
        self._transition_counter = 0
      self._last_mode = mode

  def blend_accel_transition(self, mpc_accel: float, e2e_accel: float, v_ego: float) -> float:
    if not self.dec.enabled():
      return min(mpc_accel, e2e_accel)
    self._transition_counter = min(self._transition_counter + 1, self._transition_steps)
    progress = self._transition_counter / self._transition_steps
    ease_progress = 3 * progress**2 - 2 * progress**3
    if e2e_accel < 0.0:
      # Weight E2E braking more strongly using sigmoid and low-speed scaling
      sigmoid = 1 / (1 + math.exp(-3.0 * (abs(e2e_accel / ACCEL_MIN) - 0.5)))
      low_speed_factor = min(v_ego / 5.0, 1.0)
      blend_factor = ease_progress * sigmoid * low_speed_factor
      return mpc_accel + (e2e_accel - mpc_accel) * blend_factor
    return min(mpc_accel, e2e_accel)

  def update(self, sm: messaging.SubMaster) -> None:
    self.dec.update(sm)
    self.vibe_controller.update()

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Turn Speed Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active

    pm.send('longitudinalPlanSP', plan_sp_send)
