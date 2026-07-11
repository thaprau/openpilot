#!/usr/bin/env python3
import math
import numpy as np
from types import SimpleNamespace

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
LEAD_APPROACH_MIN_SPEED = 3.0
LEAD_APPROACH_MIN_CLOSING_SPEED = 0.6
LEAD_APPROACH_TTC_START = 20.0
LEAD_APPROACH_TTC_FULL = 7.0
LEAD_APPROACH_T_FOLLOW_BUFFER = 0.5
LEAD_APPROACH_MIN_DISTANCE = 12.0
LEAD_APPROACH_MIN_DISTANCE_MARGIN = 12.0
LEAD_APPROACH_DISTANCE_MARGIN_TIME = 8.0
LEAD_APPROACH_RISK_FOR_HOLD = 0.35
LEAD_APPROACH_HOLD_ACCEL = 0.05
LEAD_APPROACH_BRAKE_START_CLOSING_SPEED = 1.2
LEAD_APPROACH_BRAKE_FULL_CLOSING_SPEED = 3.0
LEAD_APPROACH_DECEL_SCALE = 0.75
LEAD_APPROACH_DECEL_MAX = 0.65
LEAD_APPROACH_BRAKE_DISTANCE_FLOOR = 4.0
LEAD_APPROACH_DISTANCE_FILTER_RC = 0.25
LEAD_APPROACH_V_REL_FILTER_RC = 0.25
LEAD_APPROACH_FILTER_RESET_DISTANCE = 10.0

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]

def limit_accel_for_lead_approach(v_ego, radar_state, personality, accel_limits, _accel_coast):
  if v_ego < LEAD_APPROACH_MIN_SPEED:
    return accel_limits, False

  t_follow = get_T_FOLLOW(personality)
  desired_distance = max(LEAD_APPROACH_MIN_DISTANCE, v_ego * (t_follow + LEAD_APPROACH_T_FOLLOW_BUFFER))
  lead_accel_cap = accel_limits[1]

  # radarState also carries model-only leads on radarless cars, with lead.radar set to False.
  for lead in (radar_state.leadOne, radar_state.leadTwo):
    if not lead.status:
      continue

    closing_speed = -lead.vRel
    if closing_speed < LEAD_APPROACH_MIN_CLOSING_SPEED:
      continue

    ttc = lead.dRel / closing_speed if closing_speed > 0.0 else math.inf
    ttc_risk = float(np.interp(ttc, [LEAD_APPROACH_TTC_FULL, LEAD_APPROACH_TTC_START], [1.0, 0.0]))

    distance_margin = max(LEAD_APPROACH_MIN_DISTANCE_MARGIN, closing_speed * LEAD_APPROACH_DISTANCE_MARGIN_TIME)
    distance_error = lead.dRel - desired_distance
    distance_risk = float(np.interp(distance_error, [0.0, distance_margin], [1.0, 0.0]))

    risk = max(ttc_risk, distance_risk)
    if risk <= 0.0:
      continue

    brake_distance = max(distance_error, LEAD_APPROACH_BRAKE_DISTANCE_FLOOR)
    closing_brake_scale = float(np.interp(closing_speed,
                                          [LEAD_APPROACH_BRAKE_START_CLOSING_SPEED, LEAD_APPROACH_BRAKE_FULL_CLOSING_SPEED],
                                          [0.0, 1.0]))
    needed_decel = closing_speed**2 / (2.0 * brake_distance)
    target_decel = min(needed_decel * closing_brake_scale * LEAD_APPROACH_DECEL_SCALE, LEAD_APPROACH_DECEL_MAX)

    hold_limit = max(accel_limits[0], LEAD_APPROACH_HOLD_ACCEL)
    brake_limit = min(hold_limit, max(accel_limits[0], -target_decel))
    cap = float(np.interp(risk, [0.0, LEAD_APPROACH_RISK_FOR_HOLD, 1.0],
                          [accel_limits[1], hold_limit, brake_limit]))
    lead_accel_cap = min(lead_accel_cap, cap)

  lead_approach_active = lead_accel_cap < accel_limits[1] - 1e-3
  return [accel_limits[0], min(accel_limits[1], lead_accel_cap)], lead_approach_active


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    self.lead_approach_active = False
    self.lead_approach_d_rel_filters = [FirstOrderFilter(0.0, LEAD_APPROACH_DISTANCE_FILTER_RC, self.dt, initialized=False) for _ in range(2)]
    self.lead_approach_v_rel_filters = [FirstOrderFilter(0.0, LEAD_APPROACH_V_REL_FILTER_RC, self.dt, initialized=False) for _ in range(2)]
    self.lead_approach_track_ids = [None, None]

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def get_filtered_lead_approach_radar_state(self, radar_state):
    filtered_leads = []
    for idx, lead in enumerate((radar_state.leadOne, radar_state.leadTwo)):
      d_rel_filter = self.lead_approach_d_rel_filters[idx]
      v_rel_filter = self.lead_approach_v_rel_filters[idx]

      if not lead.status:
        d_rel_filter.initialized = False
        v_rel_filter.initialized = False
        self.lead_approach_track_ids[idx] = None
        filtered_leads.append(SimpleNamespace(status=False, dRel=0.0, vRel=0.0))
        continue

      d_rel = max(0.0, float(lead.dRel))
      v_rel = float(lead.vRel)
      track_id = int(lead.radarTrackId)
      track_changed = self.lead_approach_track_ids[idx] != track_id and track_id >= 0
      distance_jump = d_rel_filter.initialized and abs(d_rel - d_rel_filter.x) > LEAD_APPROACH_FILTER_RESET_DISTANCE
      if track_changed or distance_jump:
        d_rel_filter.initialized = False
        v_rel_filter.initialized = False

      self.lead_approach_track_ids[idx] = track_id
      filtered_leads.append(SimpleNamespace(status=True, dRel=d_rel_filter.update(d_rel), vRel=v_rel_filter.update(v_rel)))

    return SimpleNamespace(leadOne=filtered_leads[0], leadTwo=filtered_leads[1])

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if self.is_e2e(sm):
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    lead_approach_radar_state = self.get_filtered_lead_approach_radar_state(sm['radarState'])
    accel_clip, self.lead_approach_active = limit_accel_for_lead_approach(
      v_ego, lead_approach_radar_state, sm['selfdriveState'].personality, accel_clip, accel_coast
    )
    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
