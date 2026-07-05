from types import SimpleNamespace

from cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_planner import limit_accel_for_lead_approach


def make_lead(status=False, d_rel=0.0, v_rel=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel)


def make_radar_state(lead_one=None, lead_two=None):
  return SimpleNamespace(
    leadOne=lead_one or make_lead(),
    leadTwo=lead_two or make_lead(),
  )


def test_lead_approach_inactive_without_relevant_lead():
  accel_limits = [-3.5, 1.2]
  limited_accel, active = limit_accel_for_lead_approach(20.0, make_radar_state(), log.LongitudinalPersonality.standard, accel_limits, 0.0)

  assert limited_accel == accel_limits
  assert not active


def test_lead_approach_inactive_below_minimum_speed():
  accel_limits = [-3.5, 1.2]
  radar_state = make_radar_state(make_lead(status=True, d_rel=10.0, v_rel=-4.0))
  limited_accel, active = limit_accel_for_lead_approach(2.0, radar_state, log.LongitudinalPersonality.standard, accel_limits, 0.0)

  assert limited_accel == accel_limits
  assert not active


def test_lead_approach_inactive_when_closing_too_slowly():
  accel_limits = [-3.5, 1.2]
  radar_state = make_radar_state(make_lead(status=True, d_rel=25.0, v_rel=-0.2))
  limited_accel, active = limit_accel_for_lead_approach(20.0, radar_state, log.LongitudinalPersonality.standard, accel_limits, 0.0)

  assert limited_accel == accel_limits
  assert not active


def test_lead_approach_active_lowers_accel_cap():
  accel_limits = [-3.5, 1.2]
  radar_state = make_radar_state(make_lead(status=True, d_rel=35.0, v_rel=-2.0))
  limited_accel, active = limit_accel_for_lead_approach(20.0, radar_state, log.LongitudinalPersonality.standard, accel_limits, 0.0)

  assert limited_accel[0] == accel_limits[0]
  assert limited_accel[1] < accel_limits[1]
  assert active
