"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit_controller.common import Source, OffsetType
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit_controller import SpeedLimitControlState, REQUIRED_INITIAL_MAX_SET_SPEED, \
  PRE_ACTIVE_GUARD_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit_controller.speed_limit_controller import SpeedLimitController, ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SPEED_LIMITS = {
  'residential': 25 * CV.MPH_TO_MS,  # 25 mph
  'city': 35 * CV.MPH_TO_MS,         # 35 mph
  'highway': 65 * CV.MPH_TO_MS,      # 65 mph
  'freeway': 80 * CV.MPH_TO_MS,      # 80 mph
}


class TestSpeedLimitController:

  def _setup_platform(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)

    sunnypilot_interfaces.setup_interfaces(CI, self.params)

    return CI
  def teardown_method(self, method):
    self.reset_state()

  def reset_state(self):
    self.reset_custom_params()
    self.slc.state = SpeedLimitControlState.disabled
    self.slc.frame = -1
    self.slc.last_op_engaged_frame = 0.0
    self.slc.op_engaged = False
    self.slc.op_engaged_prev = False
    self.slc.initial_max_set = False
    self.slc._speed_limit = 0.
    self.slc.speed_limit_prev = 0.
    self.slc.last_valid_speed_limit_final = 0.
    self.slc._distance = 0.
    self.slc._source = Source.none
    self.events_sp.clear()

  def setup_method(self):
    self.params = Params()
    self.reset_custom_params()
    self.events_sp = EventsSP()
    CI = self._setup_platform(TOYOTA.TOYOTA_RAV4_TSS2_2022)
    self.slc = SpeedLimitController(CI.CP)

  def reset_custom_params(self):
    self.params.put_bool("SpeedLimitControl", True)
    self.params.put_bool("IsMetric", False)
    self.params.put("SpeedLimitOffsetType", 0)
    self.params.put("SpeedLimitValueOffset", 0)

  def initialize_active_state(self, v_cruise_setpoint):
    self.slc.state = SpeedLimitControlState.active
    self.slc.v_cruise_setpoint_prev = v_cruise_setpoint

  def test_initial_state(self):
    assert self.slc.state == SpeedLimitControlState.disabled
    assert not self.slc.is_enabled
    assert not self.slc.is_active
    assert self.slc.final_cruise_speed == V_CRUISE_UNSET

  def test_disabled(self):
    self.params.put_bool("SpeedLimitControl", False)
    for _ in range(int(10. / DT_MDL)):
      _ = self.slc.update(True, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.disabled

  def test_transition_disabled_to_preactive(self):
    for _ in range(int(3. / DT_MDL)):
      _ = self.slc.update(True, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.preActive
    assert self.slc.is_enabled and not self.slc.is_active

  def test_preactive_to_active_with_max_speed_confirmation(self):
    self.slc.state = SpeedLimitControlState.preActive
    v_cruise_slc = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.active
    assert self.slc.is_enabled and self.slc.is_active
    assert v_cruise_slc == SPEED_LIMITS['city']

  def test_preactive_timeout_to_inactive(self):
    self.slc.state = SpeedLimitControlState.preActive
    _ = self.slc.update(True, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)

    for _ in range(int(PRE_ACTIVE_GUARD_PERIOD / DT_MDL)):
      _ = self.slc.update(True, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.inactive

  def test_preactive_to_pending_no_speed_limit(self):
    self.slc.state = SpeedLimitControlState.preActive
    _ = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, 0, 0, Source.none, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.pending
    assert self.slc.is_enabled and not self.slc.is_active

  def test_pending_to_active_when_speed_limit_available(self):
    self.slc.state = SpeedLimitControlState.pending
    _ = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.active

  def test_pending_to_adapting_when_below_speed_limit(self):
    self.slc.state = SpeedLimitControlState.pending
    _ = self.slc.update(True, SPEED_LIMITS['city'] + 5, 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.adapting
    assert self.slc.is_enabled and self.slc.is_active

  def test_active_to_adapting_transition(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)

    _ = self.slc.update(True, SPEED_LIMITS['city'] + 2, 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.adapting

  def test_adapting_to_active_transition(self):
    self.slc.state = SpeedLimitControlState.adapting
    self.slc.v_cruise_setpoint_prev = REQUIRED_INITIAL_MAX_SET_SPEED

    _ = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.active

  def test_manual_cruise_change_detection(self):
    self.slc.state = SpeedLimitControlState.active
    expected_cruise = SPEED_LIMITS['highway']
    self.slc.v_cruise_setpoint_prev = expected_cruise

    different_cruise = SPEED_LIMITS['highway'] + 5
    _ = self.slc.update(True, SPEED_LIMITS['city'], 0, different_cruise, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.inactive

  @pytest.mark.parametrize("offset_type, offset_value, speed_limit, expected_offset", [
    (OffsetType.fixed, 5, SPEED_LIMITS['city'], 5 * CV.MPH_TO_MS),  # 5 MPH fixed offset
    (OffsetType.percentage, 10, SPEED_LIMITS['city'], 0.1 * SPEED_LIMITS['city']),  # 10% offset
    (OffsetType.off, 0, SPEED_LIMITS['city'], 0),  # Off
    (OffsetType.fixed, 10, SPEED_LIMITS['highway'], 10 * CV.MPH_TO_MS),  # Different speed, fixed offset
    (OffsetType.percentage, 5, SPEED_LIMITS['highway'], 0.05 * SPEED_LIMITS['highway']),  # Different speed, percentage
  ])
  def test_offset_calculations(self, offset_type, offset_value, speed_limit, expected_offset):
    self.slc._speed_limit = speed_limit
    actual_offset = self.slc.get_offset(offset_type, offset_value)
    assert actual_offset == pytest.approx(expected_offset, rel=0.01)

  def test_rapid_speed_limit_changes(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)
    speed_limits = [SPEED_LIMITS['city'], SPEED_LIMITS['highway'], SPEED_LIMITS['residential']]

    for _, speed_limit in enumerate(speed_limits):
      _ = self.slc.update(True, speed_limit, 0, REQUIRED_INITIAL_MAX_SET_SPEED, speed_limit, 0, Source.car_state, self.events_sp)
    assert self.slc.state in ACTIVE_STATES

  def test_invalid_speed_limits_handling(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)
    self.slc.last_valid_speed_limit_final = SPEED_LIMITS['city']

    invalid_limits = [-10, 0, 200 * CV.MPH_TO_MS]

    for invalid_limit in invalid_limits:
      v_cruise_slc = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, invalid_limit, 0, Source.car_state, self.events_sp)
      assert isinstance(v_cruise_slc, (int, float))
      assert v_cruise_slc == V_CRUISE_UNSET or v_cruise_slc > 0

  def test_stale_data_handling(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)
    old_speed_limit = SPEED_LIMITS['city']
    self.slc.last_valid_speed_limit_final = old_speed_limit

    v_cruise_slc = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, 0, 0, Source.car_state, self.events_sp)
    assert self.slc.state in ACTIVE_STATES
    assert v_cruise_slc == old_speed_limit

  def test_different_speed_limit_sources(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)

    for source in (Source.car_state, Source.map_data):
      v_cruise_slc = self.slc.update(True, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, source, self.events_sp)
      assert v_cruise_slc != V_CRUISE_UNSET

  def test_distance_based_adapting(self):
    self.slc.state = SpeedLimitControlState.adapting
    self.slc.v_cruise_setpoint_prev = REQUIRED_INITIAL_MAX_SET_SPEED

    distance = 100.0
    current_speed = SPEED_LIMITS['highway']
    target_speed = SPEED_LIMITS['city']

    v_cruise_slc = self.slc.update(True, current_speed, 0, REQUIRED_INITIAL_MAX_SET_SPEED, target_speed, distance, Source.map_data, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.adapting
    assert v_cruise_slc == target_speed  # TODO-SP: assert expected accel, need to enable self.acceleration_solutions

  def test_long_disengaged_to_disabled(self):
    self.initialize_active_state(REQUIRED_INITIAL_MAX_SET_SPEED)

    v_cruise_slc = self.slc.update(False, SPEED_LIMITS['city'], 0, REQUIRED_INITIAL_MAX_SET_SPEED, SPEED_LIMITS['city'], 0, Source.car_state, self.events_sp)
    assert self.slc.state == SpeedLimitControlState.disabled
    assert v_cruise_slc == V_CRUISE_UNSET
