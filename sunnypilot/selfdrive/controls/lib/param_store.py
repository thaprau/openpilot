"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import capnp

from cereal import custom
from common.swaglog import cloudlog

from opendbc.car import structs
from openpilot.common.params import Params


class ParamStore:
  keys: list[str]
  values: dict[str, str]

  def __init__(self, CP: structs.CarParams):
    universal_params: list[str] = [
      "IsMetric",
    ]
    brand_params: list[str] = []

    self.keys = universal_params + brand_params
    self.values = {}
    self.cached_params_list: list[capnp.lib.capnp._DynamicStructBuilder] | None = None

    self.frame = 0

  def update(self, params: Params) -> None:
    if self.frame % 300 == 0:
      old_values = dict(self.values)
      self.values = {k: params.get(k) or "0" for k in self.keys}
      if old_values != self.values:
        self.cached_params_list = None

    self.frame += 1

  def publish(self) -> list[capnp.lib.capnp._DynamicStructBuilder]:
    if self.cached_params_list is None:
      # TODO-SP: Why are we doing a list instead of a dictionary here?
        self.cached_params_list = []
        for key in self.keys:
          try:
            self.cached_params_list.append(custom.CarControlSP.Param(key=key, value=self.values[key]))
          except Exception:
            cloudlog.exception(f"Error publishing CarControlSP.Param for key {key} with value {self.values[key]}")
            raise
    return self.cached_params_list
