"""
Scenario criteria definitions: success/failure conditions.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ScenarioCriteria:
    """Success and failure conditions for the scenario."""
    collision_fail: bool = True          # Fail on collision
    reach_goal_success: bool = True      # Succeed when ego reaches goal
    timeout_seconds: float = 120.0       # Scenario timeout
    enable_timeout: bool = True
    route_deviation_fail: bool = False   # Fail on route deviation
    max_route_deviation: float = 10.0    # meters
    running_red_light: bool = False      # Track red light violations
    running_stop_sign: bool = False      # Track stop sign violations
    keep_lane: bool = False              # Track lane keeping

    def to_dict(self):
        return {
            "collision_fail": self.collision_fail,
            "reach_goal_success": self.reach_goal_success,
            "timeout_seconds": self.timeout_seconds,
            "enable_timeout": self.enable_timeout,
            "route_deviation_fail": self.route_deviation_fail,
            "max_route_deviation": self.max_route_deviation,
            "running_red_light": self.running_red_light,
            "running_stop_sign": self.running_stop_sign,
            "keep_lane": self.keep_lane,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: d[k] for k in d if k in cls.__dataclass_fields__})


@dataclass
class WeatherConfig:
    """Weather parameters for the scenario."""
    cloudiness: float = 0.0
    precipitation: float = 0.0
    precipitation_deposits: float = 0.0
    wind_intensity: float = 0.0
    sun_azimuth_angle: float = 0.0
    sun_altitude_angle: float = 75.0
    fog_density: float = 0.0
    fog_distance: float = 0.0
    wetness: float = 0.0

    def to_dict(self):
        return {k: getattr(self, k) for k in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: d[k] for k in d if k in cls.__dataclass_fields__})
