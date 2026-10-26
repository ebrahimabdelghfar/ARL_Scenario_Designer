"""
Data model classes for scenario actors: EgoVehicle, NPCVehicle, Pedestrian, TriggerPoint.
"""

import uuid
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Transform:
    """3D position + rotation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.5
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z,
                "yaw": self.yaw, "pitch": self.pitch, "roll": self.roll}

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: d.get(k, 0.0) for k in ("x", "y", "z", "yaw", "pitch", "roll")})


@dataclass
class RouteWaypoint:
    """Single waypoint in a route."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: Optional[float] = None  # heading in degrees; None = unspecified

    def to_dict(self):
        d = {"x": self.x, "y": self.y, "z": self.z}
        if self.yaw is not None:
            d["yaw"] = self.yaw
        return d

    @classmethod
    def from_dict(cls, d):
        return cls(x=d["x"], y=d["y"], z=d.get("z", 0.0), yaw=d.get("yaw"))


@dataclass
class BehaviorProfile:
    """Behavior configuration for an NPC or pedestrian."""
    speed: float = 10.0            # m/s
    max_brake: float = 1.0         # [0-1]
    follow_waypoints: bool = True  # follow road waypoints
    lane_change: str = "none"      # none / left / right
    stop_at_intersection: bool = True
    distance_to_stop: float = 20.0  # meters before intersection
    random_velocity: bool = False   # randomly vary target speed while driving
    velocity_std: float = 1.0       # std dev [m/s] of the random speed variation

    def to_dict(self):
        return {
            "speed": self.speed,
            "max_brake": self.max_brake,
            "follow_waypoints": self.follow_waypoints,
            "lane_change": self.lane_change,
            "stop_at_intersection": self.stop_at_intersection,
            "distance_to_stop": self.distance_to_stop,
            "random_velocity": self.random_velocity,
            "velocity_std": self.velocity_std,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: d[k] for k in d if k in cls.__dataclass_fields__})


@dataclass
class EgoVehicle:
    """Ego vehicle — only one per scenario."""
    uid: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    name: str = "ego"
    model: str = "vehicle.lincoln.mkz_2017"
    spawn: Transform = field(default_factory=Transform)
    goal: Optional[Transform] = None  # destination / finish point
    goal_radius: float = 5.0          # trigger/success radius in meters

    def to_dict(self):
        d = {"uid": self.uid, "name": self.name, "model": self.model,
             "spawn": self.spawn.to_dict(), "goal_radius": self.goal_radius}
        if self.goal:
            d["goal"] = self.goal.to_dict()
        return d

    @classmethod
    def from_dict(cls, d):
        ego = cls(uid=d.get("uid", str(uuid.uuid4())[:8]),
                  name=d.get("name", "ego"),
                  model=d.get("model", "vehicle.lincoln.mkz_2017"),
                  spawn=Transform.from_dict(d["spawn"]),
                  goal_radius=d.get("goal_radius", 5.0))
        if "goal" in d and d["goal"]:
            ego.goal = Transform.from_dict(d["goal"])
        return ego


@dataclass
class NPCVehicle:
    """NPC vehicle actor."""
    uid: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    name: str = "npc_vehicle"
    model: str = "vehicle.tesla.model3"
    spawn: Transform = field(default_factory=Transform)
    behavior: BehaviorProfile = field(default_factory=BehaviorProfile)
    route: List[RouteWaypoint] = field(default_factory=list)
    color: Optional[str] = None  # "R,G,B"
    trigger_uid: Optional[str] = None  # TriggerPoint.uid that fires this NPC; None = global scenario trigger
    parked: bool = False  # True = spawned with hand brake, no driving behavior

    def to_dict(self):
        return {
            "uid": self.uid, "name": self.name, "model": self.model,
            "spawn": self.spawn.to_dict(),
            "behavior": self.behavior.to_dict(),
            "route": [wp.to_dict() for wp in self.route],
            "color": self.color,
            "trigger_uid": self.trigger_uid,
            "parked": self.parked,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            uid=d.get("uid", str(uuid.uuid4())[:8]),
            name=d.get("name", "npc_vehicle"),
            model=d.get("model", "vehicle.tesla.model3"),
            spawn=Transform.from_dict(d["spawn"]),
            behavior=BehaviorProfile.from_dict(d.get("behavior", {})),
            route=[RouteWaypoint.from_dict(wp) for wp in d.get("route", [])],
            color=d.get("color"),
            trigger_uid=d.get("trigger_uid"),
            parked=d.get("parked", False),
        )


@dataclass
class Pedestrian:
    """Pedestrian actor."""
    uid: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    name: str = "pedestrian"
    model: str = "walker.pedestrian.0001"
    spawn: Transform = field(default_factory=Transform)
    speed: float = 1.4  # m/s (walking speed)
    route: List[RouteWaypoint] = field(default_factory=list)
    use_wheelchair: bool = False
    trigger_uid: Optional[str] = None  # TriggerPoint.uid that fires this pedestrian; None = global scenario trigger

    def to_dict(self):
        return {
            "uid": self.uid, "name": self.name, "model": self.model,
            "spawn": self.spawn.to_dict(),
            "speed": self.speed,
            "route": [wp.to_dict() for wp in self.route],
            "use_wheelchair": self.use_wheelchair,
            "trigger_uid": self.trigger_uid,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            uid=d.get("uid", str(uuid.uuid4())[:8]),
            name=d.get("name", "pedestrian"),
            model=d.get("model", "walker.pedestrian.0001"),
            spawn=Transform.from_dict(d["spawn"]),
            speed=d.get("speed", 1.4),
            route=[RouteWaypoint.from_dict(wp) for wp in d.get("route", [])],
            use_wheelchair=d.get("use_wheelchair", False),
            trigger_uid=d.get("trigger_uid"),
        )


@dataclass
class TriggerPoint:
    """Trigger point that activates scenario when ego reaches it."""
    uid: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    name: str = "trigger"
    transform: Transform = field(default_factory=Transform)
    radius: float = 5.0  # activation radius in meters

    def to_dict(self):
        return {"uid": self.uid, "name": self.name,
                "transform": self.transform.to_dict(),
                "radius": self.radius}

    @classmethod
    def from_dict(cls, d):
        return cls(uid=d.get("uid", str(uuid.uuid4())[:8]),
                   name=d.get("name", "trigger"),
                   transform=Transform.from_dict(d["transform"]),
                   radius=d.get("radius", 5.0))


@dataclass
class StaticProp:
    """Static prop (cone, barrier, debris...) placed in the world for the whole scenario."""
    uid: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    name: str = "prop"
    model: str = "static.prop.trafficcone01"
    spawn: Transform = field(default_factory=lambda: Transform(z=0.0))
    physics: bool = False  # True = can be pushed / knocked over

    def to_dict(self):
        return {"uid": self.uid, "name": self.name, "model": self.model,
                "spawn": self.spawn.to_dict(), "physics": self.physics}

    @classmethod
    def from_dict(cls, d):
        return cls(uid=d.get("uid", str(uuid.uuid4())[:8]),
                   name=d.get("name", "prop"),
                   model=d.get("model", "static.prop.trafficcone01"),
                   spawn=Transform.from_dict(d["spawn"]),
                   physics=d.get("physics", False))
