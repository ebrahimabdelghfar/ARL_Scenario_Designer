"""
ScenarioProject — the root data model that holds everything for a scenario.
Supports save/load to JSON project files.
"""

import json
import os
import re
from dataclasses import dataclass, field
from typing import List, Optional

from .actors import EgoVehicle, NPCVehicle, Pedestrian, TriggerPoint, StaticProp
from .criteria import ScenarioCriteria, WeatherConfig


@dataclass
class ScenarioProject:
    """Root data model for a scenario design project."""
    name: str = "MyScenario"
    town: str = "Town03"
    ego: Optional[EgoVehicle] = None
    npc_vehicles: List[NPCVehicle] = field(default_factory=list)
    pedestrians: List[Pedestrian] = field(default_factory=list)
    trigger_points: List[TriggerPoint] = field(default_factory=list)
    static_props: List[StaticProp] = field(default_factory=list)
    criteria: ScenarioCriteria = field(default_factory=ScenarioCriteria)
    weather: WeatherConfig = field(default_factory=WeatherConfig)
    description: str = ""
    # Path where project was last saved
    project_path: Optional[str] = None

    def to_dict(self):
        return {
            "name": self.name,
            "town": self.town,
            "ego": self.ego.to_dict() if self.ego else None,
            "npc_vehicles": [v.to_dict() for v in self.npc_vehicles],
            "pedestrians": [p.to_dict() for p in self.pedestrians],
            "trigger_points": [t.to_dict() for t in self.trigger_points],
            "static_props": [p.to_dict() for p in self.static_props],
            "criteria": self.criteria.to_dict(),
            "weather": self.weather.to_dict(),
            "description": self.description,
        }

    @classmethod
    def from_dict(cls, d):
        proj = cls(
            name=d.get("name", "MyScenario"),
            town=d.get("town", "Town03"),
            criteria=ScenarioCriteria.from_dict(d.get("criteria", {})),
            weather=WeatherConfig.from_dict(d.get("weather", {})),
            description=d.get("description", ""),
        )
        if d.get("ego"):
            proj.ego = EgoVehicle.from_dict(d["ego"])
        proj.npc_vehicles = [NPCVehicle.from_dict(v) for v in d.get("npc_vehicles", [])]
        proj.pedestrians = [Pedestrian.from_dict(p) for p in d.get("pedestrians", [])]
        proj.trigger_points = [TriggerPoint.from_dict(t) for t in d.get("trigger_points", [])]
        proj.static_props = [StaticProp.from_dict(p) for p in d.get("static_props", [])]
        return proj

    def save(self, path: str):
        """Save project to JSON file."""
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        self.project_path = path

    @classmethod
    def load(cls, path: str) -> "ScenarioProject":
        """Load project from JSON file."""
        with open(path, "r") as f:
            data = json.load(f)
        proj = cls.from_dict(data)
        proj.project_path = path
        return proj

    def issues(self):
        """Validate project: list of (level, message, obj_type, obj_ref).
        level is ERROR / WARNING / INFO; obj_type/obj_ref point at the offending actor or None."""
        out = []
        if not self.ego:
            out.append(("ERROR", "No ego vehicle defined", None, None))
        if not self.town:
            out.append(("ERROR", "No map/town selected", None, None))
        if not re.fullmatch(r"[A-Za-z_]\w*", self.name or ""):
            out.append(("ERROR", f"Scenario name '{self.name}' must be a Python identifier "
                                  "(letters, digits, _; no leading digit)", None, None))
        if self.ego and not self.ego.goal and self.criteria.reach_goal_success:
            out.append(("WARNING", "Ego goal not set but 'reach goal' criterion enabled", "ego", self.ego))
        if not self.npc_vehicles and not self.pedestrians:
            out.append(("WARNING", "No NPC actors defined — scenario may be trivial", None, None))
        if not self.trigger_points:
            out.append(("INFO", "No trigger points — scenario starts when the ego starts driving", None, None))
        for npc in self.npc_vehicles:
            if npc.spawn.x == 0 and npc.spawn.y == 0:
                out.append(("WARNING", f"NPC '{npc.name}' at origin — likely unpositioned", "npc", npc))
            if npc.parked and npc.route:
                out.append(("INFO", f"NPC '{npc.name}' is parked — its route is ignored", "npc", npc))
        for ped in self.pedestrians:
            if ped.spawn.x == 0 and ped.spawn.y == 0:
                out.append(("WARNING", f"Pedestrian '{ped.name}' at origin — likely unpositioned", "ped", ped))
        for prop in self.static_props:
            if prop.spawn.x == 0 and prop.spawn.y == 0:
                out.append(("WARNING", f"Prop '{prop.name}' at origin — likely unpositioned", "prop", prop))
        return out

    def validate(self) -> List[str]:
        """Validate project, return list of 'LEVEL: message' strings."""
        return [f"{level}: {msg}" for level, msg, _, _ in self.issues()]

    @staticmethod
    def next_name(prefix, items):
        """First free '<prefix>_<n>' among items' names (avoids duplicates after deletes)."""
        taken = {it.name for it in items}
        n = 1
        while f"{prefix}_{n}" in taken:
            n += 1
        return f"{prefix}_{n}"

    def get_all_objects(self):
        """Return flat list of all placeable objects for iteration."""
        objs = []
        if self.ego:
            objs.append(("ego", self.ego))
        for v in self.npc_vehicles:
            objs.append(("npc", v))
        for p in self.pedestrians:
            objs.append(("ped", p))
        for t in self.trigger_points:
            objs.append(("trigger", t))
        for p in self.static_props:
            objs.append(("prop", p))
        return objs
