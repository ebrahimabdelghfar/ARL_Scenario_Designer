"""
Generate ScenarioRunner XML configuration file from a ScenarioProject.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom

from ..models.project import ScenarioProject


def generate_xml(project: ScenarioProject) -> str:
    """
    Generate the ScenarioRunner XML config file content.

    Output matches the format expected by ScenarioConfigurationParser:
      <scenarios>
        <scenario name="..." type="..." town="...">
          <ego_vehicle x="..." y="..." z="..." yaw="..." model="..." />
          <other_actor ... />
          <weather ... />
        </scenario>
      </scenarios>
    """
    root = ET.Element("scenarios")

    scenario_el = ET.SubElement(root, "scenario")
    scenario_el.set("name", f"{project.name}_1")
    scenario_el.set("type", project.name)
    scenario_el.set("town", project.town)

    # Ego vehicle
    if project.ego:
        ego_el = ET.SubElement(scenario_el, "ego_vehicle")
        ego_el.set("x", str(round(project.ego.spawn.x, 2)))
        ego_el.set("y", str(round(project.ego.spawn.y, 2)))
        ego_el.set("z", str(round(project.ego.spawn.z, 2)))
        ego_el.set("yaw", str(round(project.ego.spawn.yaw, 1)))
        ego_el.set("model", project.ego.model)

    # NPC vehicles as other_actor elements
    for npc in project.npc_vehicles:
        actor_el = ET.SubElement(scenario_el, "other_actor")
        actor_el.set("x", str(round(npc.spawn.x, 2)))
        actor_el.set("y", str(round(npc.spawn.y, 2)))
        actor_el.set("z", str(round(npc.spawn.z, 2)))
        actor_el.set("yaw", str(round(npc.spawn.yaw, 1)))
        actor_el.set("model", npc.model)
        actor_el.set("speed", str(round(npc.behavior.speed, 1)))
        if npc.color:
            actor_el.set("color", npc.color)

    # Pedestrians as other_actor elements with category
    for ped in project.pedestrians:
        actor_el = ET.SubElement(scenario_el, "other_actor")
        actor_el.set("x", str(round(ped.spawn.x, 2)))
        actor_el.set("y", str(round(ped.spawn.y, 2)))
        actor_el.set("z", str(round(ped.spawn.z, 2)))
        actor_el.set("yaw", str(round(ped.spawn.yaw, 1)))
        actor_el.set("model", ped.model)
        actor_el.set("speed", str(round(ped.speed, 1)))
        actor_el.set("use_wheelchair", "true" if ped.use_wheelchair else "false")

    # Static props as other_actor elements (no speed)
    for prop in project.static_props:
        actor_el = ET.SubElement(scenario_el, "other_actor")
        actor_el.set("x", str(round(prop.spawn.x, 2)))
        actor_el.set("y", str(round(prop.spawn.y, 2)))
        actor_el.set("z", str(round(prop.spawn.z, 2)))
        actor_el.set("yaw", str(round(prop.spawn.yaw, 1)))
        actor_el.set("model", prop.model)

    # Weather
    w = project.weather
    weather_el = ET.SubElement(scenario_el, "weather")
    for attr, value in w.to_dict().items():
        weather_el.set(attr, str(value))
    # Pretty-print
    rough = ET.tostring(root, encoding="unicode")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="    ", encoding=None)


def save_xml(project: ScenarioProject, path: str):
    """Write XML config to file."""
    content = generate_xml(project)
    with open(path, "w") as f:
        f.write(content)
