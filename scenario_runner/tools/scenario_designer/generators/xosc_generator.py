"""
Generate OpenSCENARIO 1.0 (.xosc) file from a ScenarioProject.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
from ..models.project import ScenarioProject


def generate_xosc(project: ScenarioProject) -> str:
    """Generate OpenSCENARIO 1.0 XML content."""
    root = ET.Element("OpenSCENARIO")
    # FileHeader
    fh = ET.SubElement(root, "FileHeader")
    fh.set("revMajor", "1")
    fh.set("revMinor", "0")
    fh.set("date", "2026-01-01T00:00:00")
    fh.set("description", f"CARLA:{project.name}")
    fh.set("author", "ScenarioDesigner")

    ET.SubElement(root, "ParameterDeclarations")
    ET.SubElement(root, "CatalogLocations")

    # RoadNetwork
    rn = ET.SubElement(root, "RoadNetwork")
    lf = ET.SubElement(rn, "LogicFile")
    lf.set("filepath", project.town)
    ET.SubElement(rn, "SceneGraphFile").set("filepath", "")

    # Entities
    entities = ET.SubElement(root, "Entities")

    def add_vehicle_entity(parent, name, model, etype):
        so = ET.SubElement(parent, "ScenarioObject")
        so.set("name", name)
        v = ET.SubElement(so, "Vehicle")
        v.set("name", model)
        v.set("vehicleCategory", "car")
        ET.SubElement(v, "ParameterDeclarations")
        perf = ET.SubElement(v, "Performance")
        perf.set("maxSpeed", "69.444")
        perf.set("maxAcceleration", "200")
        perf.set("maxDeceleration", "10.0")
        bb = ET.SubElement(v, "BoundingBox")
        c = ET.SubElement(bb, "Center")
        c.set("x", "1.5"); c.set("y", "0.0"); c.set("z", "0.9")
        d = ET.SubElement(bb, "Dimensions")
        d.set("width", "2.1"); d.set("length", "4.5"); d.set("height", "1.8")
        ax = ET.SubElement(v, "Axles")
        fa = ET.SubElement(ax, "FrontAxle")
        for k, val in [("maxSteering","0.5"),("wheelDiameter","0.6"),("trackWidth","1.8"),("positionX","3.1"),("positionZ","0.3")]:
            fa.set(k, val)
        ra = ET.SubElement(ax, "RearAxle")
        for k, val in [("maxSteering","0.0"),("wheelDiameter","0.6"),("trackWidth","1.8"),("positionX","0.0"),("positionZ","0.3")]:
            ra.set(k, val)
        props = ET.SubElement(v, "Properties")
        p1 = ET.SubElement(props, "Property")
        p1.set("name", "type"); p1.set("value", etype)

    if project.ego:
        add_vehicle_entity(entities, "hero", project.ego.model, "ego_vehicle")
    for i, npc in enumerate(project.npc_vehicles):
        add_vehicle_entity(entities, f"adversary_{i}", npc.model, "simulation")
    for i, prop in enumerate(project.static_props):
        so = ET.SubElement(entities, "ScenarioObject")
        so.set("name", f"prop_{i}")
        mo = ET.SubElement(so, "MiscObject")
        mo.set("name", prop.model)
        mo.set("miscObjectCategory", "obstacle")
        mo.set("mass", "10.0")
        ET.SubElement(mo, "ParameterDeclarations")
        bb = ET.SubElement(mo, "BoundingBox")
        c = ET.SubElement(bb, "Center")
        c.set("x", "0.0"); c.set("y", "0.0"); c.set("z", "0.5")
        d = ET.SubElement(bb, "Dimensions")
        d.set("width", "1.0"); d.set("length", "1.0"); d.set("height", "1.0")
        ET.SubElement(mo, "Properties")

    # Storyboard
    sb = ET.SubElement(root, "Storyboard")
    init = ET.SubElement(sb, "Init")
    actions = ET.SubElement(init, "Actions")

    def add_teleport(parent, entity_name, x, y, z, yaw):
        priv = ET.SubElement(parent, "Private")
        priv.set("entityRef", entity_name)
        pa = ET.SubElement(priv, "PrivateAction")
        ta = ET.SubElement(pa, "TeleportAction")
        pos = ET.SubElement(ta, "Position")
        wp = ET.SubElement(pos, "WorldPosition")
        wp.set("x", str(x)); wp.set("y", str(y)); wp.set("z", str(z))
        wp.set("h", str(yaw * 0.0174533))  # deg to rad

    if project.ego:
        add_teleport(actions, "hero", project.ego.spawn.x, project.ego.spawn.y,
                     project.ego.spawn.z, project.ego.spawn.yaw)
    for i, npc in enumerate(project.npc_vehicles):
        add_teleport(actions, f"adversary_{i}", npc.spawn.x, npc.spawn.y,
                     npc.spawn.z, npc.spawn.yaw)
    for i, prop in enumerate(project.static_props):
        add_teleport(actions, f"prop_{i}", prop.spawn.x, prop.spawn.y,
                     prop.spawn.z, prop.spawn.yaw)

    # Story with speed actions for NPCs
    story = ET.SubElement(sb, "Story")
    story.set("name", "GeneratedStory")
    act = ET.SubElement(story, "Act")
    act.set("name", "Behavior")
    for i, npc in enumerate(project.npc_vehicles):
        mg = ET.SubElement(act, "ManeuverGroup")
        mg.set("name", f"MG_{i}"); mg.set("maximumExecutionCount", "1")
        actors = ET.SubElement(mg, "Actors")
        actors.set("selectTriggeringEntities", "false")
        er = ET.SubElement(actors, "EntityRef")
        er.set("entityRef", f"adversary_{i}")
        man = ET.SubElement(mg, "Maneuver")
        man.set("name", f"Maneuver_{i}")
        ev = ET.SubElement(man, "Event")
        ev.set("name", f"SpeedEvent_{i}"); ev.set("priority", "overwrite")
        ac = ET.SubElement(ev, "Action")
        ac.set("name", f"SpeedAction_{i}")
        pa = ET.SubElement(ac, "PrivateAction")
        la = ET.SubElement(pa, "LongitudinalAction")
        sa = ET.SubElement(la, "SpeedAction")
        sad = ET.SubElement(sa, "SpeedActionDynamics")
        sad.set("dynamicsShape", "step"); sad.set("value", "20"); sad.set("dynamicsDimension", "distance")
        sat = ET.SubElement(sa, "SpeedActionTarget")
        ats = ET.SubElement(sat, "AbsoluteTargetSpeed")
        ats.set("value", str(0.0 if npc.parked else npc.behavior.speed))
        st = ET.SubElement(ev, "StartTrigger")
        cg = ET.SubElement(st, "ConditionGroup")
        cond = ET.SubElement(cg, "Condition")
        cond.set("name", "Start"); cond.set("delay", "0"); cond.set("conditionEdge", "rising")
        bvc = ET.SubElement(cond, "ByValueCondition")
        stc = ET.SubElement(bvc, "SimulationTimeCondition")
        stc.set("value", "0"); stc.set("rule", "greaterThan")

    # Act triggers
    ast = ET.SubElement(act, "StartTrigger")
    acg = ET.SubElement(ast, "ConditionGroup")
    ac2 = ET.SubElement(acg, "Condition")
    ac2.set("name", "ActStart"); ac2.set("delay", "0"); ac2.set("conditionEdge", "rising")
    bvc2 = ET.SubElement(ac2, "ByValueCondition")
    stc2 = ET.SubElement(bvc2, "SimulationTimeCondition")
    stc2.set("value", "0"); stc2.set("rule", "greaterThan")
    ET.SubElement(act, "StopTrigger")

    # Criteria via StopTrigger
    stop = ET.SubElement(sb, "StopTrigger")
    scg = ET.SubElement(stop, "ConditionGroup")
    if project.criteria.collision_fail:
        cc = ET.SubElement(scg, "Condition")
        cc.set("name", "criteria_CollisionTest"); cc.set("delay", "0"); cc.set("conditionEdge", "rising")
        bv = ET.SubElement(cc, "ByValueCondition")
        pc = ET.SubElement(bv, "ParameterCondition")
        pc.set("parameterRef", ""); pc.set("value", ""); pc.set("rule", "lessThan")

    rough = ET.tostring(root, encoding="unicode")
    return minidom.parseString(rough).toprettyxml(indent="  ")


def save_xosc(project: ScenarioProject, path: str):
    with open(path, "w") as f:
        f.write(generate_xosc(project))
