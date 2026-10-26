"""
Generate ScenarioRunner Python scenario class from a ScenarioProject.
"""

from ..models.project import ScenarioProject


def _build_actor_init(project):
    """Build actor initialization code."""
    lines = []
    for i, npc in enumerate(project.npc_vehicles):
        lines.append(
            f"        npc_{i}_tf = carla.Transform(\n"
            f"            carla.Location(x={npc.spawn.x:.2f}, y={npc.spawn.y:.2f}, z={npc.spawn.z:.2f}),\n"
            f"            carla.Rotation(yaw={npc.spawn.yaw:.1f}))\n"
            f"        self.other_actors.append(CarlaDataProvider.request_new_actor('{npc.model}', npc_{i}_tf))"
        )
        if npc.parked:
            lines[-1] += (
                f"\n        if self.other_actors[-1] is not None:  # parked: hold in place for the whole scenario\n"
                f"            self.other_actors[-1].apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))"
            )
    for i, ped in enumerate(project.pedestrians):
        if ped.use_wheelchair:
            # Widen the model to any walker and require can_use_wheelchair so a
            # wheelchair-capable blueprint is always chosen, even if ped.model
            # itself lacks support. 'use_wheelchair' then enables it via the setter.
            ped_model = "walker.pedestrian.*"
            filter_str = ", attribute_filter={'can_use_wheelchair': True, 'use_wheelchair': 'true'}"
        else:
            ped_model = ped.model
            filter_str = ", attribute_filter={'use_wheelchair': 'false'}"
        lines.append(
            f"        ped_{i}_tf = carla.Transform(\n"
            f"            carla.Location(x={ped.spawn.x:.2f}, y={ped.spawn.y:.2f}, z={ped.spawn.z:.2f}),\n"
            f"            carla.Rotation(yaw={ped.spawn.yaw:.1f}))\n"
            f"        self.other_actors.append(CarlaDataProvider.request_new_actor('{ped_model}', ped_{i}_tf, actor_category='pedestrian'{filter_str}))"
        )
    # Static props go after NPCs/pedestrians so the behavior tree's other_actors indices stay valid;
    # a prop that fails to spawn (e.g. overlapping geometry) is skipped instead of appending None.
    for i, prop in enumerate(project.static_props):
        lines.append(
            f"        prop_{i}_tf = carla.Transform(\n"
            f"            carla.Location(x={prop.spawn.x:.2f}, y={prop.spawn.y:.2f}, z={prop.spawn.z:.2f}),\n"
            f"            carla.Rotation(yaw={prop.spawn.yaw:.1f}))\n"
            f"        prop_{i} = CarlaDataProvider.request_new_actor('{prop.model}', prop_{i}_tf, actor_category='misc')\n"
            f"        if prop_{i} is not None:\n"
            f"            prop_{i}.set_simulate_physics({prop.physics})\n"
            f"            self.other_actors.append(prop_{i})"
        )
    return "\n\n".join(lines) if lines else "        pass"


def _build_behavior(project):
    """Build behavior tree code.

    Each NPC runs as an independent branch of a top-level Parallel so they
    execute concurrently. A per-NPC trigger (npc.trigger_uid -> a TriggerPoint)
    becomes an InTriggerDistanceToLocation guard at the head of that NPC's
    Sequence; NPCs without one ("Global") start when the shared scenario
    trigger activates the tree.
    """
    lines = []

    # Check if ego goal and reach_goal_success is enabled
    has_goal = project.ego and project.ego.goal and project.criteria.reach_goal_success

    triggers_by_uid = {t.uid: t for t in project.trigger_points}

    # Top-level concurrent container: all actor branches tick together.
    lines.append('        all_actors = py_trees.composites.Parallel("AllActors", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)')

    # NPC behaviors — one independent Sequence per NPC
    for i, npc in enumerate(project.npc_vehicles):
        if npc.parked:
            continue  # no driving branch: hand brake from _initialize_actors keeps it in place
        s = npc.behavior.speed
        lines.append(f'        npc_seq_{i} = py_trees.composites.Sequence("NPC{i}_Behavior")')
        trig = triggers_by_uid.get(npc.trigger_uid) if npc.trigger_uid else None
        if trig is not None:
            # Per-NPC trigger: wait for ego to reach this NPC's own trigger point.
            lines.append(
                f'        npc_seq_{i}.add_child(InTriggerDistanceToLocation(\n'
                f'            self.ego_vehicles[0],\n'
                f'            carla.Location(x={trig.transform.x:.2f}, y={trig.transform.y:.2f}, z={trig.transform.z:.2f}),\n'
                f'            {trig.radius:.1f},\n'
                f'            name="NPC{i}_Trigger"))'
            )
        lines.append(f'        drv_{i} = py_trees.composites.Parallel("NPC{i}_Drive", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)')
        if npc.route:
            # Route present: follow the full collected plan. WaypointFollower returns
            # SUCCESS at end of plan, so the parallel ends there. Do NOT add an
            # intersection-stop trigger here — the NPC spawns near an intersection and
            # it would fire on tick 1, killing the WaypointFollower before it moves.
            plan_list = ", ".join([f"carla.Location(x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f})" for wp in npc.route])
            lines.append(f'        drv_{i}.add_child(WaypointFollower(self.other_actors[{i}], {s}, plan=[{plan_list}]))')
        else:
            # Free-drive: follow lane forward; optionally stop near the next intersection.
            lines.append(f'        drv_{i}.add_child(WaypointFollower(self.other_actors[{i}], {s}))')
            if npc.behavior.stop_at_intersection:
                lines.append(f'        drv_{i}.add_child(InTriggerDistanceToNextIntersection(self.other_actors[{i}], {npc.behavior.distance_to_stop}))')
        if npc.behavior.random_velocity:
            # Randomly vary the WaypointFollower target speed ~ N(speed, std) in
            # segments spanning the scenario. Idle keeps the profile from ending the
            # SUCCESS_ON_ONE drive early; the route/intersection child governs that.
            lines.append(f'        spd_seq_{i} = py_trees.composites.Sequence("NPC{i}_SpeedProfile")')
            lines.append(f'        _seg = max(1.0, self.timeout / 20.0)')
            lines.append(f'        for _ in range(20):')
            lines.append(f'            spd_seq_{i}.add_child(ChangeActorTargetSpeed(self.other_actors[{i}], max(0.0, random.gauss({npc.behavior.speed}, {npc.behavior.velocity_std})), duration=_seg))')
            lines.append(f'        spd_seq_{i}.add_child(Idle())')
            lines.append(f'        drv_{i}.add_child(spd_seq_{i})')
        lines.append(f'        npc_seq_{i}.add_child(drv_{i})')
        lines.append(f'        npc_seq_{i}.add_child(StopVehicle(self.other_actors[{i}], {npc.behavior.max_brake}))')
        lines.append(f'        all_actors.add_child(npc_seq_{i})')

    # Pedestrian behaviors — each an independent Sequence branch. A per-ped
    # trigger (ped.trigger_uid -> a TriggerPoint) becomes an
    # InTriggerDistanceToLocation guard at the head of that ped's Sequence;
    # peds without one ("Global") start with the shared scenario trigger.
    for i, ped in enumerate(project.pedestrians):
        idx = len(project.npc_vehicles) + i
        lines.append(f'        ped_seq_{i} = py_trees.composites.Sequence("Ped{i}_Behavior")')
        trig = triggers_by_uid.get(ped.trigger_uid) if ped.trigger_uid else None
        if trig is not None:
            lines.append(
                f'        ped_seq_{i}.add_child(InTriggerDistanceToLocation(\n'
                f'            self.ego_vehicles[0],\n'
                f'            carla.Location(x={trig.transform.x:.2f}, y={trig.transform.y:.2f}, z={trig.transform.z:.2f}),\n'
                f'            {trig.radius:.1f},\n'
                f'            name="Ped{i}_Trigger"))'
            )
        if ped.route:
            plan_list = ", ".join([f"carla.Location(x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f})" for wp in ped.route])
            lines.append(f'        ped_seq_{i}.add_child(WaypointFollower(self.other_actors[{idx}], {ped.speed}, plan=[{plan_list}]))')
        else:
            lines.append(f'        ped_seq_{i}.add_child(KeepVelocity(self.other_actors[{idx}], {ped.speed}))')
        lines.append(f'        all_actors.add_child(ped_seq_{i})')

    # End behavior
    lines.append('        end = py_trees.composites.Parallel("End", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)')
    lines.append('        end.add_child(StandStill(self.ego_vehicles[0], name="EgoStop", duration=1))')
    lines.append('        all_actors.add_child(end)')

    # Assemble and return
    n = len(project.npc_vehicles) + len(project.pedestrians)
    if has_goal:
        lines.append('        ')
        lines.append('        # Allow the scenario to finish when the ego reaches its goal')
        lines.append('        root_parallel = py_trees.composites.Parallel("ScenarioParallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)')
        lines.append('        root_parallel.add_child(all_actors)')
        lines.append(
            f"        goal_location = carla.Location(\n"
            f"            x={project.ego.goal.x:.2f},\n"
            f"            y={project.ego.goal.y:.2f},\n"
            f"            z={project.ego.goal.z:.2f}\n"
            f"        )\n"
            f"        # Goal trigger distance is {project.ego.goal_radius:.1f}m\n"
            f"        goal_check = InTriggerDistanceToLocation(\n"
            f"            self.ego_vehicles[0],\n"
            f"            goal_location,\n"
            f"            {project.ego.goal_radius:.1f},\n"
            f"            name='EgoGoalCheck'\n"
            f"        )\n"
            f"        root_parallel.add_child(goal_check)"
        )
        lines.append('        seq = py_trees.composites.Sequence("ScenarioBehavior")')
        lines.append('        seq.add_child(root_parallel)')
    else:
        lines.append('        seq = py_trees.composites.Sequence("ScenarioBehavior")')
        lines.append('        seq.add_child(all_actors)')

    for i in range(n):
        lines.append(f'        seq.add_child(ActorDestroy(self.other_actors[{i}]))')
    lines.append('        return seq')
    return "\n".join(lines)


def _build_criteria(project):
    """Build criteria code."""
    lines = ['        criteria = []']
    if project.criteria.collision_fail:
        lines.append('        criteria.append(CollisionTest(self.ego_vehicles[0]))')
    if project.ego and project.ego.goal and project.criteria.reach_goal_success:
        lines.append(
            f"        criteria.append(InRadiusRegionTest(\n"
            f"            self.ego_vehicles[0],\n"
            f"            x={project.ego.goal.x:.2f},\n"
            f"            y={project.ego.goal.y:.2f},\n"
            f"            radius={project.ego.goal_radius:.1f}\n"
            f"        ))"
        )
    lines.append('        return criteria')
    return "\n".join(lines)


def _build_trigger(project):
    """Build trigger behavior code if custom trigger points exist."""
    if not project.trigger_points:
        return ""
    
    # We will generate a custom _setup_scenario_trigger method
    lines = [
        "",
        "    def _setup_scenario_trigger(self, config):",
        "        # Custom trigger points from the designer"
    ]
    
    if len(project.trigger_points) == 1:
        trig = project.trigger_points[0]
        lines.append(
            f"        return InTriggerDistanceToLocation(\n"
            f"            self.ego_vehicles[0],\n"
            f"            carla.Location(x={trig.transform.x:.2f}, y={trig.transform.y:.2f}, z={trig.transform.z:.2f}),\n"
            f"            {trig.radius:.1f},\n"
            f"            name='{trig.name}'\n"
            f"        )"
        )
    else:
        lines.append(
            "        trigger_parallel = py_trees.composites.Parallel(\n"
            "            'TriggerParallel', policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE\n"
            "        )"
        )
        for trig in project.trigger_points:
            lines.append(
                f"        trigger_parallel.add_child(\n"
                f"            InTriggerDistanceToLocation(\n"
                f"                self.ego_vehicles[0],\n"
                f"                carla.Location(x={trig.transform.x:.2f}, y={trig.transform.y:.2f}, z={trig.transform.z:.2f}),\n"
                f"                {trig.radius:.1f},\n"
                f"                name='{trig.name}'\n"
                f"            )\n"
                f"        )"
            )
        lines.append("        return trigger_parallel")
    
    return "\n".join(lines)


def generate_python(project: ScenarioProject) -> str:
    cn = "".join(c if c.isalnum() else "_" for c in project.name)
    if cn[0].isdigit():
        cn = "S_" + cn
    to = int(project.criteria.timeout_seconds) if project.criteria.enable_timeout else 300
    trigger_method = _build_trigger(project)
    return f'''#!/usr/bin/env python
"""
{project.description or cn} — Auto-generated by Scenario Designer.
"""
import random
import py_trees
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter, ActorDestroy, KeepVelocity, StopVehicle, WaypointFollower,
    ChangeActorTargetSpeed, Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, InRadiusRegionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToVehicle, InTriggerDistanceToNextIntersection, DriveDistance, StandStill, InTriggerDistanceToLocation)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario


class {cn}(BasicScenario):
    """{project.description or cn}"""
    timeout = {to}

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout={to}):
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self.timeout = timeout
        super().__init__("{cn}", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
{_build_actor_init(project)}

    def _create_behavior(self):
{_build_behavior(project)}

    def _create_test_criteria(self):
{_build_criteria(project)}
{trigger_method}

    def __del__(self):
        self.remove_all_actors()
'''


def save_python(project: ScenarioProject, path: str):
    with open(path, "w") as f:
        f.write(generate_python(project))
