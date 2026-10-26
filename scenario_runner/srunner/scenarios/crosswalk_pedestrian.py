#!/usr/bin/env python

# Copyright (c) 2026 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Custom Scenario: Pedestrians crossing the street only through a crosswalk.
"""

from __future__ import print_function

import math
import operator
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      WaitForever,
                                                                      ActorTransformSetter)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               TriggerVelocity,
                                                                               WaitUntilInFrontPosition)
from srunner.scenarios.basic_scenario import BasicScenario


def get_value_parameter(config, name, p_type, default):
    """
    Helper function to extract parameter values from scenario config
    """
    if name in config.other_parameters:
        val = config.other_parameters[name]['value']
        if isinstance(val, str) and ',' in val:
            val = val.split(',')[0].strip()
        return p_type(val)
    else:
        return default


class CrosswalkPedestrian(BasicScenario):

    """
    This class implements a scenario where one or more pedestrians cross the street
    exclusively through a crosswalk. The pedestrians wait until the ego vehicle
    starts moving and approaches, then cross the road.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._min_trigger_dist = get_value_parameter(config, 'trigger_distance', float, 12.0)
        self.timeout = timeout
        self._crosswalks = self._wmap.get_crosswalks()

        # Check map contains valid crosswalks
        if not self._crosswalks:
            raise ValueError("This scenario can only be run on maps containing valid crosswalks.")

        super(CrosswalkPedestrian, self).__init__("CrosswalkPedestrian",
                                                 ego_vehicles,
                                                 config,
                                                 world,
                                                 debug_mode,
                                                 criteria_enable=criteria_enable)

    def _group_crosswalks(self, locations):
        """
        Group flat list of locations into closed polygons
        """
        polygons = []
        curr = []
        for loc in locations:
            if not curr:
                curr.append(loc)
            else:
                curr.append(loc)
                if loc.distance(curr[0]) < 0.1:
                    polygons.append(curr)
                    curr = []
        return polygons

    def _initialize_actors(self, config):
        # Group crosswalks into polygons
        polygons = self._group_crosswalks(self._crosswalks)
        trigger_location = config.trigger_points[0].location
        closest_crosswalk = None
        min_dist = float('inf')

        # Find closest crosswalk to the trigger location
        for poly in polygons:
            cx = sum(pt.x for pt in poly[:-1]) / len(poly[:-1])
            cy = sum(pt.y for pt in poly[:-1]) / len(poly[:-1])
            cz = sum(pt.z for pt in poly[:-1]) / len(poly[:-1])
            center = carla.Location(cx, cy, cz)
            dist = center.distance(trigger_location)
            if dist < min_dist:
                min_dist = dist
                closest_crosswalk = poly

        if not closest_crosswalk:
            raise ValueError("Could not find a valid crosswalk polygon near trigger point.")

        self._closest_crosswalk = closest_crosswalk

        # Get vertices
        vertices = closest_crosswalk[:-1]

        # Calculate Side A and Side B of the crosswalk
        if len(vertices) != 4:
            side_a = vertices[0]
            side_b = vertices[len(vertices) // 2]
            p0 = vertices[0]
            p1 = vertices[1]
            width_vector = carla.Location(x=p1.x - p0.x, y=p1.y - p0.y, z=p1.z - p0.z)
            dist_p0_p1 = p0.distance(p1)
            if dist_p0_p1 > 0.001:
                width_vector = carla.Location(x=width_vector.x / dist_p0_p1, y=width_vector.y / dist_p0_p1, z=width_vector.z / dist_p0_p1)
            else:
                width_vector = carla.Location(0, 0, 0)
        else:
            len_01 = vertices[0].distance(vertices[1])
            len_12 = vertices[1].distance(vertices[2])
            if len_01 < len_12:
                side_a = carla.Location(
                    x=(vertices[0].x + vertices[1].x) / 2.0,
                    y=(vertices[0].y + vertices[1].y) / 2.0,
                    z=(vertices[0].z + vertices[1].z) / 2.0
                )
                side_b = carla.Location(
                    x=(vertices[2].x + vertices[3].x) / 2.0,
                    y=(vertices[2].y + vertices[3].y) / 2.0,
                    z=(vertices[2].z + vertices[3].z) / 2.0
                )
                p0 = vertices[0]
                p1 = vertices[1]
                width_vector = carla.Location(x=p1.x - p0.x, y=p1.y - p0.y, z=p1.z - p0.z)
                dist_p0_p1 = p0.distance(p1)
                if dist_p0_p1 > 0.001:
                    width_vector = carla.Location(x=width_vector.x / dist_p0_p1, y=width_vector.y / dist_p0_p1, z=width_vector.z / dist_p0_p1)
                else:
                    width_vector = carla.Location(0, 0, 0)
            else:
                side_a = carla.Location(
                    x=(vertices[3].x + vertices[0].x) / 2.0,
                    y=(vertices[3].y + vertices[0].y) / 2.0,
                    z=(vertices[3].z + vertices[0].z) / 2.0
                )
                side_b = carla.Location(
                    x=(vertices[1].x + vertices[2].x) / 2.0,
                    y=(vertices[1].y + vertices[2].y) / 2.0,
                    z=(vertices[1].z + vertices[2].z) / 2.0
                )
                p3 = vertices[3]
                p0 = vertices[0]
                width_vector = carla.Location(x=p0.x - p3.x, y=p0.y - p3.y, z=p0.z - p3.z)
                dist_p3_p0 = p3.distance(p0)
                if dist_p3_p0 > 0.001:
                    width_vector = carla.Location(x=width_vector.x / dist_p3_p0, y=width_vector.y / dist_p3_p0, z=width_vector.z / dist_p3_p0)
                else:
                    width_vector = carla.Location(0, 0, 0)

        # Store locations
        self._side_a = side_a
        self._side_b = side_b

        # Calculate orientations
        dir_1 = side_b - side_a
        yaw_1 = math.degrees(math.atan2(dir_1.y, dir_1.x))
        dir_2 = side_a - side_b
        yaw_2 = math.degrees(math.atan2(dir_2.y, dir_2.x))

        # We support dynamic walker count (from XML configuration config.other_actors or defaulting to 2)
        num_pedestrians = len(config.other_actors) if config.other_actors else 2
        
        base_speed = get_value_parameter(config, 'pedestrian_speed', float, 1.3)

        self._walker_data = []
        for i in range(num_pedestrians):
            # Alternating starting sides to prevent crowding
            if i % 2 == 0:
                # Side A -> Side B
                offset_scale = (i // 2) * 1.0 + 0.5
                start_loc = carla.Location(
                    x=side_a.x + width_vector.x * offset_scale,
                    y=side_a.y + width_vector.y * offset_scale,
                    z=side_a.z + 0.5
                )
                end_loc = carla.Location(
                    x=side_b.x + width_vector.x * offset_scale,
                    y=side_b.y + width_vector.y * offset_scale,
                    z=side_b.z + 0.5
                )
                yaw = yaw_1
            else:
                # Side B -> Side A
                offset_scale = -((i // 2) * 1.0 + 0.5)
                start_loc = carla.Location(
                    x=side_b.x + width_vector.x * offset_scale,
                    y=side_b.y + width_vector.y * offset_scale,
                    z=side_b.z + 0.5
                )
                end_loc = carla.Location(
                    x=side_a.x + width_vector.x * offset_scale,
                    y=side_a.y + width_vector.y * offset_scale,
                    z=side_a.z + 0.5
                )
                yaw = yaw_2

            distance = start_loc.distance(end_loc) + 3.0
            speed = base_speed + (i * 0.1)
            duration = distance / speed
            
            model = config.other_actors[i].model if config.other_actors else 'walker.*'

            self._walker_data.append({
                'transform': carla.Transform(start_loc, carla.Rotation(pitch=0, yaw=yaw, roll=0)),
                'speed': speed,
                'distance': distance,
                'duration': duration,
                'model': model
            })

        # Spawn the walkers
        for walker_info in self._walker_data:
            spawn_transform = walker_info['transform']
            model = walker_info['model']
            
            # Sub-loop to retry spawning with slightly increased Z coordinate to avoid ground collisions
            walker = None
            retry_transform = carla.Transform(spawn_transform.location, spawn_transform.rotation)
            for _ in range(10):
                walker = CarlaDataProvider.request_new_actor(model, retry_transform)
                if walker is not None:
                    break
                retry_transform.location.z += 0.5

            if walker is None:
                self._destroy_other_actors()
                raise ValueError("Failed to spawn a pedestrian adversary")

            walker.set_location(spawn_transform.location + carla.Location(z=-200))
            walker = self._replace_walker(walker)
            self.other_actors.append(walker)

    def _replace_walker(self, walker):
        """
        Replace walker with substitute near ego vehicle to prevent dormancy issues on large maps
        """
        type_id = walker.type_id
        CarlaDataProvider.remove_actor_by_id(walker.id)
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        walker = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not walker:
            raise ValueError("Couldn't spawn walker substitute")
        walker.set_simulate_physics(False)
        walker.set_location(spawn_transform.location + carla.Location(z=-50))
        return walker

    def _create_behavior(self):
        """
        Create behavior tree:
        - Pedestrians spawn stationary.
        - Once ego vehicle starts moving and gets within trigger distance, pedestrians cross.
        - Once ego vehicle safely passes the crosswalk, the scenario succeeds.
        """
        root = py_trees.composites.Parallel(
            name="CrosswalkPedestrianRoot",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )

        pedestrian_movement = py_trees.composites.Parallel(
            name="PedestrianMovement",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )

        # Center of crosswalk
        cx = sum(pt.x for pt in self._closest_crosswalk[:-1]) / len(self._closest_crosswalk[:-1])
        cy = sum(pt.y for pt in self._closest_crosswalk[:-1]) / len(self._closest_crosswalk[:-1])
        cz = sum(pt.z for pt in self._closest_crosswalk[:-1]) / len(self._closest_crosswalk[:-1])
        collision_location = carla.Location(cx, cy, cz)

        for walker_actor, walker_info in zip(self.other_actors, self._walker_data):
            walker_seq = py_trees.composites.Sequence(name="WalkerSeq")

            # Place walker at start and enable physics
            walker_seq.add_child(ActorTransformSetter(walker_actor, walker_info['transform'], True))

            # Wait for ego vehicle to move AND approach
            trigger = py_trees.composites.Sequence(name="TriggerWalker")
            trigger.add_child(TriggerVelocity(self.ego_vehicles[0], 0.1, comparison_operator=operator.gt))
            trigger.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], collision_location, self._min_trigger_dist))
            walker_seq.add_child(trigger)

            # Keep velocity across the road
            walker_seq.add_child(KeepVelocity(
                walker_actor,
                walker_info['speed'],
                False,
                walker_info['duration'],
                walker_info['distance']
            ))

            # Destroy walker and wait forever
            walker_seq.add_child(ActorDestroy(walker_actor, name="DestroyPedestrian"))
            walker_seq.add_child(WaitForever())

            pedestrian_movement.add_child(walker_seq)

        # Success condition: ego vehicle safely passes crosswalk
        wp = self._wmap.get_waypoint(collision_location, lane_type=carla.LaneType.Driving)
        if wp:
            crosswalk_transform = carla.Transform(collision_location, wp.transform.rotation)
        else:
            crosswalk_transform = carla.Transform(collision_location, self._reference_waypoint.transform.rotation)

        end_condition = WaitUntilInFrontPosition(self.ego_vehicles[0], crosswalk_transform, check_distance=False)

        root.add_child(pedestrian_movement)
        root.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        """
        Evaluation criteria
        """
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Cleanup all actors upon deletion
        """
        self.remove_all_actors()
