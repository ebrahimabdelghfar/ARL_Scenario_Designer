#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Pedestrians crossing through the middle of the lane.
"""

from __future__ import print_function

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      WaitForever,
                                                                      Idle,
                                                                      ActorTransformSetter,
                                                                      MovePedestrianWithEgo)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import HandleJunctionScenario


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        val = config.other_parameters[name]['value']
        if isinstance(val, str) and ',' in val:
            val = val.split(',')[0].strip()
        return p_type(val)
    else:
        return default


def convert_dict_to_location(actor_dict):
    """
    Convert a JSON string to a Carla.Location
    """
    location = carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )
    return location


class PedestrianCrossing(BasicScenario):

    """
    This class holds everything required for a group of natural pedestrians crossing the road.
    The ego vehicle is passing through a road,
    And encounters a group of pedestrians crossing the road.

    This is a single ego vehicle scenario.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._walker_amount = get_value_parameter(config, 'walker_amount', int, 1)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        self._adversary_speed = get_value_parameter(config, 'adversary_speed', float, 1.5)
        self._reaction_time = get_value_parameter(config, 'reaction_time', float, 3.5)
        self._min_trigger_dist = get_value_parameter(config, 'min_trigger_dist', float, 12.0)
        self._ego_end_distance = 40
        self.timeout = timeout

        self._walker_data = []

        super().__init__("PedestrianCrossing",
                          ego_vehicles,
                          config,
                          world,
                          debug_mode,
                          criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Get the start point of the initial pedestrian
        collision_wp = self._reference_waypoint
        while True:
            next_wps = collision_wp.next(1)
            if not next_wps:
                raise ValueError("Couldn't find a waypoint to spawn the pedestrians")
            if next_wps[0].is_junction:
                break
            collision_wp = next_wps[0]

        self._collision_wp = collision_wp

        # Get start/end locations from the closest crosswalk
        ref_loc = collision_wp.transform.location
        crosswalks = self._wmap.get_crosswalks()
        use_crosswalk = False
        start_loc = None
        end_loc = None

        if crosswalks:
            # Find closest crosswalk polygon to the collision waypoint
            closest_polygon = min(
                crosswalks,
                key=lambda poly: min(pt.distance(ref_loc) for pt in poly)
            )
            # Ensure it is close enough to be our target crosswalk (e.g., within 30m)
            min_dist = min(pt.distance(ref_loc) for pt in closest_polygon)
            if min_dist < 30.0:
                points = closest_polygon[:-1] if closest_polygon[0] == closest_polygon[-1] else closest_polygon
                n = len(points)
                if n >= 4:
                    # Find adjacent edges and their lengths
                    edges = []
                    for i in range(n):
                        p1 = points[i]
                        p2 = points[(i + 1) % n]
                        edges.append((p1, p2, p1.distance(p2)))
                    edges.sort(key=lambda edge: edge[2])

                    # The two shortest edges are typically the sidewalk ends of the crosswalk
                    short_edge_1 = edges[0]
                    short_edge_2 = edges[1]

                    # Calculate midpoints of the short edges
                    mid_1 = carla.Location(
                        x=(short_edge_1[0].x + short_edge_1[1].x) / 2.0,
                        y=(short_edge_1[0].y + short_edge_1[1].y) / 2.0,
                        z=(short_edge_1[0].z + short_edge_1[1].z) / 2.0
                    )
                    mid_2 = carla.Location(
                        x=(short_edge_2[0].x + short_edge_2[1].x) / 2.0,
                        y=(short_edge_2[0].y + short_edge_2[1].y) / 2.0,
                        z=(short_edge_2[0].z + short_edge_2[1].z) / 2.0
                    )

                    # Project onto right vector of collision_wp
                    right_vec = collision_wp.transform.get_right_vector()
                    vec_1 = mid_1 - ref_loc
                    dot_1 = vec_1.x * right_vec.x + vec_1.y * right_vec.y + vec_1.z * right_vec.z

                    # If dot_1 > 0, mid_1 is to the right. Otherwise it is to the left.
                    if self._direction == 'left':
                        if dot_1 < 0:
                            start_loc, end_loc = mid_1, mid_2
                        else:
                            start_loc, end_loc = mid_2, mid_1
                    else:  # 'right'
                        if dot_1 > 0:
                            start_loc, end_loc = mid_1, mid_2
                        else:
                            start_loc, end_loc = mid_2, mid_1

                    use_crosswalk = True

        if not use_crosswalk:
            # Fallback to sidewalk-based waypoint calculation
            start_wp = collision_wp
            while start_wp.lane_type != carla.LaneType.Sidewalk:
                wp = start_wp.get_right_lane() if self._direction == 'right' else start_wp.get_left_lane()
                if wp is None:
                    raise ValueError("Couldn't find a sidewalk waypoint to start fallback")
                start_wp = wp

            # Since there is no crosswalk polygon, compute start and end based on sidewalk displacement
            opposite_wp = collision_wp
            while opposite_wp.lane_type != carla.LaneType.Sidewalk:
                wp = opposite_wp.get_left_lane() if self._direction == 'right' else opposite_wp.get_right_lane()
                if wp is None:
                    break
                opposite_wp = wp

            start_loc = start_wp.transform.location
            end_loc = opposite_wp.transform.location

        # Now spawn the walkers based on start_loc and end_loc
        crossing_dir = end_loc - start_loc
        crossing_len = math.sqrt(crossing_dir.x**2 + crossing_dir.y**2 + crossing_dir.z**2)
        if crossing_len > 0:
            v_cross = carla.Location(crossing_dir.x / crossing_len, crossing_dir.y / crossing_len, crossing_dir.z / crossing_len)
        else:
            v_cross = carla.Location(1.0, 0.0, 0.0)

        v_road = collision_wp.transform.get_forward_vector()

        self._walker_data = []

        for i in range(self._walker_amount):
            w_start_loc = start_loc + v_road * (i * 0.6) + v_cross * (i * 0.2)
            w_end_loc = end_loc + v_road * (i * 0.6) + v_cross * (i * 0.2)

            move_vec = w_end_loc - w_start_loc
            move_dist = math.sqrt(move_vec.x**2 + move_vec.y**2 + move_vec.z**2)

            speed = self._rng.uniform(self._adversary_speed - 0.2, self._adversary_speed + 0.3)
            idle_time = self._rng.uniform(0.0, 1.5)
            duration = move_dist / speed

            yaw = math.degrees(math.atan2(move_vec.y, move_vec.x))
            spawn_rotation = carla.Rotation(pitch=0, yaw=yaw, roll=0)
            spawn_transform = carla.Transform(w_start_loc + carla.Location(z=1.0), spawn_rotation)

            walker = CarlaDataProvider.request_new_actor('walker.*', spawn_transform)
            if walker is None:
                self._destroy_other_actors()
                raise ValueError("Failed to spawn walker")

            walker.set_location(spawn_transform.location + carla.Location(z=-200))
            walker = self._replace_walker(walker)
            self.other_actors.append(walker)

            self._walker_data.append({
                'transform': spawn_transform,
                'distance': move_dist,
                'speed': speed,
                'duration': duration,
                'idle_time': idle_time
            })

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="PedestrianCrossing")
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=True,
                remove_entries=[],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=0
            ))

        for walker_actor, walker_data in zip(self.other_actors, self._walker_data):
            sequence.add_child(ActorTransformSetter(walker_actor, walker_data['transform'], True))

        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the walkers
        main_behavior = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="WalkerMovement")

        for walker_actor, walker_data in zip(self.other_actors, self._walker_data):
            walker_sequence = py_trees.composites.Sequence(name="WalkerCrossing")
            walker_sequence.add_child(Idle(walker_data['idle_time']))
            walker_sequence.add_child(KeepVelocity(
                walker_actor, walker_data['speed'], False, walker_data['duration'], walker_data['distance']))
            walker_sequence.add_child(ActorDestroy(walker_actor, name="DestroyAdversary"))
            walker_sequence.add_child(WaitForever())

            main_behavior.add_child(walker_sequence)

        main_behavior.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))
        sequence.add_child(main_behavior)

        # Remove everything

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    # TODO: Pedestrian have an issue with large maps were setting them to dormant breaks them,
    # so all functions below are meant to patch it until the fix is done
    def _replace_walker(self, walker):
        """As the adversary is probably, replace it with another one"""
        type_id = walker.type_id
        CarlaDataProvider.remove_actor_by_id(walker.id)
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        walker = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not walker:
            raise ValueError("Couldn't spawn the walker substitute")
        walker.set_simulate_physics(False)
        walker.set_location(spawn_transform.location + carla.Location(z=-50))
        return walker

    def _setup_scenario_trigger(self, config):
        """Normal scenario trigger but in parallel, a behavior that ensures the pedestrian stays active"""
        trigger_tree = super()._setup_scenario_trigger(config)

        if not self.route_mode:
            return trigger_tree

        parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="ScenarioTrigger")

        for _, walker in enumerate(reversed(self.other_actors)):
            parallel.add_child(MovePedestrianWithEgo(self.ego_vehicles[0], walker, 100))

        parallel.add_child(trigger_tree)
        return parallel
