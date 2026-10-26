#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Change lane scenario:

The scenario realizes a driving behavior, in which the user-controlled ego vehicle
follows a fast driving car on the highway. There's a slow car driving in great distance to the fast vehicle.
At one point the fast vehicle is changing the lane to overtake a slow car, which is driving on the same lane.

The ego vehicle doesn't "see" the slow car before the lane change of the fast car, therefore it hast to react
fast to avoid an collision. There are two options to avoid an accident:
The ego vehicle adjusts its velocity or changes the lane as well.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, StandStill, WaitUntilInFront
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        val = config.other_parameters[name]['value']
        if isinstance(val, str) and ',' in val:
            val = val.split(',')[0].strip()
        return p_type(val)
    else:
        return default


def get_list_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        val_str = config.other_parameters[name]['value']
        return [p_type(x.strip()) for x in val_str.split(',')]
    else:
        return default


class ChangeLane(BasicScenario):

    """
    This class holds everything required for a "change lane" scenario involving three or more vehicles.
    There are multiple vehicles driving in the same direction on the highway: A fast car and one or more slow cars in front.
    The fast car will change the lane, when it is close to the first slow car.

    The ego vehicle is driving right behind the fast car.

    This is a single ego vehicle scenario
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._fast_vehicle_velocity = get_value_parameter(config, 'fast_vehicle_velocity', float, 70) / 3.6
        self._slow_vehicle_velocity = get_value_parameter(config, 'slow_vehicle_velocity', float, 0) / 3.6
        self._change_lane_velocity = get_value_parameter(config, 'change_lane_velocity', float, 36) / 3.6

        self._slow_vehicle_distance = get_value_parameter(config, 'slow_vehicle_distance', float, 100)
        self._fast_vehicle_distance = get_value_parameter(config, 'fast_vehicle_distance', float, 20)
        self._trigger_distance = get_value_parameter(config, 'trigger_distance', float, 30)
        self._max_brake = get_value_parameter(config, 'max_brake', float, 1)

        self.direction = get_value_parameter(config, 'direction', str, 'left')
        self.lane_check = get_value_parameter(config, 'lane_check', str, 'true')

        self._distance_lane_change = get_value_parameter(config, 'distance_lane_change', float, 35)
        self._kp = get_value_parameter(config, 'k_p', float, 0.5)
        self._kd = get_value_parameter(config, 'k_d', float, 0.05)

        self._slow_vehicle_distances = []
        self._slow_vehicle_velocities = []

        super(ChangeLane, self).__init__("ChangeLane",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        if randomize:
            self._fast_vehicle_distance = random.randint(10, 51)
            self._fast_vehicle_velocity = random.randint(100, 201)
            if len(self._slow_vehicle_velocities) > 0:
                self._slow_vehicle_velocities[0] = random.randint(1, 6)

    def _initialize_actors(self, config):

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        num_actors = len(self.other_actors)
        if num_actors < 2:
            raise ValueError("ChangeLane scenario requires at least 2 other actors (one fast, one slow)")

        # fast vehicle, tesla
        # transform visible
        fast_car_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._fast_vehicle_distance)
        self.fast_car_visible = carla.Transform(
            carla.Location(fast_car_waypoint.transform.location.x,
                           fast_car_waypoint.transform.location.y,
                           fast_car_waypoint.transform.location.z + 1),
            fast_car_waypoint.transform.rotation)

        # parse slow vehicle distances and velocities
        default_slow_distances = [self._slow_vehicle_distance + i * 20 for i in range(num_actors - 1)]
        self._slow_vehicle_distances = get_list_parameter(config, 'slow_vehicle_distance', float, default_slow_distances)
        while len(self._slow_vehicle_distances) < num_actors - 1:
            last_dist = self._slow_vehicle_distances[-1] if self._slow_vehicle_distances else self._slow_vehicle_distance
            self._slow_vehicle_distances.append(last_dist + 20)

        default_slow_velocities = [self._slow_vehicle_velocity * 3.6 for _ in range(num_actors - 1)]
        self._slow_vehicle_velocities = get_list_parameter(config, 'slow_vehicle_velocity', float, default_slow_velocities)
        self._slow_vehicle_velocities = [v / 3.6 for v in self._slow_vehicle_velocities]
        while len(self._slow_vehicle_velocities) < num_actors - 1:
            self._slow_vehicle_velocities.append(self._slow_vehicle_velocity)

        # slow vehicles
        # transforms visible
        self.slow_cars_visible = []
        for i in range(num_actors - 1):
            slow_car_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._slow_vehicle_distances[i])
            transform = carla.Transform(
                carla.Location(slow_car_waypoint.transform.location.x,
                               slow_car_waypoint.transform.location.y,
                               slow_car_waypoint.transform.location.z),
                slow_car_waypoint.transform.rotation)
            self.slow_cars_visible.append(transform)

    def _create_behavior(self):
        num_actors = len(self.other_actors)

        # Behavior for all slow vehicles
        slow_vehicles_behavior = py_trees.composites.Parallel(
            "Slow Vehicles", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        for i in range(1, num_actors):
            actor = self.other_actors[i]
            actor_visible_transform = self.slow_cars_visible[i - 1]
            velocity = self._slow_vehicle_velocities[i - 1]

            seq = py_trees.composites.Sequence(f"Slow Vehicle {i}")
            # Make visible
            seq.add_child(ActorTransformSetter(actor, actor_visible_transform))

            # Behavior based on velocity
            if velocity > 0:
                # Drive at specified velocity
                seq.add_child(WaypointFollower(actor, velocity))
            else:
                # Stop to avoid rolling backward, then idle
                seq.add_child(StopVehicle(actor, self._max_brake))
                seq.add_child(Idle())
            slow_vehicles_behavior.add_child(seq)

        # sequence tesla
        # make visible
        sequence_tesla = py_trees.composites.Sequence("Tesla")
        tesla_visible = ActorTransformSetter(self.other_actors[0], self.fast_car_visible)
        sequence_tesla.add_child(tesla_visible)

        # drive fast towards slow vehicle
        just_drive = py_trees.composites.Parallel("DrivingTowardsSlowVehicle",
                                                  policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        tesla_driving_fast = WaypointFollower(self.other_actors[0], self._fast_vehicle_velocity)
        just_drive.add_child(tesla_driving_fast)
        distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[1], self.other_actors[0], self._trigger_distance)
        just_drive.add_child(distance_to_vehicle)
        sequence_tesla.add_child(just_drive)

        # change lane
        lane_check_bool = self.lane_check.lower() in ('true', '1', 'yes')
        lateral_args = {'K_P': self._kp, 'K_D': self._kd, 'K_I': 0.0, 'dt': 0.05}
        lane_change_atomic = LaneChange(self.other_actors[0],
                                        speed=self._change_lane_velocity,
                                        direction=self.direction,
                                        distance_other_lane=200,
                                        distance_lane_change=self._distance_lane_change,
                                        check=lane_check_bool,
                                        args_lateral_dict=lateral_args)
        sequence_tesla.add_child(lane_change_atomic)
        sequence_tesla.add_child(Idle())

        # ego vehicle
        # end condition: success after all other vehicles are passed by ego vehicle
        endcondition = py_trees.composites.Parallel("Waiting for ego to pass all other vehicles",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        for i, actor in enumerate(self.other_actors):
            endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0], actor, factor=1.0, check_distance=False, name=f"PassVehicle{i}"))

        # build tree
        root = py_trees.composites.Parallel("Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(slow_vehicles_behavior)
        root.add_child(sequence_tesla)
        root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
