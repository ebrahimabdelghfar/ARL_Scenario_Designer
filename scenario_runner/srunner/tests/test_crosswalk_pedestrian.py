#!/usr/bin/env python

# Copyright (c) 2026 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Unit tests for CrosswalkPedestrian scenario.
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch

# Ensure srunner and tests/carla_mocks are in python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'carla_mocks')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import carla
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.crosswalk_pedestrian import CrosswalkPedestrian


class MockWaypoint(object):
    def __init__(self, location, yaw=90):
        self.transform = carla.Transform(location, carla.Rotation(yaw=yaw))
        self.lane_width = 3.5


class MockMap(object):
    def __init__(self):
        self.name = "MockMap"

    def get_crosswalks(self):
        # Closed polygon loop representing crosswalk
        return [
            carla.Location(-47.0, 0.0, 0.0),
            carla.Location(-47.0, 5.0, 0.0),
            carla.Location(-57.0, 5.0, 0.0),
            carla.Location(-57.0, 0.0, 0.0),
            carla.Location(-47.0, 0.0, 0.0),
        ]

    def get_waypoint(self, location, lane_type=None):
        return MockWaypoint(location, yaw=90)


class TestCrosswalkPedestrian(unittest.TestCase):

    def setUp(self):
        self.client = carla.Client()
        self.world = self.client.get_world()
        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)

        # Mock Map
        self.mock_map = MockMap()
        CarlaDataProvider.get_map = MagicMock(return_value=self.mock_map)

        # Setup scenario configuration
        self.config = ScenarioConfiguration()
        self.config.name = "CrosswalkPedestrian_1"
        self.config.town = "Town10HD"
        
        # Trigger point
        trigger_point = carla.Transform(carla.Location(-48.84, -17.07, 0.1), carla.Rotation(yaw=90))
        self.config.trigger_points = [trigger_point]
        
        # Ego vehicle configuration
        ego_actor = ActorConfigurationData('vehicle.lincoln.mkz_2017', trigger_point, 'hero')
        self.config.ego_vehicles = [ego_actor]

    def tearDown(self):
        CarlaDataProvider.cleanup()

    @patch('srunner.scenarios.crosswalk_pedestrian.CarlaDataProvider.request_new_actor')
    def test_scenario_creation(self, mock_request_actor):
        # Mock spawning actors
        mock_actor = carla.Vehicle()
        mock_actor.id = 123
        mock_actor.type_id = 'walker.pedestrian.0001'
        mock_actor.get_transform = MagicMock(return_value=carla.Transform(carla.Location(0,0,0), carla.Rotation()))
        mock_actor.get_location = MagicMock(return_value=carla.Location(0,0,0))
        mock_request_actor.return_value = mock_actor

        ego_vehicle = carla.Vehicle()
        ego_vehicle.id = 999
        ego_vehicle.get_transform = MagicMock(return_value=self.config.trigger_points[0])
        ego_vehicles = [ego_vehicle]

        # Initialize scenario
        scenario = CrosswalkPedestrian(
            world=self.world,
            ego_vehicles=ego_vehicles,
            config=self.config,
            criteria_enable=True,
            timeout=60
        )

        self.assertIsNotNone(scenario)
        # By default, we expect 2 walkers to be spawned
        self.assertEqual(len(scenario.other_actors), 2)
        # Check behavior tree
        self.assertIsNotNone(scenario.behavior_node)
        # Check criteria tree
        self.assertIsNotNone(scenario.criteria_node)


if __name__ == '__main__':
    unittest.main()
