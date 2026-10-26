#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for an XML-based scenario
"""

from networkx.generators import spectral_graph_forge
import carla


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', speed=0, autopilot=False,
                 random=False, color=None, category="car", args=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.speed = speed
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.category = category
        self.args = args

    @staticmethod
    def parse_from_node(node, rolename):
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = node.attrib.get('model', 'vehicle.*')

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = node.attrib.get('rolename', rolename)

        speed = node.attrib.get('speed', 0)

        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        args = {}
        if 'use_wheelchair' in node.attrib:
            val = node.attrib['use_wheelchair']
            args['use_wheelchair'] = val.lower() in ('true', '1', 'yes')

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color, args=args)

    @staticmethod
    def parse_from_dict(actor_dict, rolename):
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = actor_dict['model'] if 'model' in actor_dict else 'vehicle.*'

        pos_x = float(actor_dict['x']) if 'x' in actor_dict else 0
        pos_y = float(actor_dict['y']) if 'y' in actor_dict else 0
        pos_z = float(actor_dict['z']) if 'z' in actor_dict else 0
        yaw = float(actor_dict['yaw']) if 'yaw' in actor_dict else 0
        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = actor_dict['rolename'] if 'rolename' in actor_dict else rolename
        speed = actor_dict['speed'] if 'speed' in actor_dict else 0
        autopilot = actor_dict['autopilot'] if 'autopilot' in actor_dict else False
        random_location = actor_dict['random_location'] if 'random_location' in actor_dict else False
        color = actor_dict['color'] if 'color' in actor_dict else None

        args = {}
        if 'use_wheelchair' in actor_dict:
            args['use_wheelchair'] = actor_dict['use_wheelchair']

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color, args=args)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    def __init__(self):
        self.trigger_points = []
        self.ego_vehicles = []
        self.other_actors = []
        self.other_parameters = {}
        self.town = None
        self.name = None
        self.type = None
        self._start_distance = None
        self._start_distance2 = None
        self._start_distance3 = None
        self._actor_vel = None
        self._actor_vel2 = None
        self._actor_vel3 = None
        self._trigger_distance = None
        self._trigger_distance2 = None
        self.route = None
        self.agent = None
        self.weather = carla.WeatherParameters(sun_altitude_angle=70, cloudiness=50)
        self.friction = None
        self.subtype = None
        self.route_var_name = None
        # actor's braking coefficient [0-1]
        self._brake = None
        # model ped=0 or bike=1
        self._model = None
        # control_loss noise for steer [-0.5,0.5]
        self._new_steer_noise = None
        self._new_steer_noise2 = None
        # control_loss noise for throttle [0-1]
        self._new_throttle_noise = None
        # _traffic_distance m
        self._traffic_distance = None
