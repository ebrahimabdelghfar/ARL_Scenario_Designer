#!/usr/bin/env python

# Copyright (c) 2026 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Mock ConstantVelocityAgent for unittesting
"""

class ConstantVelocityAgent(object):
    def __init__(self, *args, **kwargs):
        pass
    def set_target_speed(self, speed):
        pass
    def run_step(self):
        import carla
        return carla.VehicleControl()
    def destroy_sensor(self):
        pass
