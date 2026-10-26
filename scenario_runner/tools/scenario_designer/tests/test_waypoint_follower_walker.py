"""WaypointFollower walker plan termination, no CARLA server needed.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tools/scenario_designer/tests/test_waypoint_follower_walker.py
"""
import os
import sys

SR_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if SR_ROOT not in sys.path:
    sys.path.insert(0, SR_ROOT)

import carla
import py_trees
import pytest

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics import atomic_behaviors as ab

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS


class FakeWalker:
    type_id = "walker.pedestrian.0001"
    is_alive = True

    def __init__(self, actor_id):
        self.id = actor_id
        self.loc = carla.Location(0, 0, 0)
        self.control = carla.WalkerControl()

    def get_control(self):
        return self.control

    def apply_control(self, control):
        self.control = control


def _patch(monkeypatch):
    monkeypatch.setattr(ab.carla, "Walker", FakeWalker)
    monkeypatch.setattr(CarlaDataProvider, "get_location", staticmethod(lambda a: a.loc))
    monkeypatch.setattr(CarlaDataProvider, "get_transform",
                        staticmethod(lambda a: carla.Transform(a.loc)))


def test_walker_stops_and_succeeds_at_end_of_plan(monkeypatch):
    _patch(monkeypatch)
    walker = FakeWalker(9001)
    wf = ab.WaypointFollower(walker, 1.4, plan=[carla.Location(5, 0, 0), carla.Location(10, 0, 0)])
    wf.initialise()

    assert wf.update() == RUNNING
    assert walker.control.speed == pytest.approx(1.4)
    walker.loc = carla.Location(5.5, 0, 0)   # reached first waypoint
    assert wf.update() == RUNNING
    walker.loc = carla.Location(10.5, 0, 0)  # reached last waypoint
    assert wf.update() == SUCCESS
    # Ticked again after the end (e.g. sibling actors still running): stays stopped
    assert wf.update() == SUCCESS
    assert walker.control.speed == 0


def test_walker_without_plan_walks_forever(monkeypatch):
    _patch(monkeypatch)
    walker = FakeWalker(9002)
    wf = ab.WaypointFollower(walker, 1.4)
    wf.initialise()
    for _ in range(5):
        assert wf.update() == RUNNING
        assert walker.control.speed == pytest.approx(1.4)
