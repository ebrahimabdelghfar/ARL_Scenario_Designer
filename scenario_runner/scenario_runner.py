#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Welcome to CARLA scenario_runner

This is the main script to be executed when running a scenario.
It loads the scenario configuration, loads the scenario and manager,
and finally triggers the scenario execution.
"""

from __future__ import print_function

import glob
import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
try:
    from packaging.version import Version
except ImportError:
    from distutils.version import LooseVersion as Version  # Python 2 fallback
import importlib
import inspect
import os
import signal
import sys
import time
import json

# Auto-configure SCENARIO_RUNNER_ROOT to current script directory if not properly set
_current_dir = os.path.dirname(os.path.abspath(__file__))
_env_root = os.getenv('SCENARIO_RUNNER_ROOT')
if not _env_root or not os.path.isdir(_env_root) or not os.path.isdir(os.path.join(_env_root, 'srunner')):
    os.environ['SCENARIO_RUNNER_ROOT'] = _current_dir


try:
    # requires Python 3.8+
    from importlib.metadata import metadata

    def get_carla_version():
        return Version(metadata("carla")["Version"])
except ModuleNotFoundError:
    # backport checking for older Python versions; module is deprecated
    import pkg_resources

    def get_carla_version():
        return Version(pkg_resources.get_distribution("carla").version)

import carla

from srunner.scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenarios.open_scenario import OpenScenario
from srunner.scenarios.route_scenario import RouteScenario
from srunner.tools.scenario_parser import ScenarioConfigurationParser
from srunner.tools.route_parser import RouteParser
from srunner.tools.osc2_helper import OSC2Helper
from srunner.scenarios.osc2_scenario import OSC2Scenario
from srunner.scenarioconfigs.osc2_scenario_configuration import OSC2ScenarioConfiguration

# Minimum version of CARLA that is required
MIN_CARLA_VERSION = '0.9.16'


class ScenarioRunner(object):

    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run()
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    wait_for_world = 20.0  # in seconds

    # CARLA world and scenario handlers
    world = None
    manager = None

    finished = False

    additional_scenario_module = None

    agent_instance = None
    module_agent = None

    def __init__(self, args):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self._args = args

        if args.timeout:
            self.client_timeout = float(args.timeout)

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(self.client_timeout)
        # --waitForEgo: the ROS 2 bridge is the sync master and ticks the world.
        # Set before any set_world() (the OpenSCENARIO/OSC2 configs call it too).
        CarlaDataProvider.set_external_ticker(args.waitForEgo)
        carla_version = get_carla_version()
        if carla_version < Version(MIN_CARLA_VERSION):
            raise ImportError(
                "CARLA version {} or newer required. CARLA version found: {}".format(
                    MIN_CARLA_VERSION, carla_version
                )
            )

        # Load agent if requested via command line args
        # If something goes wrong an exception will be thrown by importlib (ok here)
        if self._args.agent is not None:
            module_name = os.path.basename(args.agent).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.agent))
            self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(self._args.debug, self._args.sync, self._args.timeout)

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != 'win32':
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.now()

    def destroy(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self._cleanup()
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world
        if self.client is not None:
            del self.client

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._shutdown_requested = True
        if self.manager:
            self.manager.stop_scenario()
            self._cleanup()
            if not self.manager.get_running_status():
                raise RuntimeError("Timeout occurred during scenario execution")

    def _get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, exit script
        """

        # Path of all scenario at "srunner/scenarios" folder + the path of the additional scenario argument
        scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        scenarios_list.append(self._args.additionalScenario)

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                if scenario in member:
                    # Ignore imported atomic behaviors that share the same name as scenarios (e.g. CutIn)
                    if "scenarioatomics" not in getattr(member[1], "__module__", ""):
                        return member[1]

            # Remove unused Python paths
            sys.path.pop(0)

        print("Scenario '{}' not supported ... Exiting".format(scenario))
        sys.exit(-1)

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        if self.finished:
            return

        self.finished = True

        # Always shut down our Traffic Manager to release the port
        try:
            tm = self.client.get_trafficmanager(int(self._args.trafficManagerPort))
            if self._args.sync:
                tm.set_synchronous_mode(False)
            tm.shut_down()
        except Exception:
            pass

        # Simulation still running and in synchronous mode?
        if self.world is not None and self._args.sync:
            try:
                # Reset to asynchronous mode
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
            except RuntimeError:
                sys.exit(-1)

        # Prevent ego vehicles from being destroyed by CarlaDataProvider.cleanup()
        if self._args.waitForEgo:
            try:
                import srunner.scenariomanager.carla_data_provider as cdp
                for ego in self.ego_vehicles:
                    if ego is not None:
                        cdp.CarlaDataProvider._carla_actor_pool.pop(ego.id, None)
            except Exception:
                pass

        if self.manager:
            try:
                self.manager.cleanup()
            except Exception:
                pass

        # Clean up criteria sensors to prevent Invalid session stream errors
        if self.manager and self.manager.scenario:
            try:
                criteria = self.manager.scenario.get_criteria()
                for criterion in criteria:
                    for attr_name in dir(criterion):
                        if attr_name.startswith('_') and not attr_name.startswith('__'):
                            attr = getattr(criterion, attr_name, None)
                            if isinstance(attr, carla.Actor):
                                if attr.type_id.startswith('sensor.'):
                                    if hasattr(attr, 'stop'):
                                        try:
                                            attr.stop()
                                        except (RuntimeError, AttributeError):
                                            pass
                                try:
                                    attr.destroy()
                                except RuntimeError:
                                    pass
                                setattr(criterion, attr_name, None)
            except Exception:
                pass

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if not self._args.waitForEgo and self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                else:
                    # Clean up the ego vehicle from CarlaDataProvider maps to avoid KeyError on next run
                    import srunner.scenariomanager.carla_data_provider as cdp
                    for map_name in ['_actor_velocity_map', '_actor_location_map', '_actor_transform_map']:
                        d = getattr(cdp.CarlaDataProvider, map_name)
                        keys_to_remove = [k for k in d if k == self.ego_vehicles[i] or k.id == self.ego_vehicles[i].id]
                        for k in keys_to_remove:
                            d.pop(k, None)
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None

    def _prepare_ego_vehicles(self, ego_vehicles):
        """
        Spawn or update the ego vehicles
        """

        if not self._args.waitForEgo:
            for vehicle in ego_vehicles:
                self.ego_vehicles.append(CarlaDataProvider.request_new_actor(vehicle.model,
                                                                             vehicle.transform,
                                                                             vehicle.rolename,
                                                                             random_location=vehicle.random_location,
                                                                             color=vehicle.color,
                                                                             actor_category=vehicle.category))
        else:
            ego_vehicle_missing = True
            while ego_vehicle_missing:
                self.ego_vehicles = []
                ego_vehicle_missing = False
                for ego_vehicle in ego_vehicles:
                    ego_vehicle_found = False
                    carla_vehicles = CarlaDataProvider.get_world().get_actors().filter('vehicle.*')
                    for carla_vehicle in carla_vehicles:
                        if carla_vehicle.attributes['role_name'] == ego_vehicle.rolename:
                            ego_vehicle_found = True
                            self.ego_vehicles.append(carla_vehicle)
                            break
                    if not ego_vehicle_found:
                        ego_vehicle_missing = True
                        break

            for i, _ in enumerate(self.ego_vehicles):
                # No set_simulate_physics(False) here: toggling physics on the
                # bridge-spawned ego segfaults the CARLA server (Signal 11).
                # set_transform alone teleports it fine.
                self.ego_vehicles[i].set_transform(ego_vehicles[i].transform)
                
                # Wait for the transform to take effect on the server
                target_loc = ego_vehicles[i].transform.location
                for _ in range(20):
                    if CarlaDataProvider.is_sync_mode():
                        self.world.tick()
                    else:
                        self.world.wait_for_tick()
                    current_loc = self.ego_vehicles[i].get_transform().location
                    if current_loc.distance(target_loc) < 2.0:
                        break

                self.ego_vehicles[i].set_target_velocity(carla.Vector3D())
                self.ego_vehicles[i].set_target_angular_velocity(carla.Vector3D())
                self.ego_vehicles[i].apply_control(carla.VehicleControl())
                CarlaDataProvider.register_actor(self.ego_vehicles[i], self.ego_vehicles[i].get_transform())

        # sync state
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _analyze_scenario(self, config):
        """
        Provide feedback about success/failure of a scenario
        """

        # Create the filename
        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        json_filename = None
        config_name = config.name
        if self._args.outputDir != '':
            config_name = os.path.join(self._args.outputDir, config_name)

        if self._args.junit:
            junit_filename = config_name + current_time + ".xml"
        if self._args.json:
            json_filename = config_name + current_time + ".json"
        filename = None
        if self._args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(self._args.output, filename, junit_filename, json_filename):
            print("All scenario tests were passed successfully!")
        else:
            print("Not all scenario tests were successful")
            if not (self._args.output or filename or junit_filename):
                print("Please run with --output for further information")

    def _record_criteria(self, criteria, name):
        """
        Filter the JSON serializable attributes of the criterias and
        dumps them into a file. This will be used by the metrics manager,
        in case the user wants specific information about the criterias.
        """
        file_name = name[:-4] + ".json"

        # Filter the attributes that aren't JSON serializable
        with open('temp.json', 'w', encoding='utf-8') as fp:

            criteria_dict = {}
            for criterion in criteria:

                criterion_dict = criterion.__dict__
                criteria_dict[criterion.name] = {}

                for key in criterion_dict:
                    if key != "name":
                        try:
                            key_dict = {key: criterion_dict[key]}
                            json.dump(key_dict, fp, sort_keys=False, indent=4)
                            criteria_dict[criterion.name].update(key_dict)
                        except TypeError:
                            pass

        os.remove('temp.json')

        # Save the criteria dictionary into a .json file
        with open(file_name, 'w', encoding='utf-8') as fp:
            json.dump(criteria_dict, fp, sort_keys=False, indent=4)

    # ROS 2 lifecycle node name of the CARLA bridge (override with CARLA_BRIDGE_NODE).
    # ScenarioRunner coordinates world reloads via lifecycle transitions
    # to prevent "Invalid session: no stream available" errors.
    _BRIDGE_LIFECYCLE_NODE = os.environ.get("CARLA_BRIDGE_NODE", "/carla_bridge_node")

    @staticmethod
    def _ros2_lifecycle_set(node_name, transition, timeout=15):
        """
        Trigger a ROS 2 lifecycle transition via rclpy API.

        Args:
            node_name: Fully-qualified lifecycle node name.
            transition: Target state string (e.g. "deactivate", "cleanup",
                        "configure", "activate").
            timeout: Seconds to wait for the command to complete.

        Returns:
            True if the transition succeeded, False otherwise.
        """
        import rclpy
        from rclpy.node import Node
        from lifecycle_msgs.srv import ChangeState
        from lifecycle_msgs.msg import Transition
        
        print("[ScenarioRunner] Lifecycle transition: {} → {}".format(node_name, transition))
        
        shutdown_after = False
        if not rclpy.ok():
            try:
                rclpy.init()
                shutdown_after = True
            except Exception:
                pass
                
        node = None
        try:
            node = Node('scenario_runner_lifecycle_setter')
            client = node.create_client(ChangeState, f'{node_name}/change_state')
            
            if not client.wait_for_service(timeout_sec=5.0):
                print(f"[ScenarioRunner] Service {node_name}/change_state not available")
                return False
                
            req = ChangeState.Request()
            label_to_id = {
                'configure': Transition.TRANSITION_CONFIGURE,
                'cleanup': Transition.TRANSITION_CLEANUP,
                'activate': Transition.TRANSITION_ACTIVATE,
                'deactivate': Transition.TRANSITION_DEACTIVATE,
                'unconfigured_shutdown': Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
                'inactive_shutdown': Transition.TRANSITION_INACTIVE_SHUTDOWN,
                'active_shutdown': Transition.TRANSITION_ACTIVE_SHUTDOWN,
                'destroy': Transition.TRANSITION_DESTROY,
            }
            req.transition.id = label_to_id.get(transition, 0)
            req.transition.label = transition
            
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                success = future.result().success
                if not success:
                    print(f"[ScenarioRunner] Lifecycle transition '{transition}' failed")
                return success
        except Exception as e:
            print(f"[ScenarioRunner] WARNING: Lifecycle transition '{transition}' error: {e}")
        finally:
            if node is not None:
                node.destroy_node()
        return False

    @staticmethod
    def _poll_lifecycle_state(node_name, expected_state, timeout=30, poll_interval=1.0):
        """
        Poll ROS 2 lifecycle node until it reaches the expected state.

        Args:
            node_name: Fully-qualified lifecycle node name.
            expected_state: Target state string (e.g. "unconfigured", "inactive",
                            "active", "finalized").
            timeout: Max seconds to wait.
            poll_interval: Seconds between polls.

        Returns:
            True if expected state reached, False on timeout/error.
        """
        waited = 0.0
        while waited < timeout:
            state = ScenarioRunner._get_lifecycle_state(node_name)
            if state is not None and expected_state.lower() == state:
                print("[ScenarioRunner] Bridge state confirmed: {}".format(expected_state))
                return True
            time.sleep(poll_interval)
            waited += poll_interval
            
        print("[ScenarioRunner] WARNING: Bridge did not reach '{}' within {}s".format(
            expected_state, timeout))
        return False

    @staticmethod
    def _get_lifecycle_state(node_name):
        """
        Get current ROS 2 lifecycle state of the node.
        """
        import rclpy
        from rclpy.node import Node
        from lifecycle_msgs.srv import GetState

        shutdown_after = False
        if not rclpy.ok():
            try:
                rclpy.init()
                shutdown_after = True
            except Exception:
                pass

        node = None
        try:
            node = Node('scenario_runner_lifecycle_client')
            client = node.create_client(GetState, f'{node_name}/get_state')
            
            if not client.wait_for_service(timeout_sec=2.0):
                return None
                
            request = GetState.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            
            state = None
            if future.result() is not None:
                state = future.result().current_state.label.lower()
            return state
        except Exception:
            pass
        finally:
            if node is not None:
                node.destroy_node()
        return None

    @staticmethod
    def _ros2_set_map_parameter(node_name, town, timeout=10):
        """
        Use the ROS 2 SetParameters service to update the bridge's
        'world_town' parameter so it connects to the correct map on
        the next configure cycle.

        Args:
            node_name: Fully-qualified lifecycle node name.
            town: Target CARLA map name (e.g. "Town03").
            timeout: Seconds to wait for the service to be available.

        Returns:
            True if the parameter was set successfully, False otherwise.
        """
        import rclpy
        from rclpy.node import Node
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        print(f"[ScenarioRunner] Setting {node_name} 'world_town' parameter to '{town}' via ROS 2 API")

        shutdown_after = False
        if not rclpy.ok():
            try:
                rclpy.init()
                shutdown_after = True
            except Exception:
                pass

        node = None
        try:
            node = Node('scenario_runner_param_setter')
            client = node.create_client(SetParameters, f'{node_name}/set_parameters')

            if not client.wait_for_service(timeout_sec=float(timeout)):
                print(f"[ScenarioRunner] Service {node_name}/set_parameters not available")
                return False

            param = Parameter()
            param.name = 'world_town'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = town

            # Bridge NPC traffic (vehicles + pedestrians) blocks scenario actor spawn points; turn it off.
            off_params = []
            for name in ('npc_vehicles_enabled', 'pedestrians_enabled'):
                p = Parameter()
                p.name = name
                p.value = ParameterValue()
                p.value.type = ParameterType.PARAMETER_STRING
                p.value.string_value = 'false'
                off_params.append(p)

            req = SetParameters.Request()
            req.parameters = [param] + off_params

            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=float(timeout))

            if future.result() is not None:
                results = future.result().results
                for p, r in zip(off_params, results[1:]):
                    if not r.successful:
                        print(f"[ScenarioRunner] WARNING: bridge rejected '{p.name}' ({r.reason}); "
                              "its NPC traffic stays on")
                if results and results[0].successful:
                    print(f"[ScenarioRunner] Parameter 'world_town' set to '{town}' successfully")
                    return True
                else:
                    reason = results[0].reason if results else 'unknown'
                    print(f"[ScenarioRunner] Failed to set parameter: {reason}")
                    return False
            else:
                print("[ScenarioRunner] SetParameters call returned no result")
                return False
        except Exception as e:
            print(f"[ScenarioRunner] WARNING: Failed to set map parameter: {e}")
            return False
        finally:
            if node is not None:
                node.destroy_node()

    def _deactivate_bridge(self):
        """
        Cleanly shut down the ROS 2 CARLA bridge via lifecycle transitions:
        active → deactivate → cleanup.
        This ensures all sensors call stop() and all actors call destroy()
        *before* the CARLA world is reloaded.
        Uses poll-based verification instead of fixed sleeps.
        """
        state = self._get_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE)
        
        if state == "active":
            self._ros2_lifecycle_set(self._BRIDGE_LIFECYCLE_NODE, "deactivate")
            # Poll until bridge reaches inactive state (max 15s)
            self._poll_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE, "inactive", timeout=15, poll_interval=0.5)
            state = "inactive"

        if state == "inactive":
            self._ros2_lifecycle_set(self._BRIDGE_LIFECYCLE_NODE, "cleanup")
            # Poll until bridge reaches unconfigured state (max 15s)
            self._poll_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE, "unconfigured", timeout=15, poll_interval=0.5)

        # Kill bridge's Traffic Manager (port 8000) to prevent stale port binding
        # after the bridge has cleaned up its NPC vehicles.
        try:
            bridge_tm = self.client.get_trafficmanager(8000)
            bridge_tm.shut_down()
        except Exception:
            pass

    def _reactivate_bridge(self):
        """
        Re-initialize the bridge after a world reload:
        unconfigured → configure → activate.

        Waits for the CARLA server to be fully ready (new world loaded
        and responding to API calls) before triggering configure.
        Uses poll-based verification instead of fixed sleeps.
        """
        state = self._get_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE)
        if state == "active":
            return

        # Wait for the new CARLA world to be fully ready.
        # load_world() returns before the server finishes loading the map,
        # so we poll until get_world().get_map() succeeds.
        print("[ScenarioRunner] Waiting for CARLA world to be ready...")
        max_wait = 60  # seconds
        poll_interval = 2.0
        waited = 0.0
        while waited < max_wait:
            try:
                w = self.client.get_world()
                m = w.get_map()
                # Also try a tick/wait to confirm the world is fully responsive
                try:
                    w.wait_for_tick(timeout=5.0)
                except Exception:
                    pass
                print("[ScenarioRunner] World ready: {}".format(m.name.split('/')[-1]))
                break
            except Exception as e:
                waited += poll_interval
                if waited >= max_wait:
                    print("[ScenarioRunner] WARNING: CARLA world not ready after {}s: {}".format(
                        max_wait, e))
                    break
                time.sleep(poll_interval)

        state = self._get_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE)
        if state == "unconfigured":
            # Configure the bridge and poll for inactive state (configure → inactive)
            self._ros2_lifecycle_set(self._BRIDGE_LIFECYCLE_NODE, "configure", timeout=60)
            self._poll_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE, "inactive", timeout=60, poll_interval=1.0)
            state = "inactive"

        if state == "inactive":
            # Activate the bridge and poll for active state
            self._ros2_lifecycle_set(self._BRIDGE_LIFECYCLE_NODE, "activate", timeout=30)
            self._poll_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE, "active", timeout=30, poll_interval=1.0)
            
        # Give the bridge a moment to spawn everything and start streaming
        time.sleep(2.0)

    def _load_and_wait_for_world(self, town, ego_vehicles=None):
        """
        Load a new CARLA world and provide data to CarlaDataProvider.

        When --waitForEgo is active and the map must change, the method
        coordinates with the ROS 2 bridge via lifecycle transitions so
        that sensors are cleanly destroyed before load_world() and
        re-created afterwards.  This prevents the CARLA server from
        spamming "Invalid session: no stream available" errors.
        """

        if self._args.reloadWorld:
            current_world = self.client.get_world()
            current_map = current_world.get_map().name.split('/')[-1]
            map_name_match = current_map.lower().split('_')[0] == town.lower().split('_')[0]
            if self._args.waitForEgo and map_name_match:
                print("World reload skipped because map is already {} and --waitForEgo is set".format(current_map))
                self.world = current_world

                # Check if bridge is active. If not, activate it!
                state = self._get_lifecycle_state(self._BRIDGE_LIFECYCLE_NODE)
                if state != "active":
                    print("[ScenarioRunner] Bridge is {}, reactivating it...".format(state))
                    # Push correct map to bridge before configure, else on_configure()
                    # reloads the bridge's stale 'world_town' (Town03) over the correct world.
                    self._ros2_set_map_parameter(self._BRIDGE_LIFECYCLE_NODE, town)
                    self._reactivate_bridge()
            elif self._args.waitForEgo and not map_name_match:
                # ── Coordinated reload: bridge lifecycle + load_world ──
                print("[ScenarioRunner] Map change required ({} → {}) with --waitForEgo".format(
                    current_map, town))
                print("[ScenarioRunner] Coordinating bridge lifecycle for safe world reload...")

                # 1. Set the map parameter BEFORE deactivation while the bridge
                #    is still in a stable 'active' state.  The parameter is just
                #    stored — it will be read on the next on_configure() cycle.
                self._ros2_set_map_parameter(self._BRIDGE_LIFECYCLE_NODE, town)

                # 2. Deactivate + cleanup the bridge (stops sensors, destroys actors)
                self._deactivate_bridge()

                # 3. Disable synchronous mode before load_world
                try:
                    settings = current_world.get_settings()
                    if settings.synchronous_mode:
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = None
                        current_world.apply_settings(settings)
                except Exception:
                    pass

                # 4. Load the new world
                print("[ScenarioRunner] Loading world: {}".format(town))
                self.world = self.client.load_world(town)

                # 5. Reactivate the bridge (reconnects, spawns ego + sensors on new world)
                self._reactivate_bridge()
                print("[ScenarioRunner] Bridge re-initialized on {}".format(town))
            else:
                # ── Legacy reload path (no --waitForEgo) ──
                # 1. Destroy ego vehicles to trigger manual_control.py cleanup
                for actor in current_world.get_actors():
                    if actor.type_id.startswith('vehicle.'):
                        try:
                            actor.destroy()
                        except RuntimeError:
                            pass

                # 2. Wait for manual_control.py to tick and clean up its sensors
                try:
                    if current_world.get_settings().synchronous_mode:
                        for _ in range(10):
                            current_world.tick()
                            time.sleep(0.1)
                    else:
                        time.sleep(1.5)
                except Exception:
                    pass

                # 3. Disable synchronous mode before load_world to prevent server crash
                try:
                    settings = current_world.get_settings()
                    if settings.synchronous_mode:
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = None
                        current_world.apply_settings(settings)
                except Exception:
                    pass

                self.world = self.client.load_world(town)
        else:
            self.world = self.client.get_world()

        # If we are waiting for an ego vehicle, wait until all ego vehicles are ready
        if self._args.waitForEgo and ego_vehicles:
            ego_vehicle_found = False
            while not ego_vehicle_found and not self._shutdown_requested:
                vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                for ego_vehicle in ego_vehicles:
                    ego_vehicle_found = False
                    for vehicle in vehicles:
                        if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                            ego_vehicle_found = True
                            break
                    if not ego_vehicle_found:
                        print("Not all ego vehicles ready. Waiting ... ")
                        time.sleep(1)
                        break

        if self._args.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self._args.frameRate
            self.world.apply_settings(settings)

        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)

        # Wait for the world to be ready
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

        map_name = CarlaDataProvider.get_map().name.split('/')[-1]
        map_name_match = map_name.lower().split('_')[0] == town.lower().split('_')[0]
        if map_name != "OpenDriveMap" and not map_name_match:
            print("The CARLA server uses the wrong map: {}".format(map_name))
            print("This scenario requires to use map: {}".format(town))
            return False

        return True

    def _load_and_run_scenario(self, config):
        """
        Load and run the scenario given by config.

        The Watchdog is paused during the load/setup phase (world load,
        ego discovery, actor spawn) so that inherently slow operations
        in async mode do not trigger a premature timeout.  It is resumed
        just before the scenario tick loop starts.
        """
        result = False

        # Pause watchdog during load phase — async world ticks are unreliable here
        if self.manager:
            self.manager.pause_watchdog()

        if not self._load_and_wait_for_world(config.town, config.ego_vehicles):
            self._cleanup()
            return False

        if self._args.agent:
            agent_class_name = self.module_agent.__name__.title().replace('_', '')
            try:
                self.agent_instance = getattr(self.module_agent, agent_class_name)(self._args.agentConfig)
                config.agent = self.agent_instance
            except Exception as e:          # pylint: disable=broad-except
                traceback.print_exc()
                print("Could not setup required agent due to {}".format(e))
                self._cleanup()
                return False

        CarlaDataProvider.set_traffic_manager_port(int(self._args.trafficManagerPort))
        tm_port = int(self._args.trafficManagerPort)

        # Shut down any stale TM on this port from a previous run or bridge
        try:
            old_tm = self.client.get_trafficmanager(tm_port)
            old_tm.shut_down()
            time.sleep(1.0)
        except Exception:
            pass

        tm = None
        for _tm_attempt in range(10):
            try:
                tm = self.client.get_trafficmanager(tm_port)
                break
            except RuntimeError as e:
                if "bind error" in str(e).lower():
                    print("[ScenarioRunner] Traffic Manager port {} busy, retrying ({}/10)...".format(
                        tm_port, _tm_attempt + 1))
                    time.sleep(2)
                else:
                    raise
        if tm is None:
            raise RuntimeError("Could not create Traffic Manager on port {}".format(tm_port))
        tm.set_random_device_seed(int(self._args.trafficManagerSeed))
        if self._args.sync:
            tm.set_synchronous_mode(True)

        # Prepare scenario
        print("Preparing scenario: " + config.name)
        try:
            self._prepare_ego_vehicles(config.ego_vehicles)
            if self._args.openscenario:
                scenario = OpenScenario(world=self.world,
                                        ego_vehicles=self.ego_vehicles,
                                        config=config,
                                        config_file=self._args.openscenario,
                                        timeout=100000)
            elif self._args.route:
                scenario = RouteScenario(world=self.world,
                                         config=config,
                                         debug_mode=self._args.debug)
            elif self._args.openscenario2:
                scenario = OSC2Scenario(world=self.world,
                                        ego_vehicles=self.ego_vehicles,
                                        config=config,
                                        osc2_file=self._args.openscenario2,
                                        timeout=100000)
            else:
                scenario_class = self._get_scenario_class_or_fail(config.type)
                scenario = scenario_class(world=self.world,
                                          ego_vehicles=self.ego_vehicles,
                                          config=config,
                                          randomize=self._args.randomize,
                                          debug_mode=self._args.debug)
        except Exception as exception:                  # pylint: disable=broad-except
            print("The scenario cannot be loaded")
            traceback.print_exc()
            print(exception)
            self._cleanup()
            return False

        try:
            if self._args.record:
                recorder_name = "{}/{}/{}.log".format(
                    os.getenv('SCENARIO_RUNNER_ROOT', "./"), self._args.record, config.name)
                self.client.start_recorder(recorder_name, True)

            # Load scenario and run it
            self.manager.load_scenario(scenario, self.agent_instance)

            # External hook (CSV runner: system manager power_on + mission) — ego placed,
            # scenario timer not started, watchdog still paused. Raising skips the run.
            hook = getattr(self, '_on_scenario_loaded', None)
            if hook:
                hook(scenario, config)

            # Resume watchdog just before the tick loop — scenario is now fully loaded
            if self.manager:
                self.manager.resume_watchdog()

            self.manager.run_scenario()

            # Provide outputs if required
            self._analyze_scenario(config)

            # Remove all actors, stop the recorder and save all criterias (if needed)
            scenario.remove_all_actors()
            if self._args.record:
                self.client.stop_recorder()
                self._record_criteria(self.manager.scenario.get_criteria(), recorder_name)

            result = True

        except Exception as e:              # pylint: disable=broad-except
            traceback.print_exc()
            print(e)
            result = False

        self._cleanup()
        return result

    def _run_scenarios(self):
        """
        Run conventional scenarios (e.g. implemented using the Python API of ScenarioRunner)
        """
        result = False

        # Load the scenario configurations provided in the config file
        scenario_configurations = ScenarioConfigurationParser.parse_scenario_configuration(
            self._args.scenario,
            self._args.configFile)
        if not scenario_configurations:
            print("Configuration for scenario {} cannot be found!".format(self._args.scenario))
            return result

        # Execute each configuration
        for config in scenario_configurations:
            for _ in range(self._args.repetitions):
                self.finished = False
                result = self._load_and_run_scenario(config)

            self._cleanup()
        return result

    def _run_route(self):
        """
        Run the route scenario
        """
        result = False

        # retrieve routes
        route_configurations = RouteParser.parse_routes_file(self._args.route, self._args.route_id)

        for config in route_configurations:
            for _ in range(self._args.repetitions):
                result = self._load_and_run_scenario(config)

                self._cleanup()
        return result

    def _run_openscenario(self):
        """
        Run a scenario based on OpenSCENARIO
        """

        # Load the scenario configurations provided in the config file
        if not os.path.isfile(self._args.openscenario):
            print("File does not exist")
            self._cleanup()
            return False

        openscenario_params = {}
        if self._args.openscenarioparams is not None:
            for entry in self._args.openscenarioparams.split(','):
                [key, val] = [m.strip() for m in entry.split(':')]
                openscenario_params[key] = val
        config = OpenScenarioConfiguration(self._args.openscenario, self.client, openscenario_params)

        result = self._load_and_run_scenario(config)
        self._cleanup()
        return result

    def _run_osc2(self):
        """
        Run a scenario based on ASAM OpenSCENARIO 2.0.
        https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html
        """
        # Load the scenario configurations provided in the config file
        if not os.path.isfile(self._args.openscenario2):
            print("File does not exist")
            self._cleanup()
            return False

        config = OSC2ScenarioConfiguration(self._args.openscenario2, self.client)

        result = self._load_and_run_scenario(config)
        self._cleanup()

        return result

    def _final_bridge_cleanup(self):
        """
        Deactivate and clean up the bridge after all scenarios have completed.
        Ensures no stale bridge state persists after the runner exits.
        """
        if not self._args.waitForEgo:
            return
        print("[ScenarioRunner] Final bridge cleanup after all scenarios...")
        self._deactivate_bridge()
        print("[ScenarioRunner] Bridge deactivated and cleaned up.")

    def run(self):
        """
        Run all scenarios according to provided commandline args
        """
        result = True
        if self._args.openscenario:
            result = self._run_openscenario()
        elif self._args.route:
            result = self._run_route()
        elif self._args.openscenario2:
            result = self._run_osc2()
        else:
            result = self._run_scenarios()

        # Cleanly shut down the bridge after all scenarios are done.
        # A batch driver (e.g. run_csv_scenarios.py) sets _defer_bridge_cleanup
        # so the bridge stays active across runs: deactivating then reactivating
        # it without a world reload respawns sensors on a dirty world and
        # segfaults the CARLA server. The driver tears the bridge down once at end.
        if not getattr(self, '_defer_bridge_cleanup', False):
            self._final_bridge_cleanup()

        print("No more scenarios .... Exiting")
        return result


def main():
    """
    main function
    """
    description = ("CARLA Scenario Runner: Setup, Run and Evaluate scenarios using CARLA\n"
                   "Current version: " + MIN_CARLA_VERSION)

    # pylint: disable=line-too-long
    parser = argparse.ArgumentParser(description=description,
                                     formatter_class=RawTextHelpFormatter)
    parser.add_argument('-v', '--version', action='version', version='%(prog)s ' + MIN_CARLA_VERSION)
    parser.add_argument('--host', default='127.0.0.1',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='2000',
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--timeout', default="60.0",
                        help='Set the CARLA client timeout value in seconds')
    parser.add_argument('--trafficManagerPort', default='8100',
                        help='Port to use for the TrafficManager (default: 8100)')
    parser.add_argument('--trafficManagerSeed', default='0',
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--sync', action='store_true',
                        help='Forces the simulation to run synchronously')
    parser.add_argument('--list', action="store_true", help='List all supported scenarios and exit')
    parser.add_argument('--frameRate', default='20', type=float,
                        help='Frame rate (Hz) to use in \'sync\' mode (default: 20)')

    parser.add_argument(
        "--scenario",
        help="Name of the scenario to be executed. Use the preposition 'group:' to run all scenarios of one class"
             " e.g. ControlLoss or FollowLeadingVehicle",
    )
    parser.add_argument("--openscenario", help="Provide an OpenSCENARIO definition")
    parser.add_argument('--openscenarioparams', help='Overwrited for OpenSCENARIO ParameterDeclaration')
    parser.add_argument('--openscenario2', help='Provide an openscenario2 definition')
    parser.add_argument('--route', help='Run a route as a scenario', type=str)
    parser.add_argument('--route-id', help='Run a specific route inside that \'route\' file', default='', type=str)
    parser.add_argument(
        '--agent', help="Agent used to execute the route. Not compatible with non-route-based scenarios.")
    parser.add_argument('--agentConfig', type=str, help="Path to Agent's configuration file", default="")

    parser.add_argument('--output', action="store_true", help='Provide results on stdout')
    parser.add_argument('--file', action="store_true", help='Write results into a txt file')
    parser.add_argument('--junit', action="store_true", help='Write results into a junit file')
    parser.add_argument('--json', action="store_true", help='Write results into a JSON file')
    parser.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')

    parser.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    parser.add_argument('--additionalScenario', default='', help='Provide additional scenario implementations (*.py)')

    parser.add_argument('--debug', action="store_true", help='Run with debug output')
    parser.add_argument('--reloadWorld', action="store_true",
                        help='Reload the CARLA world before starting a scenario (default=True)')
    parser.add_argument(
        "--record",
        type=str,
        default="",
        help="Path were the files will be saved, relative to SCENARIO_RUNNER_ROOT."
             "\nActivates the CARLA recording feature and saves to file all the criteria information.",
    )
    parser.add_argument("--randomize", action="store_true", help="Scenario parameters are randomized")
    parser.add_argument("--repetitions", default=1, type=int, help="Number of scenario executions")
    parser.add_argument(
        "--waitForEgo",
        action="store_true",
        help="Connect the scenario to an existing ego vehicle",
    )

    arguments = parser.parse_args()
    # pylint: enable=line-too-long

    OSC2Helper.wait_for_ego = arguments.waitForEgo

    if arguments.list:
        print("Currently the following scenarios are supported:")
        print(*ScenarioConfigurationParser.get_list_of_scenarios(arguments.configFile), sep='\n')
        return 1

    if not arguments.scenario and not arguments.openscenario and not arguments.route and not arguments.openscenario2:
        print("Please specify either a scenario or use the route mode\n\n")
        parser.print_help(sys.stdout)
        return 1

    if arguments.route and (arguments.openscenario or arguments.scenario):
        print("The route mode cannot be used together with a scenario (incl. OpenSCENARIO)'\n\n")
        parser.print_help(sys.stdout)
        return 1

    if arguments.agent and (arguments.openscenario or arguments.scenario):
        print("Agents are currently only compatible with route scenarios'\n\n")
        parser.print_help(sys.stdout)
        return 1

    if arguments.openscenarioparams and not arguments.openscenario:
        print("WARN: Ignoring --openscenarioparams when --openscenario is not specified")

    if arguments.route:
        arguments.reloadWorld = True

    if arguments.agent:
        arguments.sync = True

    scenario_runner = None
    result = True
    try:
        scenario_runner = ScenarioRunner(arguments)
        result = scenario_runner.run()
    except Exception:   # pylint: disable=broad-except
        traceback.print_exc()

    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
    return not result


if __name__ == "__main__":
    sys.exit(main())
