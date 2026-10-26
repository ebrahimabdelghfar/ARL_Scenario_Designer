#!/usr/bin/env python3

"""
CSV Scenario Runner for CARLA Scenario Runner.
Executes scenarios listed in a CSV file and logs their status (PASS, Failed, Time Out, Not Found)
along with elapsed time for PASS/Failed runs.
"""

import os
import sys
import csv
import glob
import time
import shutil
import argparse
import signal
import threading
import traceback
import datetime
import xml.etree.ElementTree as ET

import yaml

from scenario_runner import ScenarioRunner

# Auto-configure SCENARIO_RUNNER_ROOT to current script directory if not properly set
_current_dir = os.path.dirname(os.path.abspath(__file__))
_env_root = os.getenv('SCENARIO_RUNNER_ROOT')
if not _env_root or not os.path.isdir(_env_root) or not os.path.isdir(os.path.join(_env_root, 'srunner')):
    os.environ['SCENARIO_RUNNER_ROOT'] = _current_dir
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

def create_scenario_runner_parser():
    """Create argparse parser matching scenario_runner.py parameters."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default='2000')
    parser.add_argument('--timeout', default="60.0")
    parser.add_argument('--trafficManagerPort', default='8100')
    parser.add_argument('--trafficManagerSeed', default='0')
    parser.add_argument('--sync', action='store_true')
    parser.add_argument('--list', action="store_true")
    parser.add_argument('--frameRate', default='20', type=float)
    parser.add_argument("--scenario")
    parser.add_argument("--openscenario")
    parser.add_argument('--openscenarioparams')
    parser.add_argument('--openscenario2')
    parser.add_argument('--route', type=str)
    parser.add_argument('--route-id', default='', type=str)
    parser.add_argument('--agent')
    parser.add_argument('--agentConfig', type=str, default="")
    parser.add_argument('--output', action="store_true")
    parser.add_argument('--file', action="store_true")
    parser.add_argument('--junit', action="store_true")
    parser.add_argument('--json', action="store_true")
    parser.add_argument('--outputDir', default='')
    parser.add_argument('--configFile', default='')
    parser.add_argument('--additionalScenario', default='')
    parser.add_argument('--debug', action="store_true")
    parser.add_argument('--reloadWorld', action="store_true")
    parser.add_argument("--record", type=str, default="")
    parser.add_argument("--randomize", action="store_true")
    parser.add_argument("--repetitions", default=1, type=int)
    parser.add_argument("--waitForEgo", action="store_true")
    return parser

class TimeoutException(Exception):
    pass

# ScenarioRunner catches the TimeoutException internally, so record that it fired.
_timed_out = False

def timeout_handler(signum, frame):
    global _timed_out
    _timed_out = True
    raise TimeoutException("Execution timed out")


# ─── ROS 2 Bag Recording ──────────────────────────────────────────────────────
# Records the ego's sensor + telemetry topics while a scenario runs. Topic names
# are resolved from config/carla_interface_config.yaml (never hardcoded); which
# groups to record is driven by config/scenario_recording_config.yaml. Uses the
# rosbag2 native API (rosbag2_py.Recorder) — no `ros2 bag record` subprocess.

def _resolve_topic(suffix, namespace):
    """Mirror the carla interface topic() rule: absolute ('/...') verbatim,
    otherwise prepend /<namespace>/."""
    suffix = str(suffix)
    if suffix.startswith('/'):
        return suffix
    ns = str(namespace).strip('/')
    return f"/{suffix}" if not ns else f"/{ns}/{suffix}"


def load_recording_targets(recording_cfg_path, carla_cfg_path):
    """Read both configs and return (topics, bag_opts). topics are fully-resolved
    ROS topic names for the enabled groups; bag_opts is the recording config 'bag' dict."""
    with open(recording_cfg_path, 'r', encoding='utf-8') as f:
        rec_cfg = yaml.safe_load(f) or {}
    with open(carla_cfg_path, 'r', encoding='utf-8') as f:
        carla_cfg = yaml.safe_load(f) or {}

    record = rec_cfg.get('record', {})
    bag_opts = rec_cfg.get('bag', {})

    ros2 = carla_cfg.get('ros2', {})
    namespace = ros2.get('namespace', 'sim')
    ros_topics = ros2.get('topics', {})

    topics = []

    def add(suffix):
        if suffix:
            topics.append(_resolve_topic(suffix, namespace))

    cams = record.get('cameras', {})
    if cams.get('enabled'):
        include_info = cams.get('include_camera_info', True)
        for cam in carla_cfg.get('cameras', []) or []:
            if not cam.get('enabled'):
                continue
            add(cam.get('topic_rgb'))
            if include_info:
                add(cam.get('topic_camera_info'))
        if cams.get('include_third_person'):
            tpv = carla_cfg.get('third_person_view', {})
            if tpv.get('enabled'):
                add(tpv.get('topic_rgb'))
                if include_info:
                    add(tpv.get('topic_camera_info'))

    if record.get('lidars', {}).get('enabled'):
        for lidar in carla_cfg.get('lidars', []) or []:
            if lidar.get('enabled'):
                add(lidar.get('topic_point_cloud'))

    if record.get('odom'):
        add(ros_topics.get('odom'))

    gps = record.get('gps', {})
    if gps.get('enabled'):
        add(ros_topics.get('feedback_gps'))
        if gps.get('include_velocity'):
            add(ros_topics.get('feedback_gps_vel'))

    if record.get('battery'):
        add(ros_topics.get('feedback_battery_state'))

    # Dedupe, preserve order
    seen = set()
    unique = [t for t in topics if not (t in seen or seen.add(t))]
    return unique, bag_opts


class BagRecorder:
    """Starts a rosbag2 recording once the scenario is ticking and stops it when
    the run ends. record() blocks, so it runs in a background thread."""

    def __init__(self, topics, bag_opts, bag_uri):
        import rosbag2_py
        self._rosbag2_py = rosbag2_py
        self.topics = topics
        self.bag_opts = bag_opts or {}
        self.bag_uri = bag_uri
        self._recorder = None
        self._thread = None
        self._stop_event = threading.Event()
        self.started = False

    def _run(self, runner):
        rosbag2_py = self._rosbag2_py
        # Wait until the scenario is loaded and actively ticking (or aborted).
        while not self._stop_event.is_set():
            mgr = getattr(runner, 'manager', None)
            if mgr is not None and getattr(mgr, 'scenario', None) is not None \
                    and getattr(mgr, '_running', False):
                break
            time.sleep(0.1)
        if self._stop_event.is_set():
            return

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_uri,
            storage_id=self.bag_opts.get('storage_id', 'mcap'),
            max_cache_size=int(self.bag_opts.get('max_cache_size', 0)),
        )
        record_options = rosbag2_py.RecordOptions()
        record_options.all = False
        record_options.topics = self.topics
        record_options.rmw_serialization_format = 'cdr'
        comp_mode = self.bag_opts.get('compression_mode', 'none')
        if comp_mode and comp_mode != 'none':
            record_options.compression_mode = comp_mode
            record_options.compression_format = self.bag_opts.get('compression_format', 'zstd')

        self._recorder = rosbag2_py.Recorder()
        self.started = True
        print(f"[recorder] scenario ticking → recording {len(self.topics)} "
              f"topic(s) to {self.bag_uri}")
        try:
            self._recorder.record(storage_options, record_options)  # blocks until cancel()
        except Exception as e:  # noqa: BLE001 - recorder must not crash the run
            print(f"[recorder] recording error: {e}", file=sys.stderr)

    def start(self, runner):
        self._thread = threading.Thread(target=self._run, args=(runner,), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._recorder is not None:
            try:
                self._recorder.cancel()
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=30.0)

    def discard(self):
        """Delete the recorded bag (used when the scenario passes)."""
        shutil.rmtree(self.bag_uri, ignore_errors=True)


def get_scenario_mapping():
    """
    Scan srunner/examples directory and build a map of scenario names to their details.
    Maps: scenario_name -> list of {'type': 'scenario'|'openscenario', 'arg': argument_value, 'file': file_path}
    Multiple files can map to the same scenario description name (duplicates).
    """
    root = os.getenv('SCENARIO_RUNNER_ROOT', './')
    mapping = {}

    # Helper to add mapping
    def add_mapping(name, item):
        if name not in mapping:
            mapping[name] = []
        mapping[name].append(item)

    # 1. XML scenarios
    xml_files = glob.glob(os.path.join(root, "srunner/examples/*.xml"))
    for file_name in xml_files:
        try:
            tree = ET.parse(file_name)
            for scenario in tree.iter("scenario"):
                name = scenario.attrib.get('name', None)
                if name:
                    add_mapping(name, {
                        'type': 'scenario',
                        'arg': name,
                        'file': file_name
                    })
        except Exception as e:
            print(f"Warning: Failed to parse XML file {file_name}: {e}", file=sys.stderr)

    # 2. OpenSCENARIO scenarios (.xosc)
    xosc_files = glob.glob(os.path.join(root, "srunner/examples/*.xosc"))
    for file_name in xosc_files:
        try:
            tree = ET.parse(file_name)
            file_header = tree.find("FileHeader")
            if file_header is not None:
                desc = file_header.attrib.get('description', None)
                if desc:
                    # Key format matches what is returned by --list
                    name = f"{desc} (OpenSCENARIO)"
                    add_mapping(name, {
                        'type': 'openscenario',
                        'arg': file_name,
                        'file': file_name
                    })
        except Exception as e:
            print(f"Warning: Failed to parse XOSC file {file_name}: {e}", file=sys.stderr)

    return mapping

def run_scenario(name, details, extra_args, exec_timeout=600, recorder=None):
    """
    Executes a single scenario and returns (status, elapsed_time).
    If a BagRecorder is given, it starts once the scenario is ticking and stops
    when the run ends.
    """
    if not details:
        return "Not Found", 0.0

    # Build command arguments for scenario runner parser
    cmd_args = ["--waitForEgo", "--reloadWorld"]
    cmd_args.extend(extra_args)

    if details['type'] == 'scenario':
        cmd_args.extend(["--scenario", details['arg']])
    elif details['type'] == 'openscenario':
        cmd_args.extend(["--openscenario", details['arg']])

    cmd_str = " ".join(["scenario_runner.py"] + cmd_args)
    print(f"\nRunning API execution roughly equivalent to: {cmd_str}")

    # Parse arguments for ScenarioRunner
    sr_parser = create_scenario_runner_parser()
    sr_args, _ = sr_parser.parse_known_args(cmd_args)

    # Apply same argument handling as scenario_runner.py main
    from srunner.tools.osc2_helper import OSC2Helper
    OSC2Helper.wait_for_ego = sr_args.waitForEgo

    if sr_args.route:
        sr_args.reloadWorld = True
    if sr_args.agent:
        sr_args.sync = True

    start_time = time.time()
    runner = None
    result_status = "PASS"
    
    # Configure signal alarm for execution timeout
    global _timed_out
    _timed_out = False
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(exec_timeout)

    try:
        runner = ScenarioRunner(sr_args)
        # Keep the bridge active between scenarios; tearing it down per-run and
        # reactivating it without a world reload segfaults the CARLA server.
        # The batch does one final teardown after the whole CSV (see main()).
        runner._defer_bridge_cleanup = True
        if recorder is not None:
            recorder.start(runner)
        runner.run()

        # Check success criteria programmatically
        if _timed_out:
            print(f"Execution timed out after {exec_timeout} seconds.")
            result_status = "Time Out"
        elif runner.manager and runner.manager.scenario:
            failure = False
            timeout = False

            criteria = runner.manager.scenario.get_criteria()
            for criterion in criteria:
                if (not criterion.optional and
                        criterion.test_status != "SUCCESS" and
                        criterion.test_status != "ACCEPTABLE"):
                    failure = True
                    break

            if getattr(runner.manager.scenario, 'timeout_node', None) and runner.manager.scenario.timeout_node.timeout and not failure:
                timeout = True

            if timeout:
                result_status = "Time Out"
            elif failure:
                result_status = "Failed"
            else:
                result_status = "PASS"
        else:
            # Scenario failed to load or crashed early
            result_status = "Failed"

    except TimeoutException:
        print(f"Execution timed out after {exec_timeout} seconds.")
        result_status = "Time Out"
    except Exception as e:
        print(f"Exception during scenario execution: {e}")
        traceback.print_exc()
        result_status = "Failed"
    finally:
        signal.alarm(0) # Disable alarm
        if recorder is not None:
            recorder.stop()
        if runner is not None:
            runner.destroy()

    elapsed = time.time() - start_time
    return result_status, elapsed

def final_bridge_teardown(extra_args):
    """Deactivate the ROS 2 bridge once after all scenarios have run.

    The per-scenario runs defer bridge cleanup (bridge stays active between
    them). This does the single teardown at the very end so no stale bridge
    persists. Best-effort: never fatal to the batch summary.
    """
    try:
        sr_parser = create_scenario_runner_parser()
        sr_args, _ = sr_parser.parse_known_args(["--waitForEgo", "--reloadWorld"] + list(extra_args))
        from srunner.tools.osc2_helper import OSC2Helper
        OSC2Helper.wait_for_ego = True
        runner = ScenarioRunner(sr_args)
        try:
            runner._final_bridge_cleanup()
        finally:
            runner.destroy()
    except Exception as e:
        print(f"Warning: final bridge teardown skipped ({e})", file=sys.stderr)

def main():
    parser = argparse.ArgumentParser(
        description="Run CARLA scenarios listed in a CSV file and record results.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Any extra arguments are forwarded directly to scenario_runner.py.\n"
               "Example:\n"
               "  python3 run_csv_scenarios.py --csv input.csv --host 127.0.0.1 --port 2000"
    )
    parser.add_argument('--csv', required=True, help='Path to the input CSV file')
    parser.add_argument('--output-csv', help='Path to save output results CSV (defaults to updating input in-place)')
    parser.add_argument('--exec-timeout', type=int, default=600, help='Max execution time for a scenario run in seconds (default: 600)')
    parser.add_argument('--recording-config',
                        default=os.path.join(_current_dir, '..', 'config', 'scenario_recording_config.yaml'),
                        help='Path to the bag recording config (default: ../config/scenario_recording_config.yaml)')
    parser.add_argument('--carla-config',
                        default=os.path.join(_current_dir, '..', 'config', 'carla_interface_config.yaml'),
                        help='Path to the CARLA interface config used to resolve topic names '
                             '(default: ../config/carla_interface_config.yaml)')
    parser.add_argument('--no-record', action='store_true', help='Disable ROS 2 bag recording')

    # Parse args separating known from forwarded args
    args, unknown_args = parser.parse_known_args()

    input_csv = args.csv
    output_csv = args.output_csv if args.output_csv else input_csv

    if not os.path.exists(input_csv):
        print(f"Error: CSV file not found at '{input_csv}'", file=sys.stderr)
        sys.exit(1)

    # Set up ROS 2 bag recording (resolve topics from the CARLA config)
    rec_topics, bag_opts = ([], {})
    recording_enabled = not args.no_record
    if recording_enabled:
        try:
            rec_topics, bag_opts = load_recording_targets(args.recording_config, args.carla_config)
            import rclpy
            rclpy.init()
            bags_dir = os.path.abspath(
                os.path.join(os.path.dirname(_current_dir), bag_opts.get('output_dir', 'scenarios_bags')))
            os.makedirs(bags_dir, exist_ok=True)
            print(f"Recording enabled: {len(rec_topics)} topic(s) → {bags_dir}")
        except Exception as e:
            print(f"Warning: bag recording disabled ({e})", file=sys.stderr)
            recording_enabled = False

    # Load mapping of scenario names
    mapping = get_scenario_mapping()

    # Read CSV records
    rows = []
    headers = []
    
    with open(input_csv, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        try:
            headers = next(reader)
        except StopIteration:
            print("Error: Empty CSV file", file=sys.stderr)
            sys.exit(1)
        rows = list(reader)

    # Identify column indices
    try:
        scenario_idx = [h.lower().strip() for h in headers].index('scenario name')
    except ValueError:
        print("Error: CSV must contain 'Scenario Name' column", file=sys.stderr)
        sys.exit(1)

    # Remove 'Command' column if it exists in the headers/rows
    try:
        command_idx = [h.lower().strip() for h in headers].index('command')
        headers.pop(command_idx)
        for r in rows:
            if len(r) > command_idx:
                r.pop(command_idx)
    except ValueError:
        pass

    # Ensure 'Status' column exists, add if missing
    try:
        status_idx = [h.lower().strip() for h in headers].index('status')
    except ValueError:
        headers.append('Status')
        status_idx = len(headers) - 1
        for r in rows:
            r.append('')

    # Ensure 'Duration (s)' column exists, add if missing
    try:
        duration_idx = [h.lower().strip() for h in headers].index('duration (s)')
    except ValueError:
        try:
            duration_idx = [h.lower().strip() for h in headers].index('duration')
        except ValueError:
            headers.append('Duration (s)')
            duration_idx = len(headers) - 1
            for r in rows:
                r.append('')

    total = len(rows)
    passed = 0
    failed = 0
    timeouts = 0
    not_found = 0
    seen_counts = {}

    print(f"Loaded {total} scenarios to run from '{input_csv}'")

    for i, row in enumerate(rows, start=1):
        if len(row) <= scenario_idx:
            continue
        scen_name = row[scenario_idx].strip()
        if not scen_name:
            continue

        print(f"\n[{i}/{total}] Processing: {scen_name}")
        
        # Track occurrence to handle duplicate names mapping to distinct files
        seen_counts[scen_name] = seen_counts.get(scen_name, 0) + 1
        occurrence = seen_counts[scen_name] - 1
        
        details_list = mapping.get(scen_name, [])
        if occurrence < len(details_list):
            details = details_list[occurrence]
        else:
            details = None
        
        if not details:
            print(f"Result: Not Found")
            row[status_idx] = "Not Found"
            row[duration_idx] = "N/A"
            not_found += 1
        else:
            recorder = None
            if recording_enabled and rec_topics:
                safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in scen_name).strip("_")
                stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                bag_uri = os.path.join(bags_dir, f"{safe_name}_{stamp}")
                recorder = BagRecorder(rec_topics, bag_opts, bag_uri)

            status, elapsed = run_scenario(scen_name, details, unknown_args,
                                           exec_timeout=args.exec_timeout, recorder=recorder)
            print(f"Result: {status} (took {elapsed:.2f}s)")

            if recorder is not None and recorder.started:
                if status == "PASS":
                    recorder.discard()
                    print(f"[recorder] PASS → discarded bag {recorder.bag_uri}")
                else:
                    print(f"[recorder] {status} → kept bag {recorder.bag_uri}")

            row[status_idx] = status
            if status in ["PASS", "Failed"]:
                row[duration_idx] = f"{elapsed:.2f}"
            else:
                row[duration_idx] = "N/A"
            
            if status == "PASS":
                passed += 1
            elif status == "Time Out":
                timeouts += 1
            else:
                failed += 1

        # Write updates iteratively back to CSV to preserve progress in case of crash/cancel
        with open(output_csv, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(rows)

    # Bridge was kept active across scenarios; tear it down once now.
    final_bridge_teardown(unknown_args)

    # Print final summary
    print("\n" + "="*40)
    print("Execution Summary:")
    print(f"  Total Scenarios: {total}")
    print(f"  Passed:          {passed}")
    print(f"  Failed:          {failed}")
    print(f"  Time Out:        {timeouts}")
    print(f"  Not Found:       {not_found}")
    print("="*40)
    print(f"Results written to: {output_csv}\n")

    if recording_enabled:
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
