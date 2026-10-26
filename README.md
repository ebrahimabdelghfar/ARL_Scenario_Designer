<p align="center">
  <img src="scenario_runner/logo/arl.png" alt="The Autotronics Research Lab — Ain Shams University" width="420">
</p>

<h1 align="center">ARL Scenario Generation</h1>

<p align="center">
  Design CARLA driving scenarios visually, export them to ScenarioRunner / OpenSCENARIO,
  and run them in batches with pass/fail reporting and ROS 2 bag recording.
</p>

---

## What's inside

| Component | What it does |
|---|---|
| **ARL Scenario Designer** (`scenario_runner/tools/scenario_designer`) | PyQt5 desktop editor. Load a CARLA map, place the ego, goal, NPC vehicles, pedestrians, props and trigger points, draw or "drive" routes, set weather and success criteria, then export **XML + Python + OpenSCENARIO (.xosc)**. |
| **ScenarioRunner** (`scenario_runner/`) | A fork of [CARLA ScenarioRunner](https://github.com/carla-simulator/scenario_runner) that can wait for an externally spawned ego (`--waitForEgo`) and coordinates world reloads with a ROS 2 lifecycle bridge. |
| **CSV batch runner** (`scenario_runner/run_csv_scenarios.py`) | Runs every scenario listed in a CSV file and writes `Status` (PASS / Failed / Time Out / Not Found) and `Duration (s)` back into it. |
| **Bag recording** (`config/scenario_recording_config.yaml`) | Records the ego's sensor topics to a ROS 2 bag while each scenario runs. Bags of passing runs are deleted; bags of failed runs are kept for debugging. |

## Requirements

- Ubuntu 22.04, Python 3.10
- CARLA **0.9.16 or newer** (server and matching `carla` Python package). `make download_carla_assets` fetches the ASU Racing Team CARLA build, which includes the `silverstone` map and the `vehicle.vehicle.asurt_fsai` vehicle used in `config/carla_interface_config.yaml`. A stock CARLA 0.9.16 release also works for the built-in towns.
- ROS 2 Humble, for the CSV batch runner, bag recording and bridge coordination (`rclpy`, `rosbag2_py`, `lifecycle_msgs`, `rcl_interfaces`).
- `netcat` (`nc`), used by the Makefile to wait for the CARLA port.

## Installation

```bash
git clone https://github.com/ebrahimabdelghfar/ARL_Scenario_Designer.git
cd ARL_Scenario_Designer

# 1. CARLA server (extracts to ./ASU_RT_Carla)
make download_carla_assets

# 2. Python dependencies
pip install -r scenario_runner/requirements.txt PyQt5 pyyaml
pip install ASU_RT_Carla/PythonAPI/carla/dist/carla-*.whl   # CARLA client, must match the server

# 3. CARLA's `agents` package (route planner etc.)
export PYTHONPATH=$PWD/ASU_RT_Carla/PythonAPI/carla:$PYTHONPATH
```

## Quick start: design a scenario

```bash
make launch_designer             # starts CARLA (off-screen) and the designer; Ctrl+C stops both
make launch_designer_no_server   # connects to a CARLA server that is already running on :2000
```

No CARLA server? Run the designer offline. There is no map in this mode, so you type exact coordinates in the Properties panel:

```bash
cd scenario_runner && python3 -m tools.scenario_designer.main --offline
```

### Workflow

1. Pick a town in the toolbar and press **Load Map**.
2. Place the **Ego** (key `1`) and its **Goal** (`2`).
3. Add **NPC** vehicles (`3`), **Pedestrians** (`4`) and static **Props** (`5`).
4. Give actors routes with **Waypoints** (`7`), or **Collect** (`8`) to drive a route with the arrow keys. You can also add a **Trigger** point (`6`).
5. **Edit → Scenario Settings** (`Ctrl+,`): name, description, weather, collision / goal / timeout criteria.
6. Fix anything the **Issues** panel reports, then **Export** (`Ctrl+E`).

On export, **Auto-Deploy → Yes** puts the XML and XOSC files in `srunner/examples/` and the Python class in `srunner/scenarios/`, so ScenarioRunner finds them by name. **No** writes everything to the folder you chose. Projects are saved as `.scenario.json` (`Ctrl+S`).

Press **F1** in the designer for the full user guide, covering snapping rules, routes, triggers, NPC options and shortcuts.

## Running scenarios

### A single scenario

With CARLA running, from `scenario_runner/`:

```bash
python3 scenario_runner.py --list                         # all available scenarios
python3 scenario_runner.py --scenario MyScenario_1        # an XML scenario (exported + deployed)
python3 scenario_runner.py --openscenario srunner/examples/MyScenario.xosc

# Exported without Auto-Deploy:
python3 scenario_runner.py --scenario MyScenario_1 \
    --configFile <dir>/MyScenario.xml --additionalScenario <dir>/myscenario.py
```

In this mode ScenarioRunner spawns the ego itself. To drive it, run `python3 manual_control.py` in another terminal. Upstream ScenarioRunner options (`--sync`, `--agent`, `--route`, `--record`, `--output`, …) work as documented in [`scenario_runner/Docs`](scenario_runner/Docs).

### A batch from a CSV file

The CSV needs a `Scenario Name` column. `Status` and `Duration (s)` are added if they are missing. See `scenario_runner/scenarios_template.csv`:

```csv
Scenario Name,Status
CutOutWithCutIn1,
CutOutWithCutIn2,
```

Each name is looked up in `srunner/examples/*.xml` (the scenario `name`) and `*.xosc` (`FileHeader description + " (OpenSCENARIO)"`). Run the batch with:

```bash
make run_csv_scenarios_standalone CSV=scenario_runner/urban_scenarios.csv
# or, with ROS 2 sourced:
python3 scenario_runner/run_csv_scenarios.py --csv my_list.csv --host localhost --port 2000
```

The CSV is rewritten after every scenario, so progress survives a crash or `Ctrl+C`.

| Option | Default | Meaning |
|---|---|---|
| `--csv` | (required) | Input CSV |
| `--output-csv` | input file | Write results somewhere else |
| `--exec-timeout` | `600` | Wall-clock limit per scenario (s) → `Time Out` |
| `--no-record` | off | Disable bag recording |
| `--recording-config` | `config/scenario_recording_config.yaml` | Which sensor groups to record |
| `--carla-config` | `config/carla_interface_config.yaml` | Where topic names and the namespace come from |
| anything else | | Forwarded to `scenario_runner.py` (e.g. `--timeout 120`) |

Makefile variables: `CONFIG` (CARLA interface config), `ROS_SETUP` (default `/opt/ros/humble/setup.bash`), `CARLA_PYAPI` (default `ASU_RT_Carla/PythonAPI/carla`).

> **The batch runner expects an external ego.** Every run uses `--waitForEgo --reloadWorld`. ScenarioRunner waits until a vehicle whose `role_name` matches the scenario's ego (normally `hero`) exists, teleports it to the scenario start, and lets the external process tick the world. That process is usually a ROS 2 CARLA bridge that spawns the ego and its sensors.

### ROS 2 bridge coordination

When a scenario needs another map, ScenarioRunner drives the bridge's ROS 2 lifecycle so that sensors are torn down before `load_world()`:

1. It sets the bridge parameters `world_town=<town>`, `npc_vehicles_enabled=false` and `pedestrians_enabled=false`.
2. It calls `deactivate` → `cleanup`, loads the world, then calls `configure` → `activate`.

The bridge node name defaults to `/carla_bridge_node`. Override it with:

```bash
export CARLA_BRIDGE_NODE=/my_bridge_node
```

During a batch the bridge stays active between scenarios and is torn down once at the end.

### Bag recording

`config/scenario_recording_config.yaml` selects which sensor groups to record: cameras (optionally `camera_info` and the third-person view), lidars, odometry, GPS (and velocity) and battery. It also sets the bag options (`mcap`, compression, cache size, output directory). Topic names are never listed there. They are resolved from the `ros2` section and the sensor entries of `config/carla_interface_config.yaml`, so renaming a topic in one place is enough.

Recording starts once the scenario is loaded and ticking. Bags go to `scenarios_bags/<scenario>_<YYYYmmdd_HHMMSS>/`. A PASS deletes the bag; Failed and Time Out keep it.

## Make targets

| Target | Purpose |
|---|---|
| `download_carla_assets` | Download and extract the CARLA build into `ASU_RT_Carla/` |
| `launch_designer` | Start CARLA off-screen (Vulkan) and the designer, and clean both up on exit |
| `launch_designer_no_server` | Designer only, against a running server on `localhost:2000` |
| `list_scenarios` | Print scenario names without importing the runtime dependencies |
| `run_csv_scenarios_standalone CSV=…` | CSV batch run (sources `ROS_SETUP` first) |
| `kill_stale_sim` | Kill a leftover CARLA server and wait for port 2000 to free |

## Repository layout

```
config/
  carla_interface_config.yaml      # CARLA + ROS 2 bridge config (topics, sensors, ego)
  scenario_recording_config.yaml   # bag recording groups + options
scripts/download_assests/          # CARLA build download
scenario_runner/
  scenario_runner.py               # ScenarioRunner entry point
  run_csv_scenarios.py             # CSV batch runner
  manual_control.py                # drive the ego by keyboard
  srunner/examples/                # scenario configs (.xml, .xosc, .osc)
  srunner/scenarios/               # scenario Python classes
  tools/scenario_designer/
    main.py                        # designer entry point
    gui/                           # main window, map canvas, tree, properties, theme
    models/                        # project / actor / criteria data model
    generators/                    # XML, Python and OpenSCENARIO exporters
    tests/                         # designer tests
  logo/arl.png
```

## Tests

```bash
cd scenario_runner
QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tools/scenario_designer/tests
```

## Credits and license

Developed at **The Autotronics Research Lab (ARL), Ain Shams University**.

Built on [CARLA ScenarioRunner](https://github.com/carla-simulator/scenario_runner), © 2018 CARLA, MIT License (see [`scenario_runner/LICENSE`](scenario_runner/LICENSE)).
