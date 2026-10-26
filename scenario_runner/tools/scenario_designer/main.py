#!/usr/bin/env python3
"""
CARLA Scenario Designer — Visual GUI tool for creating ScenarioRunner scenarios.

Usage:
    # With CARLA server running (recommended):
    python -m tools.scenario_designer.main --host localhost --port 2000

    # Without CARLA server (manual coordinate entry):
    python -m tools.scenario_designer.main --offline

    # Or directly:
    python tools/scenario_designer/main.py [--host HOST] [--port PORT] [--offline]
"""

import sys
import argparse
import os

# Add scenario_runner root to path for srunner imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sr_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
if sr_root not in sys.path:
    sys.path.insert(0, sr_root)


def main():
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser(description="CARLA Scenario Designer GUI")
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")
    parser.add_argument("--offline", action="store_true",
                        help="Run without CARLA server connection")
    args = parser.parse_args()

    # Import PyQt5
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError:
        print("ERROR: PyQt5 not installed. Install with: pip install PyQt5")
        sys.exit(1)

    # Try connecting to CARLA
    carla_client = None
    if not args.offline:
        try:
            import carla
            carla_client = carla.Client(args.host, args.port)
            carla_client.set_timeout(5.0)
            # Test connection
            world = carla_client.get_world()
            map_name = world.get_map().name.split('/')[-1]
            print(f"Connected to CARLA server at {args.host}:{args.port}")
            print(f"Current map: {map_name}")
        except ImportError:
            print("WARNING: carla Python package not found. Running in offline mode.")
            carla_client = None
        except Exception as e:
            print(f"WARNING: Cannot connect to CARLA at {args.host}:{args.port}: {e}")
            print("Running in offline mode. Start CARLA server for map visualization.")
            carla_client = None

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("Micropolis Scenario Designer")
    from .gui import theme
    theme.apply(app)

    from .gui.main_window import MainWindow
    window = MainWindow(carla_client=carla_client)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
