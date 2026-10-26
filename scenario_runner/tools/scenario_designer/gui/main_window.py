"""
MainWindow — The main application window for CARLA Scenario Designer.

Layout:
  ┌────────────────────────────────────────────────────┐
  │  Menu bar  /  Toolbar (brand · map · place · ...)  │
  ├──────────┬──────────────────────┬──────────────────┤
  │ Scenario │   Map Canvas         │ Properties       │
  │ (dock)   ├──────────────────────┤ (dock)           │
  │          │   Issues (dock)      │                  │
  ├──────────┴──────────────────────┴──────────────────┤
  │  Status Bar                                        │
  └────────────────────────────────────────────────────┘
"""

import json
import math
import os

from PyQt5.QtWidgets import (QMainWindow, QToolBar, QAction, QActionGroup,
                             QComboBox, QStatusBar, QFileDialog, QMessageBox,
                             QLabel, QHBoxLayout, QWidget, QDialog, QDockWidget,
                             QVBoxLayout, QFormLayout, QLineEdit, QDoubleSpinBox,
                             QCheckBox, QGroupBox, QTextEdit, QListWidget,
                             QListWidgetItem, QDialogButtonBox, QSizePolicy, QTextBrowser)
from PyQt5.QtCore import Qt, QSize, QTimer, QRegExp
from PyQt5.QtGui import QIcon, QKeySequence, QRegExpValidator

from . import theme
from .map_canvas import MapCanvas
from .object_tree import ObjectTree
from .properties_panel import PropertiesPanel
from ..models.project import ScenarioProject
from ..models.actors import (EgoVehicle, NPCVehicle, Pedestrian, TriggerPoint, Transform,
                             RouteWaypoint, StaticProp)
from ..generators.xml_generator import save_xml
from ..generators.python_generator import save_python
from ..generators.xosc_generator import save_xosc


# Standard CARLA maps
CARLA_MAPS = [
    "Town01", "Town02", "Town03", "Town04", "Town05",
    "Town06", "Town07", "Town10HD", "Town11", "Town12",
    "Town13", "Town15",
]

# Place tools: key -> (label, icon, shortcut, tooltip)
PLACE_TOOLS = [
    ("ego",      "Ego",        "car",    "1", "Place the ego vehicle"),
    ("ego_goal", "Goal",       "flag",   "2", "Set the ego destination / goal point"),
    ("npc",      "NPC",        "car",    "3", "Place an NPC vehicle"),
    ("ped",      "Pedestrian", "person", "4", "Place a pedestrian"),
    ("prop",     "Prop",       "cone",   "5", "Place a static prop (cone, barrier, debris…)"),
    ("trigger",  "Trigger",    "zap",    "6", "Place a trigger point"),
    ("waypoint", "Waypoints",  "pin",    "7", "Add route waypoints to the selected NPC / pedestrian"),
    ("collect",  "Collect",    "route",  "8", "Capture a route by driving the selected NPC "
                                              "(drag or arrow keys; Space toggles lane snap)"),
]
TOOL_COLORS = {"ego": "#1E90FF", "ego_goal": "#00C850", "npc": "#FF5050",
               "ped": "#FFB428", "prop": "#00C8C8", "trigger": "#B450FF"}

SHORTCUTS_HTML = """
<table cellspacing="6">
<tr><td><b>1 – 8</b></td><td>Place tools: Ego, Goal, NPC, Pedestrian, Prop, Trigger, Waypoints, Collect</td></tr>
<tr><td><b>Esc</b></td><td>Cancel the active tool / stop collecting</td></tr>
<tr><td><b>Del</b></td><td>Delete selected object</td></tr>
<tr><td><b>Ctrl+D</b></td><td>Duplicate selected NPC / pedestrian / prop / trigger</td></tr>
<tr><td><b>Ctrl+Z / Ctrl+Shift+Z</b></td><td>Undo / Redo</td></tr>
<tr><td><b>F</b></td><td>Fit map in view</td></tr>
<tr><td><b>Wheel / drag</b></td><td>Zoom / pan the map</td></tr>
<tr><td><b>Arrows, Space</b></td><td>Collect mode: drive, toggle lane snap</td></tr>
<tr><td><b>Ctrl+N / O / S / E</b></td><td>New, Open, Save, Export</td></tr>
<tr><td><b>F1</b></td><td>User Guide</td></tr>
</table>"""


GUIDE_CSS = """
h1 { color: #E6EDF2; font-size: 20px; margin-bottom: 2px; }
h2 { color: #5FB3CE; font-size: 15px; margin-top: 18px; margin-bottom: 4px; }
p, li, td { color: #E6EDF2; font-size: 13px; }
.muted { color: #93A4B1; }
.note { color: #F5A524; }
code { color: #F0BE28; }
a { color: #5FB3CE; text-decoration: none; }
td.k { color: #5FB3CE; font-weight: bold; padding-right: 14px; }
"""

GUIDE_HTML = """
<h1>Scenario Designer — User Guide</h1>
<p class="muted">Design a CARLA ScenarioRunner scenario visually: load a map, place the ego and other actors,
give them routes and triggers, then export runnable files.</p>
<p><a href="#start">1. Start</a> &nbsp;·&nbsp; <a href="#workflow">2. Quick workflow</a> &nbsp;·&nbsp;
<a href="#map">3. Map &amp; navigation</a> &nbsp;·&nbsp; <a href="#place">4. Placing objects</a> &nbsp;·&nbsp;
<a href="#edit">5. Editing</a> &nbsp;·&nbsp; <a href="#routes">6. Routes</a> &nbsp;·&nbsp;
<a href="#triggers">7. Triggers</a> &nbsp;·&nbsp; <a href="#settings">8. Settings &amp; criteria</a> &nbsp;·&nbsp;
<a href="#files">9. Save, export &amp; run</a> &nbsp;·&nbsp; <a href="#keys">10. Shortcuts</a></p>

<a name="start"></a><h2>1. Start the designer</h2>
<ul>
<li><code>make launch_designer</code> — starts a CARLA server and the designer together.</li>
<li><code>make launch_designer_no_server</code> — connects to a CARLA server that is already running.</li>
<li>Offline (no CARLA): from <code>scenario_runner/</code> run
<code>python3 -m tools.scenario_designer.main --offline</code>. There is no map, so place actors by
clicking and type exact coordinates in Properties.</li>
</ul>
<p>The status bar shows <b>CARLA connected</b> or <b>CARLA offline</b>.</p>

<a name="workflow"></a><h2>2. Quick workflow</h2>
<ol>
<li>Pick a town in the toolbar and press <b>Load Map</b>.</li>
<li>Press <b>Ego</b> (key 1) and click a lane; press <b>Goal</b> (2) and click the destination.</li>
<li>Add <b>NPC</b> vehicles (3), <b>Pedestrians</b> (4), static <b>Props</b> (5).</li>
<li>Give actors routes with <b>Waypoints</b> (7) or <b>Collect</b> (8), optionally a <b>Trigger</b> (6).</li>
<li>Open <b>Edit → Scenario Settings</b> to name the scenario and set weather and success/failure criteria.</li>
<li>Fix anything listed in the <b>Issues</b> panel, then <b>Export</b> (Ctrl+E).</li>
</ol>

<a name="map"></a><h2>3. Map &amp; navigation</h2>
<ul>
<li><b>Mouse wheel</b> zooms, <b>drag</b> on empty map pans, <b>F</b> fits the whole map.</li>
<li>The map is drawn from the CARLA lane graph: asphalt, lane markings (white/yellow, solid/dashed),
faint <b>&gt;</b> chevrons for travel direction, sidewalks, zebra crossings and grey spawn points
(hover a spawn point for its coordinates).</li>
<li><b>Right-click</b> the map to see/copy world coordinates.</li>
<li>Changing the town in the combo only sets the project's town — press <b>Load Map</b> to switch CARLA and redraw.</li>
<li><b>Sync Cam</b> (toolbar camera button) moves the CARLA spectator camera to follow the selected actor.</li>
</ul>

<a name="place"></a><h2>4. Placing objects</h2>
<p>Choose a tool in the <b>PLACE</b> toolbar (or keys 1–8) — a blue hint appears at the top of the map — then
click the map. <b>Esc</b> or clicking the active tool again cancels.</p>
<p>A click places the object exactly where you click; <b>snapping happens when you drag</b> it afterwards.
Dragging always snaps (vehicles to lanes, pedestrians to sidewalks) — to put an object off the lane,
type its X / Y in Properties. The <b>Space</b> snap toggle exists only while collecting a route (Collect, 8).</p>
<table cellspacing="4">
<tr><td class="k">Ego (1)</td><td>The vehicle under test. Only one; when dragged it snaps to the nearest driving lane and takes its heading.</td></tr>
<tr><td class="k">Goal (2)</td><td>Ego destination (needs the ego first). Its dashed circle is the success radius.</td></tr>
<tr><td class="k">NPC (3)</td><td>Other vehicle; snaps to a driving lane when dragged.</td></tr>
<tr><td class="k">Pedestrian (4)</td><td>When dragged, snaps to the nearest sidewalk, or stays put on a crosswalk.</td></tr>
<tr><td class="k">Prop (5)</td><td>Static object (cone, barrier, debris…). Placed anywhere at ground height.</td></tr>
<tr><td class="k">Trigger (6)</td><td>A point with a radius; fires when the ego enters it (see Triggers).</td></tr>
<tr><td class="k">Waypoints (7)</td><td>Add route points to the selected NPC / pedestrian (see Routes).</td></tr>
<tr><td class="k">Collect (8)</td><td>Record an NPC route by driving it (see Routes).</td></tr>
</table>

<a name="edit"></a><h2>5. Selecting &amp; editing</h2>
<ul>
<li>Click an actor on the map or in the <b>Scenario</b> tree to select it; its fields open in <b>Properties</b>.</li>
<li><b>Drag</b> actors on the map to move them, or type exact X / Y / Z / Yaw in Properties.</li>
<li><b>Del</b> deletes, <b>Ctrl+D</b> duplicates (NPC / pedestrian / prop / trigger, placed 4 m ahead).
The same actions are in the tree's right-click menu.</li>
<li><b>Ctrl+Z / Ctrl+Shift+Z</b> undo / redo every change, including drags and property edits.</li>
<li><b>NPC</b>: model, speed, max brake, stop at next intersection (only when it has no route),
random velocity variation, trigger. Tick <b>Parked</b> to keep it where it spawns for the whole scenario
(hand brake, no driving; its route and driving fields are ignored) — shown as <b>(P)</b> on the map.
<span class="note">Lane Change is saved but not used by the export yet; Color is written to the XML only.</span></li>
<li><b>Pedestrian</b>: model, walking speed, wheelchair, route, trigger.</li>
<li><b>Prop</b>: model (the full <code>static.prop.*</code> list when CARLA is connected) and
<b>Simulate Physics</b> — on: it can be pushed or knocked over; off: fixed obstacle.</li>
</ul>

<a name="routes"></a><h2>6. Routes</h2>
<p>Without a route an NPC drives along its lane (optionally stopping before the next intersection) —
use <b>Parked</b> if it should not move; a pedestrian walks straight ahead.</p>
<ul>
<li><b>Waypoints (7)</b>: select an NPC or pedestrian, press 7, then click point after point.
NPC points snap to lanes, pedestrian points to sidewalks. Press <b>Esc</b> to finish.</li>
<li><b>Collect (8)</b>: select an NPC, press 8, then <b>drag the vehicle</b> or drive it with the arrow keys
(<b>Up/Down</b> 1 m, <b>Left/Right</b> steer 5°). <b>Space</b> toggles lane snapping.
A point is recorded every <b>Sample Dist</b> metres (set in Properties). Press <b>Esc</b> or <b>Stop</b> to finish.</li>
<li><b>Snap to Start of Route</b> moves the NPC onto the first route point; <b>Clear Route</b> removes the route.</li>
<li>Routes show as green dashed lines with a dot per point.</li>
</ul>

<a name="triggers"></a><h2>7. Triggers</h2>
<ul>
<li>The scenario starts when the ego enters <b>any</b> trigger point.</li>
<li>With <b>no</b> trigger points, ScenarioRunner waits until the ego <b>starts driving</b> from its spawn —
if nothing drives the ego, NPCs never start. To start everything at once, place a Trigger on the ego's spawn.</li>
<li>Each NPC / pedestrian has a <b>Trigger</b> field: <b>Global</b> starts it with the scenario; choosing a
specific trigger makes that actor wait until the ego reaches that trigger's radius.</li>
</ul>

<a name="settings"></a><h2>8. Scenario settings &amp; criteria</h2>
<p><b>Edit → Scenario Settings</b> (Ctrl+,) or double-click <b>Criteria</b> in the tree:</p>
<ul>
<li><b>Name</b> — letters, digits and <code>_</code> only (it becomes the Python class name). Description.</li>
<li><b>Weather</b> — clouds, rain, wind, sun azimuth/altitude (negative altitude = night), fog, wetness.</li>
<li><b>Criteria</b> — collision fails the test; reaching the goal succeeds; optional timeout.</li>
</ul>
<p>The <b>Issues</b> panel lists problems live: <span class="note">errors</span> block export, warnings don't.
Click an issue to jump to the actor.</p>

<a name="files"></a><h2>9. Save, export &amp; run</h2>
<ul>
<li><b>Save</b> (Ctrl+S) / <b>Save As</b> write a <code>.scenario.json</code> project; <b>Open</b> (Ctrl+O) loads one.
A <b>*</b> in the title means unsaved changes — you are asked before closing or opening another project.</li>
<li><b>Export</b> (Ctrl+E): pick a folder, then answer <b>Auto-Deploy</b>:
<b>Yes</b> puts the XML + XOSC in <code>srunner/examples/</code> and the Python class in
<code>srunner/scenarios/</code>; <b>No</b> writes everything to the chosen folder. The project JSON always goes to the chosen folder.</li>
<li>Run it (CARLA running, from <code>scenario_runner/</code>):<br>
<code>python3 scenario_runner.py --scenario &lt;Name&gt;_1</code><br>
<span class="muted">Not deployed? Add <code>--configFile &lt;dir&gt;/&lt;Name&gt;.xml --additionalScenario &lt;dir&gt;/&lt;name&gt;.py</code>.</span></li>
<li><b>File → Delete Scenario Files</b> removes a project and every file its export created (asks first).</li>
</ul>

<a name="keys"></a><h2>10. Shortcuts</h2>
""" + SHORTCUTS_HTML


class ScenarioSettingsDialog(QDialog):
    """Dialog for scenario name, description, weather, and criteria settings."""

    def __init__(self, project, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Scenario Settings")
        self.setMinimumWidth(500)
        self.project = project

        layout = QVBoxLayout(self)

        # Basic info
        info_group = QGroupBox("Scenario Info")
        info_layout = QFormLayout()
        self.name_edit = QLineEdit(project.name)
        # Name becomes the Python class + XML type, so it must be an identifier
        self.name_edit.setValidator(QRegExpValidator(QRegExp(r"[A-Za-z_]\w*"), self))
        self.name_edit.setToolTip("Letters, digits and _ only (used as the Python class name)")
        info_layout.addRow("Name:", self.name_edit)
        self.desc_edit = QTextEdit(project.description)
        self.desc_edit.setFixedHeight(64)
        info_layout.addRow("Description:", self.desc_edit)
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # Weather
        w_group = QGroupBox("Weather")
        w_layout = QFormLayout()
        w = project.weather
        self.w_spins = {}
        ranges = {"sun_azimuth_angle": (0, 360), "sun_altitude_angle": (-90, 90)}
        for attr in ["cloudiness", "precipitation", "precipitation_deposits",
                      "wind_intensity", "sun_azimuth_angle", "sun_altitude_angle",
                      "fog_density", "wetness"]:
            spin = QDoubleSpinBox()
            spin.setRange(*ranges.get(attr, (0, 100)))
            spin.setValue(getattr(w, attr))
            w_layout.addRow(attr.replace("_", " ").title() + ":", spin)
            self.w_spins[attr] = spin
        w_group.setLayout(w_layout)
        layout.addWidget(w_group)

        # Criteria
        c_group = QGroupBox("Criteria")
        c_layout = QFormLayout()
        c = project.criteria
        self.collision_cb = QCheckBox()
        self.collision_cb.setChecked(c.collision_fail)
        c_layout.addRow("Collision → Fail:", self.collision_cb)
        self.goal_cb = QCheckBox()
        self.goal_cb.setChecked(c.reach_goal_success)
        c_layout.addRow("Reach Goal → Success:", self.goal_cb)
        self.timeout_cb = QCheckBox()
        self.timeout_cb.setChecked(c.enable_timeout)
        c_layout.addRow("Enable Timeout:", self.timeout_cb)
        self.timeout_spin = QDoubleSpinBox()
        self.timeout_spin.setRange(10, 9999)
        self.timeout_spin.setValue(c.timeout_seconds)
        self.timeout_spin.setSuffix(" s")
        c_layout.addRow("Timeout:", self.timeout_spin)
        c_group.setLayout(c_layout)
        layout.addWidget(c_group)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def apply_to_project(self):
        self.project.name = self.name_edit.text() or self.project.name
        self.project.description = self.desc_edit.toPlainText()
        for attr, spin in self.w_spins.items():
            setattr(self.project.weather, attr, spin.value())
        self.project.criteria.collision_fail = self.collision_cb.isChecked()
        self.project.criteria.reach_goal_success = self.goal_cb.isChecked()
        self.project.criteria.enable_timeout = self.timeout_cb.isChecked()
        self.project.criteria.timeout_seconds = self.timeout_spin.value()


class MainWindow(QMainWindow):
    """Main application window for CARLA Scenario Designer."""

    def __init__(self, carla_client=None):
        super().__init__()
        self.setMinimumSize(1200, 700)
        self.resize(1440, 880)
        self.setWindowIcon(QIcon(theme.LOGO_PATH))

        self._carla_client = carla_client
        self._carla_maps_dict = {}
        self._project = ScenarioProject()
        self._current_path = None  # path of loaded/saved project for in-place Save

        # Undo history: JSON snapshots of the project; dirty = snapshot != saved
        self._history = []
        self._hist_idx = -1
        self._saved_state = None
        self._commit_timer = QTimer(self)
        self._commit_timer.setSingleShot(True)
        self._commit_timer.setInterval(400)  # coalesce typing / spin edits into one undo step
        self._commit_timer.timeout.connect(self._commit)

        # 3D Sync settings
        self._sync_spectator_enabled = True
        self._last_cam_pose = None
        self._sync_timer = QTimer(self)
        self._sync_timer.setInterval(200)
        self._sync_timer.timeout.connect(self._sync_with_carla)
        self._sync_timer.start()

        self._setup_panels()
        self._setup_actions()
        self._setup_menus()
        self._setup_toolbar()
        self._setup_statusbar()
        self._connect_signals()

        self._reset_history()
        self._refresh_all()

        # Dynamic map population and auto-load on startup
        self._populate_carla_maps()
        if self._carla_client:
            QTimer.singleShot(100, self._load_map)

    # --- UI construction -----------------------------------------------------

    def _action(self, text, icon=None, slot=None, shortcut=None, tip=None, checkable=False):
        act = QAction(text, self)
        if icon:
            act.setIcon(theme.icon(icon) if isinstance(icon, str) else icon)
        if shortcut:
            act.setShortcut(QKeySequence(shortcut))
        if tip or shortcut:
            act.setToolTip(f"{tip or text}" + (f"  ({QKeySequence(shortcut).toString()})" if shortcut else ""))
        act.setCheckable(checkable)
        if slot:
            act.triggered.connect(slot)
        self.addAction(act)  # window-wide shortcut even when the toolbar hides it
        return act

    def _setup_actions(self):
        a = self._action
        self.new_action = a("New", "file_plus", self._new_project, "Ctrl+N", "New scenario")
        self.open_action = a("Open…", "folder", self._load_project, "Ctrl+O", "Open a .scenario.json project")
        self.save_action = a("Save", "save", self._save_project, "Ctrl+S", "Save to current file")
        self.save_as_action = a("Save As…", "save", self._save_project_as, "Ctrl+Shift+S")
        self.export_action = a("Export", "export", self._export_scenario, "Ctrl+E",
                               "Generate ScenarioRunner files (XML + Python + XOSC)")
        self.delete_scenario_action = a("Delete Scenario Files…", theme.icon("trash", theme.T["danger"]),
                                        self._delete_scenario,
                                        tip="Delete a scenario project and all its generated files")
        self.quit_action = a("Quit", None, self.close, "Ctrl+Q")

        self.undo_action = a("Undo", "undo", self._undo, "Ctrl+Z")
        self.redo_action = a("Redo", "redo", self._redo, "Ctrl+Shift+Z")
        self.redo_action.setShortcuts([QKeySequence("Ctrl+Shift+Z"), QKeySequence("Ctrl+Y")])
        self.duplicate_action = a("Duplicate", "copy", self._duplicate_selected, "Ctrl+D",
                                  "Duplicate selected NPC / pedestrian / prop / trigger")
        self.delete_action = a("Delete", theme.icon("trash", theme.T["danger"]),
                               self._delete_selected, "Del", "Delete selected object")
        self.settings_action = a("Scenario Settings…", "settings", self._open_settings, "Ctrl+,",
                                 "Name, weather and success / failure criteria")
        self.cancel_action = a("Cancel Tool", None, self._cancel_tools, "Esc")

        self.load_map_action = a("Load Map", "refresh", self._load_map, tip="Load the selected map from CARLA")
        self.fit_action = a("Fit Map", "fit", self._map_canvas.fit_map, "F", "Fit map in view")
        self.sync_action = a("Sync Cam", "camera", self._toggle_sync, checkable=True,
                             tip="Follow the selected actor with the CARLA spectator camera")
        self.sync_action.setChecked(True)

        # Place tools: exclusive, but clicking the active tool again turns it off
        self._place_group = QActionGroup(self)
        self._place_group.setExclusionPolicy(QActionGroup.ExclusionPolicy.ExclusiveOptional)
        self._place_actions = {}
        for key, label, icon_name, key_seq, tip in PLACE_TOOLS:
            act = a(label, theme.icon(icon_name, TOOL_COLORS.get(key)), None, key_seq, tip, checkable=True)
            act.triggered.connect(lambda checked, k=key: self._on_place_action(k, checked))
            self._place_group.addAction(act)
            self._place_actions[key] = act
        self.collect_action = self._place_actions["collect"]

        self.guide_action = a("User Guide", "help", self._show_guide, "F1", "How to use the designer")
        self.shortcuts_action = a("Keyboard Shortcuts", None, self._show_shortcuts)
        self.about_action = a("About", "info", self._show_about)

    def _setup_menus(self):
        mb = self.menuBar()
        m = mb.addMenu("&File")
        for act in (self.new_action, self.open_action, None, self.save_action, self.save_as_action,
                    None, self.export_action, self.delete_scenario_action, None, self.quit_action):
            m.addSeparator() if act is None else m.addAction(act)
        m = mb.addMenu("&Edit")
        for act in (self.undo_action, self.redo_action, None, self.duplicate_action,
                    self.delete_action, None, self.settings_action):
            m.addSeparator() if act is None else m.addAction(act)
        m = mb.addMenu("&Place")
        for key, *_ in PLACE_TOOLS:
            m.addAction(self._place_actions[key])
        m.addSeparator()
        m.addAction(self.cancel_action)
        m = mb.addMenu("&View")
        for dock in (self._tree_dock, self._props_dock, self._issues_dock):
            m.addAction(dock.toggleViewAction())
        m.addSeparator()
        m.addAction(self.fit_action)
        m.addAction(self.load_map_action)
        m.addAction(self.sync_action)
        m = mb.addMenu("&Help")
        m.addAction(self.guide_action)
        m.addAction(self.shortcuts_action)
        m.addAction(self.about_action)

    def _setup_toolbar(self):
        toolbar = QToolBar("Main Toolbar")
        toolbar.setObjectName("main_toolbar")
        toolbar.setIconSize(QSize(18, 18))
        toolbar.setMovable(False)
        toolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.addToolBar(toolbar)

        # Brand
        brand = QWidget()
        bl = QHBoxLayout(brand)
        bl.setContentsMargins(4, 0, 6, 0)
        bl.setSpacing(8)
        logo = QLabel()
        logo.setPixmap(theme.logo_pixmap(28))
        bl.addWidget(logo)
        title = QLabel("Scenario Designer")
        title.setObjectName("brand")
        bl.addWidget(title)
        toolbar.addWidget(brand)
        toolbar.addSeparator()

        # Map selector
        self._map_combo = QComboBox()
        self._map_combo.setToolTip("CARLA map (town)")
        self._map_combo.addItems(CARLA_MAPS)
        self._map_combo.setCurrentText(self._project.town)
        self._map_combo.currentTextChanged.connect(self._on_map_changed)  # after initial set: no edit yet
        toolbar.addWidget(self._map_combo)
        toolbar.addAction(self.load_map_action)
        toolbar.addSeparator()

        icon_only = [self.undo_action, self.redo_action, self.fit_action,
                     self.sync_action, self.settings_action]
        for act in icon_only:
            toolbar.addAction(act)
            toolbar.widgetForAction(act).setToolButtonStyle(Qt.ToolButtonIconOnly)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        toolbar.addWidget(spacer)

        toolbar.addAction(self.save_action)
        toolbar.addAction(self.export_action)
        export_btn = toolbar.widgetForAction(self.export_action)
        export_btn.setObjectName("primary")
        export_btn.setIcon(theme.icon("export", "#FFFFFF"))
        export_btn.style().unpolish(export_btn)
        export_btn.style().polish(export_btn)

        # Second row: place tools (full width so labels never overflow)
        self.addToolBarBreak()
        place_bar = QToolBar("Place Tools")
        place_bar.setObjectName("place_toolbar")
        place_bar.setIconSize(QSize(18, 18))
        place_bar.setMovable(False)
        place_bar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.addToolBar(place_bar)
        lbl = QLabel("PLACE")
        lbl.setObjectName("muted")
        lbl.setContentsMargins(6, 0, 8, 0)
        place_bar.addWidget(lbl)
        for key, *_ in PLACE_TOOLS:
            place_bar.addAction(self._place_actions[key])
            place_bar.widgetForAction(self._place_actions[key]).setCursor(Qt.PointingHandCursor)
        place_bar.addSeparator()
        place_bar.addAction(self.duplicate_action)
        place_bar.addAction(self.delete_action)

    def _dock(self, title, name, widget, area):
        dock = QDockWidget(title, self)
        dock.setObjectName(name)
        dock.setWidget(widget)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable |
                         QDockWidget.DockWidgetFloatable)
        self.addDockWidget(area, dock)
        return dock

    def _setup_panels(self):
        self._object_tree = ObjectTree()
        self._map_canvas = MapCanvas()
        self._properties_panel = PropertiesPanel()
        self._issues_list = QListWidget()
        self._issues_list.setCursor(Qt.PointingHandCursor)

        self.setCentralWidget(self._map_canvas)
        self.setCorner(Qt.BottomLeftCorner, Qt.LeftDockWidgetArea)
        self.setCorner(Qt.BottomRightCorner, Qt.RightDockWidgetArea)
        self._tree_dock = self._dock("Scenario", "scenario_dock", self._object_tree, Qt.LeftDockWidgetArea)
        self._props_dock = self._dock("Properties", "properties_dock", self._properties_panel,
                                      Qt.RightDockWidgetArea)
        self._issues_dock = self._dock("Issues", "issues_dock", self._issues_list, Qt.BottomDockWidgetArea)
        self.resizeDocks([self._tree_dock, self._props_dock], [250, 330], Qt.Horizontal)
        self.resizeDocks([self._issues_dock], [120], Qt.Vertical)

    def _setup_statusbar(self):
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._status_map = QLabel()
        self._status_issues = QLabel()
        connected = self._carla_client is not None
        self._status_conn = QLabel(
            f"<span style='color:{theme.T['ok' if connected else 'warn']}'>&#9679;</span> "
            f"CARLA {'connected' if connected else 'offline'}")
        for w in (self._status_map, self._status_issues, self._status_conn):
            self._statusbar.addPermanentWidget(w)

    def _connect_signals(self):
        self._object_tree.object_selected.connect(self._on_object_selected)
        self._object_tree.object_delete_requested.connect(self._on_object_delete)
        self._object_tree.object_duplicate_requested.connect(self._on_object_duplicate)
        self._object_tree.collect_waypoints_requested.connect(self._start_collect)
        self._object_tree.settings_requested.connect(self._open_settings)
        self._map_canvas.object_selected.connect(self._on_object_selected)
        self._map_canvas.object_moved.connect(self._on_object_moved)
        self._map_canvas.drag_finished.connect(self._commit)
        self._map_canvas.collect_requested.connect(self._start_collect)
        self._map_canvas.collect_mode_changed.connect(self._on_collect_mode_changed)
        self._map_canvas.placement_mode_changed.connect(self._on_placement_mode_changed)
        self._properties_panel.properties_changed.connect(self._on_properties_changed)
        self._properties_panel.snap_to_start_requested.connect(self._snap_npc_to_start)
        self._properties_panel.sample_distance_changed.connect(self._set_sample_distance)
        self._properties_panel.stop_collect_requested.connect(self._stop_collect)
        self._issues_list.itemClicked.connect(self._on_issue_clicked)

    # --- Refresh / status ----------------------------------------------------

    def _selection(self):
        return self._properties_panel._current_type, self._properties_panel._current_ref

    def _refresh_issues(self):
        issues = self._project.issues()
        self._issues_list.clear()
        if not issues:
            item = QListWidgetItem(theme.icon("check", theme.T["ok"]), "No issues — ready to export")
            item.setData(Qt.UserRole, (None, None))
            self._issues_list.addItem(item)
        style = {"ERROR": ("error", theme.T["danger"]), "WARNING": ("alert", theme.T["warn"]),
                 "INFO": ("info", theme.T["muted"])}
        for level, msg, obj_type, obj_ref in issues:
            icon_name, color = style[level]
            item = QListWidgetItem(theme.icon(icon_name, color), msg)
            item.setData(Qt.UserRole, (obj_type, obj_ref))
            if obj_ref is not None:
                item.setToolTip("Click to select")
            self._issues_list.addItem(item)
        n_err = sum(1 for i in issues if i[0] == "ERROR")
        n_warn = sum(1 for i in issues if i[0] == "WARNING")
        self._issues_dock.setWindowTitle(f"Issues ({n_err + n_warn})" if n_err + n_warn else "Issues")
        if n_err:
            chip = f"<span style='color:{theme.T['danger']}'>&#9679; {n_err} error{'s' * (n_err > 1)}</span>"
        elif n_warn:
            chip = f"<span style='color:{theme.T['warn']}'>&#9679; {n_warn} warning{'s' * (n_warn > 1)}</span>"
        else:
            chip = f"<span style='color:{theme.T['ok']}'>&#9679; Ready to export</span>"
        self._status_issues.setText(chip)
        self._status_map.setText(f"{self._project.town}  ·  {len(self._project.get_all_objects())} objects")

    def _update_title(self):
        label = os.path.basename(self._current_path) if self._current_path else self._project.name
        dirty = " *" if self._is_dirty() else ""
        self.setWindowTitle(f"Micropolis Scenario Designer — {label}{dirty}")
        self.undo_action.setEnabled(self._hist_idx > 0)
        self.redo_action.setEnabled(self._hist_idx < len(self._history) - 1)

    def _refresh_all(self):
        sel_type, sel_ref = self._selection()
        self._object_tree.refresh(self._project)
        self._object_tree.select_object(sel_type, sel_ref)
        self._map_canvas.refresh_markers(self._project)
        self._refresh_issues()
        self._update_title()

    def _changed(self):
        """Call after every structural edit: redraw + one undo step."""
        self._refresh_all()
        self._commit()

    def _warn(self, msg):
        self._statusbar.showMessage(msg, 6000)

    # --- Undo / redo / dirty -------------------------------------------------

    def _snapshot(self):
        return json.dumps(self._project.to_dict(), sort_keys=True)

    def _is_dirty(self):
        return self._saved_state is not None and self._snapshot() != self._saved_state

    def _reset_history(self):
        self._commit_timer.stop()
        self._history = [self._snapshot()]
        self._hist_idx = 0
        self._saved_state = self._history[0]
        self._update_title()

    def _commit(self):
        self._commit_timer.stop()
        snap = self._snapshot()
        if snap != self._history[self._hist_idx]:
            del self._history[self._hist_idx + 1:]
            self._history.append(snap)
            # ponytail: full-project JSON snapshots; switch to diffs if projects get huge
            if len(self._history) > 200:
                self._history.pop(0)
            self._hist_idx = len(self._history) - 1
        self._update_title()

    def _restore(self, state):
        self._map_canvas.stop_collect_mode()
        self._map_canvas.set_placement_mode(None)
        sel_type, sel_ref = self._selection()
        sel_uid = getattr(sel_ref, "uid", None)
        self._project = ScenarioProject.from_dict(json.loads(state))
        self._map_combo.blockSignals(True)
        if self._map_combo.findText(self._project.town) < 0:
            self._map_combo.addItem(self._project.town)
        self._map_combo.setCurrentText(self._project.town)
        self._map_combo.blockSignals(False)
        # Reselect the same object (by uid) if it still exists
        target = None
        for obj_type, obj in self._project.get_all_objects():
            if obj.uid == sel_uid:
                target = (sel_type if sel_type == "ego_goal" and obj.goal else obj_type, obj)
        self._properties_panel.clear()
        self._map_canvas.select_object(None, None)
        self._refresh_all()
        if target:
            self._on_object_selected(*target)

    def _undo(self):
        self._commit()  # flush a pending property edit so it can be undone
        if self._hist_idx > 0:
            self._hist_idx -= 1
            self._restore(self._history[self._hist_idx])

    def _redo(self):
        self._commit()
        if self._hist_idx < len(self._history) - 1:
            self._hist_idx += 1
            self._restore(self._history[self._hist_idx])

    def _confirm_discard(self):
        """True if it's OK to drop the current project (saved, discarded, or clean)."""
        self._commit()
        if not self._is_dirty():
            return True
        reply = QMessageBox.question(
            self, "Unsaved Changes", f"Save changes to '{self._project.name}'?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel, QMessageBox.Save)
        if reply == QMessageBox.Save:
            self._save_project()
            return not self._is_dirty()
        return reply == QMessageBox.Discard

    def closeEvent(self, event):
        if self._confirm_discard():
            self._sync_timer.stop()
            event.accept()
        else:
            event.ignore()

    # --- Toolbar actions -----------------------------------------------------

    def _populate_carla_maps(self):
        if self._carla_client:
            try:
                maps = self._carla_client.get_available_maps()
                self._carla_maps_dict.clear()
                parsed_maps = []
                for m in maps:
                    name = m.split('/')[-1]
                    if name:
                        self._carla_maps_dict[name] = m
                        if name not in parsed_maps:
                            parsed_maps.append(name)
                parsed_maps.sort()
                if parsed_maps:
                    self._map_combo.blockSignals(True)
                    self._map_combo.clear()
                    self._map_combo.addItems(parsed_maps)

                    try:
                        current_world_map_full = self._carla_client.get_world().get_map().name
                        current_world_map = current_world_map_full.split('/')[-1]
                        self._project.town = current_world_map
                        idx = self._map_combo.findText(current_world_map)
                        if idx >= 0:
                            self._map_combo.setCurrentIndex(idx)
                        else:
                            self._map_combo.addItem(current_world_map)
                            self._carla_maps_dict[current_world_map] = current_world_map_full
                            self._map_combo.setCurrentIndex(self._map_combo.count() - 1)
                    except Exception as e:
                        print(f"Error getting current CARLA map: {e}")

                    self._map_combo.blockSignals(False)
                    self._reset_history()  # town sync from server isn't a user edit
            except Exception as e:
                print(f"Error fetching available maps: {e}")
            try:
                library = self._carla_client.get_world().get_blueprint_library()
                self._properties_panel.set_prop_models(sorted(bp.id for bp in library.filter("static.prop.*")))
            except Exception as e:
                print(f"Error fetching prop blueprints: {e}")

    def _on_map_changed(self, town):
        self._project.town = town
        self._changed()
        if self._carla_client:
            self._warn(f"Map set to {town} — press Load Map to render it")

    def _load_map(self):
        town = self._map_combo.currentText()
        self._project.town = town

        if self._carla_client:
            try:
                # Load the selected map in CARLA
                current_map = self._carla_client.get_world().get_map().name.split('/')[-1]
                if current_map != town:
                    self._statusbar.showMessage(f"Loading {town} in CARLA...", 5000)
                    full_path = self._carla_maps_dict.get(town, town)
                    self._carla_client.load_world(full_path)
                    import time
                    time.sleep(3)  # wait for world to load

                success = self._map_canvas.load_map_from_carla(self._carla_client, town)
                if success:
                    self._statusbar.showMessage(f"Map {town} loaded successfully", 3000)
                    self._refresh_all()
            except Exception as e:
                QMessageBox.warning(self, "CARLA Error",
                                    f"Failed to load map from CARLA:\n{e}\n\n"
                                    "Make sure CARLA server is running.")
        else:
            QMessageBox.information(self, "No CARLA Connection",
                                    "CARLA client not connected.\n"
                                    "Start CARLA server first, then relaunch the tool.\n\n"
                                    "You can still design scenarios — coordinates "
                                    "must be entered manually.")

    # --- Place tools ---------------------------------------------------------

    def _on_place_action(self, key, checked):
        if key == "collect":
            self._toggle_collect_mode(checked)
            return
        if not checked:
            self._map_canvas.set_placement_mode(None)
            return
        self._map_canvas.stop_collect_mode()
        starters = {"ego": self._add_ego, "ego_goal": self._add_goal, "npc": self._add_npc,
                    "ped": self._add_pedestrian, "prop": self._add_prop, "trigger": self._add_trigger,
                    "waypoint": self._add_waypoint}
        starters[key]()
        self._on_placement_mode_changed(self._map_canvas._placement_mode or "")

    def _on_placement_mode_changed(self, mode):
        """Keep the checked Place tool in sync with the canvas mode."""
        for key, act in self._place_actions.items():
            if key != "collect":
                act.setChecked(key == mode)

    def _cancel_tools(self):
        self._map_canvas.set_placement_mode(None)
        self._map_canvas.stop_collect_mode()

    def _add_ego(self):
        if self._project.ego:
            self._warn("Only one ego vehicle allowed — drag it to move, or delete it first.")
            return
        self._map_canvas.set_placement_mode("ego", self._place_ego)

    def _place_ego(self, wx, wy):
        ego = EgoVehicle(spawn=Transform(x=wx, y=wy, z=0.5))
        self._project.ego = ego
        self._changed()
        self._on_object_selected("ego", ego)

    def _add_goal(self):
        if not self._project.ego:
            self._warn("Add the ego vehicle first.")
            return
        self._map_canvas.set_placement_mode("ego_goal", self._place_goal)

    def _place_goal(self, wx, wy):
        self._project.ego.goal = Transform(x=wx, y=wy, z=0.5)
        self._changed()
        self._on_object_selected("ego_goal", self._project.ego)

    def _add_npc(self):
        self._map_canvas.set_placement_mode("npc", self._place_npc)

    def _place_npc(self, wx, wy):
        name = ScenarioProject.next_name("npc", self._project.npc_vehicles)
        npc = NPCVehicle(name=name, spawn=Transform(x=wx, y=wy, z=0.5))
        self._project.npc_vehicles.append(npc)
        self._changed()
        self._on_object_selected("npc", npc)

    def _add_pedestrian(self):
        self._map_canvas.set_placement_mode("ped", self._place_ped)

    def _place_ped(self, wx, wy):
        name = ScenarioProject.next_name("ped", self._project.pedestrians)
        ped = Pedestrian(name=name, spawn=Transform(x=wx, y=wy, z=0.5))
        self._project.pedestrians.append(ped)
        self._changed()
        self._on_object_selected("ped", ped)

    def _add_prop(self):
        self._map_canvas.set_placement_mode("prop", self._place_prop)

    def _place_prop(self, wx, wy):
        name = ScenarioProject.next_name("prop", self._project.static_props)
        prop = StaticProp(name=name, spawn=Transform(x=wx, y=wy, z=self._map_canvas.ground_z(wx, wy)))
        self._project.static_props.append(prop)
        self._changed()
        self._on_object_selected("prop", prop)

    def _add_trigger(self):
        self._map_canvas.set_placement_mode("trigger", self._place_trigger)

    def _place_trigger(self, wx, wy):
        name = ScenarioProject.next_name("trigger", self._project.trigger_points)
        trig = TriggerPoint(name=name, transform=Transform(x=wx, y=wy, z=0.5))
        self._project.trigger_points.append(trig)
        self._changed()
        self._on_object_selected("trigger", trig)

    def _add_waypoint(self):
        selected_type, selected_ref = self._selection()
        if selected_type not in ("npc", "ped"):
            self._warn("Select an NPC vehicle or pedestrian first to edit its route.")
            return
        self._map_canvas.set_placement_mode("waypoint", lambda wx, wy: self._place_waypoint(wx, wy, selected_ref))
        self._statusbar.showMessage(f"Adding route waypoints for {selected_ref.name}", 5000)

    def _place_waypoint(self, wx, wy, actor_ref):
        if self._map_canvas.carla_map:
            try:
                import carla
                loc = carla.Location(x=wx, y=wy, z=0.5)
                if isinstance(actor_ref, NPCVehicle):
                    wp = self._map_canvas.carla_map.get_waypoint(loc, lane_type=carla.LaneType.Driving)
                    if wp:
                        wx = wp.transform.location.x
                        wy = wp.transform.location.y
                else:
                    from PyQt5.QtCore import QPointF
                    # Find nearest sidewalk waypoint
                    nearest_wp = None
                    min_dist = 999999.0
                    for sw_wp in self._map_canvas.sidewalk_wps:
                        dist = loc.distance(sw_wp.transform.location)
                        if dist < min_dist:
                            min_dist = dist
                            nearest_wp = sw_wp

                    # Check if on crosswalk
                    px, py = self._map_canvas.world_to_pixel_coords(wx, wy)
                    pt = QPointF(px, py)
                    on_crosswalk = False
                    for poly in self._map_canvas.crosswalk_polys:
                        if poly.containsPoint(pt, Qt.OddEvenFill):
                            on_crosswalk = True
                            break

                    if min_dist < 3.0:
                        if nearest_wp:
                            wx = nearest_wp.transform.location.x
                            wy = nearest_wp.transform.location.y
                    elif on_crosswalk:
                        pass
                    else:
                        if nearest_wp:
                            wx = nearest_wp.transform.location.x
                            wy = nearest_wp.transform.location.y
            except Exception as e:
                print(f"Waypoint snap error: {e}")

        actor_ref.route.append(RouteWaypoint(x=wx, y=wy, z=0.5))
        self._changed()
        obj_type = "npc" if isinstance(actor_ref, NPCVehicle) else "ped"
        self._on_object_selected(obj_type, actor_ref)

    # --- Collect-waypoint mode ---

    def _toggle_collect_mode(self, checked):
        if checked:
            npc = self._selection()[1]
            if not isinstance(npc, NPCVehicle):
                self.collect_action.setChecked(False)
                self._warn("Select an NPC vehicle first to collect its route.")
                return
            self._start_collect(npc)
        else:
            self._map_canvas.stop_collect_mode()

    def _stop_collect(self):
        """Stop collection from the properties-panel Stop button."""
        self.collect_action.setChecked(False)
        self._map_canvas.stop_collect_mode()

    def _start_collect(self, npc):
        self._on_object_selected("npc", npc)
        self._map_canvas.start_collect_mode(npc)
        self._statusbar.showMessage(
            f"Collecting route for {npc.name} — drag the vehicle or drive with arrow keys; "
            "Space toggles lane snap; Esc to finish.", 0)

    def _on_collect_mode_changed(self, active):
        # Keep the toolbar toggle in sync with the canvas state
        if self.collect_action.isChecked() != active:
            self.collect_action.setChecked(active)
        # Flip the label so the checked button clearly reads as a stop control
        self.collect_action.setText("Stop" if active else "Collect")
        self.collect_action.setIcon(theme.icon("stop" if active else "route"))
        # Mirror the active state to the properties-panel Stop button
        self._properties_panel.set_collecting(active)
        if not active:
            self._changed()
            self._statusbar.showMessage("Route collection finished.", 3000)

    def _snap_npc_to_start(self, npc):
        if not npc.route:
            return
        start = npc.route[0]
        npc.spawn.x = start.x
        npc.spawn.y = start.y
        if start.yaw is not None:
            npc.spawn.yaw = start.yaw
        self._changed()
        self._on_object_selected("npc", npc)

    def _set_sample_distance(self, dist):
        self._map_canvas._collect_sample_dist = dist

    def _toggle_sync(self, checked):
        self._sync_spectator_enabled = checked
        self._last_cam_pose = None
        self._statusbar.showMessage(f"Spectator camera sync {'enabled' if checked else 'disabled'}", 3000)

    def _selected_pose(self):
        selected_type, item = self._selection()
        if item is None:
            return None
        if selected_type == "ego_goal":
            t = item.goal
            return (t.x, t.y, t.z, 0.0) if t else None
        t = item.transform if selected_type == "trigger" else item.spawn
        return (t.x, t.y, t.z, t.yaw)

    def _sync_with_carla(self):
        if not self._carla_client or not self._sync_spectator_enabled:
            return
        pose = self._selected_pose()
        # Only talk to CARLA when the target moved: leaves the user's free camera alone
        # and avoids a blocking RPC on the GUI thread every tick.
        if pose is None or pose == self._last_cam_pose:
            return
        self._last_cam_pose = pose
        try:
            import carla
            x, y, z, yaw = pose
            yaw_rad = math.radians(yaw)
            # Position camera 12m back, 18m high looking down
            loc = carla.Location(x=x - 12 * math.cos(yaw_rad), y=y - 12 * math.sin(yaw_rad), z=z + 18)
            rot = carla.Rotation(pitch=-50, yaw=yaw, roll=0)
            self._carla_client.get_world().get_spectator().set_transform(carla.Transform(loc, rot))
        except Exception as e:
            self.sync_action.setChecked(False)
            self._sync_spectator_enabled = False
            self._warn(f"Camera sync disabled — CARLA not reachable ({e})")

    def _open_settings(self):
        dlg = ScenarioSettingsDialog(self._project, self)
        if dlg.exec_() == QDialog.Accepted:
            dlg.apply_to_project()
            self._changed()

    def _show_guide(self):
        """Non-modal so it can stay open next to the map while you work."""
        if getattr(self, "_guide", None) is None:
            self._guide = QDialog(self)
            self._guide.setWindowTitle("Scenario Designer — User Guide")
            self._guide.resize(780, 680)
            browser = QTextBrowser()
            browser.setObjectName("guide")
            browser.document().setDefaultStyleSheet(GUIDE_CSS)
            browser.setHtml(GUIDE_HTML)
            QVBoxLayout(self._guide).addWidget(browser)
        self._guide.show()
        self._guide.raise_()
        self._guide.activateWindow()

    def _show_shortcuts(self):
        QMessageBox.information(self, "Keyboard Shortcuts", SHORTCUTS_HTML)

    def _show_about(self):
        box = QMessageBox(self)
        box.setWindowTitle("About")
        box.setIconPixmap(theme.logo_pixmap(64, 12))
        box.setText("<h3>Micropolis Scenario Designer</h3>"
                    "<p>Visual editor for CARLA ScenarioRunner scenarios: place the ego, NPCs, "
                    "pedestrians and triggers on the map, then export XML + Python + OpenSCENARIO.</p>")
        box.exec_()

    # --- Signal handlers ---

    def _on_object_selected(self, obj_type, obj_ref):
        self._properties_panel.set_triggers(self._project.trigger_points)
        self._properties_panel.show_properties(obj_type, obj_ref)
        self._object_tree.select_object(obj_type, obj_ref)
        self._map_canvas.select_object(obj_type, obj_ref)
        self._last_cam_pose = None

    def _on_object_moved(self, obj_type, obj_ref):
        if self._selection() == (obj_type, obj_ref):
            self._properties_panel.update_transform()  # in place: no form rebuild per mouse move
        else:
            self._on_object_selected(obj_type, obj_ref)
        self._refresh_issues()

    def _on_issue_clicked(self, item):
        obj_type, obj_ref = item.data(Qt.UserRole)
        if obj_ref is None:
            return
        self._on_object_selected(obj_type, obj_ref)
        t = obj_ref.goal if obj_type == "ego_goal" else getattr(obj_ref, "spawn", None) or obj_ref.transform
        self._map_canvas.centerOn(*self._map_canvas.world_to_pixel_coords(t.x, t.y))

    def _delete_selected(self):
        obj_type, obj_ref = self._selection()
        if obj_ref is None:
            self._warn("Nothing selected to delete.")
            return
        self._on_object_delete(obj_type, obj_ref)

    def _on_object_delete(self, obj_type, obj_ref):
        if self._map_canvas._collect_npc is obj_ref:
            self._stop_collect()
        if obj_type == "ego":
            self._project.ego = None
        elif obj_type == "ego_goal":
            obj_ref.goal = None
        elif obj_type == "npc":
            self._project.npc_vehicles = [v for v in self._project.npc_vehicles if v.uid != obj_ref.uid]
        elif obj_type == "ped":
            self._project.pedestrians = [p for p in self._project.pedestrians if p.uid != obj_ref.uid]
        elif obj_type == "prop":
            self._project.static_props = [p for p in self._project.static_props if p.uid != obj_ref.uid]
        elif obj_type == "trigger":
            self._project.trigger_points = [t for t in self._project.trigger_points if t.uid != obj_ref.uid]
        self._properties_panel.clear()
        self._map_canvas.select_object(None, None)
        self._changed()

    def _duplicate_selected(self):
        self._on_object_duplicate(*self._selection())

    def _on_object_duplicate(self, obj_type, obj_ref):
        kinds = {"npc": (NPCVehicle, self._project.npc_vehicles, "npc"),
                 "ped": (Pedestrian, self._project.pedestrians, "ped"),
                 "prop": (StaticProp, self._project.static_props, "prop"),
                 "trigger": (TriggerPoint, self._project.trigger_points, "trigger")}
        if obj_type not in kinds:
            self._warn("Select an NPC, pedestrian, prop or trigger to duplicate.")
            return
        cls, items, prefix = kinds[obj_type]
        d = obj_ref.to_dict()
        d.pop("uid")  # from_dict generates a fresh uid
        d["name"] = ScenarioProject.next_name(prefix, items)
        copy = cls.from_dict(d)
        t = copy.transform if obj_type == "trigger" else copy.spawn
        t.x += 4.0 * math.cos(math.radians(t.yaw))  # 4 m ahead so both stay clickable
        t.y += 4.0 * math.sin(math.radians(t.yaw))
        items.append(copy)
        self._changed()
        self._on_object_selected(obj_type, copy)

    def _on_properties_changed(self):
        self._object_tree.refresh(self._project)
        self._object_tree.select_object(*self._selection())
        self._map_canvas.refresh_markers(self._project)
        self._refresh_issues()
        self._commit_timer.start()

    # --- File operations ---

    def _new_project(self):
        if not self._confirm_discard():
            return
        self._cancel_tools()
        self._project = ScenarioProject(town=self._map_combo.currentText() or "Town03")
        self._current_path = None
        self._properties_panel.clear()
        self._map_canvas.select_object(None, None)
        self._reset_history()
        self._refresh_all()
        self._statusbar.showMessage("New scenario", 3000)

    def _mark_saved(self, path):
        self._current_path = path
        self._saved_state = self._snapshot()
        self._update_title()
        self._statusbar.showMessage(f"Project saved: {path}", 5000)

    def _save_project(self):
        """Save in place to the current file; fall back to Save As if none."""
        if not self._current_path:
            self._save_project_as()
            return
        self._commit()
        self._project.save(self._current_path)
        self._mark_saved(self._current_path)

    def _save_project_as(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Project", self._current_path or f"{self._project.name}.scenario.json",
            "Scenario Project (*.scenario.json)")
        if path:
            self._commit()
            self._project.save(path)
            self._mark_saved(path)

    def _load_project(self):
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Project", "",
            "Scenario Project (*.scenario.json)")
        if not path:
            return
        try:
            self._cancel_tools()
            self._project = ScenarioProject.load(path)
            self._current_path = path  # remember for in-place Save
            self._properties_panel.clear()
            self._map_canvas.select_object(None, None)
            # Sync combo to the loaded town without firing _on_map_changed mid-load.
            # addItem first if the town isn't in the static CARLA_MAPS list, else
            # setCurrentText is a no-op and a later _load_map() would load the wrong map.
            town = self._project.town
            self._map_combo.blockSignals(True)
            if self._map_combo.findText(town) < 0:
                self._map_combo.addItem(town)
            self._map_combo.setCurrentText(town)
            self._map_combo.blockSignals(False)
            self._reset_history()

            # Re-render the map: _refresh_all only redraws markers, it does NOT load
            # map topology / _world_offset, so without this the map is blank and
            # actors land at stale pixel coords.
            if self._carla_client:
                self._load_map()  # loads world + renders map + _refresh_all()
            else:
                self._refresh_all()
                self._statusbar.showMessage(
                    f"Project loaded (offline — start CARLA + 'Load Map' to render {town}): {path}",
                    6000)
                return
            self._statusbar.showMessage(f"Project loaded: {path}", 5000)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", str(e))

    def _delete_scenario(self):
        """Delete a scenario project file and all its generated children
        (XML config, Python class, XOSC) wherever Export placed them."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Delete Scenario — select project file", "",
            "Scenario Project (*.scenario.json)")
        if not path:
            return

        # Derive the base name Export used from the project, not the filename.
        try:
            name = ScenarioProject.load(path).name
        except Exception:
            name = os.path.splitext(os.path.basename(path))[0].replace(".scenario", "")
        py_name = name.lower().replace(" ", "_")

        # Mirror export's srunner destinations (SCENARIO_RUNNER_ROOT or relative).
        env_root = os.getenv("SCENARIO_RUNNER_ROOT")
        if env_root and os.path.isdir(env_root):
            base_dir = os.path.abspath(env_root)
        else:
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
        srunner_examples = os.path.join(base_dir, "srunner", "examples")
        srunner_scenarios = os.path.join(base_dir, "srunner", "scenarios")

        # Candidate related files: the project dir (non-deploy export) + srunner dirs.
        proj_dir = os.path.dirname(path)
        candidates = [
            path,
            os.path.join(proj_dir, f"{name}.xml"),
            os.path.join(proj_dir, f"{name}.xosc"),
            os.path.join(proj_dir, f"{py_name}.py"),
            os.path.join(srunner_examples, f"{name}.xml"),
            os.path.join(srunner_examples, f"{name}.xosc"),
            os.path.join(srunner_scenarios, f"{py_name}.py"),
        ]
        # Dedupe, keep only existing files.
        targets = []
        for f in candidates:
            f = os.path.abspath(f)
            if f not in targets and os.path.isfile(f):
                targets.append(f)

        reply = QMessageBox.warning(
            self, "Delete Scenario",
            "Permanently delete these files? This cannot be undone.\n\n"
            + "\n".join(f"  • {t}" for t in targets),
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if reply != QMessageBox.Yes:
            return

        deleted, failed = [], []
        for t in targets:
            try:
                os.remove(t)
                deleted.append(t)
            except OSError as e:
                failed.append(f"{t} ({e})")

        # If the deleted project was the one open, reset the in-place Save target.
        if self._current_path and os.path.abspath(self._current_path) == os.path.abspath(path):
            self._current_path = None
            self._update_title()

        msg = f"Deleted {len(deleted)} file(s)."
        if failed:
            msg += "\n\nFailed:\n" + "\n".join(failed)
            QMessageBox.critical(self, "Delete Scenario", msg)
        self._statusbar.showMessage(msg.replace("\n", " "), 6000)

    def _export_scenario(self):
        """Export scenario files (XML + Python + optional XOSC)."""
        self._commit()
        warnings = self._project.validate()
        errors = [w for w in warnings if "ERROR" in w]
        if errors:
            self._issues_dock.show()
            QMessageBox.critical(self, "Validation Error",
                                 "Cannot export — fix errors first:\n\n" + "\n".join(errors))
            return

        # Choose output directory
        out_dir = QFileDialog.getExistingDirectory(self, "Choose Export Directory")
        if not out_dir:
            return

        reply = QMessageBox.question(self, 'Auto-Deploy',
                                     'Do you want to automatically copy the generated files to their respective ScenarioRunner directories?\n\n'
                                     '- XML/XOSC -> srunner/examples/\n'
                                     '- Python -> srunner/scenarios/',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        auto_deploy = (reply == QMessageBox.Yes)

        name = self._project.name
        py_name = name.lower().replace(" ", "_")
        try:
            # Pick destinations up front: when auto-deploying, scenario files go
            # straight to their srunner dirs (no duplicate copy in out_dir); only
            # the project .json stays in the chosen export dir.
            deploy_msg = ""
            if auto_deploy:
                # Honor SCENARIO_RUNNER_ROOT env var if it exists, otherwise use relative path
                env_root = os.getenv("SCENARIO_RUNNER_ROOT")
                if env_root and os.path.isdir(env_root):
                    base_dir = os.path.abspath(env_root)
                else:
                    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

                srunner_examples = os.path.join(base_dir, "srunner", "examples")
                srunner_scenarios = os.path.join(base_dir, "srunner", "scenarios")
                os.makedirs(srunner_examples, exist_ok=True)
                os.makedirs(srunner_scenarios, exist_ok=True)
                xml_dir = xosc_dir = srunner_examples
                py_dir = srunner_scenarios
                deploy_msg = f"\n\nDeployed to:\n  - {srunner_examples}\n  - {srunner_scenarios}"
            else:
                xml_dir = xosc_dir = py_dir = out_dir

            # 1. XML config
            xml_path = os.path.join(xml_dir, f"{name}.xml")
            save_xml(self._project, xml_path)

            # 2. Python scenario class
            py_path = os.path.join(py_dir, f"{py_name}.py")
            save_python(self._project, py_path)

            # 3. OpenSCENARIO (optional)
            xosc_path = os.path.join(xosc_dir, f"{name}.xosc")
            save_xosc(self._project, xosc_path)

            # 4. Project file — always kept in the chosen export dir
            proj_path = os.path.join(out_dir, f"{name}.scenario.json")
            self._project.save(proj_path)

            msg = (f"Scenario exported to:\n\n"
                   f"  XML     {xml_path}\n"
                   f"  Python  {py_path}\n"
                   f"  XOSC    {xosc_path}\n"
                   f"  Project {proj_path}{deploy_msg}\n\n"
                   f"To run with ScenarioRunner:\n"
                   f"  python scenario_runner.py --scenario {name}_1")
            QMessageBox.information(self, "Export Complete", msg)
            self._statusbar.showMessage(f"Exported to {out_dir}", 5000)

        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))
