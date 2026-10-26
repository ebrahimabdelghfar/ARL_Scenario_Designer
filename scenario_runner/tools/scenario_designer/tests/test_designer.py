"""
Scenario Designer tests — models, generators and the GUI driven offscreen (no CARLA).

Run from scenario_runner/ (autoload off: the sourced ROS launch_testing plugin breaks pytest 9):
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tools/scenario_designer/tests -q
"""

import glob
import json
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

SR_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if SR_ROOT not in sys.path:
    sys.path.insert(0, SR_ROOT)

import pytest
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from tools.scenario_designer.models.project import ScenarioProject
from tools.scenario_designer.models.actors import EgoVehicle, NPCVehicle, StaticProp, Transform
from tools.scenario_designer.generators.xml_generator import generate_xml
from tools.scenario_designer.generators.python_generator import generate_python
from tools.scenario_designer.generators.xosc_generator import generate_xosc


@pytest.fixture(scope="module")
def app():
    from tools.scenario_designer.gui import theme
    app = QApplication.instance() or QApplication([])
    theme.apply(app)
    return app


@pytest.fixture
def win(app):
    from tools.scenario_designer.gui.main_window import MainWindow
    w = MainWindow(carla_client=None)
    w._confirm_discard = lambda: True  # never block on a modal in tests
    yield w
    w._project = ScenarioProject()  # clean so closeEvent never prompts
    w.close()


def _sample_project():
    p = ScenarioProject(name="Sample")
    p.ego = EgoVehicle(spawn=Transform(x=10, y=20))
    p.ego.goal = Transform(x=50, y=20)
    p.npc_vehicles.append(NPCVehicle(name="npc_1", spawn=Transform(x=30, y=25)))
    p.weather.fog_density = 40.0
    p.weather.wetness = 55.0
    return p


# --- models / generators ---------------------------------------------------

@pytest.mark.parametrize("path", sorted(glob.glob(os.path.join(SR_ROOT, "*.scenario.json"))))
def test_repo_projects_roundtrip(path):
    with open(path) as f:
        raw = json.load(f)
    proj = ScenarioProject.load(path)
    again = ScenarioProject.from_dict(proj.to_dict())
    assert again.to_dict() == proj.to_dict()
    assert proj.name == raw["name"]


def test_python_compiles_and_xml_type_matches_class():
    p = _sample_project()
    code = generate_python(p)
    compile(code, "generated.py", "exec")
    assert f"class {p.name}(" in code
    xml = generate_xml(p)
    assert f'type="{p.name}"' in xml


def test_xml_exports_all_weather_fields():
    xml = generate_xml(_sample_project())
    for attr in ("fog_density", "fog_distance", "wetness", "sun_altitude_angle"):
        assert f'{attr}="' in xml
    assert 'fog_density="40.0"' in xml


def test_issues_and_validate():
    p = ScenarioProject()
    levels = [i[0] for i in p.issues()]
    assert "ERROR" in levels
    assert any(v.startswith("ERROR: No ego vehicle") for v in p.validate())
    p = _sample_project()
    assert not [i for i in p.issues() if i[0] == "ERROR"]
    p.name = "My Scenario-1"
    assert any(i[0] == "ERROR" and "name" in i[1].lower() for i in p.issues())


def test_next_name_skips_taken():
    taken = [NPCVehicle(name="npc_2"), NPCVehicle(name="npc_1")]
    assert ScenarioProject.next_name("npc", taken) == "npc_3"
    assert ScenarioProject.next_name("npc", [NPCVehicle(name="npc_2")]) == "npc_1"


# --- GUI -------------------------------------------------------------------

def test_delete_goal_from_tree(win):
    win._place_ego(0, 0)
    win._place_goal(20, 0)
    assert win._project.ego.goal is not None
    win._on_object_delete("ego_goal", win._project.ego)
    assert win._project.ego is not None
    assert win._project.ego.goal is None


def test_auto_names_unique_after_delete(win):
    win._place_npc(1, 1)
    win._place_npc(2, 2)
    first = win._project.npc_vehicles[0]
    win._on_object_delete("npc", first)
    win._place_npc(3, 3)
    names = [n.name for n in win._project.npc_vehicles]
    assert len(names) == len(set(names)) == 2


def test_trigger_radius_edit_propagates(win):
    win._place_trigger(5, 5)
    trig = win._project.trigger_points[0]
    hits = []
    win._properties_panel.properties_changed.connect(lambda: hits.append(1))
    win._properties_panel._widgets["radius"].setValue(12.0)
    assert trig.radius == 12.0
    assert hits


def test_name_edit_refreshes_tree(win):
    win._place_npc(1, 1)
    npc = win._project.npc_vehicles[0]
    win._properties_panel._widgets["name"].setText("lead_car")
    texts = [win._object_tree._npc_node.child(i).text(0)
             for i in range(win._object_tree._npc_node.childCount())]
    assert any("lead_car" in t for t in texts)
    assert npc.name == "lead_car"


def test_undo_redo(win):
    win._place_ego(0, 0)
    win._place_npc(5, 5)
    assert len(win._project.npc_vehicles) == 1
    win._undo()
    assert len(win._project.npc_vehicles) == 0
    assert win._project.ego is not None
    win._redo()
    assert len(win._project.npc_vehicles) == 1


def test_dirty_title_and_save(win, tmp_path):
    assert "*" not in win.windowTitle()
    win._place_ego(0, 0)
    assert win.windowTitle().endswith("*")
    win._current_path = str(tmp_path / "t.scenario.json")
    win._save_project()
    assert "*" not in win.windowTitle()
    assert os.path.isfile(win._current_path)


def test_new_project_resets(win):
    win._place_ego(0, 0)
    win._new_project()
    assert win._project.ego is None
    assert win._current_path is None


def test_escape_cancels_placement_mode(win):
    act = win._place_actions["npc"]
    act.trigger()
    assert act.isChecked()
    assert win._map_canvas._placement_mode == "npc"
    QTest.keyClick(win._map_canvas, Qt.Key_Escape)
    assert win._map_canvas._placement_mode is None
    assert not act.isChecked()


def test_issues_panel_selects_actor(win):
    win._place_ego(0, 0)
    win._place_npc(0, 0)  # at origin -> WARNING tied to the npc
    npc = win._project.npc_vehicles[0]
    npc.spawn.x = npc.spawn.y = 0.0
    win._refresh_all()
    assert win._issues_list.count() == len(win._project.issues())
    for i in range(win._issues_list.count()):
        item = win._issues_list.item(i)
        if npc.name in item.text():
            win._on_issue_clicked(item)
            break
    else:
        pytest.fail("no issue row for the npc")
    assert win._properties_panel._current_ref is npc


def test_shortcuts_dont_steal_typing(win, app):
    win.show()
    QApplication.setActiveWindow(win)
    win._place_npc(5, 5)
    spin = win._properties_panel._widgets["Spawn Position_x"]
    spin.setFocus()
    spin.lineEdit().selectAll()
    QTest.keyClicks(spin, "34")  # "3" is the NPC tool shortcut
    QTest.keyClick(spin, Qt.Key_Enter)
    assert spin.value() == 34.0
    assert win._map_canvas._placement_mode is None
    win._map_canvas.setFocus()
    QTest.keyClick(win._map_canvas, Qt.Key_3)
    assert win._map_canvas._placement_mode == "npc"
    QTest.keyClick(win._map_canvas, Qt.Key_Escape)
    win._map_canvas.setFocus()
    QTest.keyClick(win._map_canvas, Qt.Key_Delete)
    assert not win._project.npc_vehicles


# --- static props ----------------------------------------------------------

def test_props_roundtrip_and_export():
    p = _sample_project()
    p.static_props.append(StaticProp(name="cone_a", model="static.prop.constructioncone",
                                     spawn=Transform(x=12, y=21, z=0.0, yaw=90), physics=True))
    again = ScenarioProject.from_dict(json.loads(json.dumps(p.to_dict())))
    assert again.to_dict() == p.to_dict()
    assert ScenarioProject.from_dict({"name": "Old"}).static_props == []  # pre-prop projects load
    code = generate_python(p)
    compile(code, "generated.py", "exec")
    assert "request_new_actor('static.prop.constructioncone'" in code
    assert "set_simulate_physics(True)" in code
    # props spawn after NPCs so behavior indices (other_actors[0] = npc) stay valid
    assert code.index("npc_0_tf") < code.index("prop_0_tf")
    assert 'model="static.prop.constructioncone"' in generate_xml(p)
    assert "<MiscObject" in generate_xosc(p) and 'entityRef="prop_0"' in generate_xosc(p)


def test_prop_place_edit_duplicate_delete(win):
    win._place_actions["prop"].trigger()
    assert win._map_canvas._placement_mode == "prop"
    win._map_canvas._placement_callback(7, 8)
    prop = win._project.static_props[0]
    assert (prop.spawn.x, prop.spawn.y, prop.name) == (7, 8, "prop_1")
    assert win._object_tree._prop_node.text(0) == "Static Props (1)"
    assert win._map_canvas._actor_markers[prop.uid]._label.toPlainText() == "prop_1"
    win._properties_panel._widgets["physics"].setChecked(True)
    assert prop.physics
    win._duplicate_selected()
    assert [p.name for p in win._project.static_props] == ["prop_1", "prop_2"]
    win._delete_selected()
    assert [p.name for p in win._project.static_props] == ["prop_1"]
    win._undo()
    assert len(win._project.static_props) == 2


# --- HD map renderer (synthetic map, no CARLA server) ------------------------

def _fake_two_way_road(length=40):
    """Straight road along +x: lane -1 (yaw 0) / +1 (yaw 180), each with a 2 m sidewalk on its right."""
    carla = pytest.importorskip("carla")
    from types import SimpleNamespace as NS
    lanes = {}  # (lane_id, i) -> waypoint

    class WP:
        def __init__(self, lane_id, i, y, yaw, lane_type, width):
            x = i if yaw == 0 else length - i
            self.transform = carla.Transform(carla.Location(x=x, y=y), carla.Rotation(yaw=yaw))
            self.lane_id, self.i, self.lane_type, self.lane_width = lane_id, i, lane_type, width
            self.road_id, self.section_id, self.is_junction = 1, 0, False
            center = carla.LaneMarkingType.SolidSolid
            self.left_lane_marking = NS(type=center, color=carla.LaneMarkingColor.Yellow)
            self.right_lane_marking = NS(type=carla.LaneMarkingType.Solid, color=carla.LaneMarkingColor.White)

        def next_until_lane_end(self, _):
            return [lanes[(self.lane_id, j)] for j in range(self.i + 1, length + 1)]

        def get_right_lane(self):
            return lanes.get((self.lane_id + (-1 if self.lane_id < 0 else 1), self.i))

        def get_left_lane(self):
            return lanes.get((-self.lane_id, self.i)) if abs(self.lane_id) == 1 else None

    drv, side = carla.LaneType.Driving, carla.LaneType.Sidewalk
    for i in range(length + 1):
        lanes[(-1, i)] = WP(-1, i, 1.75, 0, drv, 3.5)
        lanes[(-2, i)] = WP(-2, i, 4.5, 0, side, 2.0)
        lanes[(1, i)] = WP(1, i, -1.75, 180, drv, 3.5)
        lanes[(2, i)] = WP(2, i, -4.5, 180, side, 2.0)
    m = NS(get_topology=lambda: [(lanes[(-1, 0)], lanes[(-1, length)]), (lanes[(1, 0)], lanes[(1, length)])],
           get_crosswalks=lambda: [], get_spawn_points=lambda: [], to_opendrive=lambda: "<OpenDRIVE/>",
           # driving surface is |y| <= 3.5 (two 3.5 m lanes)
           get_waypoint=lambda loc, project_to_road=True, lane_type=None: object() if abs(loc.y) <= 3.5 else None)
    return NS(get_world=lambda: NS(get_map=lambda: m)), length


def test_hd_map_renderer_on_synthetic_road(app):
    from PyQt5.QtGui import QPainterPath
    from tools.scenario_designer.gui.map_canvas import MapCanvas, COLORS
    client, length = _fake_two_way_road()
    cv = MapCanvas()
    assert cv.load_map_from_carla(client, "Fake")
    # both sidewalks (one per side) collected for pedestrian snapping
    assert len(cv.sidewalk_wps) == 2 * (length + 1)
    paths = [it for it in cv._scene.items() if hasattr(it, "path")]
    road = [p for p in paths if p.brush().color() == COLORS["road"]]
    assert len(road) == 1  # all lanes in one seamless fill
    yellow = [p for p in paths if p.pen().color() == COLORS["marking_yellow"]]
    assert len(yellow) == 1
    moves = sum(yellow[0].path().elementAt(i).type == QPainterPath.MoveToElement
                for i in range(yellow[0].path().elementCount()))
    assert moves == 2  # double center line drawn once (lane +1 skips the shared center)
    assert not cv._empty_label.isVisible()


def test_crosswalks_from_xodr_line_and_arc():
    from tools.scenario_designer.gui.map_canvas import crosswalks_from_xodr
    import math
    xodr = """<OpenDRIVE>
      <road id="1" length="20" junction="-1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="20"><line/></geometry></planView>
        <objects><object type="crosswalk" s="10" t="0" hdg="1.5707963267948966">
          <outline><cornerLocal u="5" v="1"/><cornerLocal u="-5" v="1"/><cornerLocal u="-5" v="-1"/></outline>
        </object></objects></road>
      <road id="2" length="15.7" junction="-1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="15.7"><arc curvature="0.1"/></geometry></planView>
        <objects><object type="crosswalk" s="15.707963267948966" t="2" hdg="0">
          <outline><cornerLocal u="0" v="0"/><cornerLocal u="1" v="0"/><cornerLocal u="0" v="1"/></outline>
        </object></objects></road>
    </OpenDRIVE>"""
    line, arc = crosswalks_from_xodr(xodr)
    # line road: center (10, 0), rotated 90 deg; CARLA flips y
    for got, want in zip(line, [(9, -5), (9, 5), (11, 5)]):
        assert math.dist(got, want) < 1e-6
    # quarter arc of radius 10 ends at (10, 10) heading +90 deg; t=2 shifts left (-x) -> (8, 10) -> CARLA (8, -10)
    assert math.dist(arc[0], (8, -10)) < 1e-6
    assert math.dist(arc[1], (8, -11)) < 1e-6  # u along heading (+y in ODR) -> -y in CARLA
    spiral = xodr.replace('<arc curvature="0.1"/>', '<spiral curvStart="0" curvEnd="0.1"/>')
    assert crosswalks_from_xodr(spiral) is None  # unsupported -> caller falls back to CARLA's list


def test_user_guide_opens_non_modal(win):
    from PyQt5.QtWidgets import QTextBrowser
    win.guide_action.trigger()
    assert win._guide.isVisible() and not win._guide.isModal()
    text = win._guide.findChild(QTextBrowser).toPlainText()
    for section in ("Quick workflow", "Placing objects", "Routes", "Triggers", "Save, export", "Shortcuts"):
        assert section in text
    assert win.guide_action.shortcut().toString() == "F1"
    win.guide_action.trigger()  # second open reuses the same window
    assert len([d for d in win.findChildren(type(win._guide)) if d.windowTitle().endswith("User Guide")]) == 1


def test_parked_npc_export_and_ui(win):
    p = _sample_project()
    driving = NPCVehicle(name="mover", spawn=Transform(x=40, y=25))
    p.npc_vehicles.append(driving)
    p.npc_vehicles[0].parked = True
    p.npc_vehicles[0].route.append(__import__("tools.scenario_designer.models.actors",
                                               fromlist=["RouteWaypoint"]).RouteWaypoint(x=1, y=1))
    assert ScenarioProject.from_dict(p.to_dict()).npc_vehicles[0].parked
    code = generate_python(p)
    compile(code, "generated.py", "exec")
    assert code.count("hand_brake=True") == 1
    assert "WaypointFollower(self.other_actors[0]" not in code  # parked: no drive branch
    assert "WaypointFollower(self.other_actors[1]" in code      # the other NPC still drives
    assert "ActorDestroy(self.other_actors[0])" in code         # still cleaned up
    assert any("parked" in i[1] and i[0] == "INFO" for i in p.issues())
    # UI: checkbox sets the flag and greys out the driving fields
    win._place_npc(5, 5)
    npc = win._project.npc_vehicles[0]
    win._properties_panel._widgets["parked"].setChecked(True)
    assert npc.parked and not win._properties_panel._widgets["speed"].isEnabled()
    assert win._map_canvas._actor_markers[npc.uid]._label.toPlainText() == "npc_1 (P)"
