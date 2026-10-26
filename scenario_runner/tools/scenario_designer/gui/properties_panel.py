"""
PropertiesPanel — Right panel for editing selected object attributes.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit,
                             QDoubleSpinBox, QComboBox, QCheckBox, QLabel,
                             QGroupBox, QScrollArea, QPushButton)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont

from . import theme


# Common vehicle blueprints in CARLA
VEHICLE_MODELS = [
    "vehicle.lincoln.mkz_2017",
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.audi.tt",
    "vehicle.bmw.grandtourer",
    "vehicle.chevrolet.impala",
    "vehicle.dodge.charger_2020",
    "vehicle.ford.mustang",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.mercedes.coupe_2020",
    "vehicle.mini.cooper_s",
    "vehicle.nissan.patrol",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2",
]

PEDESTRIAN_MODELS = [
    "walker.pedestrian.0001",
    "walker.pedestrian.0002",
    "walker.pedestrian.0003",
    "walker.pedestrian.0004",
    "walker.pedestrian.0005",
    "walker.pedestrian.0006",
    "walker.pedestrian.0007",
    "walker.pedestrian.0008",
    "walker.pedestrian.0009",
    "walker.pedestrian.0010",
]

# Fallback prop blueprints (replaced by the server's static.prop.* list when CARLA is connected)
PROP_MODELS = [
    "static.prop.trafficcone01",
    "static.prop.trafficcone02",
    "static.prop.constructioncone",
    "static.prop.streetbarrier",
    "static.prop.chainbarrier",
    "static.prop.warningconstruction",
    "static.prop.warningaccident",
    "static.prop.trafficwarning",
    "static.prop.barrel",
    "static.prop.box01",
    "static.prop.creasedbox01",
    "static.prop.ironplank",
    "static.prop.dirtdebris01",
    "static.prop.garbage01",
]

# obj_type -> (title prefix, icon, color)
HEADERS = {
    "ego":      ("Ego Vehicle", "car", "#1E90FF"),
    "ego_goal": ("Ego Goal", "flag", "#00C850"),
    "npc":      ("NPC", "car", "#FF5050"),
    "ped":      ("Pedestrian", "person", "#FFB428"),
    "prop":     ("Prop", "cone", "#00C8C8"),
    "trigger":  ("Trigger", "zap", "#B450FF"),
}


class PropertiesPanel(QWidget):
    """Property editor for selected scenario objects."""

    properties_changed = pyqtSignal()
    snap_to_start_requested = pyqtSignal(object)   # npc ref
    sample_distance_changed = pyqtSignal(float)    # collect-mode spacing (m)
    stop_collect_requested = pyqtSignal()          # user clicked Stop Collecting

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(290)

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(0)

        # Header: type icon + title
        header = QWidget()
        hl = QHBoxLayout(header)
        hl.setContentsMargins(12, 10, 12, 10)
        self._icon = QLabel()
        hl.addWidget(self._icon)
        self._title = QLabel("No Selection")
        self._title.setFont(QFont(self.font().family(), 12, QFont.Bold))
        hl.addWidget(self._title, 1)
        self._layout.addWidget(header)

        # Scroll area for properties
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self._form_widget = QWidget()
        self._form_layout = QFormLayout(self._form_widget)
        self._form_layout.setContentsMargins(12, 4, 12, 12)
        self._form_layout.setVerticalSpacing(8)
        self._form_layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        scroll.setWidget(self._form_widget)
        self._layout.addWidget(scroll)

        self._current_type = None
        self._current_ref = None
        self._widgets = {}
        self._transform_spins = []     # (transform_obj, attr, spin) for in-place drag updates
        self._triggers = []            # available TriggerPoint list for the NPC trigger combo
        self._prop_models = list(PROP_MODELS)
        self._collecting = False       # collect-mode active state
        self._stop_collect_btn = None  # Stop-Collecting button on the current NPC form (or None)
        self._show_empty()

    def set_triggers(self, triggers):
        """Update the list of trigger points offered in the NPC trigger selector."""
        self._triggers = list(triggers)

    def set_prop_models(self, models):
        """Prop blueprint ids offered in the model combo (e.g. from the CARLA server)."""
        if models:
            self._prop_models = list(models)

    def set_collecting(self, active):
        """Reflect collect-mode state on the NPC Stop-Collecting button."""
        self._collecting = active
        if self._stop_collect_btn is not None:
            self._stop_collect_btn.setEnabled(active)

    def show_properties(self, obj_type, obj_ref):
        """Display properties for selected object."""
        self._current_type = obj_type
        self._current_ref = obj_ref
        self._clear_form()
        self._set_header(obj_type, getattr(obj_ref, "name", ""))

        if obj_type == "ego":
            self._show_ego(obj_ref)
        elif obj_type == "ego_goal":
            self._show_ego_goal(obj_ref)
        elif obj_type == "npc":
            self._show_npc(obj_ref)
        elif obj_type == "ped":
            self._show_pedestrian(obj_ref)
        elif obj_type == "prop":
            self._show_prop(obj_ref)
        elif obj_type == "trigger":
            self._show_trigger(obj_ref)

    def update_transform(self):
        """Refresh coordinate fields in place (drag updates) without rebuilding the form."""
        for t, attr, spin in self._transform_spins:
            spin.blockSignals(True)
            spin.setValue(getattr(t, attr))
            spin.blockSignals(False)

    def _set_header(self, obj_type, name=""):
        prefix, icon_name, color = HEADERS.get(obj_type, ("No Selection", "info", theme.T["muted"]))
        self._icon.setPixmap(theme.icon(icon_name, color).pixmap(20, 20))
        show_name = name and obj_type in ("npc", "ped", "prop", "trigger")
        self._title.setText(f"{prefix}: {name}" if show_name else prefix)

    def _show_empty(self):
        self._set_header(None)
        hint = QLabel("Select an object on the map or in the Scenario tree, "
                      "or pick a tool from the Place toolbar.")
        hint.setObjectName("muted")
        hint.setWordWrap(True)
        self._form_layout.addRow(hint)

    def _clear_form(self):
        while self._form_layout.count():
            item = self._form_layout.takeAt(0)
            if item.widget():
                item.widget().hide()  # deleteLater is deferred; don't paint under the new form
                item.widget().deleteLater()
        self._widgets.clear()
        self._transform_spins = []
        self._stop_collect_btn = None  # button belongs to the form being cleared

    def _set(self, obj, attr, value):
        """Single write path for every edit: update the model and notify listeners."""
        setattr(obj, attr, value)
        self.properties_changed.emit()

    def _spin(self, lo, hi, value, decimals=2, step=1.0):
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _add_name_field(self, obj):
        name_edit = QLineEdit(obj.name)
        name_edit.textChanged.connect(lambda t: (self._set(obj, "name", t),
                                                 self._set_header(self._current_type, t)))
        self._form_layout.addRow("Name:", name_edit)
        self._widgets["name"] = name_edit

    def _add_model_field(self, obj, models):
        model_combo = QComboBox()
        model_combo.addItems(models)
        model_combo.setEditable(True)
        idx = model_combo.findText(obj.model)
        if idx >= 0:
            model_combo.setCurrentIndex(idx)
        else:
            model_combo.setEditText(obj.model)
        model_combo.currentTextChanged.connect(lambda t: self._set(obj, "model", t))
        self._form_layout.addRow("Model:", model_combo)
        self._widgets["model"] = model_combo

    def _add_transform_fields(self, label, transform_obj):
        """Add x/y/z/yaw spin boxes for a transform."""
        group = QGroupBox(label)
        layout = QFormLayout()
        layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        for attr, unit in [("x", " m"), ("y", " m"), ("z", " m"), ("yaw", "°")]:
            spin = self._spin(-10000, 10000, getattr(transform_obj, attr), 2, 0.5)
            spin.setSuffix(unit)
            spin.setFont(theme.mono_font())
            spin.valueChanged.connect(lambda v, a=attr, t=transform_obj: self._set(t, a, v))
            layout.addRow(attr.upper() + ":", spin)
            self._widgets[f"{label}_{attr}"] = spin
            self._transform_spins.append((transform_obj, attr, spin))
        group.setLayout(layout)
        self._form_layout.addRow(group)

    def _add_trigger_selector(self, actor):
        """Which point fires this actor ("Global" = shared scenario trigger)."""
        trigger_combo = QComboBox()
        trigger_combo.addItem("Global (scenario trigger)", None)
        sel_idx = 0
        for t in self._triggers:
            trigger_combo.addItem(t.name, t.uid)
            if actor.trigger_uid is not None and t.uid == actor.trigger_uid:
                sel_idx = trigger_combo.count() - 1
        trigger_combo.setCurrentIndex(sel_idx)
        trigger_combo.currentIndexChanged.connect(
            lambda _i, c=trigger_combo: self._set(actor, "trigger_uid", c.currentData()))
        self._form_layout.addRow("Trigger:", trigger_combo)
        self._widgets["trigger"] = trigger_combo

    def _danger_button(self, text, slot):
        btn = QPushButton(theme.icon("trash", theme.T["danger"]), text)
        btn.setObjectName("danger")
        btn.setCursor(Qt.PointingHandCursor)
        btn.clicked.connect(slot)
        return btn

    def _show_ego(self, ego):
        self._add_name_field(ego)
        self._add_model_field(ego, VEHICLE_MODELS)
        self._add_transform_fields("Spawn Position", ego.spawn)

    def _show_ego_goal(self, ego):
        if ego.goal:
            self._add_transform_fields("Goal Position", ego.goal)
            radius = self._spin(0.5, 100, ego.goal_radius)
            radius.setSuffix(" m")
            radius.valueChanged.connect(lambda v: self._set(ego, "goal_radius", v))
            self._form_layout.addRow("Goal Radius:", radius)
            self._widgets["goal_radius"] = radius

    def _show_npc(self, npc):
        self._add_name_field(npc)
        self._add_model_field(npc, VEHICLE_MODELS)
        self._add_transform_fields("Spawn Position", npc.spawn)
        # Behavior
        b = npc.behavior
        group = QGroupBox("Behavior")
        blayout = QFormLayout()
        blayout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        parked = QCheckBox()
        parked.setChecked(npc.parked)
        parked.setToolTip("Stays where it spawns for the whole scenario (hand brake, no driving).")
        blayout.addRow("Parked:", parked)
        self._widgets["parked"] = parked
        speed = self._spin(0, 100, b.speed, 1)
        speed.setSuffix(" m/s")
        speed.valueChanged.connect(lambda v: self._set(b, "speed", v))
        blayout.addRow("Speed:", speed)
        brake = self._spin(0, 1, b.max_brake, 2, 0.05)
        brake.valueChanged.connect(lambda v: self._set(b, "max_brake", v))
        blayout.addRow("Max Brake:", brake)
        lane = QComboBox()
        lane.addItems(["none", "left", "right"])
        lane.setCurrentText(b.lane_change)
        lane.currentTextChanged.connect(lambda t: self._set(b, "lane_change", t))
        blayout.addRow("Lane Change:", lane)
        stop_int = QCheckBox()
        stop_int.setChecked(b.stop_at_intersection)
        stop_int.toggled.connect(lambda v: self._set(b, "stop_at_intersection", v))
        blayout.addRow("Stop at Intersection:", stop_int)
        dist = self._spin(0, 200, b.distance_to_stop)
        dist.setSuffix(" m")
        dist.valueChanged.connect(lambda v: self._set(b, "distance_to_stop", v))
        blayout.addRow("Stop Distance:", dist)
        rand_vel = QCheckBox()
        rand_vel.setChecked(b.random_velocity)
        blayout.addRow("Random Velocity:", rand_vel)
        vel_std = self._spin(0, 50, b.velocity_std, 1)
        vel_std.setSuffix(" m/s")
        vel_std.setEnabled(b.random_velocity)
        vel_std.valueChanged.connect(lambda v: self._set(b, "velocity_std", v))
        rand_vel.toggled.connect(lambda v: (self._set(b, "random_velocity", v), vel_std.setEnabled(v)))
        blayout.addRow("Velocity Std:", vel_std)

        driving = [speed, brake, lane, stop_int, dist, rand_vel]

        def apply_parked(v):
            for w in driving:
                w.setEnabled(not v)
            vel_std.setEnabled(not v and rand_vel.isChecked())
        apply_parked(npc.parked)
        parked.toggled.connect(lambda v: (self._set(npc, "parked", v), apply_parked(v)))
        self._widgets["speed"] = speed
        group.setLayout(blayout)
        self._form_layout.addRow(group)
        # Route info
        route_group = QGroupBox("Route / Path")
        route_layout = QVBoxLayout()
        route_layout.addWidget(QLabel(f"Waypoints defined: {len(npc.route)}"))
        # Collect-mode sample spacing (used while capturing a route)
        sample_row = QFormLayout()
        sample_spin = self._spin(0.5, 50.0, 2.0, 1, 0.5)
        sample_spin.setSuffix(" m")
        sample_spin.setToolTip("Distance between waypoints recorded in Collect Route mode")
        sample_spin.valueChanged.connect(self.sample_distance_changed.emit)
        sample_row.addRow("Sample Dist:", sample_spin)
        route_layout.addLayout(sample_row)
        # Snap vehicle to the first waypoint of the captured route
        snap_btn = QPushButton(theme.icon("crosshair"), "Snap to Start of Route")
        snap_btn.setEnabled(bool(npc.route))
        snap_btn.clicked.connect(lambda: self.snap_to_start_requested.emit(npc))
        route_layout.addWidget(snap_btn)
        # Stop-Collecting button (enabled only while collect mode is active)
        stop_btn = QPushButton(theme.icon("stop"), "Stop Collecting")
        stop_btn.setEnabled(self._collecting)
        stop_btn.clicked.connect(lambda: self.stop_collect_requested.emit())
        route_layout.addWidget(stop_btn)
        self._stop_collect_btn = stop_btn
        route_layout.addWidget(self._danger_button("Clear Route", lambda: self._clear_actor_route(npc)))
        route_group.setLayout(route_layout)
        self._form_layout.addRow(route_group)
        self._add_trigger_selector(npc)
        # Color
        color_edit = QLineEdit(npc.color or "")
        color_edit.setPlaceholderText("R,G,B (e.g. 255,0,0)")
        color_edit.textChanged.connect(lambda t: self._set(npc, "color", t if t else None))
        self._form_layout.addRow("Color:", color_edit)

    def _show_pedestrian(self, ped):
        self._add_name_field(ped)
        self._add_model_field(ped, PEDESTRIAN_MODELS)
        self._add_transform_fields("Spawn Position", ped.spawn)
        speed = self._spin(0, 10, ped.speed, 1)
        speed.setSuffix(" m/s")
        speed.valueChanged.connect(lambda v: self._set(ped, "speed", v))
        self._form_layout.addRow("Speed:", speed)

        wheelchair_check = QCheckBox()
        wheelchair_check.setChecked(ped.use_wheelchair)
        wheelchair_check.toggled.connect(lambda v: self._set(ped, "use_wheelchair", v))
        self._form_layout.addRow("Use Wheelchair:", wheelchair_check)

        # Route info
        route_group = QGroupBox("Route / Path")
        route_layout = QVBoxLayout()
        route_layout.addWidget(QLabel(f"Waypoints defined: {len(ped.route)}"))
        route_layout.addWidget(self._danger_button("Clear Route", lambda: self._clear_actor_route(ped)))
        route_group.setLayout(route_layout)
        self._form_layout.addRow(route_group)
        self._add_trigger_selector(ped)

    def _show_prop(self, prop):
        self._add_name_field(prop)
        self._add_model_field(prop, self._prop_models)
        self._add_transform_fields("Spawn Position", prop.spawn)
        physics = QCheckBox()
        physics.setChecked(prop.physics)
        physics.setToolTip("On: the prop can be pushed / knocked over. Off: fixed obstacle.")
        physics.toggled.connect(lambda v: self._set(prop, "physics", v))
        self._form_layout.addRow("Simulate Physics:", physics)
        self._widgets["physics"] = physics

    def _show_trigger(self, trigger):
        self._add_name_field(trigger)
        self._add_transform_fields("Position", trigger.transform)
        radius = self._spin(0.5, 100, trigger.radius)
        radius.setSuffix(" m")
        radius.valueChanged.connect(lambda v: self._set(trigger, "radius", v))
        self._form_layout.addRow("Radius:", radius)
        self._widgets["radius"] = radius

    def _clear_actor_route(self, actor):
        actor.route.clear()
        self.properties_changed.emit()
        self.show_properties(self._current_type, self._current_ref)

    def clear(self):
        self._clear_form()
        self._current_type = None
        self._current_ref = None
        self._show_empty()
