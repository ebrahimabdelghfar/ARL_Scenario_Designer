"""
ObjectTree — QTreeWidget showing the scenario object hierarchy.
"""

from PyQt5.QtWidgets import QTreeWidget, QTreeWidgetItem, QMenu
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QColor, QBrush, QIcon

from . import theme

# (obj_type, category label, icon, color) — colors match the map markers
CATEGORIES = [
    ("ego",     "Ego Vehicle",    "car",    "#1E90FF"),
    ("npc",     "NPC Vehicles",   "car",    "#FF5050"),
    ("ped",     "Pedestrians",    "person", "#FFB428"),
    ("prop",    "Static Props",   "cone",   "#00C8C8"),
    ("trigger", "Trigger Points", "zap",    "#B450FF"),
    ("criteria", "Criteria",      "check",  theme.T["ok"]),
]


class ObjectTree(QTreeWidget):
    """Scenario object tree: Ego → NPCs → Pedestrians → Props → Triggers → Criteria."""

    object_selected = pyqtSignal(str, object)
    object_delete_requested = pyqtSignal(str, object)
    object_duplicate_requested = pyqtSignal(str, object)
    collect_waypoints_requested = pyqtSignal(object)  # npc ref
    settings_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setHeaderHidden(True)
        self.setMinimumWidth(200)
        self.setIndentation(14)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._context_menu)
        self.itemClicked.connect(self._on_item_clicked)
        self.itemDoubleClicked.connect(self._on_item_double_clicked)

        self._nodes = {}
        for key, label, icon_name, color in CATEGORIES:
            node = QTreeWidgetItem(self, [label])
            # Off-state only: expanded tree nodes would otherwise show the dark On (checked) glyph.
            node.setIcon(0, QIcon(theme.icon(icon_name, color).pixmap(36, 36)))
            node.setFlags(Qt.ItemIsEnabled)
            font = node.font(0)
            font.setBold(True)
            node.setFont(0, font)
            node.setExpanded(True)
            self._nodes[key] = node
        self._ego_node = self._nodes["ego"]
        self._npc_node = self._nodes["npc"]
        self._ped_node = self._nodes["ped"]
        self._prop_node = self._nodes["prop"]
        self._trigger_node = self._nodes["trigger"]
        self._criteria_node = self._nodes["criteria"]

        self._item_map = {}  # id(QTreeWidgetItem) -> (type, obj_ref)

    def _add(self, parent, text, obj_type, obj_ref, color):
        item = QTreeWidgetItem(parent, [text])
        item.setForeground(0, QBrush(QColor(color)))
        self._item_map[id(item)] = (obj_type, obj_ref)
        return item

    def refresh(self, project):
        """Rebuild tree from project data."""
        for node in self._nodes.values():
            node.takeChildren()
        self._item_map.clear()

        colors = {key: color for key, _, _, color in CATEGORIES}
        if project.ego:
            self._add(self._ego_node, f"{project.ego.name}  ·  {project.ego.model.split('.')[-1]}",
                      "ego", project.ego, colors["ego"])
            if project.ego.goal:
                goal = self._add(self._ego_node, "Goal Point", "ego_goal", project.ego, "#00C850")
                goal.setIcon(0, QIcon(theme.icon("flag", "#00C850").pixmap(36, 36)))
        for npc in project.npc_vehicles:
            parked = "  ·  parked" if npc.parked else ""
            self._add(self._npc_node, f"{npc.name}  ·  {npc.model.split('.')[-1]}{parked}", "npc", npc, colors["npc"])
        for ped in project.pedestrians:
            self._add(self._ped_node, f"{ped.name}  ·  {ped.model.split('.')[-1]}", "ped", ped, colors["ped"])
        for prop in project.static_props:
            self._add(self._prop_node, f"{prop.name}  ·  {prop.model.split('.')[-1]}", "prop", prop, colors["prop"])
        for trig in project.trigger_points:
            self._add(self._trigger_node, trig.name, "trigger", trig, colors["trigger"])

        for key, label, _, _ in CATEGORIES[1:5]:
            self._nodes[key].setText(0, f"{label} ({self._nodes[key].childCount()})")

        # Criteria summary (double-click opens Scenario Settings)
        c = project.criteria
        lines = []
        if c.collision_fail:
            lines.append("Collision → Fail")
        if c.reach_goal_success:
            lines.append("Reach Goal → Success")
        if c.enable_timeout:
            lines.append(f"Timeout: {c.timeout_seconds:g}s")
        for text in lines:
            item = QTreeWidgetItem(self._criteria_node, [text])
            item.setForeground(0, QBrush(QColor(theme.T["muted"])))
        self._criteria_node.setToolTip(0, "Double-click to edit criteria")

        self.expandAll()

    def select_object(self, obj_type, obj_ref):
        """Highlight the tree row for obj_ref (no signal emitted)."""
        for i in range(self.topLevelItemCount()):
            node = self.topLevelItem(i)
            for j in range(node.childCount()):
                child = node.child(j)
                if self._item_map.get(id(child)) == (obj_type, obj_ref):
                    self.setCurrentItem(child)
                    return
        self.clearSelection()

    def _on_item_clicked(self, item, column):
        key = id(item)
        if key in self._item_map:
            obj_type, obj_ref = self._item_map[key]
            self.object_selected.emit(obj_type, obj_ref)

    def _on_item_double_clicked(self, item, column):
        if item is self._criteria_node or item.parent() is self._criteria_node:
            self.settings_requested.emit()

    def _context_menu(self, pos):
        item = self.itemAt(pos)
        if not item:
            return
        key = id(item)
        if key not in self._item_map:
            return

        obj_type, obj_ref = self._item_map[key]
        menu = QMenu(self)
        collect_action = dup_action = None
        if obj_type == "npc":
            collect_action = menu.addAction(theme.icon("route"), "Collect Waypoints")
        if obj_type in ("npc", "ped", "prop", "trigger"):
            dup_action = menu.addAction(theme.icon("copy"), "Duplicate")
        if not menu.isEmpty():
            menu.addSeparator()
        delete_action = menu.addAction(theme.icon("trash", theme.T["danger"]), "Delete")
        action = menu.exec_(self.viewport().mapToGlobal(pos))
        if action is None:
            return
        if action == delete_action:
            self.object_delete_requested.emit(obj_type, obj_ref)
        elif action == dup_action:
            self.object_duplicate_requested.emit(obj_type, obj_ref)
        elif action == collect_action:
            self.collect_waypoints_requested.emit(obj_ref)
