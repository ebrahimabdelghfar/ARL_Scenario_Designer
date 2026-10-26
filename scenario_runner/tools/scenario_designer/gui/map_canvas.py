"""
MapCanvas — QGraphicsView-based CARLA map visualization and interactive editor.

Connects to running CARLA server to extract map topology, renders roads,
and allows placing/moving scenario objects (ego, NPCs, pedestrians, triggers).
"""

import math
from PyQt5.QtWidgets import (QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
                             QGraphicsPolygonItem, QGraphicsLineItem, QGraphicsTextItem,
                             QGraphicsPathItem, QMenu, QAction, QMessageBox)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import (QPen, QBrush, QColor, QPainterPath, QPolygonF,
                          QFont, QTransform, QPainter)
# Colors for different object types
COLORS = {
    "road":       QColor(50, 55, 62),     # asphalt
    "shoulder":   QColor(50, 55, 62),     # = road: shoulders are asphalt; edge line marks the lane
    "parking":    QColor(56, 64, 78),     # bluish grey parking
    "biking":     QColor(84, 58, 62),     # reddish biking
    "sidewalk":   QColor(92, 99, 108),    # concrete
    "curb":       QColor(112, 120, 130),  # border lanes between road and sidewalk
    "median":     QColor(42, 56, 50),     # greenish median
    "restricted": QColor(66, 58, 64),
    "crosswalk":  QColor(235, 238, 240, 190),  # zebra stripes
    "marking_white":  QColor(235, 238, 240, 220),
    "marking_yellow": QColor(240, 190, 40, 230),
    "arrow":      QColor(255, 255, 255, 60),
    "building":   QColor(95, 90, 85, 180),   # warm grey buildings
    "building_edge": QColor(70, 65, 60),
    "vegetation": QColor(45, 80, 55, 120),  # soft green
    "ego":        QColor(30, 144, 255),   # dodger blue
    "ego_goal":   QColor(0, 200, 80),     # green
    "npc":        QColor(255, 80, 80),    # red
    "pedestrian": QColor(255, 180, 40),   # orange
    "trigger":    QColor(180, 80, 255),   # purple
    "prop":       QColor(0, 200, 200),    # cyan
    "route":      QColor(100, 255, 100, 160),
    "spawn_pt":   QColor(255, 255, 255, 70),
    "grid":       QColor(45, 45, 50),
    "background": QColor(17, 23, 30),     # matches theme bg
}

# HD map rendering
MAP_PRECISION = 1.0  # m between lane samples (ponytail: raise for Town12/13 if load gets slow)
JUNCTION_CLOSE_GAP = 2.5  # m: widest corner gap paved inside junctions

# Size of actor markers (in world meters, scaled by pixels_per_meter)
ACTOR_SIZE = 3.0
TRIGGER_SIZE = 4.0
SELECT_RING = QColor(95, 179, 206)

# Placement-mode hints shown in the on-canvas pill
MODE_HINTS = {
    "ego": "Click the map to place the ego vehicle",
    "ego_goal": "Click the map to set the ego goal",
    "npc": "Click the map to place an NPC vehicle",
    "ped": "Click the map to place a pedestrian",
    "trigger": "Click the map to place a trigger point",
    "prop": "Click the map to place a static prop",
    "waypoint": "Click to add route waypoints",
}

from PyQt5.QtWidgets import (QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
                             QGraphicsPolygonItem, QGraphicsLineItem, QGraphicsTextItem,
                             QGraphicsPathItem, QMenu, QAction, QMessageBox, QGraphicsItem,
                             QLabel)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import (QPen, QBrush, QColor, QPainterPath, QPolygonF,
                           QFont, QTransform, QPainter)

def crosswalks_from_xodr(xodr):
    """Crosswalk outlines [(x, y), ...] in CARLA coords, computed from OpenDRIVE crosswalk objects.

    CARLA 0.9.13's Map.get_crosswalks() shifts some crosswalks by about a lane width (Town03),
    so evaluate the road reference line (line/arc) + object s/t/hdg + cornerLocal ourselves.
    Returns None if a crosswalk road uses geometry we don't evaluate (spiral, poly3).
    """
    import xml.etree.ElementTree as ET
    out = []
    for road in ET.fromstring(xodr).iter('road'):
        objs = [o for o in road.iter('object') if o.get('type', '').lower() == 'crosswalk']
        if not objs:
            continue
        geoms = []
        for g in road.find('planView'):
            kind = g[0].tag
            if kind not in ('line', 'arc'):
                return None
            geoms.append((float(g.get('s')), float(g.get('x')), float(g.get('y')), float(g.get('hdg')),
                          float(g[0].get('curvature')) if kind == 'arc' else 0.0))
        geoms.sort()
        for o in objs:
            s = float(o.get('s'))
            s0, x, y, h, k = [g for g in geoms if g[0] <= s + 1e-6][-1]
            ds = s - s0
            if abs(k) < 1e-12:
                rx, ry = x + ds * math.cos(h), y + ds * math.sin(h)
            else:
                rx = x + (math.sin(h + k * ds) - math.sin(h)) / k
                ry = y - (math.cos(h + k * ds) - math.cos(h)) / k
                h += k * ds
            t = float(o.get('t', 0.0))
            px, py = rx - math.sin(h) * t, ry + math.cos(h) * t
            a = h + float(o.get('hdg', 0.0))
            pts = [(px + u * math.cos(a) - v * math.sin(a), -(py + u * math.sin(a) + v * math.cos(a)))
                   for u, v in ((float(c.get('u')), float(c.get('v'))) for c in o.iter('cornerLocal'))]
            if len(pts) >= 3:
                out.append(pts)
    return out


def _side_lane_tables():
    import carla
    lt = carla.LaneType
    types = [lt.Sidewalk, lt.Shoulder, lt.Parking, lt.Biking, lt.Border, lt.Median, lt.Restricted]
    colors = dict(zip(types, [COLORS["sidewalk"], COLORS["shoulder"], COLORS["parking"], COLORS["biking"],
                              COLORS["curb"], COLORS["median"], COLORS["restricted"]]))
    z = dict(zip(types, [2, 1.1, 1.2, 1.3, 1.9, 1.05, 1.05]))
    return types, colors, z


def extend_crosswalk_polygon(vertices, extend_dist=2.5):
    if len(vertices) < 4:
        return vertices
    
    v = list(vertices)
    if len(v) >= 5 and v[0].distance(v[-1]) < 0.1:
        v = v[:-1]
        has_closed = True
    else:
        has_closed = False
        
    if len(v) != 4:
        return vertices
        
    import carla
    d_A = (v[0].distance(v[1]) + v[2].distance(v[3])) / 2.0
    d_B = (v[1].distance(v[2]) + v[3].distance(v[0])) / 2.0
    
    def norm_dir(p1, p2):
        d = p1.distance(p2)
        if d < 0.001:
            return carla.Location(0, 0, 0)
        return carla.Location(x=(p1.x - p2.x)/d, y=(p1.y - p2.y)/d, z=(p1.z - p2.z)/d)
        
    if d_A > d_B:
        dir0 = norm_dir(v[0], v[1])
        dir3 = norm_dir(v[3], v[2])
        dir1 = norm_dir(v[1], v[0])
        dir2 = norm_dir(v[2], v[3])
        
        v[0] = v[0] + dir0 * extend_dist
        v[3] = v[3] + dir3 * extend_dist
        v[1] = v[1] + dir1 * extend_dist
        v[2] = v[2] + dir2 * extend_dist
    else:
        dir1 = norm_dir(v[1], v[2])
        dir0 = norm_dir(v[0], v[3])
        dir2 = norm_dir(v[2], v[1])
        dir3 = norm_dir(v[3], v[0])
        
        v[1] = v[1] + dir1 * extend_dist
        v[0] = v[0] + dir0 * extend_dist
        v[2] = v[2] + dir2 * extend_dist
        v[3] = v[3] + dir3 * extend_dist
        
    if has_closed:
        v.append(v[0])
    return v

class ActorMarker(QGraphicsItem):
    """Draggable marker representing a scenario object on the map."""

    def __init__(self, obj_type, obj_ref, radius, color, canvas):
        super().__init__()
        self.obj_type = obj_type
        self.obj_ref = obj_ref
        self.radius = radius
        self.color = color
        self.canvas = canvas
        self._updating_position = True
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setZValue(10)
        self.setCursor(Qt.PointingHandCursor)

        # Label
        self._label = QGraphicsTextItem(self._get_label(), self)
        self._label.setDefaultTextColor(QColor(255, 255, 255))
        font = QFont("Arial", 8)
        font.setBold(True)
        self._label.setFont(font)
        self._label.setPos(-radius, -radius - 18)

        # Set initial visual rotation from yaw
        yaw = getattr(self.obj_ref, 'spawn', getattr(self.obj_ref, 'transform', None)).yaw
        self.setRotation(yaw)

    def setRotation(self, angle):
        super().setRotation(angle)
        # Keep the name upright and centered above the marker whatever the heading:
        # counter-rotate the label and express its scene-frame offset in local coords.
        lbl = self._label
        lbl.setRotation(-angle)
        br = lbl.boundingRect()
        dx, dy = -br.width() / 2, -self.radius - br.height()
        a = math.radians(-angle)
        lbl.setPos(dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a))

    def _get_label(self):
        if self.obj_type == "ego":
            return "EGO"
        elif self.obj_type == "ego_goal":
            return "GOAL"
        elif self.obj_type == "npc":
            return self.obj_ref.name + (" (P)" if self.obj_ref.parked else "")
        elif self.obj_type == "ped":
            return self.obj_ref.name
        elif self.obj_type in ("trigger", "prop"):
            return self.obj_ref.name
        return "?"

    def boundingRect(self):
        # Return bounds containing the visual shape plus margin
        if self.obj_type in ("ego", "npc"):
            l = 4.5 * self.canvas._pixels_per_meter
            w = 2.0 * self.canvas._pixels_per_meter
            return QRectF(-l/2 - 5, -w/2 - 5, l + 10, w + 10)
        elif self.obj_type == "ped":
            r = 0.5 * 1.5 * self.canvas._pixels_per_meter
            return QRectF(-r - 5, -r - 5, 2*r + 10, 2*r + 10)
        elif self.obj_type == "prop":
            r = 0.6 * self.canvas._pixels_per_meter
            return QRectF(-r - 5, -r - 5, 2*r + 10, 2*r + 10)
        elif self.obj_type == "trigger":
            r = max(5.0, self.obj_ref.radius) * self.canvas._pixels_per_meter
            return QRectF(-r - 5, -r - 5, 2*r + 10, 2*r + 10)
        elif self.obj_type == "ego_goal":
            r = max(1.5, self.obj_ref.goal_radius) * self.canvas._pixels_per_meter
            return QRectF(-r - 5, -r - 5, 2*r + 10, 2*r + 10)
        else:
            r = 1.5 * self.canvas._pixels_per_meter
            return QRectF(-r - 5, -r - 5, 2*r + 10, 2*r + 10)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(self.color.darker(150), 1.5)
        brush = QBrush(self.color)
        painter.setPen(pen)
        painter.setBrush(brush)

        if self.obj_type in ("ego", "npc"):
            # Vehicle shape: oriented rectangle
            l = 4.5 * self.canvas._pixels_per_meter
            w = 2.0 * self.canvas._pixels_per_meter
            rect = QRectF(-l/2, -w/2, l, w)
            painter.drawRect(rect)
            
            # Forward arrow
            painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
            arrow = QPolygonF([
                QPointF(l/4, 0),
                QPointF(0, -w/4),
                QPointF(0, w/4)
            ])
            painter.drawPolygon(arrow)

        elif self.obj_type == "ped":
            # Pedestrian shape: smaller circle with forward line
            r = 0.5 * 1.5 * self.canvas._pixels_per_meter
            painter.drawEllipse(QRectF(-r, -r, 2*r, 2*r))
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawLine(QPointF(0.0, 0.0), QPointF(r, 0.0))

        elif self.obj_type == "prop":
            # Prop shape: small square (1.2 m) with a diagonal so heading/rotation is visible
            r = 0.6 * self.canvas._pixels_per_meter
            painter.drawRect(QRectF(-r, -r, 2*r, 2*r))
            painter.setPen(QPen(QColor(255, 255, 255, 200), 1.5))
            painter.drawLine(QPointF(-r, -r), QPointF(r, r))

        elif self.obj_type == "trigger":
            # Trigger shape: center dot + dashed radius
            r = self.obj_ref.radius * self.canvas._pixels_per_meter
            painter.drawEllipse(-4, -4, 8, 8)
            pen = QPen(self.color, 1.5, Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(-r, -r, 2*r, 2*r))

        elif self.obj_type == "ego_goal":
            # Goal shape: bullseye
            r = 1.5 * self.canvas._pixels_per_meter
            painter.drawEllipse(QRectF(-r, -r, 2*r, 2*r))
            painter.drawEllipse(QRectF(-r/2, -r/2, r, r))
            
            # Dashed success radius circle
            r_trigger = self.obj_ref.goal_radius * self.canvas._pixels_per_meter
            pen = QPen(self.color, 1.5, Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(-r_trigger, -r_trigger, 2*r_trigger, 2*r_trigger))

        if self.isSelected():
            ring = QPen(SELECT_RING, 2)
            ring.setCosmetic(True)
            painter.setPen(ring)
            painter.setBrush(Qt.NoBrush)
            box = self.boundingRect().adjusted(3, 3, -3, -3)
            if self.obj_type in ("ego", "npc"):
                painter.drawRoundedRect(box, 3, 3)
            else:
                painter.drawEllipse(box)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            if getattr(self, '_updating_position', False):
                return super().itemChange(change, value)

            world_pos = self.canvas.pixel_to_world(value)
            wx, wy = world_pos.x(), world_pos.y()
            yaw = getattr(self.obj_ref, 'spawn', getattr(self.obj_ref, 'transform', None)).yaw

            # Are we capturing a route by moving this NPC vehicle?
            collecting = (self.obj_type == "npc" and
                          getattr(self.canvas, "_collect_npc", None) is self.obj_ref)
            # In collect mode the vehicle yaw is fixed (set by the user / arrow steering),
            # and lane snapping is optional (spacebar toggles it).
            do_snap = (not collecting) or self.canvas._collect_lane_restrict

            # Snapping logic
            if self.canvas.carla_map:
                try:
                    import carla
                    loc = carla.Location(x=wx, y=wy, z=0.5)
                    wp = None
                    if self.obj_type in ("ego", "npc"):
                        if do_snap:
                            wp = self.canvas.carla_map.get_waypoint(loc, lane_type=carla.LaneType.Driving)
                        if wp:
                            wx = wp.transform.location.x
                            wy = wp.transform.location.y
                            if not collecting:
                                yaw = wp.transform.rotation.yaw
                            px, py = self.canvas.world_to_pixel_coords(wx, wy)
                            self._updating_position = True
                            self.setPos(px, py)
                            self._updating_position = False
                    elif self.obj_type == "prop":
                        # Props go anywhere; only follow the ground height
                        self.obj_ref.spawn.z = self.canvas.ground_z(wx, wy, self.obj_ref.spawn.z)
                    elif self.obj_type == "ped":
                        # Find nearest sidewalk waypoint
                        nearest_wp = None
                        min_dist = 999999.0
                        for sw_wp in self.canvas.sidewalk_wps:
                            dist = loc.distance(sw_wp.transform.location)
                            if dist < min_dist:
                                min_dist = dist
                                nearest_wp = sw_wp

                        # Check if on crosswalk
                        on_crosswalk = False
                        for poly in self.canvas.crosswalk_polys:
                            if poly.containsPoint(value, Qt.OddEvenFill):
                                on_crosswalk = True
                                break

                        if min_dist < 3.0:
                            # Snap to sidewalk
                            if nearest_wp:
                                wx = nearest_wp.transform.location.x
                                wy = nearest_wp.transform.location.y
                                yaw = nearest_wp.transform.rotation.yaw
                                px, py = self.canvas.world_to_pixel_coords(wx, wy)
                                self._updating_position = True
                                self.setPos(px, py)
                                self._updating_position = False
                        elif on_crosswalk:
                            # Keep exact coordinates (no snap)
                            pass
                        else:
                            # Snap to nearest sidewalk
                            if nearest_wp:
                                wx = nearest_wp.transform.location.x
                                wy = nearest_wp.transform.location.y
                                yaw = nearest_wp.transform.rotation.yaw
                                px, py = self.canvas.world_to_pixel_coords(wx, wy)
                                self._updating_position = True
                                self.setPos(px, py)
                                self._updating_position = False
                except Exception as e:
                    print(f"Snapping error: {e}")

            # Update data ref
            if self.obj_type == "ego":
                self.obj_ref.spawn.x = wx
                self.obj_ref.spawn.y = wy
                self.obj_ref.spawn.yaw = yaw
            elif self.obj_type == "ego_goal":
                self.obj_ref.goal.x = wx
                self.obj_ref.goal.y = wy
            elif self.obj_type in ("npc", "ped", "prop"):
                self.obj_ref.spawn.x = wx
                self.obj_ref.spawn.y = wy
                self.obj_ref.spawn.yaw = yaw
            elif self.obj_type == "trigger":
                self.obj_ref.transform.x = wx
                self.obj_ref.transform.y = wy
                self.obj_ref.transform.yaw = yaw

            self.setRotation(yaw)
            self.canvas.object_moved.emit(self.obj_type, self.obj_ref)

            if collecting:
                self.canvas._collect_sample(wx, wy, yaw)

        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        self.canvas.object_selected.emit(self.obj_type, self.obj_ref)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.canvas.drag_finished.emit()  # one undo step per drag, not per move


class MapCanvas(QGraphicsView):
    """
    Main map canvas for visualizing CARLA map and placing scenario objects.
    """

    object_selected = pyqtSignal(str, object)  # (type, ref)
    object_moved = pyqtSignal(str, object)
    map_clicked = pyqtSignal(float, float)  # world x, y
    collect_requested = pyqtSignal(object)   # npc ref (right-clicked on canvas)
    collect_mode_changed = pyqtSignal(bool)  # True = collect mode active
    placement_mode_changed = pyqtSignal(str)  # "" = no placement mode
    drag_finished = pyqtSignal()             # marker released after a drag

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        # Rendering settings
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(COLORS["background"]))
        self.setMinimumSize(600, 400)

        # Map data
        self._pixels_per_meter = 5.0
        self._world_offset = (0.0, 0.0)
        self._road_items = []
        self._spawn_point_items = []
        self._actor_markers = {}  # uid -> ActorMarker
        self._route_lines = []
        self.carla_map = None
        self.sidewalk_wps = []
        self.crosswalk_polys = []

        # Placement mode
        self._placement_mode = None  # "ego", "npc", "ped", "trigger", "ego_goal", "waypoint"
        self._placement_callback = None

        # Collect-waypoint mode (NPC route capture by moving the vehicle)
        self._collect_npc = None            # NPCVehicle ref being collected, None = off
        self._collect_lane_restrict = True  # toggled by spacebar; snap to driving lane
        self._collect_sample_dist = 2.0     # min spacing (m) between recorded waypoints
        self._collect_last_pt = None        # (x, y) of last recorded waypoint
        self._collect_route_lines = []      # incremental QGraphicsItems drawn live

        # Receive keyboard events (spacebar / arrows) for collect mode
        self.setFocusPolicy(Qt.StrongFocus)

        # Zoom
        self._zoom = 1.0

        self._selected = None     # (obj_type, obj_ref) highlighted on the map
        self._map_loaded = False
        self._setup_overlays()

    # --- Overlays (child widgets: no repaint artifacts when scrolling) --------

    def _setup_overlays(self):
        def overlay(text=""):
            lbl = QLabel(text, self)  # not viewport(): viewport scrolling would drag overlays along
            lbl.setAttribute(Qt.WA_TransparentForMouseEvents)
            return lbl

        self._empty_label = overlay(
            "<div style='font-size:15px; font-weight:600'>No map loaded</div>"
            "<div style='color:#93A4B1; margin-top:6px'>Pick a map in the toolbar and press "
            "<b>Load Map</b> (needs a running CARLA server).<br>"
            "Offline you can still place actors by clicking and edit coordinates in Properties.<br><br>"
            "New here? <b>Help → User Guide</b> (F1).</div>")
        self._empty_label.setAlignment(Qt.AlignCenter)
        self._empty_label.setStyleSheet("background: rgba(22,33,43,230); border: 1px solid #2C3B47;"
                                        " border-radius: 10px; padding: 18px 24px;")

        dot = "<span style='color:{}'>&#9679;</span> {}"
        self._legend = overlay("&nbsp;&nbsp;".join(dot.format(c.name(), n) for n, c in [
            ("Ego", COLORS["ego"]), ("Goal", COLORS["ego_goal"]), ("NPC", COLORS["npc"]),
            ("Pedestrian", COLORS["pedestrian"]), ("Prop", COLORS["prop"]), ("Trigger", COLORS["trigger"]),
            ("Route", COLORS["route"])]))
        self._legend.setStyleSheet("background: rgba(15,23,32,210); border-radius: 6px;"
                                   " padding: 4px 8px; font-size: 11px;")

        self._mode_pill = overlay()
        self._mode_pill.setStyleSheet("background: #36768F; color: white; border-radius: 12px;"
                                      " padding: 4px 14px; font-weight: 600;")
        self._mode_pill.hide()
        self._layout_overlays()

    def _layout_overlays(self):
        vp = self.viewport().geometry()
        vw, vh = vp.width(), vp.height()
        for lbl in (self._empty_label, self._legend, self._mode_pill):
            lbl.adjustSize()
        e = self._empty_label
        e.move(vp.x() + (vw - e.width()) // 2, vp.y() + (vh - e.height()) // 2)
        e.setVisible(not self._map_loaded and not self._actor_markers)
        self._legend.move(vp.x() + 10, vp.y() + vh - self._legend.height() - 10)
        self._mode_pill.move(vp.x() + (vw - self._mode_pill.width()) // 2, vp.y() + 10)

    def _set_mode_hint(self, text):
        self._mode_pill.setText(f"{text}  ·  Esc to cancel" if text else "")
        self._mode_pill.setVisible(bool(text))
        self._layout_overlays()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._layout_overlays()

    def fit_map(self):
        rect = self._scene.itemsBoundingRect()
        if not rect.isEmpty():
            self.fitInView(rect.adjusted(-20, -20, 20, 20), Qt.KeepAspectRatio)

    def select_object(self, obj_type, obj_ref):
        """Highlight the marker for obj_ref (None clears)."""
        self._selected = (obj_type, obj_ref) if obj_ref is not None else None
        for marker in self._actor_markers.values():
            marker.setSelected(self._is_selected(marker))

    def _is_selected(self, marker):
        return (self._selected is not None and marker.obj_type == self._selected[0]
                and marker.obj_ref is self._selected[1])

    def set_placement_mode(self, mode, callback=None):
        """Enter placement mode — next click on map will place object/waypoint."""
        self._placement_mode = mode
        self._placement_callback = callback
        self._set_mode_hint(MODE_HINTS.get(mode, "") if mode else "")
        self.placement_mode_changed.emit(mode or "")
        if mode:
            self.setCursor(Qt.CrossCursor)
            self.setDragMode(QGraphicsView.NoDrag)
        else:
            self.setCursor(Qt.ArrowCursor)
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def load_map_from_carla(self, carla_client, town_name):
        """
        Connect to CARLA, load map, render the HD map from its lane topology:
        continuous lane surfaces, sidewalks/shoulders, lane markings, direction arrows, crosswalks.
        """
        try:
            world = carla_client.get_world()
            self.carla_map = world.get_map()
        except Exception as e:
            QMessageBox.warning(self, "CARLA Error", f"Cannot get CARLA map: {e}")
            return False

        self._scene.clear()
        self._actor_markers.clear()
        self._route_lines.clear()

        segments = self._lane_segments()
        if not segments:
            return False

        # Compute world bounds
        margin = 50
        xs = [w.transform.location.x for seg in segments for w in seg]
        ys = [w.transform.location.y for seg in segments for w in seg]
        self._world_offset = (min(xs) - margin, min(ys) - margin)

        side_runs = self._side_lane_runs(segments)
        import carla
        self.sidewalk_wps = [w for (lt, _), run in side_runs if lt == carla.LaneType.Sidewalk for w in run]

        self._render_side_lanes(side_runs)
        self._render_driving_lanes(segments)
        self._render_lane_markings(segments)
        self._render_direction_arrows(segments)
        self._render_crosswalks()

        # 6. Draw spawn points
        spawn_points = self.carla_map.get_spawn_points()
        sp_pen = QPen(COLORS["spawn_pt"], 1)
        sp_brush = QBrush(COLORS["spawn_pt"])
        for sp in spawn_points:
            px, py = self.world_to_pixel_coords(sp.location.x, sp.location.y)
            r = 3
            item = self._scene.addEllipse(px - r, py - r, 2 * r, 2 * r, sp_pen, sp_brush)
            item.setZValue(5)
            item.setToolTip(f"Spawn: ({sp.location.x:.1f}, {sp.location.y:.1f}, {sp.location.z:.1f}) yaw={sp.rotation.yaw:.0f}")

        # Fit view
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        self._zoom = 1.0
        self._map_loaded = True
        self._layout_overlays()

        return True

    # --- HD map rendering ----------------------------------------------------

    def _lane_segments(self):
        """Driving lanes as ordered waypoint lists, one per topology segment (junction lanes included)."""
        segments = []
        for start, end in self.carla_map.get_topology():
            wps = [start] + start.next_until_lane_end(MAP_PRECISION)
            # Close the seam to the next segment; some junction "end" waypoints sit 10-30 m
            # away from the real lane end (seen in Town03) and would draw long slivers.
            if 0.1 < wps[-1].transform.location.distance(end.transform.location) <= 2 * MAP_PRECISION:
                wps.append(end)
            if len(wps) >= 2:
                segments.append(wps)
        return segments

    def _side_lane_runs(self, segments):
        """Non-driving lanes (sidewalk, shoulder, parking, biking) as continuous runs.

        Walk outward from the outermost driving lane of every sample, so each side lane is
        visited once; junction side lanes come from Junction.get_waypoints (corner sidewalks).
        Returns [((lane_type, width), [waypoints...]), ...].
        """
        import carla
        side_types = _side_lane_tables()[0]
        runs = []
        for seg in segments:
            per_lane = {}  # (road, section, lane) -> waypoints in travel order
            if seg[len(seg) // 2].is_junction:
                continue  # junction side lanes come from Junction.get_waypoints below
            for w in seg:
                for step in ("get_right_lane", "get_left_lane"):
                    n = getattr(w, step)()
                    # Only start from the outermost driving lane on that side
                    if n is None or n.lane_type == carla.LaneType.Driving:
                        continue
                    while n is not None and n.lane_type not in (carla.LaneType.NONE, carla.LaneType.Driving):
                        if n.lane_type in side_types:
                            per_lane.setdefault((n.road_id, n.section_id, n.lane_id), []).append(n)
                        n = getattr(n, step)()
            runs.extend(((run[0].lane_type, run[0].lane_width), run) for run in per_lane.values() if len(run) >= 2)

        seen = set()
        for seg in segments:
            w = seg[len(seg) // 2]
            if not w.is_junction:
                continue
            junction = w.get_junction()
            if junction.id in seen:
                continue
            seen.add(junction.id)
            for lane_type in (carla.LaneType.Sidewalk, carla.LaneType.Shoulder):
                for start, _ in junction.get_waypoints(lane_type):
                    run = [start] + start.next_until_lane_end(MAP_PRECISION)
                    if len(run) >= 2:
                        runs.append(((lane_type, start.lane_width), run))
        # Junction (and roundabout) side lanes often run straight across the driving surface;
        # drawn, they show as grey lines over the road. Keep only runs that stay off it.
        return [r for r in runs if not self._mostly_on_driving(r[1])]

    def _on_driving(self, x, y):
        import carla
        return self.carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=False,
                                           lane_type=carla.LaneType.Driving) is not None

    def _mostly_on_driving(self, run, share=0.3):
        samples = run[::3] or run
        hits = sum(self._on_driving(w.transform.location.x, w.transform.location.y) for w in samples)
        return hits > share * len(samples)

    def _polyline(self, wps, offset=0.0):
        """QPainterPath through the waypoints, shifted `offset` meters to the right of travel."""
        path = QPainterPath()
        for i, w in enumerate(wps):
            loc = w.transform.location
            yaw = math.radians(w.transform.rotation.yaw)
            pt = QPointF(*self.world_to_pixel_coords(loc.x - math.sin(yaw) * offset,
                                                     loc.y + math.cos(yaw) * offset))
            path.lineTo(pt) if i else path.moveTo(pt)
        return path

    def _render_side_lanes(self, runs):
        """Side lanes as round-capped strokes: continuous strips, no seams or bowtie holes."""
        _, colors, z_values = _side_lane_tables()
        paths = {}  # (lane_type, width bucket) -> combined path
        for (lane_type, width), run in runs:
            key = (lane_type, round(max(width, 0.5) * 4) / 4)
            paths.setdefault(key, QPainterPath()).addPath(self._polyline(run))
        for (lane_type, width), path in paths.items():
            pen = QPen(colors[lane_type], width * self._pixels_per_meter, Qt.SolidLine,
                       Qt.RoundCap, Qt.RoundJoin)
            item = self._scene.addPath(path, pen)
            item.setZValue(z_values[lane_type])

    def _render_driving_lanes(self, segments):
        """All driving lanes as ONE winding-filled path: continuous asphalt, no AA seams between lanes.

        Junction lanes are also stroked wide underneath ("closing"): OpenDRIVE leaves the corners
        between right-turn arcs and the curb empty, which otherwise shows as dark wedges.
        Sidewalks are drawn on top, so the overflow past the curb stays hidden.
        """
        road = QPainterPath()
        road.setFillRule(Qt.WindingFill)
        junctions = QPainterPath()
        for seg in segments:
            left, right = [], []
            for w in seg:
                loc = w.transform.location
                yaw = math.radians(w.transform.rotation.yaw)
                rx, ry = -math.sin(yaw), math.cos(yaw)
                h = w.lane_width / 2.0
                left.append(QPointF(*self.world_to_pixel_coords(loc.x - rx * h, loc.y - ry * h)))
                right.append(QPointF(*self.world_to_pixel_coords(loc.x + rx * h, loc.y + ry * h)))
            road.addPolygon(QPolygonF(left + right[::-1]))
            road.closeSubpath()
            if seg[len(seg) // 2].is_junction:
                junctions.addPolygon(QPolygonF(left + right[::-1]))
                junctions.closeSubpath()
        closing = self._scene.addPath(junctions, QPen(COLORS["road"], JUNCTION_CLOSE_GAP * 2 * self._pixels_per_meter,
                                                      Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        closing.setZValue(0.9)
        # Same-color outline covers hairline gaps where curved lanes meet
        item = self._scene.addPath(road, QPen(COLORS["road"], 0.8 * self._pixels_per_meter, Qt.SolidLine,
                                              Qt.RoundCap, Qt.RoundJoin), QBrush(COLORS["road"]))
        item.setZValue(1)

    def _render_lane_markings(self, segments):
        """Lane markings as continuous polylines per run of equal type/color (skipped inside junctions).

        Each boundary is drawn once: every lane draws its left marking (except lane +1, whose left
        is the center line already drawn by lane -1) and the outermost lane also draws its right one.
        """
        import carla
        solid_types = {carla.LaneMarkingType.Solid: (False,), carla.LaneMarkingType.Broken: (True,),
                       carla.LaneMarkingType.SolidSolid: (False, False),
                       carla.LaneMarkingType.SolidBroken: (False, True),
                       carla.LaneMarkingType.BrokenSolid: (True, False),
                       carla.LaneMarkingType.BrokenBroken: (True, True)}
        paths = {}  # (yellow, broken) -> path

        def emit(run, side, kinds, yellow):
            half_gap = 0.12  # m between the two lines of a double marking
            for i, broken in enumerate(kinds):
                shift = 0.0 if len(kinds) == 1 else (-half_gap if i == 0 else half_gap)
                pts = []
                for w in run:
                    edge = (w.lane_width / 2.0) * (-1 if side == "left" else 1) + shift
                    pts.append((w, edge))
                path = QPainterPath()
                for j, (w, edge) in enumerate(pts):
                    loc = w.transform.location
                    yaw = math.radians(w.transform.rotation.yaw)
                    pt = QPointF(*self.world_to_pixel_coords(loc.x - math.sin(yaw) * edge,
                                                             loc.y + math.cos(yaw) * edge))
                    path.lineTo(pt) if j else path.moveTo(pt)
                paths.setdefault((yellow, broken), QPainterPath()).addPath(path)

        for seg in segments:
            if seg[len(seg) // 2].is_junction:
                continue
            for side in ("left", "right"):
                run, run_key = [], None
                for w in seg:
                    if side == "left":
                        left = w.get_left_lane()
                        skip = (w.lane_id == 1 and left is not None and left.lane_type == carla.LaneType.Driving)
                        mark = None if skip else w.left_lane_marking
                    else:
                        right = w.get_right_lane()
                        mark = w.right_lane_marking if (right is None or right.lane_type != carla.LaneType.Driving) else None
                    key = None
                    if mark is not None and mark.type in solid_types:
                        key = (solid_types[mark.type], mark.color == carla.LaneMarkingColor.Yellow)
                    if key != run_key:
                        if run_key is not None and len(run) >= 2:
                            emit(run, side, *run_key)
                        run, run_key = [], key
                    if key is not None:
                        run.append(w)
                if run_key is not None and len(run) >= 2:
                    emit(run, side, *run_key)

        width = 0.15 * self._pixels_per_meter
        for (yellow, broken), path in paths.items():
            pen = QPen(COLORS["marking_yellow" if yellow else "marking_white"], width, Qt.SolidLine, Qt.FlatCap)
            if broken:
                pen.setDashPattern([3.0 * self._pixels_per_meter / width, 4.5 * self._pixels_per_meter / width])
            item = self._scene.addPath(path, pen)
            item.setZValue(4)

    def _render_direction_arrows(self, segments, spacing=25.0):
        """Faint chevrons every `spacing` m along driving lanes, so travel direction reads at a glance."""
        path = QPainterPath()
        step = max(1, int(spacing / MAP_PRECISION))
        for seg in segments:
            if seg[len(seg) // 2].is_junction:
                continue
            for w in seg[step // 2::step]:
                loc = w.transform.location
                yaw = math.radians(w.transform.rotation.yaw)
                fx, fy, rx, ry = math.cos(yaw), math.sin(yaw), -math.sin(yaw), math.cos(yaw)
                s = min(0.6, w.lane_width * 0.18)
                tip = self.world_to_pixel_coords(loc.x + fx * s, loc.y + fy * s)
                path.moveTo(*self.world_to_pixel_coords(loc.x - fx * s + rx * s, loc.y - fy * s + ry * s))
                path.lineTo(*tip)
                path.lineTo(*self.world_to_pixel_coords(loc.x - fx * s - rx * s, loc.y - fy * s - ry * s))
        item = self._scene.addPath(path, QPen(COLORS["arrow"], 0.2 * self._pixels_per_meter, Qt.SolidLine,
                                              Qt.RoundCap, Qt.RoundJoin))
        item.setZValue(3.5)

    def _render_crosswalks(self):
        """Zebra stripes (0.5 m bars every 1 m) inside each crosswalk; extended polygons kept for ped snapping.
        Crosswalks that don't cross any driving lane are dropped (orphans in the map data)."""
        import carla
        self.crosswalk_polys = []
        polys = None
        try:
            xodr = crosswalks_from_xodr(self.carla_map.to_opendrive())
            if xodr is not None:
                polys = [[carla.Location(x=x, y=y) for x, y in p] for p in xodr]
        except Exception as e:
            print(f"OpenDRIVE crosswalks failed, using CARLA's: {e}")
        if polys is None:
            try:
                crosswalk_locs = self.carla_map.get_crosswalks()
            except Exception as e:
                print(f"get_crosswalks render skipped: {e}")
                return
            # Group flat list of locations into closed polygons
            polys, curr = [], []
            for loc in crosswalk_locs:
                curr.append(loc)
                if len(curr) > 1 and loc.distance(curr[0]) < 0.1:
                    polys.append(curr)
                    curr = []

        stripes = QPainterPath()
        for cw in polys:
            v = cw[:-1] if cw[0].distance(cw[-1]) < 0.1 else cw
            if len(v) != 4:
                continue
            # Stripes run between the two long edges, stepping along them
            if v[0].distance(v[1]) + v[2].distance(v[3]) >= v[1].distance(v[2]) + v[3].distance(v[0]):
                a0, a1, b0, b1 = v[0], v[1], v[3], v[2]
            else:
                a0, a1, b0, b1 = v[1], v[2], v[0], v[3]

            def mid(t):  # point on the crosswalk's center line, t in [0, 1] along its length
                return ((a0.x + (a1.x - a0.x) * t + b0.x + (b1.x - b0.x) * t) / 2,
                        (a0.y + (a1.y - a0.y) * t + b0.y + (b1.y - b0.y) * t) / 2)
            # Some maps (Town03) carry crosswalk objects on orphan junction stubs, 5-80 m away
            # from any road; a real crosswalk crosses a driving lane somewhere along its length.
            if not any(self._on_driving(*mid(k / 8)) for k in range(9)):
                continue
            self.crosswalk_polys.append(QPolygonF(
                [QPointF(*self.world_to_pixel_coords(p.x, p.y)) for p in extend_crosswalk_polygon(cw, 2.5)]))
            n = max(1, int(a0.distance(a1) / 1.0))
            for k in range(n):
                t = (k + 0.5) / n
                stripes.moveTo(*self.world_to_pixel_coords(a0.x + (a1.x - a0.x) * t, a0.y + (a1.y - a0.y) * t))
                stripes.lineTo(*self.world_to_pixel_coords(b0.x + (b1.x - b0.x) * t, b0.y + (b1.y - b0.y) * t))
        item = self._scene.addPath(stripes, QPen(COLORS["crosswalk"], 0.5 * self._pixels_per_meter,
                                                 Qt.SolidLine, Qt.FlatCap))
        item.setZValue(3)

    def _render_buildings(self, world):
        import carla
        try:
            # Query all buildings
            buildings = world.get_environment_objects(carla.CityObjectLabel.Buildings)
            if not buildings:
                return
            
            brush = QBrush(COLORS["building"])
            pen = QPen(COLORS["building_edge"], 1.2)
            
            for b in buildings:
                box = b.bounding_box
                transform = b.transform
                extent = box.extent
                
                if extent.x <= 0 or extent.y <= 0:
                    continue
                
                # Get the 4 corners in building local space
                c1 = carla.Vector3D(extent.x, extent.y, 0)
                c2 = carla.Vector3D(-extent.x, extent.y, 0)
                c3 = carla.Vector3D(-extent.x, -extent.y, 0)
                c4 = carla.Vector3D(extent.x, -extent.y, 0)
                
                # Transform corners to world coordinates
                w1 = transform.transform(c1)
                w2 = transform.transform(c2)
                w3 = transform.transform(c3)
                w4 = transform.transform(c4)
                
                # Convert to screen/pixel coordinates
                p1 = QPointF(*self.world_to_pixel_coords(w1.x, w1.y))
                p2 = QPointF(*self.world_to_pixel_coords(w2.x, w2.y))
                p3 = QPointF(*self.world_to_pixel_coords(w3.x, w3.y))
                p4 = QPointF(*self.world_to_pixel_coords(w4.x, w4.y))
                
                polygon = QPolygonF([p1, p2, p3, p4])
                poly_item = self._scene.addPolygon(polygon, pen, brush)
                poly_item.setZValue(2.5) # Under actors, on top of roads
                poly_item.setToolTip(f"Building: {b.name}")
        except Exception as e:
            print(f"Building render skipped: {e}")

    def _render_vegetation(self, world):
        import carla
        try:
            vegetation = world.get_environment_objects(carla.CityObjectLabel.Vegetation)
            if not vegetation:
                return
            
            brush = QBrush(COLORS["vegetation"])
            pen = QPen(COLORS["vegetation"].darker(120), 1.0)
            
            for veg in vegetation:
                box = veg.bounding_box
                transform = veg.transform
                extent = box.extent
                
                # Render as circle at the object transform location
                r = max(1.0, max(extent.x, extent.y))
                px, py = self.world_to_pixel_coords(transform.location.x, transform.location.y)
                pr = r * self._pixels_per_meter
                
                circle_item = self._scene.addEllipse(px - pr, py - pr, 2 * pr, 2 * pr, pen, brush)
                circle_item.setZValue(2.6) # Slightly above buildings
                circle_item.setToolTip(f"Vegetation: {veg.name}")
        except Exception as e:
            print(f"Vegetation render skipped: {e}")

    def load_map_offline(self, waypoint_data):
        self._scene.clear()
        self._actor_markers.clear()
        self._route_lines.clear()

        if not waypoint_data:
            return

        margin = 50
        xs = [p[0] for p in waypoint_data]
        ys = [p[1] for p in waypoint_data]
        min_x, max_x = min(xs) - margin, max(xs) + margin
        min_y, max_y = min(ys) - margin, max(ys) + margin
        self._world_offset = (min_x, min_y)

        pen = QPen(COLORS["road"], 3)
        brush = QBrush(COLORS["road"])
        for x, y in waypoint_data:
            px, py = self.world_to_pixel_coords(x, y)
            self._scene.addEllipse(px - 1, py - 1, 2, 2, pen, brush)

        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        self._map_loaded = True
        self._layout_overlays()

    def ground_z(self, wx, wy, default=0.0):
        """Road/ground height at (wx, wy) from the CARLA map, or default when offline."""
        if not self.carla_map:
            return default
        try:
            import carla
            wp = self.carla_map.get_waypoint(carla.Location(x=wx, y=wy, z=0.0), project_to_road=True,
                                             lane_type=carla.LaneType.Any)
            return wp.transform.location.z if wp else default
        except Exception as e:
            print(f"Ground height error: {e}")
            return default

    def world_to_pixel_coords(self, wx, wy):
        """Convert CARLA world coordinates to pixel coordinates."""
        px = self._pixels_per_meter * (wx - self._world_offset[0])
        py = self._pixels_per_meter * (wy - self._world_offset[1])
        return px, py

    def pixel_to_world(self, pixel_pos):
        """Convert pixel position to CARLA world coordinates."""
        wx = pixel_pos.x() / self._pixels_per_meter + self._world_offset[0]
        wy = pixel_pos.y() / self._pixels_per_meter + self._world_offset[1]
        return QPointF(wx, wy)

    def add_actor_marker(self, obj_type, obj_ref):
        """Add a visual marker for a scenario object."""
        color_map = {
            "ego": COLORS["ego"],
            "ego_goal": COLORS["ego_goal"],
            "npc": COLORS["npc"],
            "ped": COLORS["pedestrian"],
            "trigger": COLORS["trigger"],
            "prop": COLORS["prop"],
        }
        size_map = {
            "ego": ACTOR_SIZE * self._pixels_per_meter,
            "ego_goal": ACTOR_SIZE * self._pixels_per_meter * 0.7,
            "npc": ACTOR_SIZE * self._pixels_per_meter * 0.9,
            "ped": ACTOR_SIZE * self._pixels_per_meter * 0.5,
            "trigger": TRIGGER_SIZE * self._pixels_per_meter,
            "prop": ACTOR_SIZE * self._pixels_per_meter * 0.4,
        }

        color = color_map.get(obj_type, COLORS["npc"])
        radius = size_map.get(obj_type, ACTOR_SIZE * self._pixels_per_meter)

        if obj_type == "ego":
            wx, wy = obj_ref.spawn.x, obj_ref.spawn.y
            uid = obj_ref.uid + "_ego"
        elif obj_type == "ego_goal":
            wx, wy = obj_ref.goal.x, obj_ref.goal.y
            uid = obj_ref.uid + "_goal"
        elif obj_type in ("npc", "ped", "prop"):
            wx, wy = obj_ref.spawn.x, obj_ref.spawn.y
            uid = obj_ref.uid
        elif obj_type == "trigger":
            wx, wy = obj_ref.transform.x, obj_ref.transform.y
            uid = obj_ref.uid
        else:
            return

        px, py = self.world_to_pixel_coords(wx, wy)
        marker = ActorMarker(obj_type, obj_ref, radius, color, self)
        marker.setPos(px, py)
        marker._updating_position = False
        self._scene.addItem(marker)
        marker.setSelected(self._is_selected(marker))
        self._actor_markers[uid] = marker

    def remove_actor_marker(self, uid):
        if uid in self._actor_markers:
            self._scene.removeItem(self._actor_markers[uid])
            del self._actor_markers[uid]

    def refresh_markers(self, project):
        """Re-draw all markers and route lines from project data."""
        # Clear live collect-mode overlay lines (not removed by the route-line loop below)
        self._clear_collect_lines()
        # Clear existing markers
        for uid, marker in list(self._actor_markers.items()):
            self._scene.removeItem(marker)
        self._actor_markers.clear()

        # Clear existing routes
        for line in self._route_lines:
            self._scene.removeItem(line)
        self._route_lines.clear()

        # Re-add markers
        if project.ego:
            self.add_actor_marker("ego", project.ego)
            if project.ego.goal:
                self.add_actor_marker("ego_goal", project.ego)
        for npc in project.npc_vehicles:
            self.add_actor_marker("npc", npc)
            # Draw NPC route lines
            if npc.route:
                px_prev, py_prev = self.world_to_pixel_coords(npc.spawn.x, npc.spawn.y)
                for i, wp in enumerate(npc.route):
                    px, py = self.world_to_pixel_coords(wp.x, wp.y)
                    # Green dashed connection
                    line = self._scene.addLine(px_prev, py_prev, px, py, QPen(COLORS["route"], 2, Qt.DashLine))
                    line.setZValue(5)
                    self._route_lines.append(line)
                    # Node dot
                    dot = self._scene.addEllipse(px - 3, py - 3, 6, 6, QPen(COLORS["route"], 1), QBrush(COLORS["route"]))
                    dot.setZValue(6)
                    dot.setToolTip(f"Waypoint {i} for {npc.name}")
                    self._route_lines.append(dot)
                    px_prev, py_prev = px, py

        for ped in project.pedestrians:
            self.add_actor_marker("ped", ped)
            # Draw Ped route lines
            if ped.route:
                px_prev, py_prev = self.world_to_pixel_coords(ped.spawn.x, ped.spawn.y)
                for i, wp in enumerate(ped.route):
                    px, py = self.world_to_pixel_coords(wp.x, wp.y)
                    line = self._scene.addLine(px_prev, py_prev, px, py, QPen(COLORS["route"], 1.5, Qt.DashLine))
                    line.setZValue(5)
                    self._route_lines.append(line)
                    dot = self._scene.addEllipse(px - 2.5, py - 2.5, 5, 5, QPen(COLORS["route"], 1), QBrush(COLORS["route"]))
                    dot.setZValue(6)
                    dot.setToolTip(f"Waypoint {i} for {ped.name}")
                    self._route_lines.append(dot)
                    px_prev, py_prev = px, py

        for prop in project.static_props:
            self.add_actor_marker("prop", prop)
        for trig in project.trigger_points:
            self.add_actor_marker("trigger", trig)
        self._layout_overlays()

    def wheelEvent(self, event):
        factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(factor, factor)
            self._zoom *= factor
        else:
            self.scale(1 / factor, 1 / factor)
            self._zoom /= factor

    def mousePressEvent(self, event):
        if self._placement_mode and event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            world_pos = self.pixel_to_world(scene_pos)
            if self._placement_callback:
                self._placement_callback(world_pos.x(), world_pos.y())
            # "waypoint" stays active for continuous adding (Esc / button to finish);
            # one-shot placements (ego/npc/ped/trigger/goal) reset after a single click.
            if self._placement_mode != "waypoint":
                self.set_placement_mode(None)
            return
        super().mousePressEvent(event)

    # --- Collect-waypoint mode -------------------------------------------------

    def start_collect_mode(self, npc):
        """Begin capturing a route by moving the given NPC vehicle."""
        self.set_placement_mode(None)
        self._collect_npc = npc
        self._collect_last_pt = (npc.spawn.x, npc.spawn.y)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setCursor(Qt.CrossCursor)
        self._set_mode_hint(f"Collecting route for {npc.name}: drag or arrow keys, Space = lane snap")
        self.setFocus()
        self.collect_mode_changed.emit(True)

    def stop_collect_mode(self):
        """Exit collect mode and restore normal navigation."""
        if self._collect_npc is None:
            return
        self._collect_npc = None
        self._collect_last_pt = None
        self._clear_collect_lines()
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setCursor(Qt.ArrowCursor)
        self._set_mode_hint("")
        self.collect_mode_changed.emit(False)

    def _clear_collect_lines(self):
        for item in self._collect_route_lines:
            self._scene.removeItem(item)
        self._collect_route_lines.clear()

    def _collect_sample(self, wx, wy, yaw):
        """Append a waypoint if the vehicle moved at least the sample distance."""
        npc = self._collect_npc
        if npc is None:
            return
        from ..models.actors import RouteWaypoint
        if self._collect_last_pt is not None:
            dx = wx - self._collect_last_pt[0]
            dy = wy - self._collect_last_pt[1]
            if (dx * dx + dy * dy) < (self._collect_sample_dist ** 2):
                return
        npc.route.append(RouteWaypoint(x=wx, y=wy, z=0.5, yaw=yaw))
        # Draw the new segment + node live (without rebuilding markers mid-drag)
        if self._collect_last_pt is not None:
            px0, py0 = self.world_to_pixel_coords(*self._collect_last_pt)
            px1, py1 = self.world_to_pixel_coords(wx, wy)
            line = self._scene.addLine(px0, py0, px1, py1, QPen(COLORS["route"], 2, Qt.DashLine))
            line.setZValue(5)
            self._collect_route_lines.append(line)
            dot = self._scene.addEllipse(px1 - 3, py1 - 3, 6, 6, QPen(COLORS["route"], 1), QBrush(COLORS["route"]))
            dot.setZValue(6)
            self._collect_route_lines.append(dot)
        self._collect_last_pt = (wx, wy)

    def _drive_npc(self, dx_forward, dyaw):
        """Arrow-key game driving: advance along heading / steer, then sample."""
        npc = self._collect_npc
        if npc is None:
            return
        npc.spawn.yaw += dyaw
        yaw_rad = math.radians(npc.spawn.yaw)
        wx = npc.spawn.x + dx_forward * math.cos(yaw_rad)
        wy = npc.spawn.y + dx_forward * math.sin(yaw_rad)

        if self.carla_map and self._collect_lane_restrict:
            try:
                import carla
                wp = self.carla_map.get_waypoint(carla.Location(x=wx, y=wy, z=0.5),
                                                 lane_type=carla.LaneType.Driving)
                if wp:
                    wx = wp.transform.location.x
                    wy = wp.transform.location.y
            except Exception as e:
                print(f"Drive snap error: {e}")

        npc.spawn.x, npc.spawn.y = wx, wy
        marker = self._actor_markers.get(npc.uid)
        if marker:
            px, py = self.world_to_pixel_coords(wx, wy)
            marker._updating_position = True
            marker.setPos(px, py)
            marker._updating_position = False
            marker.setRotation(npc.spawn.yaw)
        self.object_moved.emit("npc", npc)
        self._collect_sample(wx, wy, npc.spawn.yaw)

    def keyPressEvent(self, event):
        if self._collect_npc is not None:
            key = event.key()
            if key == Qt.Key_Escape:
                self.stop_collect_mode()
                return
            if key == Qt.Key_Space and not event.isAutoRepeat():
                self._collect_lane_restrict = not self._collect_lane_restrict
                state = "ON (snap to lane)" if self._collect_lane_restrict else "OFF (free)"
                self.window().statusBar().showMessage(f"Collect lane restrict: {state}", 2000)
                return
            step = 1.0      # meters per forward/back press
            turn = 5.0      # degrees per steer press
            if key == Qt.Key_Up:
                self._drive_npc(step, 0.0); return
            if key == Qt.Key_Down:
                self._drive_npc(-step, 0.0); return
            if key == Qt.Key_Left:
                self._drive_npc(0.0, -turn); return
            if key == Qt.Key_Right:
                self._drive_npc(0.0, turn); return
        elif event.key() == Qt.Key_Escape and self._placement_mode:
            self.set_placement_mode(None)
            return
        super().keyPressEvent(event)

    def contextMenuEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        world_pos = self.pixel_to_world(scene_pos)

        # Detect an NPC marker under the cursor for the collect action
        npc_ref = None
        item = self.itemAt(event.pos())
        while item is not None:
            if isinstance(item, ActorMarker) and item.obj_type == "npc":
                npc_ref = item.obj_ref
                break
            item = item.parentItem()

        menu = QMenu(self)
        info_action = menu.addAction(f"World: ({world_pos.x():.1f}, {world_pos.y():.1f})")
        info_action.setEnabled(False)
        menu.addSeparator()
        collect_action = None
        if npc_ref is not None:
            collect_action = menu.addAction("🎯 Collect Waypoints")
            menu.addSeparator()
        copy_action = menu.addAction("Copy Coordinates")

        action = menu.exec_(event.globalPos())
        if action == copy_action:
            from PyQt5.QtWidgets import QApplication
            QApplication.clipboard().setText(f"{world_pos.x():.2f}, {world_pos.y():.2f}")
        elif collect_action is not None and action == collect_action:
            self.collect_requested.emit(npc_ref)

