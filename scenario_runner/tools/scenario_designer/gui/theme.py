"""
Theme — ARL design tokens, Qt stylesheet, SVG icons and the logo.

Brand color is sampled from logo/arl.png. Icons are Lucide-style strokes
(24x24 viewBox) rendered through QtSvg, so no emoji / icon font dependency.
"""

import os
from functools import lru_cache

from PyQt5.QtCore import QByteArray, QRect, Qt
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QIcon, QPainter, QPalette, QPixmap
from PyQt5.QtSvg import QSvgRenderer

LOGO_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         "..", "..", "..", "logo", "arl.png"))
# Car-in-ring mark on the left of the wordmark; the black text is unreadable on the dark UI.
LOGO_MARK = QRect(0, 0, 148, 136)

T = {
    "bg":          "#0F1720",
    "surface":     "#16212B",
    "surface2":    "#1E2B36",
    "hover":       "#253541",
    "border":      "#2C3B47",
    "text":        "#E6EDF2",
    "muted":       "#93A4B1",
    "brand":       "#00D77C",
    "brand_hover": "#3BE39A",
    "on_brand":    "#03140B",   # white on #00D77C is ~2:1, near-black is ~11:1
    "brand_soft":  "rgba(0, 215, 124, 56)",
    "ring":        "#7CF0BE",
    "danger":      "#E5484D",
    "warn":        "#F5A524",
    "ok":          "#30A46C",
}

_ICONS = {
    "map": '<polygon points="3 6 9 3 15 6 21 3 21 18 15 21 9 18 3 21"/><line x1="9" y1="3" x2="9" y2="18"/><line x1="15" y1="6" x2="15" y2="21"/>',
    "refresh": '<polyline points="23 4 23 10 17 10"/><polyline points="1 20 1 14 7 14"/><path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15"/>',
    "car": '<path d="M19 17h2c.6 0 1-.4 1-1v-3c0-.9-.7-1.7-1.5-1.9C18.7 10.6 16 10 16 10s-1.3-1.4-2.2-2.3c-.5-.4-1.1-.7-1.8-.7H5c-.6 0-1.1.4-1.4.9l-1.4 2.9A3.7 3.7 0 0 0 2 12v4c0 .6.4 1 1 1h2"/><circle cx="7" cy="17" r="2"/><path d="M9 17h6"/><circle cx="17" cy="17" r="2"/>',
    "flag": '<path d="M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z"/><line x1="4" y1="22" x2="4" y2="15"/>',
    "person": '<circle cx="12" cy="5" r="1"/><path d="m9 20 3-6 3 6"/><path d="m6 8 6 2 6-2"/><path d="M12 10v4"/>',
    "cone": '<path d="M9.3 6.2a4.55 4.55 0 0 0 5.4 0"/><path d="M7.9 10.7c.9.8 2.4 1.3 4.1 1.3s3.2-.5 4.1-1.3"/><path d="M13.9 3.5a1.93 1.93 0 0 0-3.8-.1l-3 10c-.1.2-.1.4-.1.6 0 1.7 2.2 3 5 3s5-1.3 5-3c0-.2 0-.4-.1-.5Z"/><path d="m7.5 12.2-4.7 2.7c-.5.3-.8.7-.8 1.1s.3.8.8 1.1l7.6 4.5c.9.5 2.1.5 3 0l7.6-4.5c.7-.3 1-.7 1-1.1s-.3-.8-.8-1.1l-4.7-2.8"/>',
    "zap": '<polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/>',
    "pin": '<path d="M20 10c0 6-8 12-8 12s-8-6-8-12a8 8 0 0 1 16 0Z"/><circle cx="12" cy="10" r="3"/>',
    "route": '<circle cx="6" cy="19" r="3"/><path d="M9 19h8.5a3.5 3.5 0 0 0 0-7h-11a3.5 3.5 0 0 1 0-7H15"/><circle cx="18" cy="5" r="3"/>',
    "undo": '<path d="M3 7v6h6"/><path d="M21 17a9 9 0 0 0-9-9 9 9 0 0 0-6 2.3L3 13"/>',
    "redo": '<path d="M21 7v6h-6"/><path d="M3 17a9 9 0 0 1 9-9 9 9 0 0 1 6 2.3l3 2.7"/>',
    "save": '<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><polyline points="17 21 17 13 7 13 7 21"/><polyline points="7 3 7 8 15 8"/>',
    "folder": '<path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"/>',
    "file_plus": '<path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/><polyline points="14 2 14 8 20 8"/><line x1="12" y1="18" x2="12" y2="12"/><line x1="9" y1="15" x2="15" y2="15"/>',
    "export": '<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/>',
    "settings": '<line x1="4" y1="21" x2="4" y2="14"/><line x1="4" y1="10" x2="4" y2="3"/><line x1="12" y1="21" x2="12" y2="12"/><line x1="12" y1="8" x2="12" y2="3"/><line x1="20" y1="21" x2="20" y2="16"/><line x1="20" y1="12" x2="20" y2="3"/><line x1="1" y1="14" x2="7" y2="14"/><line x1="9" y1="8" x2="15" y2="8"/><line x1="17" y1="16" x2="23" y2="16"/>',
    "camera": '<polygon points="23 7 16 12 23 17 23 7"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/>',
    "trash": '<polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>',
    "copy": '<rect x="9" y="9" width="13" height="13" rx="2" ry="2"/><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/>',
    "fit": '<path d="M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3m0 18h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3"/>',
    "alert": '<path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/>',
    "error": '<circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/>',
    "info": '<circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/>',
    "check": '<path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/>',
    "help": '<circle cx="12" cy="12" r="10"/><path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"/><line x1="12" y1="17" x2="12.01" y2="17"/>',
    "stop": '<rect x="6" y="6" width="12" height="12" rx="1"/>',
    "crosshair": '<circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/>',
}


def _render(name, color, px=48):
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" '
           f'stroke="{color}" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
           f'{_ICONS[name]}</svg>')
    pm = QPixmap(px, px)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    QSvgRenderer(QByteArray(svg.encode())).render(p)
    p.end()
    return pm


@lru_cache(maxsize=None)
def icon(name, color=None):
    """QIcon for a named glyph; checked (On) state renders on_brand."""
    ic = QIcon()
    ic.addPixmap(_render(name, color or T["text"]), QIcon.Normal, QIcon.Off)
    ic.addPixmap(_render(name, T["on_brand"]), QIcon.Normal, QIcon.On)
    ic.addPixmap(_render(name, T["border"]), QIcon.Disabled, QIcon.Off)
    return ic


def logo_pixmap(size=28):
    """ARL logo mark scaled to size (null pixmap if missing)."""
    src = QPixmap(LOGO_PATH)
    if src.isNull():
        return src
    out = src.copy(LOGO_MARK).scaled(size * 2, size * 2, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    out.setDevicePixelRatio(2.0)
    return out


def _pick(families, fallback):
    have = set(QFontDatabase().families())
    return next((f for f in families if f in have), fallback)


def mono_font(pt=10):
    return QFont(_pick(["JetBrains Mono", "DejaVu Sans Mono", "Noto Sans Mono"], "Monospace"), pt)


QSS = """
QMainWindow, QDialog {{ background: {bg}; }}
QMainWindow::separator {{ background: {border}; width: 1px; height: 1px; }}
QMainWindow::separator:hover {{ background: {brand}; }}
QToolTip {{ background: {surface2}; color: {text}; border: 1px solid {border}; padding: 4px 6px; }}

QMenuBar {{ background: {surface}; border-bottom: 1px solid {border}; padding: 2px; }}
QMenuBar::item {{ padding: 4px 10px; border-radius: 4px; background: transparent; }}
QMenuBar::item:selected {{ background: {hover}; }}
QMenu {{ background: {surface}; border: 1px solid {border}; padding: 4px; }}
QMenu::item {{ padding: 6px 28px 6px 12px; border-radius: 4px; }}
QMenu::item:selected {{ background: {brand_soft}; color: {text}; }}
QMenu::item:disabled {{ color: {muted}; }}
QMenu::separator {{ height: 1px; background: {border}; margin: 4px 6px; }}
QMenu::icon {{ padding-left: 8px; }}

QToolBar {{ background: {surface}; border: none; border-bottom: 1px solid {border}; spacing: 2px; padding: 4px 8px; }}
QToolBar::separator {{ background: {border}; width: 1px; margin: 6px 6px; }}
QToolButton {{ background: transparent; border: 1px solid transparent; border-radius: 6px; padding: 5px 8px; }}
QToolButton:hover {{ background: {hover}; }}
QToolButton:pressed {{ background: {border}; }}
QToolButton:checked {{ background: {brand}; color: {on_brand}; border-color: {brand_hover}; }}
QToolButton:focus {{ border-color: {ring}; }}
QToolButton:disabled {{ color: {border}; }}
QToolButton#primary {{ background: {brand}; color: {on_brand}; font-weight: 600; padding: 5px 14px; }}
QToolButton#primary:hover {{ background: {brand_hover}; }}
QLabel#brand {{ font-size: 14px; font-weight: 700; padding: 0 10px 0 4px; }}
QLabel#muted {{ color: {muted}; }}

QDockWidget::title {{ background: {surface}; padding: 6px 10px; border-bottom: 1px solid {border}; font-weight: 600; color: {muted}; }}

QTreeWidget, QListWidget {{ background: {surface}; border: none; outline: 0; }}
QTreeWidget::item, QListWidget::item {{ padding: 4px 2px; border-radius: 4px; }}
QTreeWidget::item:hover, QListWidget::item:hover {{ background: {hover}; }}
QTreeWidget::item:selected, QListWidget::item:selected {{ background: {brand_soft}; color: {text}; }}
QHeaderView::section {{ background: {surface}; color: {muted}; border: none; padding: 4px; }}

QScrollArea {{ border: none; background: {surface}; }}
QScrollArea > QWidget > QWidget {{ background: {surface}; }}
QGroupBox {{ border: 1px solid {border}; border-radius: 6px; margin-top: 14px; padding: 10px 6px 6px 6px;
    font-weight: 600; color: {muted}; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}
QLabel {{ background: transparent; }}

QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QTextEdit {{ background: {surface2}; border: 1px solid {border};
    border-radius: 4px; padding: 4px 6px; selection-background-color: {brand}; min-height: 18px; }}
QLineEdit:hover, QDoubleSpinBox:hover, QSpinBox:hover, QComboBox:hover, QTextEdit:hover {{ border-color: {muted}; }}
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus, QTextEdit:focus {{ border-color: {ring}; }}
QComboBox {{ min-width: 120px; }}
QAbstractSpinBox {{ padding-right: 18px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{ subcontrol-origin: border; width: 16px;
    border: none; background: transparent; }}
QAbstractSpinBox::up-button {{ subcontrol-position: top right; border-top-right-radius: 4px; }}
QAbstractSpinBox::down-button {{ subcontrol-position: bottom right; border-bottom-right-radius: 4px; }}
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: {hover}; }}
QAbstractSpinBox::up-arrow {{ image: url({assets}/up.svg); width: 9px; height: 9px; }}
QAbstractSpinBox::down-arrow {{ image: url({assets}/down.svg); width: 9px; height: 9px; }}
QComboBox QAbstractItemView {{ background: {surface}; border: 1px solid {border}; selection-background-color: {brand_soft}; selection-color: {text}; }}
QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 14px; height: 14px; border: 1px solid {muted}; border-radius: 3px; background: {surface2}; }}
QCheckBox::indicator:hover, QCheckBox::indicator:focus {{ border-color: {ring}; }}
QCheckBox::indicator:checked {{ background: {brand}; border-color: {brand_hover}; image: url({assets}/check.svg); }}
QCheckBox::indicator:disabled {{ border-color: {border}; }}

QPushButton {{ background: {surface2}; border: 1px solid {border}; border-radius: 6px; padding: 6px 14px; font-weight: 600; }}
QPushButton:hover {{ background: {hover}; border-color: {muted}; }}
QPushButton:focus {{ border-color: {ring}; }}
QPushButton:disabled {{ color: {border}; border-color: {surface2}; }}
QPushButton:default, QPushButton#primary {{ background: {brand}; border-color: {brand}; color: {on_brand}; }}
QPushButton:default:hover, QPushButton#primary:hover {{ background: {brand_hover}; }}
QPushButton#danger {{ background: transparent; border-color: {danger}; color: {danger}; }}
QPushButton#danger:hover {{ background: {danger}; color: white; }}

QStatusBar {{ background: {surface}; border-top: 1px solid {border}; color: {muted}; }}
QStatusBar::item {{ border: none; }}
QStatusBar QLabel {{ color: {muted}; padding: 0 8px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle {{ background: {border}; border-radius: 4px; min-height: 24px; min-width: 24px; }}
QScrollBar::handle:hover {{ background: {muted}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: none; }}
""".format(assets=os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets").replace(os.sep, "/"), **T)


def apply(app):
    """Apply palette, font, stylesheet and window icon to the QApplication."""
    app.setStyle("Fusion")
    app.setFont(QFont(_pick(["Noto Sans", "Inter", "Ubuntu", "DejaVu Sans"], "Sans Serif"), 10))
    pal = QPalette()
    for role, key in [(QPalette.Window, "bg"), (QPalette.WindowText, "text"),
                      (QPalette.Base, "surface2"), (QPalette.AlternateBase, "surface"),
                      (QPalette.ToolTipBase, "surface2"), (QPalette.ToolTipText, "text"),
                      (QPalette.Text, "text"), (QPalette.Button, "surface2"),
                      (QPalette.ButtonText, "text"), (QPalette.BrightText, "danger"),
                      (QPalette.Link, "ring"), (QPalette.Highlight, "brand")]:
        pal.setColor(role, QColor(T[key]))
    pal.setColor(QPalette.HighlightedText, QColor(T["on_brand"]))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor(T["muted"]))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(T["muted"]))
    app.setPalette(pal)
    app.setStyleSheet(QSS)
    logo = logo_pixmap(64)
    if not logo.isNull():
        app.setWindowIcon(QIcon(logo))
