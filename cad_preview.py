"""Small cross-platform Qt preview for STEP selection and hot-wire motion."""

import time
import numpy as np

from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import (QColor, QPainter, QPainterPath, QPen, QPixmap,
                         QPolygonF)
from PyQt6.QtWidgets import QWidget


class CADPreview(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(720, 430)
        self.setStyleSheet("background-color: #171a1f; border: 1px solid #3d444d;")
        self.model = None
        self.toolpath = None
        self.machine = None
        self.root_face = None
        self.tip_face = None
        self.move_index = 0
        self.focus = "machine"
        self.yaw = np.radians(-28.0)
        self.pitch = np.radians(18.0)
        self.zoom = 1.0
        self._last_position = None
        self._dragged = False
        self._static_cache = None
        self._cached_project = None
        self._last_drag_paint = 0.0

    def _invalidate_static(self):
        self._static_cache = None
        self._cached_project = None

    def clear(self):
        self.model = None
        self.toolpath = None
        self.machine = None
        self._invalidate_static()
        self.update()

    def set_scene(self, model, root_section=None, tip_section=None,
                  toolpath=None, machine=None):
        self.model = model
        self.toolpath = toolpath
        self.machine = machine
        self.root_face = root_section.face_index if root_section else None
        self.tip_face = tip_section.face_index if tip_section else None
        if toolpath is not None:
            self.move_index = min(self.move_index, len(toolpath.tower_left) - 1)
        self._invalidate_static()
        self.update()

    def set_move_index(self, index):
        self.move_index = max(0, int(index))
        self.update()

    def set_focus(self, focus):
        self.focus = focus
        self.zoom = 1.0
        self._invalidate_static()
        self.update()

    def _camera_coordinates(self, points):
        points = np.asarray(points, dtype=float)
        cosine, sine = np.cos(self.yaw), np.sin(self.yaw)
        x_view = points[:, 0] * cosine - points[:, 2] * sine
        depth = points[:, 0] * sine + points[:, 2] * cosine
        cosine, sine = np.cos(self.pitch), np.sin(self.pitch)
        y_view = points[:, 1] * cosine - depth * sine
        depth_view = points[:, 1] * sine + depth * cosine
        return np.column_stack((x_view, y_view, depth_view))

    def _scene_points(self, vertices):
        points = [array for array in vertices if len(array)]
        if (self.toolpath is not None and self.machine is not None
                and self.focus == "machine"):
            width = self.machine.horizontal_travel
            height = self.machine.vertical_travel
            span = self.machine.tower_span
            frame = np.asarray([
                [0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0],
                [0, 0, span], [width, 0, span], [width, height, span], [0, height, span],
            ], dtype=float)
            points.append(frame)
        return np.vstack(points) if points else np.zeros((1, 3))

    def _projector(self, vertices):
        camera = self._camera_coordinates(self._scene_points(vertices))
        minimum = camera[:, :2].min(axis=0)
        maximum = camera[:, :2].max(axis=0)
        size = np.maximum(maximum - minimum, 1.0)
        padding = 36.0
        scale = min((self.width() - 2 * padding) / size[0],
                    (self.height() - 2 * padding) / size[1]) * self.zoom
        center = 0.5 * (minimum + maximum)
        screen_center = np.array([self.width() * 0.5, self.height() * 0.5])

        def project(points):
            transformed = self._camera_coordinates(points)
            screen = (transformed[:, :2] - center) * scale
            screen[:, 1] *= -1
            screen += screen_center
            return screen, transformed[:, 2]
        return project

    @staticmethod
    def _polygon(points):
        return QPolygonF([QPointF(float(point[0]), float(point[1])) for point in points])

    def _draw_line_3d(self, painter, project, start, end, color, width=1.0,
                      style=Qt.PenStyle.SolidLine):
        screen, _ = project(np.asarray([start, end], dtype=float))
        painter.setPen(QPen(color, width, style))
        painter.drawLine(QPointF(*screen[0]), QPointF(*screen[1]))

    def _draw_polyline_3d(self, painter, project, points, color, width=1.0):
        if len(points) < 2:
            return
        screen, _ = project(np.asarray(points, dtype=float))
        path = QPainterPath(QPointF(*screen[0]))
        for point in screen[1:]:
            path.lineTo(QPointF(*point))
        painter.setPen(QPen(color, width))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(path)

    def _build_static_cache(self):
        """Render everything except the moving wire once per camera view."""
        cache = QPixmap(self.size())
        cache.fill(QColor(23, 26, 31))
        painter = QPainter(cache)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, not self._dragged)
        if self.model is None:
            painter.setPen(QColor(165, 172, 182))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                             "Import a STEP model to begin")
            painter.end()
            self._static_cache = cache
            return

        if self.focus == "cut" and self.toolpath is not None:
            vertices = [self.toolpath.cut_root, self.toolpath.cut_tip]
            project = self._projector(vertices)
            self._draw_cut_surface(painter, project)
            painter.setPen(QColor(210, 215, 224))
            painter.drawText(
                14, 22,
                "Generated wire-swept cut surface  •  drag to rotate  •  wheel to zoom",
            )
            painter.end()
            self._static_cache = cache
            self._cached_project = project
            return

        vertices = (self.toolpath.model_vertices if self.toolpath is not None
                    else [mesh.vertices for mesh in self.model.face_meshes])
        project = self._projector(vertices)
        triangle_count = sum(len(mesh.triangles) for mesh in self.model.face_meshes)
        triangle_stride = 1
        if self._dragged and triangle_count > 2500:
            triangle_stride = int(np.ceil(triangle_count / 2500.0))
        faces = []
        for mesh, face_vertices in zip(self.model.face_meshes, vertices):
            screen, depth = project(face_vertices)
            face_path = QPainterPath()
            face_path.setFillRule(Qt.FillRule.WindingFill)
            # During rotation, cap very dense models to a temporary low-detail
            # representation. Mouse release restores the complete cached view.
            for triangle in mesh.triangles[::triangle_stride]:
                indices = np.asarray(triangle, dtype=int)
                face_path.addPolygon(self._polygon(screen[indices]))
                face_path.closeSubpath()
            faces.append((float(np.mean(depth)), mesh.face_index, face_path))

        # Face-level depth sorting and batching avoids thousands of individual
        # draw calls while keeping the selected sections visually distinct.
        faces.sort(key=lambda item: item[0])
        for depth, face_index, face_path in faces:
            if face_index == self.root_face:
                fill = QColor(24, 190, 110, 150)
                edge = QColor(80, 255, 160, 210)
            elif face_index == self.tip_face:
                fill = QColor(25, 135, 235, 150)
                edge = QColor(90, 185, 255, 210)
            else:
                shade = int(np.clip(86 + depth * 0.025, 58, 126))
                fill = QColor(shade, shade + 7, shade + 15, 105)
                edge = QColor(125, 135, 148, 100)
            painter.setBrush(fill)
            painter.setPen(QPen(edge, 0.55))
            painter.drawPath(face_path)

        if self.toolpath is not None and self.machine is not None:
            self._draw_machine_static(painter, project)

        painter.setPen(QColor(210, 215, 224))
        painter.drawText(14, 22, "Drag to rotate  •  wheel to zoom")
        painter.end()
        self._static_cache = cache
        self._cached_project = project

    def _draw_cut_surface(self, painter, project):
        """Draw the surface swept by the wire between successive machine moves."""
        root = self.toolpath.cut_root
        tip = self.toolpath.cut_tip
        root_screen, _ = project(root)
        tip_screen, _ = project(tip)

        surface = QPainterPath()
        surface.setFillRule(Qt.FillRule.WindingFill)
        for index in range(min(len(root), len(tip)) - 1):
            surface.addPolygon(self._polygon([
                root_screen[index], root_screen[index + 1],
                tip_screen[index + 1], tip_screen[index],
            ]))
            surface.closeSubpath()
        painter.setBrush(QColor(244, 139, 45, 135))
        painter.setPen(QPen(QColor(255, 202, 105, 105), 0.7))
        painter.drawPath(surface)

        self._draw_polyline_3d(
            painter, project, root, QColor(35, 225, 125, 235), 2.2
        )
        self._draw_polyline_3d(
            painter, project, tip, QColor(40, 155, 255, 235), 2.2
        )
        count = min(len(root), len(tip))
        for index in np.unique(np.linspace(0, count - 1, min(32, count), dtype=int)):
            self._draw_line_3d(
                painter, project, root[index], tip[index],
                QColor(255, 225, 135, 115), 0.9,
            )

    def paintEvent(self, event):
        if (self._static_cache is None
                or self._static_cache.size() != self.size()):
            self._build_static_cache()

        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._static_cache)
        if self.toolpath is not None:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            self._draw_live_wire(painter, self._cached_project)
            painter.setPen(QColor(255, 202, 80))
            deviation_label = (
                "outer-only deviation" if self.toolpath.ignored_interior_count
                else "surface deviation"
            )
            painter.drawText(
                14, self.height() - 14,
                "Wire {}/{}   {} {:.3f} mm   wire length {:.1f}–{:.1f} mm".format(
                    self.move_index + 1, len(self.toolpath.tower_left),
                    deviation_label, self.toolpath.surface_error,
                    self.toolpath.wire_length_min, self.toolpath.wire_length_max,
                )
            )

    def _tower_paths(self):
        span = self.machine.tower_span
        left_path = np.column_stack((
            self.toolpath.tower_left,
            np.zeros(len(self.toolpath.tower_left)),
        ))
        right_path = np.column_stack((
            self.toolpath.tower_right,
            np.full(len(self.toolpath.tower_right), span),
        ))
        return left_path, right_path

    def _draw_machine_static(self, painter, project):
        width = self.machine.horizontal_travel
        height = self.machine.vertical_travel
        span = self.machine.tower_span
        frame_color = QColor(170, 180, 195, 125)
        for z in (0.0, span):
            corners = [[0, 0, z], [width, 0, z], [width, height, z], [0, height, z], [0, 0, z]]
            self._draw_polyline_3d(painter, project, corners, frame_color, 1.2)

        left_path, right_path = self._tower_paths()
        self._draw_polyline_3d(painter, project, left_path, QColor(35, 225, 125, 210), 1.8)
        self._draw_polyline_3d(painter, project, right_path, QColor(40, 155, 255, 210), 1.8)
        self._draw_line_3d(
            painter, project, [0, 0, 0], left_path[0],
            QColor(185, 190, 198, 120), 1.0, Qt.PenStyle.DashLine,
        )
        self._draw_line_3d(
            painter, project, [0, 0, span], right_path[0],
            QColor(185, 190, 198, 120), 1.0, Qt.PenStyle.DashLine,
        )

        # A sparse set of rulings makes the swept surface legible without hiding
        # the imported model. The bright line is the current physical wire.
        count = len(left_path)
        for index in np.unique(np.linspace(0, count - 1, min(18, count), dtype=int)):
            self._draw_line_3d(
                painter, project, left_path[index], right_path[index],
                QColor(255, 173, 55, 55), 0.8,
            )

        for label, point, color in (
            ("XY tower", [0, height, 0], QColor(70, 245, 145)),
            ("UV tower", [0, height, span], QColor(80, 180, 255)),
        ):
            screen, _ = project(np.asarray([point], dtype=float))
            painter.setPen(color)
            painter.drawText(QPointF(screen[0, 0] + 6, screen[0, 1] - 5), label)

    def _draw_live_wire(self, painter, project):
        if self.focus == "cut":
            count = min(len(self.toolpath.cut_root), len(self.toolpath.cut_tip))
            index = min(self.move_index, count - 1)
            endpoints = np.asarray([
                self.toolpath.cut_root[index], self.toolpath.cut_tip[index]
            ])
            self._draw_line_3d(
                painter, project, endpoints[0], endpoints[1],
                QColor(255, 235, 95), 3.0,
            )
            screen, _ = project(endpoints)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 245, 135))
            for point in screen:
                painter.drawEllipse(QPointF(*point), 4.5, 4.5)
            return

        left_path, right_path = self._tower_paths()
        count = len(left_path)
        index = min(self.move_index, count - 1)
        self._draw_line_3d(
            painter, project, left_path[index], right_path[index],
            QColor(255, 205, 70), 3.0,
        )
        screen, _ = project(np.asarray([left_path[index], right_path[index]]))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(255, 225, 110))
        for point in screen:
            painter.drawEllipse(QPointF(*point), 4.5, 4.5)

    def mousePressEvent(self, event):
        self._last_position = event.position()
        self._dragged = False

    def mouseMoveEvent(self, event):
        if self._last_position is None:
            return
        delta = event.position() - self._last_position
        if abs(delta.x()) + abs(delta.y()) > 1:
            self._dragged = True
            self.yaw += delta.x() * 0.008
            self.pitch = float(np.clip(self.pitch + delta.y() * 0.008, -1.35, 1.35))
            self._last_position = event.position()
            self._invalidate_static()
            now = time.monotonic()
            if now - self._last_drag_paint >= 1.0 / 60.0:
                self._last_drag_paint = now
                self.update()

    def mouseReleaseEvent(self, event):
        was_dragged = self._dragged
        self._dragged = False
        if was_dragged:
            # Replace the low-cost drag frame with an antialiased final frame.
            self._invalidate_static()
            self.update()
        self._last_position = None

    def wheelEvent(self, event):
        factor = 1.12 if event.angleDelta().y() > 0 else 1 / 1.12
        self.zoom = float(np.clip(self.zoom * factor, 0.35, 12.0))
        self._invalidate_static()
        self.update()

    def resizeEvent(self, event):
        self._invalidate_static()
        super().resizeEvent(event)
