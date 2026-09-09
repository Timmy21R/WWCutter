import sys
import time
import numpy as np
import serial.tools.list_ports
import ezdxf
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QWidget, QSlider, QLabel, QFileDialog, QHBoxLayout, 
                             QDoubleSpinBox, QSpinBox, QComboBox)
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal

class HomingWorker(QThread):
    finished = pyqtSignal()
    
    def __init__(self, serial_conn):
        super().__init__()
        self.serial_conn = serial_conn
        self.is_running = True

    def run(self):
        self.serial_conn.write(b"HOME\n")
        while self.is_running:
            if self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if "OK HOME" in line:
                    break
        self.finished.emit()
        
    def stop(self):
        self.is_running = False

class HotWireController(QMainWindow):
    # Hardware calibration only. No digital scaling required for DXFs.
    STEPS_PER_MM = 1309.6  
    MAX_SEGMENT_MM = 1.0
    DXF_JOIN_TOLERANCE_MM = 0.5

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HotWire CNC Controller (DXF)")
        self.serial_conn = None
        self.dxf_a = None
        self.dxf_b = None
        self.setup_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(500)
        
        self.auto_connect()

    def setup_ui(self):
        layout = QVBoxLayout()
        self.status_label = QLabel("Status: Disconnected")
        layout.addWidget(self.status_label)
        
        self.time_label = QLabel("Est. Job Time: 0s")
        layout.addWidget(self.time_label)
        
        layout.addWidget(QPushButton("Home", clicked=self.home_machine))
        layout.addWidget(QPushButton("Start Job", clicked=self.start_job))
        layout.addWidget(QPushButton("Stop / Force Origin", clicked=self.stop_machine))
        layout.addWidget(QPushButton("Go to Origin", clicked=self.go_to_origin))
        layout.addWidget(QPushButton("Set Current Position as Origin", clicked=self.set_current_as_origin))
        
        layout.addWidget(QPushButton("Import DXF Side A", clicked=lambda: self.load_dxf("A")))
        layout.addWidget(QPushButton("Import DXF Side B", clicked=lambda: self.load_dxf("B")))
        
        origin_layout = QHBoxLayout()
        origin_layout.addWidget(QLabel("Job Origin X (mm):"))
        self.origin_x = QDoubleSpinBox(); self.origin_x.setRange(-2000, 2000)
        origin_layout.addWidget(self.origin_x)
        
        origin_layout.addWidget(QLabel("Y:"))
        self.origin_y = QDoubleSpinBox(); self.origin_y.setRange(-2000, 2000)
        origin_layout.addWidget(self.origin_y)
        
        origin_layout.addWidget(QLabel("U:"))
        self.origin_u = QDoubleSpinBox(); self.origin_u.setRange(-2000, 2000)
        origin_layout.addWidget(self.origin_u)
        
        origin_layout.addWidget(QLabel("V:"))
        self.origin_v = QDoubleSpinBox(); self.origin_v.setRange(-2000, 2000)
        origin_layout.addWidget(self.origin_v)
        for box in (self.origin_x, self.origin_y, self.origin_u, self.origin_v):
            box.valueChanged.connect(self.update_preview)
        
        layout.addLayout(origin_layout)
                
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Maximum Cruise Speed (steps/sec):"))
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(100, 5000)
        self.speed_slider.setValue(2500)
        
        self.speed_box = QSpinBox()
        self.speed_box.setRange(100, 5000)
        self.speed_box.setValue(2500)
        
        self.speed_slider.valueChanged.connect(self.speed_box.setValue)
        self.speed_box.valueChanged.connect(self.speed_slider.setValue)
        self.speed_box.valueChanged.connect(self.update_speed)
        
        self.speed_box.valueChanged.connect(self.update_speed)
        self.speed_box.valueChanged.connect(self.update_time_estimate)

        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("DXF Rotation (Degrees):"))
        self.rot_combo = QComboBox()
        self.rot_combo.addItems(["0", "90", "180", "270"])
        self.rot_combo.currentTextChanged.connect(self.update_preview)
        self.rot_combo.currentTextChanged.connect(self.update_time_estimate)
        rot_layout.addWidget(self.rot_combo)
        layout.addLayout(rot_layout)
        
        layout.addWidget(QLabel("Toolpath Preview (XY = green, UV = blue):"))
        self.preview_label = QLabel("Load a DXF to preview")
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_label.setMinimumHeight(250)
        self.preview_label.setStyleSheet("background-color: #1e1e1e; color: #aaa; border: 1px solid #444;")
        layout.addWidget(self.preview_label)
        self.preview_move = QSlider(Qt.Orientation.Horizontal)
        self.preview_move.valueChanged.connect(self.update_preview)
        self.preview_move.setEnabled(False)
        layout.addWidget(self.preview_move)
        self.preview_coords = QLabel("Move: -   X: -   Y: -   U: -   V: -")
        layout.addWidget(self.preview_coords)
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_box)
        layout.addLayout(speed_layout)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def set_status(self, text):
        self.status_label.setText(f"Status: {text}")
        
    def calculate_estimate(self, points_a, points_b, speeds):
        total_time = 0.0
        min_len = min(len(points_a), len(points_b))
        
        for i in range(1, min_len):
            # Calculate physical distance for each tower independently
            dist_a = np.hypot(points_a[i][0] - points_a[i-1][0], points_a[i][1] - points_a[i-1][1])
            dist_b = np.hypot(points_b[i][0] - points_b[i-1][0], points_b[i][1] - points_b[i-1][1])
            
            # The machine coordinates timing based on the longest move
            max_dist_mm = max(dist_a, dist_b)
            
            speed = max(speeds[i], 100) 
            total_time += (max_dist_mm * self.STEPS_PER_MM) / speed
            
        total_seconds = int(total_time)
        hours = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        seconds = total_seconds % 60
        
        if hours > 0:
            time_str = f"{hours}h {minutes}m {seconds}s"
        elif minutes > 0:
            time_str = f"{minutes}m {seconds}s"
        else:
            time_str = f"{seconds}s"
            
        self.time_label.setText(f"Est. Job Time: {time_str}")
     
    def update_time_estimate(self):
        # 1. Bail out if both DXFs aren't loaded yet
        if not (self.dxf_a and self.dxf_b): 
            return
            
        # 2. Extract raw points (You likely missed these two lines)
        points_a, points_b = self.get_job_toolpaths()

        # 3. Bail out if the extraction failed or returned empty
        if not points_a or not points_b:
            return

        # 5. Calculate and display final estimate
        smoothed_speeds = self.calculate_job_speeds(points_a, points_b)
        self.calculate_estimate(points_a, points_b, smoothed_speeds)

    def calculate_dynamic_speeds(self, points):
        base_speed = self.speed_box.value()
        corner_ratio = 0.35  
        
        raw_speeds = []
        for i in range(len(points)):
            if i == 0 or i == len(points) - 1:
                raw_speeds.append(base_speed)
                continue
                
            v1_x = points[i][0] - points[i-1][0]
            v1_y = points[i][1] - points[i-1][1]
            v2_x = points[i+1][0] - points[i][0]
            v2_y = points[i+1][1] - points[i][1]
            
            angle1 = np.arctan2(v1_y, v1_x)
            angle2 = np.arctan2(v2_y, v2_x)
            diff = abs(angle2 - angle1)
            if diff > np.pi: diff = 2 * np.pi - diff
                
            normalized_diff = min(diff / (np.pi / 2), 1.0)
            multiplier = 1.0 - (normalized_diff * (1.0 - corner_ratio))
            raw_speeds.append(int(base_speed * multiplier))
            
        smoothed_speeds = []
        for i in range(len(raw_speeds)):
            prev_s = raw_speeds[i-1] if i > 0 else raw_speeds[i]
            next_s = raw_speeds[i+1] if i < len(raw_speeds)-1 else raw_speeds[i]
            smoothed_speeds.append(min(raw_speeds[i], prev_s, next_s))
            
        return smoothed_speeds

    def calculate_job_speeds(self, points_a, points_b):
        return [min(a, b) for a, b in zip(self.calculate_dynamic_speeds(points_a),
                                          self.calculate_dynamic_speeds(points_b))]

    def poll_status(self):
        if self.serial_conn and self.serial_conn.is_open:
            while self.serial_conn.in_waiting:
                try:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if "OK UPDONE" in line:
                        self.set_status("Ready")
                except Exception as e:
                    print(f"Serial read error: {e}")

    def auto_connect(self):
        self.set_status("Connecting...")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or "USB" in port.description:
                self.serial_conn = serial.Serial(port.device, 115200, timeout=1)
                self.set_status("Ready")
                return
        self.set_status("Connection Failed")

    def load_dxf(self, side):
        path, _ = QFileDialog.getOpenFileName(self, f"Select DXF for {side}", filter="DXF Files (*.dxf)")
        if path:
            if side == "A": 
                self.dxf_a = path
            else: 
                self.dxf_b = path
            self.set_status(f"Loaded DXF {side}")
            self.update_preview()  
            self.update_time_estimate()
    
    def rotate_points(self, points, angle_deg):
        if not points: return []
        angle_rad = np.radians(angle_deg)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        rotated = []
        for x, y in points:
            nx = x * cos_a - y * sin_a
            ny = x * sin_a + y * cos_a
            rotated.append((nx, ny))
        return rotated

    def update_preview(self):
        if not (self.dxf_a and self.dxf_b): return
        paths = self.get_job_toolpaths()
        if not all(paths): return
        moves = self.get_machine_moves(*paths)
        if not moves: return
        draw_paths = [[(m[a] / self.STEPS_PER_MM, m[a + 1] / self.STEPS_PER_MM) for m in moves]
                      for a in (0, 2)]
        points = draw_paths[0] + draw_paths[1]
        move_count = len(moves)
        self.preview_move.setEnabled(True)
        self.preview_move.setMaximum(max(0, move_count - 1))

        min_x, max_x = min(p[0] for p in points), max(p[0] for p in points)
        min_y, max_y = min(p[1] for p in points), max(p[1] for p in points)
        width, height = max_x - min_x, max_y - min_y
        if width == 0: width = 1
        if height == 0: height = 1
        
        cw, ch, padding = 600, 250, 20
        scale = min((cw - padding*2) / width, (ch - padding*2) / height)
        
        pixmap = QPixmap(cw, ch)
        pixmap.fill(QColor(30, 30, 30))
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        screen = lambda p: (padding + (p[0] - min_x) * scale,
                            ch - padding - (p[1] - min_y) * scale)
        for toolpath, color in zip(draw_paths, (QColor(0, 255, 100), QColor(0, 160, 255))):
            painter.setPen(QPen(color, 2))
            for p1, p2 in zip(toolpath, toolpath[1:]):
                painter.drawLine(*map(int, (*screen(p1), *screen(p2))))
            if toolpath:
                painter.setBrush(color); painter.setPen(Qt.PenStyle.NoPen)
                x, y = screen(toolpath[min(self.preview_move.value(), len(toolpath) - 1)])
                painter.drawEllipse(int(x) - 5, int(y) - 5, 10, 10)

        i = self.preview_move.value()
        if i < move_count:
            x, y, u, v = (n / self.STEPS_PER_MM for n in moves[i])
            self.preview_coords.setText(f"Move {i + 1}/{move_count}   X: {x:.2f}   Y: {y:.2f}   U: {u:.2f}   V: {v:.2f} mm")
        
        painter.end()
        self.preview_label.setPixmap(pixmap)

    def get_job_toolpaths(self):
        angle = int(self.rot_combo.currentText())
        paths = [self.process_toolpath(self.get_dxf_points(f), angle) for f in (self.dxf_a, self.dxf_b)]
        return self.synchronize_toolpaths(*paths) if all(paths) else paths

    def synchronize_toolpaths(self, *paths):
        contours, fractions, lengths = [], [], []
        orientation = None
        for path in paths:
            points = np.asarray(path[1:], dtype=float)
            if np.allclose(points[0], points[-1]):
                direction = np.sign(np.sum(points[:-1, 0] * points[1:, 1] - points[1:, 0] * points[:-1, 1]))
                if orientation is None: orientation = direction
                elif direction and orientation and direction != orientation: points = points[::-1]
            distance = np.r_[0.0, np.cumsum(np.hypot(*np.diff(points, axis=0).T))]
            keep = np.r_[True, np.diff(distance) > 1e-9]
            points, distance = points[keep], distance[keep]
            if len(points) < 2 or distance[-1] == 0: return list(paths)
            contours.append(points); lengths.append(distance[-1]); fractions.append(distance / distance[-1])
        samples = np.unique(np.concatenate((*fractions, np.linspace(0, 1, int(np.ceil(max(lengths) / self.MAX_SEGMENT_MM)) + 1))))
        return [[path[0]] + list(zip(np.interp(samples, fraction, points[:, 0]),
                                     np.interp(samples, fraction, points[:, 1])))
                for path, points, fraction in zip(paths, contours, fractions)]

    def get_machine_moves(self, points_a, points_b):
        origins = [int(box.value() * self.STEPS_PER_MM) for box in
                   (self.origin_x, self.origin_y, self.origin_u, self.origin_v)]
        mins = [min(p[a] for p in points) for points in (points_a, points_b) for a in (0, 1)]
        return [tuple(int((p[a] - mins[j]) * self.STEPS_PER_MM) + origins[j]
                      for j, (p, a) in enumerate(((pa, 0), (pa, 1), (pb, 0), (pb, 1))))
                for pa, pb in zip(points_a, points_b)]
        
    def process_toolpath(self, points, angle):
        if not points: return []
        
        # 1. Rotate
        points = self.rotate_points(points, angle)
        
        # 2. Find bounding box bottom-left (Job Origin)
        pts = np.array(points)
        min_x, min_y = np.min(pts[:, 0]), np.min(pts[:, 1])
        
        # 3. Shift array so the cut starts at the point closest to the origin
        is_closed = np.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6
        if is_closed:
            dists = np.hypot(pts[:, 0] - min_x, pts[:, 1] - min_y)
            best_start_idx = np.argmin(dists)
            if best_start_idx != 0:
                pts_no_end = pts[:-1]
                pts_rolled = np.roll(pts_no_end, -best_start_idx, axis=0)
                final_pts = np.vstack((pts_rolled, pts_rolled[0])).tolist()
            else:
                final_pts = pts.tolist()
        else:
            dist_start = np.hypot(pts[0][0] - min_x, pts[0][1] - min_y)
            dist_end = np.hypot(pts[-1][0] - min_x, pts[-1][1] - min_y)
            if dist_end < dist_start:
                final_pts = pts[::-1].tolist()
            else:
                final_pts = pts.tolist()
                
        # The first item is the positioning target kept separate by
        # synchronize_toolpaths().  Duplicating the real cut start makes that
        # positioning segment zero-length instead of inventing a diagonal from
        # the bounding-box corner to the DXF geometry.
        final_pts.insert(0, list(final_pts[0]))
        return [tuple(x) for x in final_pts]

    @staticmethod
    def _point_segment_projection(point, start, end):
        px, py = point
        ax, ay = start
        bx, by = end
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-18:
            return (ax, ay), 0.0, np.hypot(px - ax, py - ay)
        fraction = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
        projected = (ax + fraction * dx, ay + fraction * dy)
        return projected, fraction, np.hypot(px - projected[0], py - projected[1])

    def _join_dxf_paths(self, paths, tolerance):
        """Snap entity endpoints to connected geometry and insert junctions."""
        paths = [[tuple(map(float, point)) for point in path] for path in paths]
        endpoint_refs = []
        for path_index, path in enumerate(paths):
            if np.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1]) <= 1e-7:
                path[-1] = path[0]
                continue
            endpoint_refs.extend(((path_index, 0), (path_index, len(path) - 1)))

        parents = list(range(len(endpoint_refs)))

        def find(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        def union(left, right):
            left_root, right_root = find(left), find(right)
            if left_root != right_root:
                parents[right_root] = left_root

        for left in range(len(endpoint_refs)):
            left_path, left_vertex = endpoint_refs[left]
            left_point = paths[left_path][left_vertex]
            for right in range(left + 1, len(endpoint_refs)):
                right_path, right_vertex = endpoint_refs[right]
                right_point = paths[right_path][right_vertex]
                if np.hypot(left_point[0] - right_point[0], left_point[1] - right_point[1]) <= tolerance:
                    union(left, right)

        endpoint_groups = {}
        for index, reference in enumerate(endpoint_refs):
            endpoint_groups.setdefault(find(index), []).append(reference)

        insertions = {index: [] for index in range(len(paths))}
        for references in endpoint_groups.values():
            source_paths = {path_index for path_index, _ in references}
            point = paths[references[0][0]][references[0][1]]
            best = None

            # Entry-line endpoints often meet the middle of a flattened circle,
            # spline, or polyline segment. Find that attachment explicitly.
            for path_index, path in enumerate(paths):
                if path_index in source_paths:
                    continue
                for segment_index, (start, end) in enumerate(zip(path, path[1:])):
                    projected, fraction, distance = self._point_segment_projection(point, start, end)
                    if distance <= tolerance and (best is None or distance < best[0]):
                        best = (distance, path_index, segment_index, fraction, projected)

            if best is not None:
                _, target_path, segment_index, fraction, junction = best
                point = junction
                if 1e-9 < fraction < 1.0 - 1e-9:
                    insertions[target_path].append((segment_index, fraction, point))

            for path_index, vertex_index in references:
                paths[path_index][vertex_index] = point

        for path_index, pending in insertions.items():
            unique = {}
            for segment_index, fraction, point in pending:
                key = (segment_index, round(fraction, 12))
                unique[key] = (segment_index, fraction, point)
            for segment_index, _, point in sorted(unique.values(), reverse=True):
                paths[path_index].insert(segment_index + 1, point)

        return paths

    @staticmethod
    def _dxf_graph_bridges(node_count, edges, adjacency):
        discovery = [-1] * node_count
        low = [0] * node_count
        parent_node = [-1] * node_count
        parent_edge = [-1] * node_count
        bridges = set()
        next_time = 0

        discovery[0] = low[0] = next_time
        next_time += 1
        stack = [(0, 0)]
        while stack:
            node, next_edge = stack[-1]
            if next_edge < len(adjacency[node]):
                edge_index = adjacency[node][next_edge]
                stack[-1] = (node, next_edge + 1)
                if edge_index == parent_edge[node]:
                    continue
                left, right = edges[edge_index]
                neighbour = right if left == node else left
                if discovery[neighbour] < 0:
                    parent_node[neighbour] = node
                    parent_edge[neighbour] = edge_index
                    discovery[neighbour] = low[neighbour] = next_time
                    next_time += 1
                    stack.append((neighbour, 0))
                else:
                    low[node] = min(low[node], discovery[neighbour])
                continue

            stack.pop()
            if parent_node[node] >= 0:
                parent = parent_node[node]
                if low[node] > discovery[parent]:
                    bridges.add(parent_edge[node])
                low[parent] = min(low[parent], low[node])

        return bridges, discovery

    def _trace_dxf_paths(self, paths):
        """Return an Euler trail made solely from original DXF segments."""
        coordinates = []
        node_by_coordinate = {}
        edges = []

        def node_for(point):
            key = (round(point[0], 9), round(point[1], 9))
            if key not in node_by_coordinate:
                node_by_coordinate[key] = len(coordinates)
                coordinates.append(tuple(point))
            return node_by_coordinate[key]

        for path in paths:
            for start, end in zip(path, path[1:]):
                if np.hypot(start[0] - end[0], start[1] - end[1]) <= 1e-9:
                    continue
                edges.append((node_for(start), node_for(end)))

        if not edges:
            return []

        adjacency = [[] for _ in coordinates]
        for edge_index, (left, right) in enumerate(edges):
            adjacency[left].append(edge_index)
            adjacency[right].append(edge_index)

        bridges, discovery = self._dxf_graph_bridges(len(coordinates), edges, adjacency)
        if any(value < 0 for value in discovery):
            raise ValueError("DXF contains disconnected geometry without entry lines")

        traversal_edges = list(edges)
        if bridges and len(bridges) < len(edges):
            # A closed contour joined to another contour by an entry line has
            # odd junctions. Retracing graph bridges is the required exit;
            # unlike the former fallback, it never adds a new cutting line.
            traversal_edges.extend(edges[index] for index in sorted(bridges))

        traversal_adjacency = [[] for _ in coordinates]
        for edge_index, (left, right) in enumerate(traversal_edges):
            traversal_adjacency[left].append(edge_index)
            traversal_adjacency[right].append(edge_index)

        odd_nodes = [node for node, incident in enumerate(traversal_adjacency) if len(incident) % 2]
        if len(odd_nodes) not in (0, 2):
            raise ValueError("DXF geometry cannot be followed as one continuous toolpath")

        min_x = min(point[0] for point in coordinates)
        min_y = min(point[1] for point in coordinates)
        candidates = odd_nodes or range(len(coordinates))
        start_node = min(candidates, key=lambda node: np.hypot(
            coordinates[node][0] - min_x, coordinates[node][1] - min_y))

        used = [False] * len(traversal_edges)
        stack = [start_node]
        circuit = []
        while stack:
            node = stack[-1]
            while traversal_adjacency[node] and used[traversal_adjacency[node][-1]]:
                traversal_adjacency[node].pop()
            if not traversal_adjacency[node]:
                circuit.append(stack.pop())
                continue
            edge_index = traversal_adjacency[node].pop()
            if used[edge_index]:
                continue
            used[edge_index] = True
            left, right = traversal_edges[edge_index]
            stack.append(right if left == node else left)

        if not all(used):
            raise ValueError("DXF geometry could not be fully traversed")
        return [coordinates[node] for node in reversed(circuit)]

    def get_dxf_points(self, dxf_file):
        try:
            doc = ezdxf.readfile(dxf_file)
            msp = doc.modelspace()
            paths = []
            
            # 1. Extract geometry in solid, continuous chunks
            for e in msp:
                try:
                    p = ezdxf.path.make_path(e)
                    verts = list(p.flattening(distance=0.05))
                    if len(verts) > 1:
                        paths.append([(v.x, v.y) for v in verts])
                except: continue
                    
            if not paths: return []
            
            joined = self._join_dxf_paths(paths, self.DXF_JOIN_TOLERANCE_MM)
            return self._trace_dxf_paths(joined)
            
        except Exception as e:
            print(f"Failed to parse DXF: {e}")
            return []

    def set_current_as_origin(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        
        # 1. Force the Arduino to reset its internal absolute coordinates to 0
        self.serial_conn.write(b"SETPOS,0,0,0,0\n")
        
        # 2. Zero out the UI offset boxes
        self.origin_x.setValue(0)
        self.origin_y.setValue(0)
        self.origin_u.setValue(0)
        self.origin_v.setValue(0)
        
        self.set_status("Origin set to current position")

    def go_to_origin(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        self.set_status("Moving to Origin...")
        x_steps = int(self.origin_x.value() * self.STEPS_PER_MM)
        y_steps = int(self.origin_y.value() * self.STEPS_PER_MM)
        u_steps = int(self.origin_u.value() * self.STEPS_PER_MM)
        v_steps = int(self.origin_v.value() * self.STEPS_PER_MM)
        self.serial_conn.write(f"MOVE,{x_steps},{y_steps},{u_steps},{v_steps}\n".encode())

    def start_job(self):
        if not (self.dxf_a and self.dxf_b and self.serial_conn): return
        
        points_a, points_b = self.get_job_toolpaths()
        
        if not points_a or not points_b:
            self.set_status("Error: Empty or invalid DXF profile")
            return
            
        self.set_status("Calculating Toolpath Dynamics...")

        smoothed_speeds = self.calculate_job_speeds(points_a, points_b)
        self.calculate_estimate(points_a, points_b, smoothed_speeds)
        
        self.serial_conn.write(f"UPLOAD,{len(points_a)}\n".encode())
        self.set_status("Uploading...")
        
        for i, (x, y, u, v) in enumerate(self.get_machine_moves(points_a, points_b)):
            self.serial_conn.write(f"QUEUE,{x},{y},{u},{v},{smoothed_speeds[i]}\n".encode())
            
            if i > 0:
                dist_a = np.hypot(points_a[i][0] - points_a[i-1][0], points_a[i][1] - points_a[i-1][1])
                dist_b = np.hypot(points_b[i][0] - points_b[i-1][0], points_b[i][1] - points_b[i-1][1])
                max_dist_mm = max(dist_a, dist_b)
                
                time_to_cut = (max_dist_mm * self.STEPS_PER_MM) / max(smoothed_speeds[i], 100)
                time.sleep(time_to_cut)
                QApplication.processEvents() 
            else:
                time.sleep(0.05)
            
        self.set_status("Running")

    def home_machine(self): 
        self.set_status("Homing...")
        self.worker = HomingWorker(self.serial_conn)
        self.worker.finished.connect(lambda: self.set_status("Ready"))
        self.worker.start()

    def stop_machine(self): 
        if hasattr(self, 'worker') and self.worker.isRunning():
            self.set_status("Resetting Board...")
            self.serial_conn.setDTR(False)
            time.sleep(0.1)
            self.serial_conn.setDTR(True)
            self.worker.stop()     
            self.worker.wait()     
            QTimer.singleShot(2000, lambda: self.serial_conn.write(b"SETPOS,0,0,0,0\n"))
            QTimer.singleShot(2000, lambda: self.set_status("Ready (Forced Home)"))
        else:
            self.serial_conn.write(b"ABORT\n")
            self.set_status("Stopped")

    def update_speed(self, val): 
        if self.serial_conn: self.serial_conn.write(f"CFG,CUTMAXSPEED,{val}\n".encode())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HotWireController()
    window.show()
    sys.exit(app.exec())
