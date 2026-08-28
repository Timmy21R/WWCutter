import sys
import time
from pathlib import Path
import numpy as np
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QWidget, QSlider, QLabel, QFileDialog, QHBoxLayout, 
                             QDoubleSpinBox, QSpinBox, QComboBox, QGridLayout,
                             QGroupBox, QScrollArea)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal

from cad_preview import CADPreview
from cad_toolpath import (CADGeometryError, CADImportError, MachineGeometry,
                          auto_detect_section_pair, build_cad_toolpath,
                          load_step_model)

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
    # Hardware calibration from machine coordinates to motor steps.
    STEPS_PER_MM = 1309.6  
    MAX_SEGMENT_MM = 1.0

    def __init__(self):
        super().__init__()
        self.setWindowTitle("WWCutter — Four-Axis Hot-Wire CAM")
        self.serial_conn = None
        self.step_model = None
        self.cad_path = None
        self.cad_valid = False
        self.cad_rebuild_timer = QTimer(self)
        self.cad_rebuild_timer.setSingleShot(True)
        self.cad_rebuild_timer.setInterval(140)
        self.cad_rebuild_timer.timeout.connect(self.rebuild_cad_toolpath)
        self.setup_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(500)
        
        self.auto_connect()

    def setup_ui(self):
        layout = QVBoxLayout()
        header = QHBoxLayout()
        self.status_label = QLabel("Status: Disconnected")
        self.time_label = QLabel("Est. Job Time: 0s")
        header.addWidget(self.status_label, 1)
        header.addWidget(self.time_label)
        layout.addLayout(header)

        motion = QHBoxLayout()
        motion.addWidget(QPushButton("Home", clicked=self.home_machine))
        self.start_button = QPushButton("Start Job", clicked=self.start_job)
        motion.addWidget(self.start_button)
        motion.addWidget(QPushButton("Stop", clicked=self.stop_machine))
        motion.addWidget(QPushButton("Go to Origin", clicked=self.go_to_origin))
        motion.addWidget(QPushButton("Move to Cut Start", clicked=self.go_to_cut_start))
        motion.addWidget(QPushButton("Set Current as Origin", clicked=self.set_current_as_origin))
        layout.addLayout(motion)

        manual_group = QGroupBox("Manual axes / job origin (mm)")
        manual_layout = QHBoxLayout(manual_group)
        for axis_name in ("X", "Y", "U", "V"):
            manual_layout.addWidget(QLabel(
                "Job Origin X (mm):" if axis_name == "X" else "{}:".format(axis_name)
            ))
            box = QDoubleSpinBox()
            box.setRange(-2000.0, 2000.0)
            box.setDecimals(2)
            box.setSuffix(" mm")
            setattr(self, "origin_{}".format(axis_name.lower()), box)
            manual_layout.addWidget(box)
        manual_group.setToolTip(
            "Enter absolute X/Y/U/V machine coordinates, then choose Go to "
            "Origin. These values do not offset the validated STEP toolpath."
        )
        layout.addWidget(manual_group)

        cad_group = QGroupBox("STEP model")
        cad_layout = QGridLayout(cad_group)
        cad_layout.addWidget(QPushButton("Import STEP…", clicked=self.load_step), 0, 0)
        cad_layout.addWidget(QPushButton("Auto-detect End Faces", clicked=self.select_auto_sections), 0, 1)
        self.step_name = QLabel("No STEP model loaded")
        cad_layout.addWidget(self.step_name, 0, 2, 1, 3)

        cad_layout.addWidget(QLabel("Root section:"), 1, 0)
        self.root_section = QComboBox()
        self.root_section.currentIndexChanged.connect(self.rebuild_cad_toolpath)
        cad_layout.addWidget(self.root_section, 1, 1, 1, 4)

        cad_layout.addWidget(QLabel("Tip section:"), 2, 0)
        self.tip_section = QComboBox()
        self.tip_section.currentIndexChanged.connect(self.rebuild_cad_toolpath)
        cad_layout.addWidget(self.tip_section, 2, 1, 1, 4)
        layout.addWidget(cad_group)

        machine_group = QGroupBox("Machine and stock (mm)")
        machine_layout = QGridLayout(machine_group)

        def geometry_box(value, minimum, maximum, on_change=None):
            box = QDoubleSpinBox()
            box.setRange(minimum, maximum)
            box.setDecimals(1)
            box.setValue(value)
            box.valueChanged.connect(on_change or self.schedule_cad_rebuild)
            return box

        self.tower_span = geometry_box(1000.0, 10.0, 10000.0)
        self.foam_left_gap = geometry_box(100.0, 0.0, 10000.0)
        self.horizontal_travel = geometry_box(1000.0, 10.0, 10000.0)
        self.vertical_travel = geometry_box(1000.0, 10.0, 10000.0)
        self.workspace_margin = geometry_box(10.0, 0.0, 500.0)
        self.surface_tolerance = geometry_box(
            0.5, 0.01, 20.0, self.update_cad_validation
        )
        for column, (label, widget) in enumerate((
            ("Tower spacing", self.tower_span),
            ("Left tower → root", self.foam_left_gap),
            ("Horizontal travel", self.horizontal_travel),
            ("Vertical travel", self.vertical_travel),
            ("Safety margin", self.workspace_margin),
            ("Allowed deviation", self.surface_tolerance),
        )):
            machine_layout.addWidget(QLabel(label), 0, column)
            machine_layout.addWidget(widget, 1, column)
        machine_layout.addWidget(QLabel("Interior rods / contours"), 2, 0)
        self.interior_features = QComboBox()
        self.interior_features.addItem("Ignore them", False)
        self.interior_features.addItem("Cut out with entry slit", True)
        self.interior_features.setToolTip(
            "Ignoring measures and cuts only the outer skin. Cutting requires "
            "the contour to pass through both foam end faces."
        )
        self.interior_features.currentIndexChanged.connect(
            self.schedule_cad_rebuild
        )
        machine_layout.addWidget(self.interior_features, 2, 1)

        machine_layout.addWidget(QLabel("Stock rotation"), 2, 2)
        self.stock_rotation = geometry_box(0.0, -180.0, 180.0)
        self.stock_rotation.setSuffix("°")
        self.stock_rotation.setWrapping(True)
        self.stock_rotation.setToolTip(
            "Rotate the STEP model about its root-to-tip span axis."
        )
        machine_layout.addWidget(self.stock_rotation, 2, 3)

        machine_layout.addWidget(QLabel("Stock center X"), 3, 0)
        self.stock_position_x = geometry_box(500.0, -10000.0, 10000.0)
        self.stock_position_x.setSuffix(" mm")
        machine_layout.addWidget(self.stock_position_x, 3, 1)
        machine_layout.addWidget(QLabel("Stock center Y"), 3, 2)
        self.stock_position_y = geometry_box(500.0, -10000.0, 10000.0)
        self.stock_position_y.setSuffix(" mm")
        machine_layout.addWidget(self.stock_position_y, 3, 3)
        stock_position_tip = (
            "Position of the root-section center in the XY/UV workspace. The "
            "allowed range is constrained by both complete tower paths."
        )
        self.stock_position_x.setToolTip(stock_position_tip)
        self.stock_position_y.setToolTip(stock_position_tip)
        layout.addWidget(machine_group)

        preview_header = QHBoxLayout()
        preview_header.addWidget(QLabel(
            "Preview — green: root/XY, blue: tip/UV, yellow: current wire"
        ), 1)
        preview_header.addWidget(QLabel("Focus:"))
        self.preview_focus = QComboBox()
        self.preview_focus.addItem("Complete machine", "machine")
        self.preview_focus.addItem("Resulting cut surface", "cut")
        self.preview_focus.addItem("Imported model", "model")
        self.preview_focus.currentIndexChanged.connect(
            lambda: self.cad_preview.set_focus(self.preview_focus.currentData())
        )
        preview_header.addWidget(self.preview_focus)
        layout.addLayout(preview_header)
        self.cad_preview = CADPreview()
        layout.addWidget(self.cad_preview, 1)

        self.preview_move = QSlider(Qt.Orientation.Horizontal)
        self.preview_move.valueChanged.connect(self.on_preview_move)
        self.preview_move.setEnabled(False)
        layout.addWidget(self.preview_move)
        self.preview_coords = QLabel("Move: -   X: -   Y: -   U: -   V: -")
        layout.addWidget(self.preview_coords)

        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Maximum cruise speed (steps/sec):"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(100, 5000)
        self.speed_slider.setValue(2500)
        self.speed_box = QSpinBox()
        self.speed_box.setRange(100, 5000)
        self.speed_box.setValue(2500)
        self.speed_slider.valueChanged.connect(self.speed_box.setValue)
        self.speed_box.valueChanged.connect(self.speed_slider.setValue)
        self.speed_box.valueChanged.connect(self.update_speed)
        self.speed_box.valueChanged.connect(self.update_time_estimate)
        speed_layout.addWidget(self.speed_slider, 1)
        speed_layout.addWidget(self.speed_box)
        layout.addLayout(speed_layout)
        
        container = QWidget()
        container.setLayout(layout)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(container)
        self.setCentralWidget(scroll_area)
        self.resize(1040, 920)

    def set_status(self, text):
        self.status_label.setText(f"Status: {text}")
        self.status_label.setToolTip(text)

    def machine_geometry(self):
        return MachineGeometry(
            self.tower_span.value(), self.foam_left_gap.value(),
            self.horizontal_travel.value(), self.vertical_travel.value(),
            self.workspace_margin.value(),
        )

    def load_step(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select STEP model", filter="STEP Files (*.step *.stp)"
        )
        if not path:
            return
        self.set_status("Importing STEP model…")
        QApplication.processEvents()
        try:
            self.step_model = load_step_model(path)
        except (CADImportError, CADGeometryError) as exc:
            self.step_model = None
            self.cad_path = None
            self.cad_preview.clear()
            self.set_status("STEP error: {}".format(exc))
            return

        self.step_name.setText(Path(path).name)
        for combo in (self.root_section, self.tip_section):
            combo.blockSignals(True)
            combo.clear()
            for section in self.step_model.sections:
                combo.addItem(section.label, section.index)
            combo.blockSignals(False)
        self.select_auto_sections()

    def select_auto_sections(self):
        if self.step_model is None:
            return
        try:
            root, tip = auto_detect_section_pair(self.step_model)
        except CADGeometryError as exc:
            self.set_status(str(exc))
            self.cad_preview.set_scene(self.step_model)
            return
        self.root_section.blockSignals(True)
        self.tip_section.blockSignals(True)
        self.root_section.setCurrentIndex(self.root_section.findData(root.index))
        self.tip_section.setCurrentIndex(self.tip_section.findData(tip.index))
        self.root_section.blockSignals(False)
        self.tip_section.blockSignals(False)
        self.rebuild_cad_toolpath()

    def _selected_sections(self):
        if self.step_model is None:
            return None, None
        root_index = self.root_section.currentData()
        tip_index = self.tip_section.currentData()
        if root_index is None or tip_index is None:
            return None, None
        return (self.step_model.sections[int(root_index)],
                self.step_model.sections[int(tip_index)])

    def schedule_cad_rebuild(self, *_):
        """Coalesce rapid machine-setting edits into one CAD calculation."""
        if self.step_model is not None:
            self.set_status("Updating STEP preview…")
            self.cad_rebuild_timer.start()

    def update_cad_validation(self, *_):
        """A tolerance edit only rechecks the existing path; it does not rebuild it."""
        if self.cad_path is None:
            return
        tolerance = self.surface_tolerance.value()
        within_tolerance = self.cad_path.surface_error <= tolerance
        self.cad_valid = within_tolerance and not self.cad_path.limitations
        deviation_label = (
            "outer-only deviation" if self.cad_path.ignored_interior_count
            else "max deviation"
        )
        if self.cad_valid:
            feature_text = (
                " including {} interior contour{}".format(
                    self.cad_path.interior_count,
                    "" if self.cad_path.interior_count == 1 else "s",
                ) if self.cad_path.interior_count else ""
            )
            self.set_status(
                "STEP ready — {} rulings{}, {} {:.3f} mm".format(
                    len(self.cad_path.tower_left), feature_text,
                    deviation_label, self.cad_path.surface_error,
                )
            )
        else:
            reasons = []
            if not within_tolerance:
                reasons.append(self.cad_path.deviation_detail)
            reasons.extend(self.cad_path.limitations)
            if not within_tolerance:
                self.set_status(
                    "Cannot cut within tolerance — {} {:.3f} mm exceeds {:.3f} mm".format(
                        deviation_label, self.cad_path.surface_error, tolerance
                    )
                )
            else:
                self.set_status("Cannot cut the selected interior contours")
            self.status_label.setToolTip("\n\n".join(reasons))

    def rebuild_cad_toolpath(self, *_):
        self.cad_rebuild_timer.stop()
        if self.step_model is None:
            return
        root, tip = self._selected_sections()
        if root is None or tip is None:
            return
        try:
            self.cad_path = build_cad_toolpath(
                self.step_model, root.index, tip.index,
                self.machine_geometry(), self.MAX_SEGMENT_MM,
                include_internal=bool(self.interior_features.currentData()),
                rotation_degrees=self.stock_rotation.value(),
                stock_position=(self.stock_position_x.value(),
                                self.stock_position_y.value()),
                clamp_stock_position=True,
            )
        except CADGeometryError as exc:
            self.cad_path = None
            self.cad_valid = False
            self.preview_move.setEnabled(False)
            self.cad_preview.set_scene(self.step_model, root, tip)
            self.set_status("Geometry error: {}".format(exc))
            return

        self._update_stock_position_controls()
        self.preview_move.setEnabled(True)
        self.preview_move.setMaximum(max(0, len(self.cad_path.tower_left) - 1))
        self.cad_preview.set_scene(
            self.step_model, root, tip, self.cad_path, self.machine_geometry()
        )
        self.on_preview_move(self.preview_move.value())
        self.update_cad_validation()
        self.update_time_estimate()

    def _update_stock_position_controls(self):
        minimum = self.cad_path.stock_position_min
        maximum = self.cad_path.stock_position_max
        actual = self.cad_path.stock_position
        for axis_name, box, low, high, value in zip(
                ("X", "Y"),
                (self.stock_position_x, self.stock_position_y),
                minimum, maximum, actual):
            box.blockSignals(True)
            box.setRange(float(low), float(high))
            box.setValue(float(value))
            box.setToolTip(
                "Allowed {} stock-center range for the current orientation and "
                "tower paths: {:.1f} to {:.1f} mm.".format(
                    axis_name, low, high
                )
            )
            box.blockSignals(False)

    def on_preview_move(self, value):
        if self.step_model is not None and self.cad_path is not None:
            self.cad_preview.set_move_index(value)
            index = min(int(value), len(self.cad_path.tower_left) - 1)
            x, y = self.cad_path.tower_left[index]
            u, v = self.cad_path.tower_right[index]
            self.preview_coords.setText(
                "Wire {}/{}   X: {:.2f}   Y: {:.2f}   U: {:.2f}   V: {:.2f} mm".format(
                    index + 1, len(self.cad_path.tower_left), x, y, u, v
                )
            )

    def calculate_estimate(self, speeds, moves):
        total_time = 0.0
        for i in range(1, len(moves)):
            # MultiStepper times a segment from the largest individual axis
            # displacement, not the Euclidean length of either 2-D endpoint.
            step_distance = max(abs(moves[i][axis] - moves[i - 1][axis])
                                for axis in range(4))
            total_time += step_distance / max(speeds[i], 100)
            
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
        if self.cad_path is None:
            return
        points_a, points_b = self.get_job_toolpaths()
        if not points_a or not points_b:
            return
        smoothed_speeds = self.calculate_job_speeds(points_a, points_b)
        self.calculate_estimate(smoothed_speeds, self.get_machine_moves())

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

    def get_job_toolpaths(self):
        if self.cad_path is None:
            return [], []
        return ([tuple(point) for point in self.cad_path.root_xy],
                [tuple(point) for point in self.cad_path.tip_xy])

    def get_machine_moves(self):
        if self.cad_path is None:
            return []
        return [
            tuple(int(value * self.STEPS_PER_MM) for value in (*left, *right))
            for left, right in zip(
                self.cad_path.tower_left, self.cad_path.tower_right
            )
        ]

    def set_current_as_origin(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        
        self.serial_conn.write(b"SETPOS,0,0,0,0\n")
        for box in (self.origin_x, self.origin_y, self.origin_u, self.origin_v):
            box.setValue(0.0)
        self.set_status("Origin set to current position")

    def go_to_origin(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        self.set_status("Moving to Origin...")
        target_steps = [
            int(box.value() * self.STEPS_PER_MM)
            for box in (self.origin_x, self.origin_y, self.origin_u, self.origin_v)
        ]
        self.serial_conn.write(
            "MOVE,{},{},{},{}\n".format(*target_steps).encode()
        )

    def go_to_cut_start(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            self.set_status("Cannot position: machine is disconnected")
            return
        points_a, points_b = self.get_job_toolpaths()
        if not points_a or not points_b:
            self.set_status("Load a valid job before positioning")
            return
        x, y, u, v = self.get_machine_moves()[0]
        self.serial_conn.write(f"MOVE,{x},{y},{u},{v}\n".encode())
        self.set_status("Moving to cut start (keep the wire heater off)")

    def start_job(self):
        has_geometry = self.step_model is not None and self.cad_path is not None
        if not (has_geometry and self.serial_conn):
            return
        if not self.cad_valid:
            self.set_status("Cannot start: STEP deviation or interior selection is invalid")
            return
        
        points_a, points_b = self.get_job_toolpaths()
        
        if not points_a or not points_b:
            self.set_status("Error: Empty or invalid STEP toolpath")
            return
            
        self.set_status("Calculating Toolpath Dynamics...")

        smoothed_speeds = self.calculate_job_speeds(points_a, points_b)
        moves = self.get_machine_moves()
        self.calculate_estimate(smoothed_speeds, moves)
        
        self.serial_conn.write(f"UPLOAD,{len(points_a)}\n".encode())
        self.set_status("Uploading...")
        
        for i, (x, y, u, v) in enumerate(moves):
            self.serial_conn.write(f"QUEUE,{x},{y},{u},{v},{smoothed_speeds[i]}\n".encode())
            
            if i > 0:
                step_distance = max(abs(moves[i][axis] - moves[i - 1][axis])
                                    for axis in range(4))
                time_to_cut = step_distance / max(smoothed_speeds[i], 100)
                time.sleep(time_to_cut)
                QApplication.processEvents() 
            else:
                time.sleep(0.05)
            
        self.set_status("Running")

    def home_machine(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            self.set_status("Cannot home: machine is disconnected")
            return
        self.set_status("Homing...")
        self.worker = HomingWorker(self.serial_conn)
        self.worker.finished.connect(lambda: self.set_status("Ready"))
        self.worker.start()

    def stop_machine(self): 
        if not self.serial_conn or not self.serial_conn.is_open:
            self.set_status("Machine is disconnected")
            return
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
