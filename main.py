import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                           QTabWidget, QPushButton, QLabel, QLineEdit, QGridLayout,
                           QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog,
                           QMessageBox, QGroupBox, QSplitter, QComboBox, QSizePolicy)
from PyQt6.QtGui import QColor, QVector3D, QQuaternion, QFont
from PyQt6.QtCore import Qt, pyqtSignal, QEvent

import pyqtgraph as pg
import pyqtgraph.opengl as gl

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

class PathPoint:
    def __init__(self, x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0, name=""):
        self.x = x
        self.y = y
        self.z = z
        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.name = name if name else f"Point {id(self) % 1000}"

class Obstacle:
    def __init__(self, name="", x_min=0, y_min=0, z_min=0, x_max=0, y_max=0, z_max=0, color=None):
        self.name = name
        self.x_min = x_min
        self.y_min = y_min
        self.z_min = z_min
        self.x_max = x_max
        self.y_max = y_max
        self.z_max = z_max
        self.color = color if color else QColor(0, 0, 255, 128)  # Default blue semi-transparent

class PathVisualizationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.obstacles = []
        self.current_point_index = -1
        self.init_ui()
        
    def init_ui(self):
        # Initialize graphics items for paths and points (must be before any method that uses them)
        self.path_lines = None
        self.point_markers = []
        self.obstacle_boxes = []
        self.grid = None
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        self.x_label = None
        self.y_label = None
        self.z_label = None
        
        # Main layout
        main_layout = QHBoxLayout(self)
        
        # Controls panel on the left
        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)
        
        # Set a wider minimum width for the controls panel
        controls_panel.setMinimumWidth(420)
        controls_panel.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        
        # Set a smaller font for tables
        table_font = QFont()
        table_font.setPointSize(12)
        
        # Points table
        self.points_table = QTableWidget(0, 8)
        self.points_table.setFont(table_font)
        self.points_table.setHorizontalHeaderLabels(["Name", "X", "Y", "Z", "qW", "qX", "qY", "qZ"])
        self.points_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self.points_table.horizontalHeader().setStretchLastSection(True)
        self.points_table.cellChanged.connect(self.on_table_cell_changed)
        self.points_table.verticalHeader().setDefaultSectionSize(12)  # Reduce row height
        self.points_table.horizontalHeader().setFont(table_font)
        controls_layout.addWidget(QLabel("Path Points:"))
        controls_layout.addWidget(self.points_table)
        # Install event filter for paste handling
        self.points_table.installEventFilter(self)
        
        # Add point button
        add_point_layout = QHBoxLayout()
        self.add_point_btn = QPushButton("Add Point")
        self.add_point_btn.clicked.connect(self.add_new_point)
        self.remove_point_btn = QPushButton("Remove Point")
        self.remove_point_btn.clicked.connect(self.remove_selected_point)
        add_point_layout.addWidget(self.add_point_btn)
        add_point_layout.addWidget(self.remove_point_btn)
        controls_layout.addLayout(add_point_layout)
        
        # File operations
        file_ops_layout = QHBoxLayout()
        self.save_btn = QPushButton("Save Path")
        self.save_btn.clicked.connect(self.save_path)
        self.load_btn = QPushButton("Load Path")
        self.load_btn.clicked.connect(self.load_path)
        file_ops_layout.addWidget(self.save_btn)
        file_ops_layout.addWidget(self.load_btn)
        controls_layout.addLayout(file_ops_layout)
        
        # Obstacle controls
        obstacle_group = QGroupBox("Obstacles")
        obstacle_layout = QVBoxLayout(obstacle_group)
        
        # Obstacle table
        self.obstacle_table = QTableWidget(0, 8)
        self.obstacle_table.setFont(table_font)
        self.obstacle_table.setHorizontalHeaderLabels(["Name", "X_min", "Y_min", "Z_min", "X_max", "Y_max", "Z_max", "Color"])
        self.obstacle_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self.obstacle_table.horizontalHeader().setStretchLastSection(True)
        self.obstacle_table.cellChanged.connect(self.on_obstacle_cell_changed)
        self.obstacle_table.verticalHeader().setDefaultSectionSize(18)  # Reduce row height
        self.obstacle_table.horizontalHeader().setFont(table_font)
        obstacle_layout.addWidget(self.obstacle_table)
        
        # Add obstacle button
        obstacle_btn_layout = QHBoxLayout()
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.clicked.connect(self.add_new_obstacle)
        self.remove_obstacle_btn = QPushButton("Remove Obstacle")
        self.remove_obstacle_btn.clicked.connect(self.remove_selected_obstacle)
        obstacle_btn_layout.addWidget(self.add_obstacle_btn)
        obstacle_btn_layout.addWidget(self.remove_obstacle_btn)
        obstacle_layout.addLayout(obstacle_btn_layout)
        
        controls_layout.addWidget(obstacle_group)
        
        # 3D Visualization area on the right
        viz_panel = QWidget()
        viz_layout = QVBoxLayout(viz_panel)
        # Add explanation label
        self.explanation_label = QLabel("Red: X (forward), Green: Y (right), Blue: Z (up). Arrows show orientation.")
        self.explanation_label.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.explanation_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        viz_layout.addWidget(self.explanation_label)
        # Add the view widget
        self.view_widget = gl.GLViewWidget()
        viz_layout.addWidget(self.view_widget, stretch=1)
        # Add intersection status label
        self.intersection_label = QLabel("")
        self.intersection_label.setStyleSheet("color: red; font-weight: bold;")
        viz_layout.addWidget(self.intersection_label)
        
        # Add to main layout
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(controls_panel)
        splitter.addWidget(viz_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([320, 2000])  # Match the new width
        main_layout.addWidget(splitter)
        self.load_sample_obstacles()
        
    def load_sample_obstacles(self):
        # Load the sample obstacles from the provided data
        oasis_obstacles = [
            Obstacle("Area 1", 10.425, -10.2, 4.445, 11.425, -9.5, 4.945, QColor(0, 100, 255, 150)),
            Obstacle("Area 2", 10.925, -9.5, 4.945, 11.425, -8.45, 5.445, QColor(0, 100, 255, 150)),
            Obstacle("Area 3", 10.425, -8.45, 4.945, 10.975, -7.4, 5.445, QColor(0, 100, 255, 150)),
            Obstacle("Area 4", 10.925, -7.4, 4.425, 11.425, -6.35, 4.945, QColor(0, 100, 255, 150))
        ]
        
        kiz_obstacles = [
            Obstacle("KIZ 1", 10.3, -10.2, 4.32, 11.55, -6.0, 5.57, QColor(0, 200, 0, 120)),
            Obstacle("KIZ 2", 9.5, -10.5, 4.02, 10.5, -9.6, 4.8, QColor(0, 200, 0, 120))
        ]
        
        # Add the new red plane obstacles from the table
        area_planes = [
            # name, x_min, y_min, z_min, x_max, y_max, z_max
            ("Area Plane 1", 10.42, -10.58, 4.82, 11.48, -10.58, 5.57),
            ("Area Plane 2", 10.3, -9.25, 3.76203, 11.55, -8.5, 3.76203),
            ("Area Plane 3", 10.3, -8.4, 3.76093, 11.55, -7.45, 3.76093),
            ("Area Plane 4", 9.866984, -7.34, 4.32, 9.866984, -6.365, 5.57),
        ]
        for name, x_min, y_min, z_min, x_max, y_max, z_max in area_planes:
            self.obstacles.append(Obstacle(name, x_min, y_min, z_min, x_max, y_max, z_max, QColor(255, 0, 0, 180)))
            self.draw_filled_plane(x_min, y_min, z_min, x_max, y_max, z_max, QColor(255, 0, 0, 120))
        
        # Store astronaut position and radius as a special marker (not an obstacle)
        self.astronaut_pos = (11.143, -6.7607, 4.9654)
        self.astronaut_radius = 0.1
        
        for obs in oasis_obstacles + kiz_obstacles:
            self.obstacles.append(obs)
        
        # Update the obstacle table
        self.update_obstacle_table()
        # Visualize the obstacles
        self.update_visualization()
    
    def add_new_point(self):
        # Default new point coordinates
        new_point = PathPoint()
        self.points.append(new_point)
        self.current_point_index = len(self.points) - 1
        self.update_points_table()
        self.update_visualization()
    
    def remove_selected_point(self):
        selected_rows = self.points_table.selectedIndexes()
        if not selected_rows:
            return
            
        row = selected_rows[0].row()
        if 0 <= row < len(self.points):
            self.points.pop(row)
            if self.current_point_index >= row:
                self.current_point_index = max(-1, self.current_point_index - 1)
            self.update_points_table()
            self.update_visualization()
    
    def add_new_obstacle(self):
        new_obstacle = Obstacle(f"Obstacle {len(self.obstacles) + 1}")
        self.obstacles.append(new_obstacle)
        self.update_obstacle_table()
        self.update_visualization()
    
    def remove_selected_obstacle(self):
        selected_rows = self.obstacle_table.selectedIndexes()
        if not selected_rows:
            return
            
        row = selected_rows[0].row()
        if 0 <= row < len(self.obstacles):
            self.obstacles.pop(row)
            self.update_obstacle_table()
            self.update_visualization()
    
    def on_table_cell_changed(self, row, column):
        if row >= len(self.points) or self.points_table.item(row, column) is None:
            return
            
        try:
            value = self.points_table.item(row, column).text()
            point = self.points[row]
            
            if column == 0:  # Name
                point.name = value
            elif column == 1:  # X
                point.x = float(value)
            elif column == 2:  # Y
                point.y = float(value)
            elif column == 3:  # Z
                point.z = float(value)
            elif column == 4:  # qW
                point.qw = float(value)
            elif column == 5:  # qX
                point.qx = float(value)
            elif column == 6:  # qY
                point.qy = float(value)
            elif column == 7:  # qZ
                point.qz = float(value)
                
            self.update_visualization()
        except ValueError:
            pass  # Ignore invalid input
    
    def on_obstacle_cell_changed(self, row, column):
        if row >= len(self.obstacles) or self.obstacle_table.item(row, column) is None:
            return
            
        try:
            value = self.obstacle_table.item(row, column).text()
            obstacle = self.obstacles[row]
            
            if column == 0:  # Name
                obstacle.name = value
            elif column == 1:  # X_min
                obstacle.x_min = float(value)
            elif column == 2:  # Y_min
                obstacle.y_min = float(value)
            elif column == 3:  # Z_min
                obstacle.z_min = float(value)
            elif column == 4:  # X_max
                obstacle.x_max = float(value)
            elif column == 5:  # Y_max
                obstacle.y_max = float(value)
            elif column == 6:  # Z_max
                obstacle.z_max = float(value)
            elif column == 7:  # Color
                # Placeholder for color picking
                pass
                
            self.update_visualization()
        except ValueError:
            pass  # Ignore invalid input
    
    def update_points_table(self):
        self.points_table.blockSignals(True)
        self.points_table.setRowCount(len(self.points))
        
        for i, point in enumerate(self.points):
            # Set data for each column
            self.points_table.setItem(i, 0, QTableWidgetItem(point.name))
            self.points_table.setItem(i, 1, QTableWidgetItem(str(point.x)))
            self.points_table.setItem(i, 2, QTableWidgetItem(str(point.y)))
            self.points_table.setItem(i, 3, QTableWidgetItem(str(point.z)))
            self.points_table.setItem(i, 4, QTableWidgetItem(str(point.qw)))
            self.points_table.setItem(i, 5, QTableWidgetItem(str(point.qx)))
            self.points_table.setItem(i, 6, QTableWidgetItem(str(point.qy)))
            self.points_table.setItem(i, 7, QTableWidgetItem(str(point.qz)))
            
        self.points_table.blockSignals(False)
    
    def update_obstacle_table(self):
        self.obstacle_table.blockSignals(True)
        self.obstacle_table.setRowCount(len(self.obstacles))
        
        for i, obstacle in enumerate(self.obstacles):
            # Set data for each column
            self.obstacle_table.setItem(i, 0, QTableWidgetItem(obstacle.name))
            self.obstacle_table.setItem(i, 1, QTableWidgetItem(str(obstacle.x_min)))
            self.obstacle_table.setItem(i, 2, QTableWidgetItem(str(obstacle.y_min)))
            self.obstacle_table.setItem(i, 3, QTableWidgetItem(str(obstacle.z_min)))
            self.obstacle_table.setItem(i, 4, QTableWidgetItem(str(obstacle.x_max)))
            self.obstacle_table.setItem(i, 5, QTableWidgetItem(str(obstacle.y_max)))
            self.obstacle_table.setItem(i, 6, QTableWidgetItem(str(obstacle.z_max)))
            
            # Color column - just show the color name
            color_item = QTableWidgetItem("Edit")
            color_item.setBackground(obstacle.color)
            self.obstacle_table.setItem(i, 7, color_item)
            
        self.obstacle_table.blockSignals(False)
    
    def draw_wireframe_box(self, x_min, y_min, z_min, x_max, y_max, z_max, color):
        # 8 corners
        corners = np.array([
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max],
        ])
        # 12 edges as pairs of indices
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
            (4, 5), (5, 6), (6, 7), (7, 4),  # top
            (0, 4), (1, 5), (2, 6), (3, 7)   # sides
        ]
        for i1, i2 in edges:
            line = gl.GLLinePlotItem(pos=np.array([corners[i1], corners[i2]]), color=color.getRgbF(), width=2)
            self.view_widget.addItem(line)
            self.obstacle_boxes.append(line)

    def get_obstacles_grid_origin(self):
        if not self.obstacles:
            return QVector3D(0, 0, 0)
        x_min = min(o.x_min for o in self.obstacles)
        y_min = min(o.y_min for o in self.obstacles)
        z_min = min(o.z_min for o in self.obstacles)
        x_max = max(o.x_max for o in self.obstacles)
        y_max = max(o.y_max for o in self.obstacles)
        # Center in x/y, a bit below the lowest z_min
        z_grid = z_min - 0.2  # 0.2 units below
        return QVector3D((x_min + x_max) / 2, (y_min + y_max) / 2, z_grid)

    def update_axes_and_grid(self):
        # Place grid/axes at the center of the obstacles in x/y, and just below in z
        origin = self.get_obstacles_grid_origin()
        # Remove old grid/axes/labels if they exist
        for item in [self.grid, self.x_axis, self.y_axis, self.z_axis, self.x_label, self.y_label, self.z_label]:
            if item is not None:
                self.view_widget.removeItem(item)
        # Add coordinate grid at origin.z
        self.grid = gl.GLGridItem()
        self.grid.setSize(20, 20, 1)
        self.grid.setSpacing(1, 1, 1)
        self.grid.translate(origin.x(), origin.y(), origin.z())
        self.view_widget.addItem(self.grid)
        # Add coordinate axes at origin, with opacity 0.5
        axis_size = 5
        axis_alpha = 0.5
        self.x_axis = gl.GLLinePlotItem(pos=np.array([[origin.x(), origin.y(), origin.z()], [origin.x() + axis_size, origin.y(), origin.z()]]), color=(1, 0, 0, axis_alpha), width=2)
        self.y_axis = gl.GLLinePlotItem(pos=np.array([[origin.x(), origin.y(), origin.z()], [origin.x(), origin.y() + axis_size, origin.z()]]), color=(0, 1, 0, axis_alpha), width=2)
        self.z_axis = gl.GLLinePlotItem(pos=np.array([[origin.x(), origin.y(), origin.z()], [origin.x(), origin.y(), origin.z() + axis_size]]), color=(0, 0, 1, axis_alpha), width=2)
        self.view_widget.addItem(self.x_axis)
        self.view_widget.addItem(self.y_axis)
        self.view_widget.addItem(self.z_axis)
        # Add axis labels at the ends
        self.x_label = gl.GLTextItem(pos=np.array([origin.x() + axis_size, origin.y(), origin.z()]), text="X", color=(1, 0, 0, 1))
        self.y_label = gl.GLTextItem(pos=np.array([origin.x(), origin.y() + axis_size, origin.z()]), text="Y", color=(0, 1, 0, 1))
        self.z_label = gl.GLTextItem(pos=np.array([origin.x(), origin.y(), origin.z() + axis_size]), text="Z", color=(0, 0, 1, 1))
        self.view_widget.addItem(self.x_label)
        self.view_widget.addItem(self.y_label)
        self.view_widget.addItem(self.z_label)
        # Camera: look from above, center on new origin
        self.view_widget.setCameraPosition(pos=origin + QVector3D(0, 0, 20), elevation=90, azimuth=0, distance=20)
        self.view_widget.opts['center'] = origin

    def update_visualization(self):
        self.update_axes_and_grid()
        # Clear previous visualizations
        if self.path_lines is not None:
            self.view_widget.removeItem(self.path_lines)
        for marker in self.point_markers:
            self.view_widget.removeItem(marker)
        self.point_markers = []
        for box in self.obstacle_boxes:
            self.view_widget.removeItem(box)
        self.obstacle_boxes = []
        # Draw obstacles as wireframes
        for obstacle in self.obstacles:
            self.draw_wireframe_box(
                obstacle.x_min, obstacle.y_min, obstacle.z_min,
                obstacle.x_max, obstacle.y_max, obstacle.z_max,
                obstacle.color
            )
        # Draw astronaut as a filled yellow circle
        if hasattr(self, 'astronaut_pos'):
            x, y, z = self.astronaut_pos
            self.draw_filled_circle(x, y, z, self.astronaut_radius, QColor(255, 255, 255, 200))
        
        # Draw path points and lines
        intersected_areas = set()
        intersecting_segments = set()
        if len(self.points) > 0:
            # Draw points
            for i, point in enumerate(self.points):
                # Create a sphere for each point
                point_size = 0.15
                point_mesh = gl.GLMeshItem(
                    meshdata=gl.MeshData.sphere(rows=10, cols=10, radius=point_size),
                    smooth=True,
                    color=(1, 0.5, 0, 1) if i == self.current_point_index else (1, 1, 0, 1),
                    shader='shaded',
                    glOptions='opaque'
                )
                point_mesh.translate(point.x, point.y, point.z)
                self.view_widget.addItem(point_mesh)
                self.point_markers.append(point_mesh)
                
                # Add a text label for the point name
                text = gl.GLTextItem(pos=np.array([point.x, point.y, point.z + point_size]), text=point.name, color=(1, 1, 1, 1))
                self.view_widget.addItem(text)
                self.point_markers.append(text)
                
                # Draw quaternion orientation arrows if quaternion is valid
                quat_magnitude = np.sqrt(point.qw**2 + point.qx**2 + point.qy**2 + point.qz**2)
                if abs(quat_magnitude - 1.0) < 0.1:  # Check if quaternion is roughly normalized
                    # Convert quaternion to rotation matrix
                    quat = QQuaternion(point.qw, point.qx, point.qy, point.qz)
                    
                    # Create orientation arrows (forward, right, up)
                    arrow_length = 0.3
                    
                    # Forward arrow (X axis) - Red
                    forward = quat.rotatedVector(QVector3D(arrow_length, 0, 0))
                    x_arrow = gl.GLLinePlotItem(
                        pos=np.array([[point.x, point.y, point.z], 
                                     [point.x + forward.x(), point.y + forward.y(), point.z + forward.z()]]),
                        color=(1, 0, 0, 1),
                        width=2
                    )
                    self.view_widget.addItem(x_arrow)
                    self.point_markers.append(x_arrow)
            
            # Draw path lines connecting points
            if len(self.points) > 1:
                path_points = np.array([[p.x, p.y, p.z] for p in self.points])
                # Check for intersection with blue rectangles
                for i in range(len(self.points) - 1):
                    p1 = self.points[i]
                    p2 = self.points[i+1]
                    for j, obstacle in enumerate(self.obstacles):
                        # Only check blue obstacles (default blue or blueish)
                        c = obstacle.color
                        if (c.red() == 0 and c.green() == 100 and c.blue() == 255) or (c.red() == 0 and c.green() == 0 and c.blue() == 255):
                            if self.segment_aabb_intersect(p1, p2, obstacle):
                                intersected_areas.add(obstacle.name)
                                intersecting_segments.add(i)
                # Draw all segments, highlight intersecting ones in red
                for i in range(len(self.points) - 1):
                    seg_points = np.array([[self.points[i].x, self.points[i].y, self.points[i].z],
                                           [self.points[i+1].x, self.points[i+1].y, self.points[i+1].z]])
                    color = (1, 1, 1, 1) if i in intersecting_segments else (0, 1, 1, 1)
                    seg_line = gl.GLLinePlotItem(pos=seg_points, color=color, width=3, mode='lines')
                    self.view_widget.addItem(seg_line)
                    self.point_markers.append(seg_line)
        # Show intersection status
        if intersected_areas:
            self.intersection_label.setText(f"Path intersects: {', '.join(sorted(intersected_areas))}")
        else:
            self.intersection_label.setText("")
    
    def segment_aabb_intersect(self, p1, p2, box):
        # 3D segment-AABB intersection using the slab method
        # p1, p2: PathPoint; box: Obstacle
        from numpy import inf
        x1, y1, z1 = p1.x, p1.y, p1.z
        x2, y2, z2 = p2.x, p2.y, p2.z
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        tmin = 0.0
        tmax = 1.0
        for axis, bmin, bmax, d, p in zip(
            ['x', 'y', 'z'],
            [box.x_min, box.y_min, box.z_min],
            [box.x_max, box.y_max, box.z_max],
            [dx, dy, dz],
            [x1, y1, z1]
        ):
            if abs(d) < 1e-8:
                if p < bmin or p > bmax:
                    return False
            else:
                t1 = (bmin - p) / d
                t2 = (bmax - p) / d
                t1, t2 = min(t1, t2), max(t1, t2)
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
                if tmin > tmax:
                    return False
        return True
    
    def save_path(self):
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Path", "", "CSV Files (*.csv);;All Files (*)")
        if not file_path:
            return
            
        try:
            with open(file_path, 'w') as f:
                # Write header
                f.write("Name,X,Y,Z,qW,qX,qY,qZ\n")
                
                # Write points
                for point in self.points:
                    f.write(f"{point.name},{point.x},{point.y},{point.z},{point.qw},{point.qx},{point.qy},{point.qz}\n")
                    
            QMessageBox.information(self, "Success", "Path saved successfully!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save path: {str(e)}")
    
    def load_path(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Load Path", "", "CSV Files (*.csv);;All Files (*)")
        if not file_path:
            return
            
        try:
            self.points = []
            with open(file_path, 'r') as f:
                lines = f.readlines()
                
                # Skip header
                for i in range(1, len(lines)):
                    line = lines[i].strip()
                    if not line:
                        continue
                        
                    parts = line.split(',')
                    if len(parts) >= 8:
                        name = parts[0]
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        qw = float(parts[4])
                        qx = float(parts[5])
                        qy = float(parts[6])
                        qz = float(parts[7])
                        
                        self.points.append(PathPoint(x, y, z, qw, qx, qy, qz, name))
                        
            self.current_point_index = len(self.points) - 1 if self.points else -1
            self.update_points_table()
            self.update_visualization()
            QMessageBox.information(self, "Success", "Path loaded successfully!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load path: {str(e)}")

    def draw_filled_plane(self, x_min, y_min, z_min, x_max, y_max, z_max, color):
        # If z_min == z_max, it's a flat plane in XY
        if abs(z_min - z_max) < 1e-6:
            z = z_min
            corners = np.array([
                [x_min, y_min, z],
                [x_max, y_min, z],
                [x_max, y_max, z],
                [x_min, y_max, z],
            ])
        # If x_min == x_max, it's a YZ plane
        elif abs(x_min - x_max) < 1e-6:
            x = x_min
            corners = np.array([
                [x, y_min, z_min],
                [x, y_max, z_min],
                [x, y_max, z_max],
                [x, y_min, z_max],
            ])
        # If y_min == y_max, it's an XZ plane
        elif abs(y_min - y_max) < 1e-6:
            y = y_min
            corners = np.array([
                [x_min, y, z_min],
                [x_max, y, z_min],
                [x_max, y, z_max],
                [x_min, y, z_max],
            ])
        else:
            # General quad
            corners = np.array([
                [x_min, y_min, z_min],
                [x_max, y_min, z_max],
                [x_max, y_max, z_max],
                [x_min, y_max, z_min],
            ])
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        mesh = gl.GLMeshItem(vertexes=corners, faces=faces, color=color.getRgbF(), smooth=False, shader='shaded', glOptions='translucent')
        self.view_widget.addItem(mesh)
        self.obstacle_boxes.append(mesh)

    def draw_filled_circle(self, x, y, z, radius, color):
        # Create a sphere mesh using PyQtGraph's built-in function
        mesh = gl.GLMeshItem(
            meshdata=gl.MeshData.sphere(rows=10, cols=20, radius=radius),
            smooth=True,
            color=color.getRgbF(),
            shader='shaded',
            glOptions='translucent'
        )
        mesh.translate(x, y, z)
        self.view_widget.addItem(mesh)
        self.obstacle_boxes.append(mesh)

    def eventFilter(self, obj, event):
        # Handle paste into the points table for quaternion auto-fill
        if obj == self.points_table and event.type() == QEvent.Type.KeyPress:
            # Check for Ctrl+V (Windows/Linux) or Cmd+V (Mac)
            key = event.key()
            ctrl = event.modifiers() & Qt.KeyboardModifier.ControlModifier
            meta = event.modifiers() & Qt.KeyboardModifier.MetaModifier
            if (ctrl or meta) and key == Qt.Key.Key_V:
                selected = self.points_table.selectedIndexes()
                if selected:
                    row = selected[0].row()
                    col = selected[0].column()
                    # Only handle if in qW, qX, qY, qZ columns
                    if col in [4, 5, 6, 7]:
                        clipboard = QApplication.instance().clipboard()
                        text = clipboard.text()
                        if ',' in text:
                            parts = [p.strip() for p in text.split(',')]
                            if len(parts) == 4:
                                for i, val in enumerate(parts):
                                    if col + i <= 7:
                                        self.points_table.setItem(row, col + i, QTableWidgetItem(val))
                                return True  # Handled
        return super().eventFilter(obj, event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Path Visualization Tool")
        self.resize(1280, 800)
        
        # Create tab widget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        
        # Create path visualization tab
        self.path_tab = PathVisualizationWidget()
        self.tabs.addTab(self.path_tab, "Path Planning")
        
        # Create quaternion visualization tab
        self.quat_tab = QuaternionVisualizerWidget()
        self.tabs.addTab(self.quat_tab, "Quaternion Visualizer")
        
        # Set up the status bar
        self.statusBar().showMessage("Ready")


class QuaternionVisualizerWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        
        # Initialize graphics items for visualization
        self.orientation_arrows = []
        self.orientation_model = None
        
        # Initialize UI
        self.init_ui()
        
        # Update visualization after UI is fully initialized
        self.update_rpy()  # This will call update_visualization_with_quaternion internally
        
    def init_ui(self):
        # Initialize graphics items for paths and points (must be before any method that uses them)
        self.path_lines = None
        self.point_markers = []
        self.obstacle_boxes = []
        self.grid = None
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        self.x_label = None
        self.y_label = None
        self.z_label = None
        
        # Main layout
        main_layout = QHBoxLayout(self)
        
        # Controls panel on the left
        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)
        
        # Roll/Pitch/Yaw input fields
        rpy_group = QGroupBox("Orientation Input (degrees)")
        rpy_layout = QGridLayout(rpy_group)
        
        rpy_layout.addWidget(QLabel("Roll (X):"), 0, 0)
        self.roll_edit = QLineEdit(str(self.roll_deg))
        self.roll_edit.textChanged.connect(self.update_rpy)
        rpy_layout.addWidget(self.roll_edit, 0, 1)
        
        rpy_layout.addWidget(QLabel("Pitch (Y):"), 1, 0)
        self.pitch_edit = QLineEdit(str(self.pitch_deg))
        self.pitch_edit.textChanged.connect(self.update_rpy)
        rpy_layout.addWidget(self.pitch_edit, 1, 1)
        
        rpy_layout.addWidget(QLabel("Yaw (Z):"), 2, 0)
        self.yaw_edit = QLineEdit(str(self.yaw_deg))
        self.yaw_edit.textChanged.connect(self.update_rpy)
        rpy_layout.addWidget(self.yaw_edit, 2, 1)
        
        controls_layout.addWidget(rpy_group)
        
        # Add quaternion display
        quat_group = QGroupBox("Equivalent Quaternion")
        quat_layout = QGridLayout(quat_group)
        
        quat_layout.addWidget(QLabel("qW:"), 0, 0)
        self.qw_label = QLineEdit("1.000")
        self.qw_label.textChanged.connect(self.handle_quaternion_paste)
        quat_layout.addWidget(self.qw_label, 0, 1)
        
        quat_layout.addWidget(QLabel("qX:"), 1, 0)
        self.qx_label = QLineEdit("0.000")
        quat_layout.addWidget(self.qx_label, 1, 1)
        
        quat_layout.addWidget(QLabel("qY:"), 2, 0)
        self.qy_label = QLineEdit("0.000")
        quat_layout.addWidget(self.qy_label, 2, 1)
        
        quat_layout.addWidget(QLabel("qZ:"), 3, 0)
        self.qz_label = QLineEdit("0.000")
        quat_layout.addWidget(self.qz_label, 3, 1)
        
        # Add Copy button
        self.copy_quat_btn = QPushButton("Copy")
        self.copy_quat_btn.clicked.connect(self.copy_quaternion_to_clipboard)
        quat_layout.addWidget(self.copy_quat_btn, 4, 0, 1, 2)
        
        controls_layout.addWidget(quat_group)
        
        # Add preset orientations
        preset_group = QGroupBox("Preset Orientations")
        preset_layout = QVBoxLayout(preset_group)
        
        self.preset_combo = QComboBox()
        self.preset_combo.addItem("Identity (No Rotation)")
        self.preset_combo.addItem("90° Roll (X)")
        self.preset_combo.addItem("90° Pitch (Y)")
        self.preset_combo.addItem("90° Yaw (Z)")
        self.preset_combo.addItem("180° Roll (X)")
        self.preset_combo.addItem("45° All Axes")
        self.preset_combo.currentIndexChanged.connect(self.apply_preset)
        preset_layout.addWidget(self.preset_combo)
        
        controls_layout.addWidget(preset_group)
        
        # Add spacer
        controls_layout.addStretch()
        
        # 3D Visualization area on the right
        viz_panel = QWidget()
        viz_layout = QVBoxLayout(viz_panel)
        # Add explanation label
        self.explanation_label = QLabel("Red: X (forward), Green: Y (right), Blue: Z (up). Arrows show orientation.")
        self.explanation_label.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.explanation_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        viz_layout.addWidget(self.explanation_label)
        # Add the view widget
        self.view_widget = gl.GLViewWidget()
        self.view_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        viz_layout.addWidget(self.view_widget, stretch=1)
        # Add coordinate grid at z=0
        grid = gl.GLGridItem()
        grid.setSize(20, 20, 1)
        grid.setSpacing(1, 1, 1)
        grid.translate(0, 0, 0)
        self.view_widget.addItem(grid)
        # Add coordinate axes for reference, with opacity 0.4
        axis_size = 5
        axis_alpha = 0.4
        x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [axis_size, 0, 0]]), color=(1, 0, 0, axis_alpha), width=3)
        y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, axis_size, 0]]), color=(0, 1, 0, axis_alpha), width=3)
        z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, axis_size]]), color=(0, 0, 1, axis_alpha), width=3)
        self.view_widget.addItem(x_axis)
        self.view_widget.addItem(y_axis)
        self.view_widget.addItem(z_axis)
        # Add axis labels
        self.x_label = gl.GLTextItem(pos=np.array([axis_size, 0, 0]), text="X", color=(1, 0, 0, 1))
        self.y_label = gl.GLTextItem(pos=np.array([0, axis_size, 0]), text="Y", color=(0, 1, 0, 1))
        self.z_label = gl.GLTextItem(pos=np.array([0, 0, axis_size]), text="Z", color=(0, 0, 1, 1))
        self.view_widget.addItem(self.x_label)
        self.view_widget.addItem(self.y_label)
        self.view_widget.addItem(self.z_label)
        
        # Add panels to main layout
        splitter = QSplitter(Qt.Orientation.Horizontal)
        controls_panel.setMinimumWidth(220)
        controls_panel.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        splitter.addWidget(controls_panel)
        splitter.addWidget(viz_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([220, 2000])
        main_layout.addWidget(splitter)
    
    def update_rpy(self):
        try:
            roll = float(self.roll_edit.text())
            pitch = float(self.pitch_edit.text())
            yaw = float(self.yaw_edit.text())
            
            # Convert degrees to radians
            r = np.deg2rad(roll)
            p = np.deg2rad(pitch)
            y = np.deg2rad(yaw)
            
            # Compute quaternion from roll, pitch, yaw (ZYX convention)
            # First convert to quaternion components
            cy = np.cos(y * 0.5)
            sy = np.sin(y * 0.5)
            cp = np.cos(p * 0.5)
            sp = np.sin(p * 0.5)
            cr = np.cos(r * 0.5)
            sr = np.sin(r * 0.5)
            
            # Calculate quaternion components
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            
            # Update quaternion display
            self.qw_label.setText(f"{qw:.6f}")
            self.qx_label.setText(f"{qx:.6f}")
            self.qy_label.setText(f"{qy:.6f}")
            self.qz_label.setText(f"{qz:.6f}")
            
            # Store the angles
            self.roll_deg = roll
            self.pitch_deg = pitch
            self.yaw_deg = yaw
            
            # Update visualization with the new quaternion
            self.update_visualization_with_quaternion(qw, qx, qy, qz)
        except ValueError:
            pass  # Ignore invalid input
    
    def update_visualization_with_quaternion(self, qw, qx, qy, qz):
        # Clear previous visualization
        for arrow in self.orientation_arrows:
            self.view_widget.removeItem(arrow)
        self.orientation_arrows = []
        if self.orientation_model is not None:
            self.view_widget.removeItem(self.orientation_model)
        self.orientation_model = None
        
        # Create quaternion
        quat = QQuaternion(qw, qx, qy, qz)
        
        # Draw orientation axes (thicker, with arrowheads)
        arrow_length = 4.0
        arrow_width = 10
        arrowhead_length = 0.7  # Arrowhead length
        arrowhead_width = 0.4   # Arrowhead width
        
        # Forward arrow (X axis) - Red
        forward_normalized = quat.rotatedVector(QVector3D(1, 0, 0)).normalized()
        shaft_end = forward_normalized * (arrow_length - arrowhead_length)
        arrow_tip = forward_normalized * (arrow_length + arrowhead_length * 0.2)
        
        # Main arrow shaft
        x_arrow = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [shaft_end.x(), shaft_end.y(), shaft_end.z()]]),
            color=(1, 0, 0, 1),
            width=arrow_width
        )
        self.view_widget.addItem(x_arrow)
        self.orientation_arrows.append(x_arrow)
        
        # Robust perpendicular vector calculation for arrowhead
        perp = QVector3D.crossProduct(forward_normalized, QVector3D(0, 0, 1))
        if perp.length() < 1e-6:
            perp = QVector3D.crossProduct(forward_normalized, QVector3D(0, 1, 0))
        if perp.length() < 1e-6:
            perp = QVector3D(0, 1, 0)
        perp.normalize()
        
        # Generate points for arrowhead lines
        left = shaft_end + perp * arrowhead_width
        right = shaft_end - perp * arrowhead_width
        
        # Draw arrowhead as two lines (V shape)
        x_arrowhead_left = gl.GLLinePlotItem(
            pos=np.array([[arrow_tip.x(), arrow_tip.y(), arrow_tip.z()], [left.x(), left.y(), left.z()]]),
            color=(1, 0, 0, 1),
            width=arrow_width
        )
        x_arrowhead_right = gl.GLLinePlotItem(
            pos=np.array([[arrow_tip.x(), arrow_tip.y(), arrow_tip.z()], [right.x(), right.y(), right.z()]]),
            color=(1, 0, 0, 1),
            width=arrow_width
        )
        self.view_widget.addItem(x_arrowhead_left)
        self.view_widget.addItem(x_arrowhead_right)
        self.orientation_arrows.append(x_arrowhead_left)
        self.orientation_arrows.append(x_arrowhead_right)
        
        # Right arrow (Y axis) - Green
        right = quat.rotatedVector(QVector3D(0, arrow_length, 0))
        y_arrow = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [right.x(), right.y(), right.z()]]),
            color=(0, 1, 0, 1),
            width=arrow_width
        )
        self.view_widget.addItem(y_arrow)
        self.orientation_arrows.append(y_arrow)
        
        # Up arrow (Z axis) - Blue
        up = quat.rotatedVector(QVector3D(0, 0, arrow_length))
        z_arrow = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [up.x(), up.y(), up.z()]]),
            color=(0, 0, 1, 1),
            width=arrow_width
        )
        self.view_widget.addItem(z_arrow)
        self.orientation_arrows.append(z_arrow)
    
    def apply_preset(self, index):
        if index == 0:  # Identity
            self.roll_deg, self.pitch_deg, self.yaw_deg = 0.0, 0.0, 0.0
        elif index == 1:  # 90° Roll
            self.roll_deg, self.pitch_deg, self.yaw_deg = 90.0, 0.0, 0.0
        elif index == 2:  # 90° Pitch
            self.roll_deg, self.pitch_deg, self.yaw_deg = 0.0, 90.0, 0.0
        elif index == 3:  # 90° Yaw
            self.roll_deg, self.pitch_deg, self.yaw_deg = 0.0, 0.0, 90.0
        elif index == 4:  # 180° Roll
            self.roll_deg, self.pitch_deg, self.yaw_deg = 180.0, 0.0, 0.0
        elif index == 5:  # 45° All Axes
            self.roll_deg, self.pitch_deg, self.yaw_deg = 45.0, 45.0, 45.0
            
        # Update input fields
        self.roll_edit.setText(f"{self.roll_deg:.1f}")
        self.pitch_edit.setText(f"{self.pitch_deg:.1f}")
        self.yaw_edit.setText(f"{self.yaw_deg:.1f}")
        
        # Update visualization
        self.update_rpy()

    def copy_quaternion_to_clipboard(self):
        # Copy the current quaternion values to the clipboard as comma-separated values
        qw = self.qw_label.text()
        qx = self.qx_label.text()
        qy = self.qy_label.text()
        qz = self.qz_label.text()
        text = f"{qw}, {qx}, {qy}, {qz}"
        clipboard = QApplication.instance().clipboard()
        clipboard.setText(text)

    def handle_quaternion_paste(self, text):
        # If the pasted text contains commas, auto-populate all fields
        if ',' in text:
            parts = [p.strip() for p in text.split(',')]
            if len(parts) == 4:
                self.qw_label.blockSignals(True)
                self.qx_label.blockSignals(True)
                self.qy_label.blockSignals(True)
                self.qz_label.blockSignals(True)
                self.qw_label.setText(parts[0])
                self.qx_label.setText(parts[1])
                self.qy_label.setText(parts[2])
                self.qz_label.setText(parts[3])
                self.qw_label.blockSignals(False)
                self.qx_label.blockSignals(False)
                self.qy_label.blockSignals(False)
                self.qz_label.blockSignals(False)


if __name__ == '__main__':
    main()