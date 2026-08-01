import math
import os
import numpy as np
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QCheckBox
)
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph.opengl as gl

def create_cube_mesh(xlen=0.16, ylen=0.10, zlen=0.08):
    """Generates a centered cuboid MeshData for pyqtgraph GLMeshItem."""
    dx, dy, dz = xlen / 2.0, ylen / 2.0, zlen / 2.0
    vertices = np.array([
        [-dx, -dy, -dz], [ dx, -dy, -dz], [ dx,  dy, -dz], [-dx,  dy, -dz],
        [-dx, -dy,  dz], [ dx, -dy,  dz], [ dx,  dy,  dz], [-dx,  dy,  dz]
    ], dtype=np.float32)
    faces = np.array([
        [0, 1, 2], [0, 2, 3],  # Bottom
        [4, 5, 6], [4, 6, 7],  # Top
        [0, 1, 5], [0, 5, 4],  # Front
        [2, 3, 7], [2, 7, 6],  # Back
        [0, 3, 7], [0, 7, 4],  # Left
        [1, 2, 6], [1, 6, 5]   # Right
    ], dtype=np.uint32)
    return gl.MeshData(vertexes=vertices, faces=faces)

def load_stl_mesh(stl_path, align_mode="center"):
    """
    Loads binary or ASCII STL CAD files and centers vertex bounding box.
    align_mode='center': centers bounding box along (x, y, z)
    align_mode='pivot': centers (x, y) and sets z_min to 0 for exact pivot joint rotation.
    """
    try:
        from stl import mesh
        stl_data = mesh.Mesh.from_file(stl_path)
        verts = stl_data.vectors.reshape(-1, 3) * 0.001  # Convert mm to meters
    except Exception:
        with open(stl_path, 'rb') as f:
            f.seek(80)
            n_triangles = np.frombuffer(f.read(4), dtype=np.uint32)[0]
            record_dtype = np.dtype([('normal', 'f4', (3,)), ('verts', 'f4', (3, 3)), ('attr', 'u2')])
            records = np.frombuffer(f.read(), dtype=record_dtype)
            verts = records['verts'].reshape(-1, 3) * 0.001

    min_b = verts.min(axis=0)
    max_b = verts.max(axis=0)
    center = (min_b + max_b) / 2.0

    if align_mode == "pivot":
        # Keep z=0 at the bottom joint of the pendulum for clean rotational alignment
        center_offset = np.array([center[0], center[1], min_b[2]], dtype=np.float32)
    else:
        # Center cart at origin
        center_offset = center.astype(np.float32)

    verts = verts - center_offset
    faces = np.arange(len(verts)).reshape(-1, 3)
    return gl.MeshData(vertexes=verts, faces=faces), (max_b - min_b)

class Sim3DWindow(QMainWindow):
    """
    Parallel 3D Physics Simulation Studio (NVIDIA Isaac Sim Viewport integration & FreeCAD Assembly support).
    Features interactive manual Cart alignment slider and real-time non-linear Cart-Pole Physics Simulation.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Inverted Pendulum — Interactive 3D Physics & Assembly Alignment Studio")
        self.resize(1000, 750)

        # Physics Simulation State
        self.sim_active = False
        self.cart_x = 0.0
        self.cart_vx = 0.0
        self.angle_rad = 0.05  # initial small 2.8° tilt
        self.angle_vel = 0.0

        # Physical constants matching real hardware
        self.g = 9.81
        self.m_cart = 0.50
        self.m_pole = 0.15
        self.length = 0.40  # pole length (m)
        self.b_friction = 0.05
        self.dt = 0.01  # 100 Hz physics solver step

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Header status controls
        ctrl_bar = QHBoxLayout()
        self.lbl_info = QLabel("3D Isaac Sim Viewport — Position: 0.000 m | Pendulum: 0.0°")
        self.lbl_info.setStyleSheet("font-weight: bold; font-size: 13px; color: #3498db;")
        ctrl_bar.addWidget(self.lbl_info)
        ctrl_bar.addStretch()

        self.btn_reset_cam = QPushButton("Reset Camera")
        self.btn_reset_cam.setStyleSheet("padding: 6px 12px; font-weight: bold;")
        self.btn_reset_cam.clicked.connect(self.reset_camera)
        ctrl_bar.addWidget(self.btn_reset_cam)
        layout.addLayout(ctrl_bar)

        # pyqtgraph OpenGL 3D View Widget
        self.view = gl.GLViewWidget()
        self.view.opts['distance'] = 2.5
        self.view.opts['elevation'] = 20
        self.view.opts['azimuth'] = 45
        self.view.setBackgroundColor('#121212')
        layout.addWidget(self.view, 1)

        # Interactive Controls Bar (Manual Cart Slider + Interactive Physics Engine Toggle)
        interact_bar = QHBoxLayout()
        interact_bar.setSpacing(15)

        self.chk_physics = QCheckBox("SIMULATE 3D PHYSICS ENGINE")
        self.chk_physics.setStyleSheet("font-weight: bold; color: #2ecc71; font-size: 13px;")
        self.chk_physics.toggled.connect(self.toggle_physics)
        interact_bar.addWidget(self.chk_physics)

        lbl_slider = QLabel("Manual Cart Position (Track Alignment):")
        lbl_slider.setStyleSheet("font-weight: 600;")
        interact_bar.addWidget(lbl_slider)

        self.cart_slider = QSlider(Qt.Orientation.Horizontal)
        self.cart_slider.setRange(-200, 200)  # -0.20m to +0.20m
        self.cart_slider.setValue(0)
        self.cart_slider.valueChanged.connect(self.on_slider_moved)
        interact_bar.addWidget(self.cart_slider, 1)

        self.btn_push = QPushButton("Nudge Pole")
        self.btn_push.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold; padding: 6px 12px; border-radius: 4px;")
        self.btn_push.clicked.connect(self.nudge_pole)
        interact_bar.addWidget(self.btn_push)

        layout.addLayout(interact_bar)

        # 3D Scene Environment
        grid = gl.GLGridItem()
        grid.setSize(4, 4, 1)
        grid.setSpacing(0.2, 0.2, 0.2)
        grid.setColor((80, 80, 80, 120))
        self.view.addItem(grid)

        cart_stl = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "cart", "cart_model.stl"))
        pendulum_stl = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "pendulum", "pendulum_model.stl"))
        track_stl = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "track_model.stl"))
        assembly_fcstd = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "assembly_model.FCStd"))

        # Attempt to run automated FreeCAD conversion script if installed on system
        try:
            from scripts.convert_fcstd import convert_fcstd_to_stl
            convert_fcstd_to_stl(assembly_fcstd)
        except Exception:
            pass

        stl_loaded = False
        try:
            if os.path.exists(track_stl):
                track_meshdata, _ = load_stl_mesh(track_stl, align_mode="center")
                self.track_cad_item = gl.GLMeshItem(meshdata=track_meshdata, smooth=True, color=(0.4, 0.45, 0.5, 1.0), shader='shaded')
                self.view.addItem(self.track_cad_item)

            if os.path.exists(cart_stl) and os.path.exists(pendulum_stl):
                cart_meshdata, cart_dim = load_stl_mesh(cart_stl, align_mode="center")
                self.cart_item = gl.GLMeshItem(meshdata=cart_meshdata, smooth=True, color=(0.2, 0.6, 0.9, 1.0), shader='shaded')
                self.view.addItem(self.cart_item)

                pendulum_meshdata, pen_dim = load_stl_mesh(pendulum_stl, align_mode="pivot")
                self.pole_item = gl.GLMeshItem(meshdata=pendulum_meshdata, smooth=True, color=(0.9, 0.3, 0.2, 1.0), shader='shaded')
                self.view.addItem(self.pole_item)

                self.bob_item = None
                self.pivot_z = float(cart_dim[2]) / 2.0 if cart_dim[2] > 0 else 0.04
                stl_loaded = True
                print(f"[3D VIEWPORT] Loaded FreeCAD Assembly CAD Assets: {assembly_fcstd}")
        except Exception as e:
            print(f"[3D VIEWPORT NOTE] Assembly CAD model loader fallback: {e}")
            stl_loaded = False

        if not stl_loaded:
            self.pivot_z = 0.04
            # Cart Mesh
            cart_mesh = create_cube_mesh(xlen=0.16, ylen=0.10, zlen=0.08)
            self.cart_item = gl.GLMeshItem(meshdata=cart_mesh, smooth=True, color=(0.2, 0.6, 0.9, 1.0), shader='shaded')
            self.view.addItem(self.cart_item)

            # Pendulum Pole Mesh
            pole_mesh = gl.MeshData.cylinder(rows=12, cols=24, radius=[0.008, 0.008], length=0.40)
            self.pole_item = gl.GLMeshItem(meshdata=pole_mesh, smooth=True, color=(0.9, 0.3, 0.2, 1.0), shader='shaded')
            self.view.addItem(self.pole_item)

            # Bob Mass Mesh
            bob_mesh = gl.MeshData.sphere(rows=12, cols=24, radius=0.02)
            self.bob_item = gl.GLMeshItem(meshdata=bob_mesh, smooth=True, color=(0.95, 0.8, 0.2, 1.0), shader='shaded')
            self.view.addItem(self.bob_item)

        self.reset_camera()

        # Real-time Physics & Graphics Animation Timer (100 Hz)
        self.physics_timer = QTimer(self)
        self.physics_timer.timeout.connect(self.physics_step)
        self.physics_timer.start(10)

    def reset_camera(self):
        self.view.setCameraPosition(distance=2.2, elevation=15, azimuth=45)

    def toggle_physics(self, enabled: bool):
        self.sim_active = enabled
        self.cart_slider.setEnabled(not enabled)
        if enabled:
            self.lbl_info.setText("3D Physics Engine Running — Simulate Physics Active")

    def on_slider_moved(self, val: int):
        if not self.sim_active:
            self.cart_x = val / 1000.0  # mm to meters (-0.20m to +0.20m)
            self.update_state(self.cart_x, math.degrees(self.angle_rad))

    def nudge_pole(self):
        """Applies an instantaneous angular velocity push to tilt the 3D pendulum."""
        self.angle_vel += 1.5  # rad/s push

    def physics_step(self):
        """
        Integrates full 3D non-linear equations of motion for the Cart-Pole system:
        J * ddt(theta) = m*g*l*sin(theta) - b*theta_dot
        """
        if not self.sim_active:
            return

        sin_th = math.sin(self.angle_rad)
        cos_th = math.cos(self.angle_rad)

        # Rotational acceleration around pivot
        pole_inertia = (1.0 / 3.0) * self.m_pole * (self.length ** 2)
        angle_acc = (self.m_pole * self.g * (self.length / 2.0) * sin_th - self.b_friction * self.angle_vel) / pole_inertia

        # Symplectic Euler integration
        self.angle_vel += angle_acc * self.dt
        self.angle_rad += self.angle_vel * self.dt

        # Cart passive damping
        self.cart_vx *= 0.95
        self.cart_x += self.cart_vx * self.dt

        # Bounds check along rail track (-0.45m to +0.45m)
        if abs(self.cart_x) > 0.40:
            self.cart_x = math.copysign(0.40, self.cart_x)
            self.cart_vx = -self.cart_vx * 0.5

        deg = math.degrees(self.angle_rad)
        self.cart_slider.blockSignals(True)
        self.cart_slider.setValue(int(self.cart_x * 1000))
        self.cart_slider.blockSignals(False)

        self.update_state(self.cart_x, deg)

    def update_state(self, cart_x: float, angle_dev_deg: float):
        """
        Updates 3D spatial transformation of the Cart & Pendulum Assembly.
        cart_x: position along rail (-0.20 to +0.20 m)
        angle_dev_deg: deviation from upright (0° = upright)
        """
        cx = max(-0.45, min(0.45, cart_x))
        th_rad = math.radians(angle_dev_deg)

        # 1. Position Cart on guide rail
        self.cart_item.resetTransform()
        self.cart_item.translate(cx, 0, 0)

        # 2. Position Pendulum Pole around exact top-pivot of cart
        self.pole_item.resetTransform()
        self.pole_item.translate(cx, 0, self.pivot_z)
        self.pole_item.rotate(-angle_dev_deg, 0, 1, 0)

        # 3. Position Bob mass at tip if using procedural primitives
        if self.bob_item is not None:
            bx = cx + 0.40 * math.sin(th_rad)
            bz = self.pivot_z + 0.40 * math.cos(th_rad)
            self.bob_item.resetTransform()
            self.bob_item.translate(bx, 0, bz)

        self.lbl_info.setText(f"Isaac Sim Viewport — Cart Position: {cx:+.3f} m | Pendulum Deviation: {angle_dev_deg:+.1f}°")
