import math
import os
import numpy as np
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph.opengl as gl

class Sim3DWindow(QMainWindow):
    """
    Parallel 3D Simulation & Viewport Window for the Inverted Pendulum.
    Renders 3D mesh representations (cart, rail, pivot, pole) driven by live physics or HIL telemetry.
    Supports loading STL CAD models when stl package is available.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Inverted Pendulum — Parallel 3D Physics Simulation Studio")
        self.resize(900, 700)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Header controls
        ctrl_bar = QHBoxLayout()
        self.lbl_info = QLabel("3D Viewport — Cart Position: 0.00 m | Pendulum Deviation: 0.0°")
        self.lbl_info.setStyleSheet("font-weight: bold; font-size: 13px;")
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

        # 3D Elements Construction
        # 1) Grid floor
        grid = gl.GLGridItem()
        grid.setSize(4, 4, 1)
        grid.setSpacing(0.2, 0.2, 0.2)
        grid.setColor((100, 100, 100, 100))
        self.view.addItem(grid)

        # Check for STL models in models/cart/cart_model.stl and models/pendulum/pendulum_model.stl
        cart_stl = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "cart", "cart_model.stl"))
        pendulum_stl = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "pendulum", "pendulum_model.stl"))

        stl_loaded = False
        try:
            from stl import mesh
            if os.path.exists(cart_stl) and os.path.exists(pendulum_stl):
                # Load Cart STL
                c_mesh = mesh.Mesh.from_file(cart_stl)
                cart_verts = c_mesh.vectors.reshape(-1, 3) * 0.001 # mm to meters
                cart_faces = np.arange(len(cart_verts)).reshape(-1, 3)
                cart_meshdata = gl.MeshData(vertexes=cart_verts, faces=cart_faces)
                self.cart_item = gl.GLMeshItem(meshdata=cart_meshdata, smooth=True, color=(0.2, 0.6, 0.9, 1.0), shader='shaded')
                self.view.addItem(self.cart_item)

                # Load Pendulum STL
                p_mesh = mesh.Mesh.from_file(pendulum_stl)
                p_verts = p_mesh.vectors.reshape(-1, 3) * 0.001 # mm to meters
                p_faces = np.arange(len(p_verts)).reshape(-1, 3)
                p_meshdata = gl.MeshData(vertexes=p_verts, faces=p_faces)
                self.pole_item = gl.GLMeshItem(meshdata=p_meshdata, smooth=True, color=(0.9, 0.3, 0.2, 1.0), shader='shaded')
                self.view.addItem(self.pole_item)

                self.bob_item = None
                stl_loaded = True
                print("[3D VIEWPORT] Successfully loaded custom STL CAD models from models/ directory!")
        except Exception as e:
            print(f"[3D VIEWPORT NOTE] Custom STL mesh loader fallback: {e}")
            stl_loaded = False

        if not stl_loaded:
            # Fallback procedurally generated meshes using pyqtgraph gl.MeshData
            # 2) Physical Rail (thin metallic cylinder along X)
            rail_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.012, 0.012], length=1.0)
            self.rail_item = gl.GLMeshItem(meshdata=rail_mesh, smooth=True, color=(0.7, 0.7, 0.7, 1.0), shader='shaded')
            self.rail_item.rotate(90, 0, 1, 0)
            self.rail_item.translate(-0.5, 0, 0)
            self.view.addItem(self.rail_item)

            # 3) Cart (Cuboid mesh)
            cart_mesh = gl.MeshData.cube(x=0.16, y=0.10, z=0.08)
            self.cart_item = gl.GLMeshItem(meshdata=cart_mesh, smooth=True, color=(0.2, 0.6, 0.9, 1.0), shader='shaded')
            self.view.addItem(self.cart_item)

            # 4) Pendulum Pole (Cylinder)
            pole_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.008, 0.008], length=0.40)
            self.pole_item = gl.GLMeshItem(meshdata=pole_mesh, smooth=True, color=(0.9, 0.3, 0.2, 1.0), shader='shaded')
            self.view.addItem(self.pole_item)

            # 5) Bob/Tip Mass (Sphere)
            bob_mesh = gl.MeshData.sphere(rows=10, cols=20, radius=0.02)
            self.bob_item = gl.GLMeshItem(meshdata=bob_mesh, smooth=True, color=(0.95, 0.8, 0.2, 1.0), shader='shaded')
            self.view.addItem(self.bob_item)

        self.reset_camera()

    def reset_camera(self):
        self.view.setCameraPosition(distance=2.2, elevation=15, azimuth=45)

    def update_state(self, cart_x: float, angle_dev_deg: float):
        """
        Updates 3D spatial transformation of the Cart, Pendulum Pole, and Tip Bob.
        cart_x: position along rail (-0.20 to +0.20 m)
        angle_dev_deg: deviation from upright (0° = upright)
        """
        cx = max(-0.45, min(0.45, cart_x))
        th_rad = math.radians(angle_dev_deg)

        # Update Cart mesh position (centered at x = cx, y = 0, z = 0)
        self.cart_item.resetTransform()
        self.cart_item.translate(cx - 0.08, -0.05, -0.04)

        # Pendulum Pole pivot sits at top of cart (cx, 0, 0.04)
        self.pole_item.resetTransform()
        self.pole_item.translate(cx, 0, 0.04)
        self.pole_item.rotate(angle_dev_deg, 0, 1, 0)
        self.pole_item.rotate(90, 0, 1, 0)

        # Tip Bob position if using procedurally generated bob
        if self.bob_item is not None:
            bx = cx + 0.40 * math.sin(th_rad)
            bz = 0.04 + 0.40 * math.cos(th_rad)
            self.bob_item.resetTransform()
            self.bob_item.translate(bx, 0, bz)

        self.lbl_info.setText(f"3D Viewport — Cart Position: {cx:+.3f} m | Pendulum Deviation: {angle_dev_deg:+.1f}°")
