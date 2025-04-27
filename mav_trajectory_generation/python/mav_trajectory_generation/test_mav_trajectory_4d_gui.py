#!/usr/bin/env python3
"""
test_mav_trajectory_4d_gui.py

Interactive GUI for generating and publishing a 4D (x,y,z,yaw) drone trajectory
for an urban façade inspection mission, using mav_trajectory_generation and ROS.

Features:
  • Edit waypoints in a table (x, y, z, yaw in degrees).
  • Adjust v_max, a_max, sampling_dt parameters.
  • Choose which derivative to optimize (position, velocity, acceleration, jerk, snap).
  • Click “Publish” or press Enter to regenerate and publish /mav_trajectory/path
    and /mav_trajectory/waypoints.
  • Window stays on top and auto-resizes to fit content.

Dependencies:
  • PyQt5
  • rospy
  • mav_trajectory_generation Python bindings
  • numpy
  • tf
  • geometry_msgs, nav_msgs, visualization_msgs
"""
import sys
import numpy as np
import rospy
import tf
from PyQt5 import QtWidgets, QtCore, QtGui
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

# Import mission utilities
test_mod = __import__('test_mav_trajectory_4d')
Waypoint = test_mod.Waypoint
TrajectoryPlanner = test_mod.TrajectoryPlanner
make_waypoint_markers = test_mod.make_waypoint_markers

from mav_trajectory_generation import (
    derivative_order,
)


class TrajectoryGui(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("4D Trajectory GUI")
        # Always on top
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        self.resize(800, 600)
        self.setMinimumSize(800, 600)

        self._init_ros()
        self._build_ui()
        self._load_default_mission()

        # Fit table and window
        self.table.resizeColumnsToContents()
        self.table.resizeRowsToContents()
        self.adjustSize()

        # Enter key triggers publish
        enter = QtGui.QKeySequence(QtCore.Qt.Key_Return)
        self.pub_shortcut = QtWidgets.QShortcut(enter, self)
        self.pub_shortcut.setContext(QtCore.Qt.ApplicationShortcut)
        self.pub_shortcut.activated.connect(self._publish)
        enter2 = QtGui.QKeySequence(QtCore.Qt.Key_Enter)
        self.pub_shortcut2 = QtWidgets.QShortcut(enter2, self)
        self.pub_shortcut2.setContext(QtCore.Qt.ApplicationShortcut)
        self.pub_shortcut2.activated.connect(self._publish)

    def _init_ros(self):
        rospy.init_node('mav_trajectory_gui', anonymous=True, disable_signals=True)
        self.path_pub = rospy.Publisher(
            '/mav_trajectory/path', Path, queue_size=1, latch=True
        )
        self.wp_pub = rospy.Publisher(
            '/mav_trajectory/waypoints', MarkerArray, queue_size=1, latch=True
        )

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Parameter controls
        params_layout = QtWidgets.QHBoxLayout()
        self.vmax_spin = QtWidgets.QDoubleSpinBox()
        self.vmax_spin.setRange(0.1, 10.0)
        self.vmax_spin.setValue(3.0)
        self.vmax_spin.setSuffix(" m/s")
        params_layout.addWidget(QtWidgets.QLabel("v_max:"))
        params_layout.addWidget(self.vmax_spin)

        self.amax_spin = QtWidgets.QDoubleSpinBox()
        self.amax_spin.setRange(0.1, 10.0)
        self.amax_spin.setValue(1.5)
        self.amax_spin.setSuffix(" m/s²")
        params_layout.addWidget(QtWidgets.QLabel("a_max:"))
        params_layout.addWidget(self.amax_spin)

        self.dt_spin = QtWidgets.QDoubleSpinBox()
        self.dt_spin.setRange(0.01, 1.0)
        self.dt_spin.setSingleStep(0.01)
        self.dt_spin.setValue(0.1)
        self.dt_spin.setSuffix(" s")
        params_layout.addWidget(QtWidgets.QLabel("dt:"))
        params_layout.addWidget(self.dt_spin)

        # Derivative order combobox
        self.derivative_combo = QtWidgets.QComboBox()
        # Populate with derivative_order constants
        items = [
            ("Position", derivative_order.POSITION),
            ("Velocity", derivative_order.VELOCITY),
            ("Acceleration", derivative_order.ACCELERATION),
            ("Jerk", derivative_order.JERK),
            ("Snap", derivative_order.SNAP),
        ]
        for name, val in items:
            self.derivative_combo.addItem(name, val)
        self.derivative_combo.setCurrentIndex(4)  # default to Snap
        self.derivative_combo.currentIndexChanged.connect(self._publish)
        params_layout.addWidget(QtWidgets.QLabel("Optimize:"))
        params_layout.addWidget(self.derivative_combo)

        layout.addLayout(params_layout)

        # Waypoints table: X, Y, Z, Yaw (°)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["X (m)", "Y (m)", "Z (m)", "Yaw (°)"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table)

        # Add/Remove waypoint buttons
        btn_layout = QtWidgets.QHBoxLayout()
        add_btn = QtWidgets.QPushButton("Add Waypoint")
        add_btn.clicked.connect(self._add_row)
        btn_layout.addWidget(add_btn)
        rem_btn = QtWidgets.QPushButton("Remove Waypoint")
        rem_btn.clicked.connect(self._remove_row)
        btn_layout.addWidget(rem_btn)
        layout.addLayout(btn_layout)

        # Publish button
        pub_btn = QtWidgets.QPushButton("Publish Trajectory")
        pub_btn.clicked.connect(self._publish)
        layout.addWidget(pub_btn)

    def _load_default_mission(self):
        self.table.setRowCount(0)
        default = [
            Waypoint(0, 0, 0, 0),
            Waypoint(0, 0, 3, 0),
            Waypoint(10, 5, 5, -np.pi/2),
            Waypoint(10, 5, 3, -np.pi/2),
            Waypoint(15, 10, 3, np.pi/2),
            Waypoint(15, 15, 3, 0.0),
            Waypoint(20, 15, 3, -np.pi/2),
            Waypoint(20, 10, 3, -np.pi),
            Waypoint(10, 10, 5, -3*np.pi/4),
            Waypoint(0, 0, 3, 0),
            Waypoint(0, 0, 0, 0),
        ]
        for wp in default:
            self._add_row(wp.x, wp.y, wp.z, wp.yaw)

    def _add_row(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw_rad: float = 0.0):
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c, val in enumerate((x, y, z, yaw_rad)):
            if c == 3:
                disp = np.degrees(val)
                text = f"{disp:.1f}"
            else:
                text = f"{val:.3f}"
            item = QtWidgets.QTableWidgetItem(text)
            item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.table.setItem(r, c, item)

    def _remove_row(self):
        r = self.table.currentRow()
        if r < 0:
            r = self.table.rowCount() - 1
        if r >= 0:
            self.table.removeRow(r)

    def _publish(self):
        waypoints = []
        for r in range(self.table.rowCount()):
            vals = []
            try:
                for c in range(4):
                    text = self.table.item(r, c).text()
                    f = float(text)
                    if c == 3:
                        f = np.radians(f)
                    vals.append(f)
            except Exception:
                rospy.logerr(f"Invalid number at row {r+1}")
                return
            waypoints.append(Waypoint(*vals))

        v_max = self.vmax_spin.value()
        a_max = self.amax_spin.value()
        dt = self.dt_spin.value()
        derivative = self.derivative_combo.currentData()

        planner = TrajectoryPlanner(
            waypoints, v_max=v_max, a_max=a_max, derivative=derivative
        )
        planner.build()
        rospy.loginfo("Generated new trajectory optimizing derivative %s",
                      self.derivative_combo.currentText())

        path = planner.sample_trajectory(planner.pos_traj, planner.yaw_traj, dt)
        markers = make_waypoint_markers(waypoints)
        self.path_pub.publish(path)
        self.wp_pub.publish(markers)
        rospy.loginfo("Published Path and Waypoint markers to ROS topics")


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = TrajectoryGui()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
