#!/usr/bin/env python3
"""
test_mav_trajectory_4d.py

Generate and visualize a smooth 4D (x,y,z,yaw) drone trajectory for an urban façade
inspection mission, using mav_trajectory_generation python bindings.

Scenario steps:
  1. Take off vertically from ground to 3 m, facing north.
  2. Fly to façade inspection point A (10, 5, 3), rotate to face west.
  3. Pan down under an overhang at A (10, 5, 2.5), rotate to face south.
  4. Move to window corner B (15, 10, 3), rotate to 45°.
  5. Orbit a column in four segments around (15, 12), completing 360°.
  6. Return to above launch (0, 0, 3), then land (0, 0, 0), facing north.

Outputs:
  • Logs per-segment durations for position and yaw.
  • Publishes a nav_msgs/Path and MarkerArray for RViz.
"""

from dataclasses import dataclass
from typing import List

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mav_trajectory_generation import (
    Vertex, PolynomialOptimization, 
    derivative_order,
    Trajectory,
    estimate_segment_times, 
    estimate_segment_times_nfabian,
)
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------- #
#                                  Data Types                                  #
# ---------------------------------------------------------------------------- #

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float  # radians

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def unwrap_yaw(self, reference: float) -> None:
        """
        Adjust self.yaw so that |self.yaw - reference| <= π by adding/subtracting 2π.
        """
        delta = (self.yaw - reference + np.pi) % (2 * np.pi) - np.pi
        self.yaw = reference + delta


# ---------------------------------------------------------------------------- #
#                              Trajectory Builder                              #
# ---------------------------------------------------------------------------- #

class TrajectoryPlanner:
    """
    Build both 3D position and 1D yaw optimal‐snap trajectories from a sequence
    of Waypoints.
    """

    def __init__(
        self,
        waypoints: List[Waypoint],
        v_max: float = 2.0,
        a_max: float = 1.0,
        derivative: int = derivative_order.SNAP,
    ) -> None:
        self.waypoints = waypoints
        self.v_max = v_max
        self.a_max = a_max
        self.derivative = derivative

        # Output trajectories
        self.pos_traj = Trajectory()
        self.yaw_traj = Trajectory()

    def build(self) -> None:
        """
        1) Create nanobind::Vertex lists for position and yaw.
        2) Estimate segment times.
        3) Solve two linear optimal‐snap problems.
        """
        pos_vertices = []
        yaw_vertices = []

        prev_yaw = None
        for i, wp in enumerate(self.waypoints):
            # --- Position vertex ---
            v = Vertex(dimension=3)
            pos = wp.position()

            if i == 0 or i == len(self.waypoints) - 1:
                # fix up to snap at start/end (zero higher‐order derivatives)
                v.make_start_or_end_vec(pos, self.derivative)
            else:
                # only fix position at interior waypoints
                v.add_constraint_vec(derivative_order.POSITION, pos)
            pos_vertices.append(v)

            # --- Yaw vertex ---
            yv = Vertex(dimension=1)
            if prev_yaw is not None:
                wp.unwrap_yaw(prev_yaw)
            yv.add_constraint(derivative_order.ORIENTATION, wp.yaw)
            yaw_vertices.append(yv)

            prev_yaw = wp.yaw

        # 2) Time allocation
        segment_times = estimate_segment_times(pos_vertices, self.v_max, self.a_max)
        for i, t in enumerate(estimate_segment_times_nfabian(pos_vertices, self.v_max, self.a_max, 1)):
            print(f"  segment {i}: t = {t:.2f} s")

        # 3a) Solve for position
        pos_opt = PolynomialOptimization(dimension=3)
        pos_opt.setup_from_vertices(pos_vertices, segment_times, self.derivative)
        pos_opt.solve_linear()
        pos_opt.get_trajectory(self.pos_traj)

        # 3b) Solve for yaw
        yaw_opt = PolynomialOptimization(dimension=1)
        yaw_opt.setup_from_vertices(yaw_vertices, segment_times, self.derivative)
        yaw_opt.solve_linear()
        yaw_opt.get_trajectory(self.yaw_traj)

    def log_segment_summary(self) -> None:
        """
        Log each segment's duration for both position and yaw.
        """
        rospy.loginfo("Position trajectory segments:")
        for seg in self.pos_traj.get_segments():
            rospy.loginfo(f"  → duration = {seg.get_time():.2f} s")

        rospy.loginfo("Yaw trajectory segments:")
        for seg in self.yaw_traj.get_segments():
            rospy.loginfo(f"  → duration = {seg.get_time():.2f} s")
            
        print(f"PosTraj[Vmax,Amax]: {self.pos_traj.compute_max_velocity_and_acceleration()}")
        print(f"YawTraj[Vmax,Amax]: {self.yaw_traj.compute_max_velocity_and_acceleration()}")

    @staticmethod
    def sample_trajectory(
        pos_traj: Trajectory,
        yaw_traj: Trajectory,
        dt: float,
        frame_id: str = "world",
    ) -> Path:
        """
        Sample both pos_traj and yaw_traj at uniform dt and return a nav_msgs/Path
        whose pose.orientation quaternions correspond to yaw.
        """
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = rospy.Time.now()
        samples = []

        for pos_seg, yaw_seg in zip(pos_traj.get_segments(), yaw_traj.get_segments()):
            T = pos_seg.get_time()
            coeffs_xyz = pos_seg.get_polynomials()   # 3 lists
            coeffs_yaw = yaw_seg.get_polynomials()[0]

            t = 0.0
            while t <= T + 1e-6:
                # position
                p = [sum(c[i] * (t**i) for i in range(len(c))) for c in coeffs_xyz]
                # yaw
                yaw = sum(coeffs_yaw[i] * (t**i) for i in range(len(coeffs_yaw)))
                # build PoseStamped
                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.header.stamp = rospy.Time.now()
                ps.pose.position = Point(*p)
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                ps.pose.orientation = Quaternion(*q)
                samples.append(ps)
                t += dt

        path.poses = samples
        rospy.loginfo(f"Sampled {len(samples)} poses at dt = {dt:.2f}s")
        return path


# ---------------------------------------------------------------------------- #
#                            RViz Visualization Helpers                        #
# ---------------------------------------------------------------------------- #

def make_waypoint_markers(waypoints: List[Waypoint], frame_id: str = "world") -> MarkerArray:
    """
    Create an arrow marker at each waypoint, pointing in the specified yaw.
    """
    markers = MarkerArray()
    for idx, wp in enumerate(waypoints):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "waypoints"
        m.id = idx
        m.type = Marker.ARROW
        m.action = Marker.ADD
        # arrow shaft length = 0.5m, diameter = 0.1m
        m.scale.x, m.scale.y, m.scale.z = 0.5, 0.1, 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        m.pose.position = Point(wp.x, wp.y, wp.z)
        q = tf.transformations.quaternion_from_euler(0, 0, wp.yaw)
        m.pose.orientation = Quaternion(*q)
        markers.markers.append(m)
    return markers


# ---------------------------------------------------------------------------- #
#                                    Main                                      #
# ---------------------------------------------------------------------------- #

def main():
    rospy.init_node("mav_trajectory_4d")

    # Define mission waypoints
    mission: List[Waypoint] = [
        Waypoint(0,   0, 0,   0),         # 1) launch
        Waypoint(0,   0, 3,   np.pi/2),   #    ascend to 3 m, yaw north
        Waypoint(10,  5, 3,   np.pi/4),   # 2) façade A, face west
        Waypoint(10,  5, 2.5, -np.pi/2),  #    pan under overhang, face south
        Waypoint(15, 10, 3,   np.pi/2),   # 3) corner B
        # 4) orbit around (15,12) in four 90° steps
        Waypoint(15, 12, 3,   0.0),
        Waypoint(17, 12, 3,   -np.pi/2),
        Waypoint(17, 10, 3,   -np.pi),
        Waypoint(15, 10, 3.5, -3*np.pi/4), # slight climb, correct yaw
        # 5) return & land
        Waypoint(0,   0, 3,   np.pi/2),
        Waypoint(0,   0, 0,   0),
    ]

    planner = TrajectoryPlanner(mission, v_max=3.0, a_max=1.5)
    planner.build()
    planner.log_segment_summary()

    # Sampling & publishing
    dt = 0.25  # 4 Hz sampling
    path = planner.sample_trajectory(planner.pos_traj, planner.yaw_traj, dt)

    path_pub = rospy.Publisher("/mav_trajectory/path", Path, queue_size=1, latch=True)
    wp_pub   = rospy.Publisher("/mav_trajectory/waypoints", MarkerArray, queue_size=1, latch=True)

    # Wait a moment for connections
    rospy.sleep(0.5)
    path_pub.publish(path)
    wp_pub.publish(make_waypoint_markers(mission))
    rospy.loginfo("Trajectory & waypoints published. Add /mav_trajectory/path and /mav_trajectory/waypoints to RViz.")

    rospy.spin()


if __name__ == "__main__":
    main()
