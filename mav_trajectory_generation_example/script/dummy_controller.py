#! /usr/bin/env python3

import rospy
import math
from enum import Enum
from tf.transformations import euler_from_quaternion

# Different control modes for the drone.
from uav_gazebo_msgs.msg import (
    PositionYawControl,
    VelocityYawRateControl,
    ThrustAttitudeControl,
    ThrustVelocityControl,
    ThrustTorqueControl,
)

# Trajectory messages
from trajectory_msgs.msg import MultiDOFJointTrajectory


class ControlOutputType(Enum):
    POSITION_YAW = "PositionYawControl"
    VELOCITY_YAWRATE = "VelocityYawRateControl"
    THRUST_ATTITUDE = "ThrustAttitudeControl"
    THRUST_VELOCITY = "ThrustVelocityControl"
    THRUST_TORQUE = "ThrustTorqueControl"


class DummyController:
    def __init__(self):
        # Initialize node
        rospy.init_node("dummy_controller", anonymous=True)

        # Get control output type from ROS parameter
        control_output_type_str = rospy.get_param(
            "~control_output_type", "PositionYawControl"
        )

        # Try to parse the control output type
        try:
            self.control_output_type = ControlOutputType(control_output_type_str)
        except ValueError:
            rospy.logerr(f"Invalid control output type: {control_output_type_str}")
            rospy.signal_shutdown("Invalid control output type")
            return

        # Mapping from control type to message type and topic suffix
        self.control_types = {
            ControlOutputType.POSITION_YAW: (PositionYawControl, "position_yaw"),
            ControlOutputType.VELOCITY_YAWRATE: (
                VelocityYawRateControl,
                "velocity_yawrate",
            ),
            ControlOutputType.THRUST_ATTITUDE: (
                ThrustAttitudeControl,
                "thrust_attitude",
            ),
            ControlOutputType.THRUST_VELOCITY: (
                ThrustVelocityControl,
                "thrust_velocity",
            ),
            ControlOutputType.THRUST_TORQUE: (ThrustTorqueControl, "thrust_torque"),
        }

        # Get message type and topic suffix
        self.msg_type, topic_suffix = self.control_types[self.control_output_type]

        # Publisher
        self.pub = rospy.Publisher(
            f"/drone/{topic_suffix}/command", self.msg_type, queue_size=1
        )

        # Trajectory subscriber
        self.sub = rospy.Subscriber(
            "/command/trajectory", MultiDOFJointTrajectory, self.trajectory_callback
        )

        # Set up shutdown handler
        rospy.on_shutdown(self.send_zero_command)

        rospy.loginfo(
            f"DummyController initialized with control output type: {self.control_output_type.value}"
        )

    def trajectory_callback(self, msg):
        # Depending on control output type, create appropriate command message
        try:
            # Extract common data
            data = self.extract_data_from_msg(msg)

            if self.control_output_type == ControlOutputType.POSITION_YAW:
                cmd = self.create_position_yaw_control_msg(data)
            elif self.control_output_type == ControlOutputType.VELOCITY_YAWRATE:
                cmd = self.create_velocity_yawrate_control_msg(data)
            elif self.control_output_type == ControlOutputType.THRUST_ATTITUDE:
                cmd = self.create_thrust_attitude_control_msg(data)
            elif self.control_output_type == ControlOutputType.THRUST_VELOCITY:
                cmd = self.create_thrust_velocity_control_msg(data)
            elif self.control_output_type == ControlOutputType.THRUST_TORQUE:
                cmd = self.create_thrust_torque_control_msg(data)
            else:
                rospy.logerr("Invalid control output type in callback")
                return

            # Publish the command
            self.pub.publish(cmd)
        except Exception as e:
            rospy.logerr(f"Error in trajectory_callback: {e}")

    def extract_data_from_msg(self, msg):
        if not msg.points:
            rospy.logerr("Received trajectory message with no points.")
            return None

        point = msg.points[0]

        if not point.transforms:
            rospy.logerr("Trajectory point has no transforms.")
            return None

        if not point.velocities:
            rospy.logerr("Trajectory point has no velocities.")
            return None

        if not point.accelerations:
            rospy.logerr("Trajectory point has no accelerations.")
            return None

        # Extract position
        position = point.transforms[0].translation

        # Extract quaternion and convert to Euler angles
        rotation = point.transforms[0].rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Extract velocities
        linear_velocity = point.velocities[0].linear
        angular_velocity = point.velocities[0].angular

        # Extract accelerations
        linear_acceleration = point.accelerations[0].linear
        angular_acceleration = point.accelerations[0].angular

        # Time from start
        time_from_start = point.time_from_start.to_sec()

        data = {
            "position": position,
            "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "linear_velocity": linear_velocity,
            "angular_velocity": angular_velocity,
            "linear_acceleration": linear_acceleration,
            "angular_acceleration": angular_acceleration,
            "time_from_start": time_from_start,
        }

        return data

    def create_position_yaw_control_msg(self, data):
        cmd = PositionYawControl()
        cmd.position.x = data["position"].x
        cmd.position.y = data["position"].y
        cmd.position.z = data["position"].z

        # Velocity
        cmd.velocity.x = data["linear_velocity"].x
        cmd.velocity.y = data["linear_velocity"].y
        cmd.velocity.z = data["linear_velocity"].z

        # Acceleration
        cmd.acceleration.x = data["linear_acceleration"].x
        cmd.acceleration.y = data["linear_acceleration"].y
        cmd.acceleration.z = data["linear_acceleration"].z

        # Yaw and Yaw Rate
        cmd.yaw = data["orientation"]["yaw"]
        cmd.yaw_rate = data["angular_velocity"].z

        return cmd

    def create_velocity_yawrate_control_msg(self, data):
        cmd = VelocityYawRateControl()
        cmd.body_frame = True

        # Velocity
        cmd.velocity.x = data["linear_velocity"].x
        cmd.velocity.y = data["linear_velocity"].y
        cmd.velocity.z = data["linear_velocity"].z

        # Acceleration
        cmd.acceleration.x = data["linear_acceleration"].x
        cmd.acceleration.y = data["linear_acceleration"].y
        cmd.acceleration.z = data["linear_acceleration"].z

        # Yaw Rate
        cmd.yaw_rate = data["angular_velocity"].z

        return cmd

    def create_thrust_attitude_control_msg(self, data):
        cmd = ThrustAttitudeControl()
        cmd.thrust = self.calculate_thrust(data)

        # Convert Euler angles to desired attitude representation (e.g., degrees)
        cmd.attitude.x = math.degrees(data["orientation"]["roll"])
        cmd.attitude.y = math.degrees(data["orientation"]["pitch"])
        cmd.attitude.z = math.degrees(data["orientation"]["yaw"])

        # Attitude rates
        cmd.attitude_rates.x = data["angular_velocity"].x
        cmd.attitude_rates.y = data["angular_velocity"].y
        cmd.attitude_rates.z = data["angular_velocity"].z

        return cmd

    def create_thrust_velocity_control_msg(self, data):
        cmd = ThrustVelocityControl()
        cmd.thrust = self.calculate_thrust(data)

        # Velocity
        cmd.velocity.x = data["linear_velocity"].x
        cmd.velocity.y = data["linear_velocity"].y
        cmd.velocity.z = data["linear_velocity"].z

        # Acceleration
        cmd.acceleration.x = data["linear_acceleration"].x
        cmd.acceleration.y = data["linear_acceleration"].y
        cmd.acceleration.z = data["linear_acceleration"].z

        return cmd

    def create_thrust_torque_control_msg(self, data):
        cmd = ThrustTorqueControl()
        cmd.thrust = self.calculate_thrust(data)

        # Torque
        cmd.torque.x = data["angular_acceleration"].x
        cmd.torque.y = data["angular_acceleration"].y
        cmd.torque.z = data["angular_acceleration"].z

        return cmd

    def calculate_thrust(self, data):
        # Placeholder calculation for thrust (should be replaced with actual computation)
        # For example, thrust could be proportional to desired acceleration along z-axis
        mass = rospy.get_param("~drone_mass", 1.0)  # Get drone mass from parameter
        gravity = 9.81  # Gravitational acceleration

        # Calculate thrust required to achieve the desired acceleration plus gravity
        thrust = mass * (data["linear_acceleration"].z + gravity)

        return thrust

    def send_zero_command(self):
        rospy.loginfo("Stopping...")
        try:
            cmd = self.msg_type()

            # Set all relevant fields to zero
            if hasattr(cmd, "velocity"):
                cmd.velocity.x = 0
                cmd.velocity.y = 0
                cmd.velocity.z = 0

            if hasattr(cmd, "acceleration"):
                cmd.acceleration.x = 0
                cmd.acceleration.y = 0
                cmd.acceleration.z = 0

            if hasattr(cmd, "yaw_rate"):
                cmd.yaw_rate = 0

            if hasattr(cmd, "yaw"):
                cmd.yaw = 0

            if hasattr(cmd, "attitude"):
                cmd.attitude.x = 0
                cmd.attitude.y = 0
                cmd.attitude.z = 0

            if hasattr(cmd, "attitude_rates"):
                cmd.attitude_rates.x = 0
                cmd.attitude_rates.y = 0
                cmd.attitude_rates.z = 0

            if hasattr(cmd, "thrust"):
                cmd.thrust = 0

            if hasattr(cmd, "torque"):
                cmd.torque.x = 0
                cmd.torque.y = 0
                cmd.torque.z = 0

            self.pub.publish(cmd)
        except Exception as e:
            rospy.logerr(f"Error in send_zero_command: {e}")


if __name__ == "__main__":
    controller = DummyController()
    rospy.spin()
