#! /usr/bin/env python3

import math
import rospy

# Different control modes for the drone.
# Select the desired one for publishing.
from uav_gazebo_msgs.msg import PositionYawControl
from uav_gazebo_msgs.msg import VelocityYawRateControl
from uav_gazebo_msgs.msg import ThrustAttitudeControl
from uav_gazebo_msgs.msg import ThrustVelocityControl
from uav_gazebo_msgs.msg import ThrustTorqueControl

# Trajectory messages
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# Command message
cmd = PositionYawControl()  

pub = None

def trajectory_callback(msg):
    # header
    # joint_names
    # trajectory_msgs/MultiDOFJointTrajectoryPoint[] points
    #   geometry_msgs/Transform[] transforms
    #   geometry_msgs/Twist[] velocities
    #   geometry_msgs/Twist[] accelerations
    #   duration time_from_start
    if pub:
        # print(msg.points[0].accelerations[0])

        # Generate control command
        # Position, x, y, z
        cmd.position.x = msg.points[0].transforms[0].translation.x
        cmd.position.y = msg.points[0].transforms[0].translation.y
        cmd.position.z = msg.points[0].transforms[0].translation.z

        # Velocity: vx, vy, vz
        cmd.velocity.x = msg.points[0].velocities[0].linear.x
        cmd.velocity.y = msg.points[0].velocities[0].linear.y
        cmd.velocity.z = msg.points[0].velocities[0].linear.z

        # acceleration: ax, ay, az
        cmd.acceleration.x = msg.points[0].accelerations[0].linear.x
        cmd.acceleration.y = msg.points[0].accelerations[0].linear.y
        cmd.acceleration.z = msg.points[0].accelerations[0].linear.z

        # Yaw/YawRate
        # cmd.yaw = w * t
        # cmd.yaw_rate = w

        # Send drone command
        pub.publish(cmd)


if __name__ == "__main__":

    rospy.init_node("dummy_controller", 
                    anonymous=True)

    # Trajectory subscriber
    sub = rospy.Subscriber("command/trajectory", 
                           MultiDOFJointTrajectory, 
                           trajectory_callback)

    # Drone command publisher
    pub = rospy.Publisher("drone/position_yaw/command", 
                          PositionYawControl, 
                          queue_size=1)

    # Wait for Ctrl+C
    rospy.spin()

    # Send zero velocities
    rospy.loginfo(f"Stopping...")
    cmd.velocity.x = 0
    cmd.velocity.y = 0
    cmd.velocity.z = 0
    cmd.acceleration.x = 0
    cmd.acceleration.y = 0
    cmd.acceleration.z = 0
    cmd.yaw_rate = 0
    pub.publish(cmd)    
