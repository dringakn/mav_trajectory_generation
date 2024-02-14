#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PURSUIT_CONTROLLER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PURSUIT_CONTROLLER_H

#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include <mav_trajectory_generation/trajectory.h>
#include <tf/tf.h>

class PursuitController
{
public:
  PursuitController() : lookahead_distance_(0.5), max_angular_velocity_(0.35), kp_position_(1.0)
  {
  }

  geometry_msgs::Twist calculateControl(const nav_msgs::Odometry& current_pose,
                                        const mav_trajectory_generation::Trajectory& trajectory)
  {
    geometry_msgs::Twist control_cmd;

    // Find the closest point on the trajectory
    double closest_distance = std::numeric_limits<double>::max();
    double closest_point_time = 0.0;

    for (double t = 0.0; t <= trajectory.getMaxTime(); t += 0.1)  // Adjust the step size as needed
    {
      auto point = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);

      double dx = point[0] - current_pose.pose.pose.position.x;
      double dy = point[1] - current_pose.pose.pose.position.y;
      double distance = std::hypot(dx, dy);

      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest_point_time = t;
      }
    }

    // Check if the closest point is ahead of the robot
    if (closest_point_time >= 0)
    {
      auto closest_point =
          trajectory.evaluate(closest_point_time, mav_trajectory_generation::derivative_order::POSITION);

      // Compute the target velocity at the closest point
      auto target_velocity =
          trajectory.evaluate(closest_point_time, mav_trajectory_generation::derivative_order::VELOCITY);

      double dx = closest_point[0] - current_pose.pose.pose.position.x;
      double dy = closest_point[1] - current_pose.pose.pose.position.y;

      double angle_to_point = atan2(dy, dx);
      double angle_difference = angle_to_point - tf::getYaw(current_pose.pose.pose.orientation);

      // Adjust angular velocity
      control_cmd.angular.z = std::min(max_angular_velocity_, angle_difference);

      // Adjust linear velocity based on the distance to the closest point and target velocity
      double distance_to_point = std::hypot(dx, dy);
      double lookahead_distance = std::min(lookahead_distance_, distance_to_point);

      // Introduce a proportional control term based on position error
      double position_error = distance_to_point - lookahead_distance;
      control_cmd.linear.x = target_velocity[0] + kp_position_ * position_error;

      // Print debug information
      std::cout << "Debug Info:[Trajectory Total Time: " << trajectory.getMaxTime() << "]"
                << "\n  Closest Distance: " << closest_distance << "\n  Closest Point Time: " << closest_point_time
                << "\n  Closest Point: (" << closest_point[0] << ", " << closest_point[1] << ")"
                << "\n  Target Velocity: (" << target_velocity[0] << ", " << target_velocity[1] << ")"
                << "\n  Current Pose: (" << current_pose.pose.pose.position.x << ", "
                << current_pose.pose.pose.position.y << ")"
                << "\n  Angle to Point: " << angle_to_point << "\n  Angle Difference: " << angle_difference
                << "\n  Linear Velocity: " << control_cmd.linear.x << "\n  Angular Velocity: " << control_cmd.angular.z
                << "\n  Position Error: " << position_error << std::endl;
    }

    return control_cmd;
  }

private:
  double lookahead_distance_;
  double max_linear_velocity_ = 1.0;
  double max_angular_velocity_;
  double kp_position_;
};

#endif  // MAV_TRAJECTORY_GENERATION_EXAMPLE_PURSUIT_CONTROLLER_H