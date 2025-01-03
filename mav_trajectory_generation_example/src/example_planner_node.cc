/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);

  Eigen::Vector3d goal_position, goal_velocity, start_position, start_velocity;
  start_position << 0.0, 0.0, 0.0;
  start_velocity << 2.0, 0.0, 0.0;
  goal_position << 0.0, 10.0, 20.0;
  goal_velocity << 0.0, 0.0, 0.0;

  mav_trajectory_generation::Trajectory trajectory;
  // planner.planTrajectory(goal_position, goal_velocity, &trajectory);
  planner.planTrajectory(goal_position, 
                         goal_velocity, 
                         start_position, 
                         start_velocity, 
                         5, 
                         3, 
                         &trajectory);
  planner.publishTrajectory(trajectory);
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}