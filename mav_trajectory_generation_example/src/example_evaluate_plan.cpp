#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include "pursuit_controller.h"

class MyNode
{
public:
  MyNode() : nh_("~"), loop_rate_(50), pursuit_controller()
  {
    odom_sub_ = nh_.subscribe("odom", 10, &MyNode::odomCallback, this);
    keyboard_sub_ = nh_.subscribe("keyboard_cmd", 10, &MyNode::keyboardControlCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 10, true);
    trajectory_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 10, true);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10, true);

    max_linear_velocity = 1.0;
    max_angular_velocity = 1.0;
    max_linear_acceleration = 1.0;
    max_angular_acceleration = 1.0;
    distance_threshold_ = 0.1;
    
    initialize_edges();
    polygon_vertices_.push_back(Point{ -5, -5 });
    polygon_vertices_.push_back(Point{ 5, -5 });
    polygon_vertices_.push_back(Point{ 5, 5 });
    polygon_vertices_.push_back(Point{ -5, 5 });
    for (size_t i = 0; i < polygon_vertices_.size(); ++i)
    {
      Edge edge;
      edge.start = polygon_vertices_[i];
      edge.end = polygon_vertices_[(i + 1) % polygon_vertices_.size()];
      polygon_edges_.push_back(edge);
    }
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    path.poses.push_back(pose);

    for (uint64_t idx = 0; idx < 10; idx++)
    {
      size_t edge_index = rand() % polygon_edges_.size();
      const Edge& edge = polygon_edges_[edge_index];
      double t = static_cast<double>(rand()) / RAND_MAX;
      pose.pose.position.x = static_cast<double>(edge.start.x + t * (edge.end.x - edge.start.x));
      pose.pose.position.y = static_cast<double>(edge.start.y + t * (edge.end.y - edge.start.y));
      pose.pose.position.z = 0.5;
      path.poses.push_back(pose);
    }
    path_pub_.publish(path);

    if (generate_plan(path))
    {
      publishTrajectory(trajectory);
    }
    else
    {
      std::cout << "Failed to generate plan." << std::endl;
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    odom_msg_ = *odom_msg;
    // std::cout << std::fixed << std::showpos << std::setprecision(2) << std::setw(3) << std::setfill(' ') << "Odom:
    // pos["
    //           << odom_msg_.pose.pose.position.x << ", " << odom_msg_.pose.pose.position.y << ", "
    //           << odom_msg_.pose.pose.position.z << "]"
    //           << ", vel[" << odom_msg_.twist.twist.linear.x << ", " << odom_msg_.twist.twist.linear.y << ", "
    //           << odom_msg_.twist.twist.linear.z << "]" << std::endl;
  }

  void keyboardControlCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    keyboard_msg_ = *msg;
    std::cout << std::fixed << std::showpos << std::setprecision(2) << std::setw(3) << std::setfill(' ')
              << "Keyboard: V[" << keyboard_msg_.linear.x << ", " << keyboard_msg_.linear.y << "], W["
              << keyboard_msg_.angular.z << "]" << std::endl;
  }

  void pursuitController()
  {
    auto start = trajectory.evaluate(trajectory.getMinTime(), mav_trajectory_generation::derivative_order::POSITION);
    goto_position(start[0], start[1]);
    while (ros::ok())
    {
      geometry_msgs::Twist cmd_vel_msg = pursuit_controller.calculateControl(odom_msg_, trajectory);

      // Mix the controls with teleop inputs
      cmd_vel_msg.linear.x += keyboard_msg_.linear.x;
      cmd_vel_msg.angular.z += keyboard_msg_.angular.z;

      cmd_vel_pub_.publish(cmd_vel_msg);

      loop_rate_.sleep();
      ros::spinOnce();
    }
  }

private:
  static const int DIM = 4;   // Define the dimension of the trajectory (3/4D).
  static const int N6 = 6;    // Define the order for polynomial optimization for boustrophedon planner.
  static const int N10 = 10;  // Define the order for polynomial optimization for relaxed planner.

  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist keyboard_msg_;
  nav_msgs::Path path;
  ros::Subscriber odom_sub_, keyboard_sub_;
  ros::Publisher cmd_vel_pub_, path_pub_, trajectory_pub_, markers_pub_;

  mav_trajectory_generation::Trajectory trajectory;             // Output trajectory
  mav_trajectory_generation::Vertex::Vector waypoint_vertices;  // List of vertices for trajectory generation.
  std::vector<double> segment_times;                            // List to estimate initial waypoints segment times.
  double max_linear_velocity, max_angular_velocity, max_linear_acceleration, max_angular_acceleration;
  double distance_threshold_;

  PursuitController pursuit_controller;

  struct Point
  {
    int x;
    int y;
  };

  struct Edge
  {
    Point start;
    Point end;
  };

  std::vector<Point> polygon_vertices_;
  std::vector<Edge> polygon_edges_;

  void initialize_edges()
  {
    polygon_edges_.clear();
    for (size_t i = 0; i < polygon_vertices_.size(); ++i)
    {
      Edge edge;
      edge.start = polygon_vertices_[i];
      edge.end = polygon_vertices_[(i + 1) % polygon_vertices_.size()];
      polygon_edges_.push_back(edge);
    }
  }

  bool generate_plan(const nav_msgs::Path& waypoints)
  {
    if (waypoints.poses.size() < 2)
    {
      std::cout << "Input path contains less than 2 points. " << std::endl;
      return false;
    }

    trajectory.clear();
    waypoint_vertices.clear();
    segment_times.clear();

    if (!waypoints.poses.empty())
    {
      auto&& first_wp = waypoints.poses.front();
      mav_trajectory_generation::Vertex first_vtx(DIM);
      first_vtx.makeStartOrEnd(
          Eigen::Vector4d(first_wp.pose.position.x, first_wp.pose.position.y, first_wp.pose.position.z, 0),
          mav_trajectory_generation::derivative_order::SNAP);
      first_vtx.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(0.1, 0, 0, 0));
      waypoint_vertices.push_back(first_vtx);

      for (auto it = std::next(waypoints.poses.begin()); it != std::prev(waypoints.poses.end()); ++it)
      {
        auto&& wp = *it;
        mav_trajectory_generation::Vertex vtx(DIM);
        vtx.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                          Eigen::Vector4d(wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, 0));
        waypoint_vertices.push_back(vtx);
      }

      auto&& last_wp = waypoints.poses.back();
      mav_trajectory_generation::Vertex last_vtx(DIM);
      last_vtx.makeStartOrEnd(
          Eigen::Vector4d(last_wp.pose.position.x, last_wp.pose.position.y, last_wp.pose.position.z, 0),
          mav_trajectory_generation::derivative_order::SNAP);
      waypoint_vertices.push_back(last_vtx);
    }

    segment_times = mav_trajectory_generation::estimateSegmentTimes(waypoint_vertices, max_linear_velocity,
                                                                    max_linear_acceleration);

    mav_trajectory_generation::NonlinearOptimizationParameters optimizerParameters;
    // optimizerParameters.max_iterations = 1000;
    // optimizerParameters.f_rel = 0.05;
    // optimizerParameters.x_rel = 0.1;
    // optimizerParameters.time_penalty = 500.0;
    // optimizerParameters.initial_stepsize_rel = 0.1;
    // optimizerParameters.inequality_constraint_tolerance = 0.1;
    // optimizerParameters.print_debug_info = true;
    // optimizerParameters.print_debug_info_time_allocation = true;
    // optimizerParameters.use_soft_constraints = true;
    // optimizerParameters.soft_constraint_weight = 100;

    /*  SETUP OPTIMIZER  */
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N10> optimizer(DIM, optimizerParameters);
    optimizer.setupFromVertices(waypoint_vertices, segment_times, mav_trajectory_generation::derivative_order::SNAP);
    // optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION,
    // DEG2RAD(max_angular_acceleration));
    // optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY,
    // DEG2RAD(max_angular_velocity));
    optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_linear_velocity);
    optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                                            max_linear_acceleration);

    auto startTime = ros::Time::now();  // Start the timer
    optimizer.optimize();
    auto endTime = ros::Time::now();  // Stop the timer
    auto elapsedTime = (endTime - startTime).toSec();
    optimizer.getTrajectory(&trajectory);
    // trajectory.scaleSegmentTimesToMeetConstraints(max_linear_velocity, max_linear_acceleration);
    auto traj_length = getTrajectoryLength(trajectory, 0.5);
    std::cout << "Optimization took " << elapsedTime << " seconds" << std::endl;
    std::cout << "Trajectory execution time: " << trajectory.getMaxTime() << " seconds" << std::endl;
    std::cout << "Trajectory segments:" << trajectory.segments().size() << std::endl;
    std::cout << "Trajectory length: " << traj_length << " meters" << std::endl;
    if (trajectory.empty())
    {
      return false;
    }

    return true;
  }

  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory)
  {
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2;  // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "odom";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    markers_pub_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "odom";
    trajectory_pub_.publish(msg);

    return true;
  }

  double getTrajectoryLength(const mav_trajectory_generation::Trajectory& trajectory, double dt)
  {
    double trajectory_length = 0.0;
    // Integrate velocities to get the trajectory length
    for (double t = 0.0; t <= trajectory.getMaxTime(); t += dt)
    {
      auto point = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
      auto velocity = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
      auto acceleration = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);

      trajectory_length += std::hypot(velocity[0], velocity[1]) * dt;  // Assuming a constant step size

      // Print debug information
      std::cout << std::fixed << std::showpos << std::setprecision(2) << std::setw(3) << std::setfill(' ') << "T: " << t
                << "\t"
                << "L: " << trajectory_length << "\t"
                << "V: " << std::hypot(velocity[0], velocity[1]) << "\t"
                << "A: " << std::hypot(acceleration[0], acceleration[1]) << std::endl;
    }

    return trajectory_length;
  }

  void goto_position(double target_position_x, double target_position_y)
  {
    ros::Rate loop_rate(50);

    // PD control gains
    const double k_p_linear = 1.0;
    const double k_d_linear = 0.1;
    const double k_p_angular = 1.0;
    const double k_d_angular = 0.1;

    while (ros::ok())
    {
      // Get the current robot position from the odometry topic
      const double current_x = odom_msg_.pose.pose.position.x;
      const double current_y = odom_msg_.pose.pose.position.y;

      // Calculate the error terms
      const double dx = target_position_x - current_x;
      const double dy = target_position_y - current_y;
      const double distance_to_target = std::hypot(dx, dy);

      // Calculate the angle to the target
      const double angle_to_target = atan2(dy, dx);

      // Calculate the angular error (angle difference)
      const double angular_error = angle_to_target - tf::getYaw(odom_msg_.pose.pose.orientation);

      // PD control for linear velocity
      const double linear_error = distance_to_target;
      const double linear_control = k_p_linear * linear_error - k_d_linear * odom_msg_.twist.twist.linear.x;

      // PD control for angular velocity
      const double angular_control = k_p_angular * angular_error - k_d_angular * odom_msg_.twist.twist.angular.z;

      // Create control command
      geometry_msgs::Twist control_cmd;
      control_cmd.linear.x = std::max(0.0, std::min(max_linear_velocity, linear_control));
      control_cmd.angular.z = std::max(-max_angular_velocity, std::min(max_angular_velocity, angular_control));

      // Publish control command
      cmd_vel_pub_.publish(control_cmd);

      // Break the loop if the robot is close to the target position
      if (distance_to_target < distance_threshold_)
      {
        break;
      }

      loop_rate.sleep();
      ros::spinOnce();
    }

    // Stop the robot when the target position is reached
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_evaluate_plan");
  auto example_evaluate_plan = MyNode();
  example_evaluate_plan.pursuitController();
  ros::shutdown();
  return 0;
}
