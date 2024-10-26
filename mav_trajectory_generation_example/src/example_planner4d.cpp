// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
// MAV Trajectory Generator
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
// JSON: sudo apt-get install nlohmann-json3-dev
#include <nlohmann/json.hpp>
// C++
#include <fstream>
#include <iostream>

#define R2D(x) ((x)*(180.0/M_PI))
#define D2R(x) ((x)*(M_PI/180.0))

class MyNode
{
public:
  MyNode() : 
  nh_("~"), 
  max_linear_velocity_(1.0),
  max_linear_acceleration_(1.0),
  max_angular_velocity_(M_PI_2),
  max_angular_acceleration_(M_PI_2),
  derivative_to_optimize_(mav_trajectory_generation::derivative_order::SNAP)
  {
    waypoints_file_path_sub = nh_.subscribe("waypoints_file_path", 10, &MyNode::waypointsFileNameCallback, this);
    waypoints_sub_ = nh_.subscribe("waypoints_in", 10, &MyNode::waypointsCallback, this);
    waypoints_pub_ = nh_.advertise<nav_msgs::Path>("waypoints_out", 10, true);
    trajectory_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory4d", 10, true);
    trajectory_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory4d_markers", 10, true);
  }

  /**
   * @brief Plan path from the path topic messages.
   * 
   * @param msg 
   */
  void waypointsCallback(const nav_msgs::Path::ConstPtr & msg)
  {
    this->waypoints_ = *msg;

    if (waypoints_pub_){
      waypoints_pub_.publish(this->waypoints_);
    }

    if (generatePlan(this->waypoints_, this->trajectory_)){
      publishTrajectory(this->trajectory_, 1.0);
    }
    else{
      std::cout << "Failed to generate plan." << std::endl;
    }
  }

  /**
   * @brief Plan path from the path file name topic messages.
   * 
   * @param msg 
   */
  void waypointsFileNameCallback(const std_msgs::String::ConstPtr& msg) {
      // Get the file path from the incoming message
      std::string file_path = msg->data;

      // Create a nav_msgs::Path message to hold the waypoints
      nav_msgs::Path path_msg;

      // Try to load the waypoints from the given JSON file
      if (loadPathFromJSONFile(file_path, path_msg)) {
          this->waypoints_ = path_msg;
          // If successful, publish the path message
          ROS_INFO("Successfully loaded waypoints from file: %s", file_path.c_str());
          
          if (waypoints_pub_)
            waypoints_pub_.publish(this->waypoints_);
          
          if (generatePlan(this->waypoints_, this->trajectory_)){
            publishTrajectory(this->trajectory_, 1.0);
          }
          else{
            std::cout << "Failed to generate plan." << std::endl;
          }

      } else {
          // If loading fails, print an error message
          ROS_ERROR("Failed to load waypoints from file: %s", file_path.c_str());
      }
  }

  // Getters/Setters
  void setMaxLinearVel(double vel) { this->max_linear_velocity_ = vel; };
  double getMaxLinearVel() { return this->max_linear_velocity_; };
  void setMaxLinearAccel(double accel) { this->max_linear_acceleration_ = accel; };
  double getMaxLinearAccel() { return this->max_linear_acceleration_; };
  void setMaxAngularVel(double vel) { this->max_angular_velocity_ = vel; };
  double getMaxAngularVel() { return this->max_angular_velocity_; };
  void setMaxAngularAccel(double accel) { this->max_angular_acceleration_ = accel; };
  double getMaxAngularAccel() { return this->max_angular_acceleration_; };
  void setDerivativeToOptimize(int val) { this->derivative_to_optimize_ = val; };
  int getDerivativeToOptimize() { return this->derivative_to_optimize_; };

private:
  // Private variables
  static const int DIM = 4;   // Define the dimension of the trajectory (3/4D).
  static const int N6 = 6;    // Define the order for polynomial optimization for boustrophedon planner.
  static const int N10 = 10;  // Define the order for polynomial optimization for relaxed planner.
  double max_linear_velocity_, max_angular_velocity_;
  double max_linear_acceleration_, max_angular_acceleration_;
  int derivative_to_optimize_; // Derivative to optimize

  ros::NodeHandle nh_; // Node handle
  nav_msgs::Path waypoints_; 

  // Input topics
  ros::Subscriber waypoints_sub_;
  ros::Subscriber waypoints_file_path_sub;
  // Ouput topics
  ros::Publisher waypoints_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher trajectory_markers_pub_;
  // Output trajectory
  mav_trajectory_generation::Trajectory trajectory_;

private:
  /**
   * @brief Load Path JSON file.
   * 
   * @param file_path Absolute path for the input json filename with extension.
   * @param path_msg The loaded ROS path message.
   * @return true If message loaded successfully.
   * @return false Otherwise.
   */
  bool loadPathFromJSONFile(const std::string& file_path, nav_msgs::Path& path_msg) {
      std::ifstream file(file_path);
      if (!file.is_open()) {
          std::cout << "Could not open JSON file: " << file_path << std::endl;
          return false;
      }

      nlohmann::json path_json;
      file >> path_json;

      // Load Velocity/Acceleration Information if present
      if (path_json.contains("max_linear_velocity")) {
          this->max_linear_velocity_ = path_json.value("max_linear_velocity", this->max_linear_velocity_);
      }
      if (path_json.contains("max_angular_velocity")) {
          this->max_angular_velocity_ = path_json.value("max_angular_velocity", this->max_angular_velocity_);
      }
      if (path_json.contains("max_linear_acceleration")) {
          this->max_linear_acceleration_ = path_json.value("max_linear_acceleration", this->max_linear_acceleration_);
      }
      if (path_json.contains("max_angular_acceleration")) {
          this->max_angular_acceleration_ = path_json.value("max_angular_acceleration", this->max_angular_acceleration_);
      }

      // Fill the header for the Path message
      if (path_json.contains("header")) {
          const auto& header_json = path_json["header"];
          path_msg.header.frame_id = header_json.value("frame_id", "");
          path_msg.header.stamp.sec = header_json.value("stamp", nlohmann::json::object()).value("sec", 0);
          path_msg.header.stamp.nsec = header_json.value("stamp", nlohmann::json::object()).value("nanosec", 0);
      }

      // Iterate over the poses in the JSON
      if (path_json.contains("poses") && path_json["poses"].is_array()) {
          for (const auto& pose_json : path_json["poses"]) {
              geometry_msgs::PoseStamped pose_stamped;

              // Fill the header for each PoseStamped
              if (pose_json.contains("header")) {
                  const auto& pose_header_json = pose_json["header"];
                  pose_stamped.header.frame_id = pose_header_json.value("frame_id", "");
                  pose_stamped.header.stamp.sec = pose_header_json.value("stamp", nlohmann::json::object()).value("sec", 0);
                  pose_stamped.header.stamp.nsec = pose_header_json.value("stamp", nlohmann::json::object()).value("nanosec", 0);
              }

              // Fill the pose information
              if (pose_json.contains("pose")) {
                  const auto& pose_data = pose_json["pose"];
                  if (pose_data.contains("position")) {
                      pose_stamped.pose.position.x = pose_data["position"].value("x", 0.0);
                      pose_stamped.pose.position.y = pose_data["position"].value("y", 0.0);
                      pose_stamped.pose.position.z = pose_data["position"].value("z", 0.0);
                  }

                  if (pose_data.contains("orientation")) {
                      pose_stamped.pose.orientation.x = pose_data["orientation"].value("x", 0.0);
                      pose_stamped.pose.orientation.y = pose_data["orientation"].value("y", 0.0);
                      pose_stamped.pose.orientation.z = pose_data["orientation"].value("z", 0.0);
                      pose_stamped.pose.orientation.w = pose_data["orientation"].value("w", 1.0);
                  }
              }

              // Append the pose to the path message
              path_msg.poses.push_back(pose_stamped);
          }
      }

      return true;
  }

  /**
   * @brief Generate the MAV trajectory given set of waypoints.
   * 
   * @param waypoints input waypoints.
   * @param trajectory output trajectory.
   * @return true if successful,
   * @return false otherwise
   */
  bool generatePlan(const nav_msgs::Path& waypoints,
                    mav_trajectory_generation::Trajectory& trajectory)
  {    

    if (waypoints.poses.size() < 2)
    {
      std::cout << "Input path contains less than 2 points. " << std::endl;
      return false;
    }

    mav_trajectory_generation::Vertex::Vector waypoint_vertices;  // List of vertices for trajectory generation.

    if (!waypoints.poses.empty())
    {
      auto&& first_wp = waypoints.poses.front();
      mav_trajectory_generation::Vertex first_vtx(DIM);
      first_vtx.makeStartOrEnd(
          Eigen::Vector4d(first_wp.pose.position.x, 
                          first_wp.pose.position.y, 
                          first_wp.pose.position.z, 
                          0), 
                          derivative_to_optimize_);
      first_vtx.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 
                              Eigen::Vector4d(0.1, 0, 0, 0));
      waypoint_vertices.push_back(first_vtx);

      for (auto it = std::next(waypoints.poses.begin()); it != std::prev(waypoints.poses.end()); ++it)
      {
        auto&& wp = *it;
        mav_trajectory_generation::Vertex vtx(DIM);
        vtx.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                          Eigen::Vector4d(wp.pose.position.x, 
                                          wp.pose.position.y, 
                                          wp.pose.position.z, 
                                          0));
        waypoint_vertices.push_back(vtx);
      }

      auto&& last_wp = waypoints.poses.back();
      mav_trajectory_generation::Vertex last_vtx(DIM);
      last_vtx.makeStartOrEnd(
          Eigen::Vector4d(last_wp.pose.position.x, 
                          last_wp.pose.position.y, 
                          last_wp.pose.position.z, 
                          0),
                          derivative_to_optimize_);
      waypoint_vertices.push_back(last_vtx);
    }

    // List to estimate initial waypoints segment times.
    std::vector<double> segment_times;  
    segment_times = mav_trajectory_generation::estimateSegmentTimes(waypoint_vertices, 
                                                                    max_linear_velocity_,
                                                                    max_linear_acceleration_);

    mav_trajectory_generation::NonlinearOptimizationParameters optimizerParameters;
    optimizerParameters.max_iterations = 1000;
    optimizerParameters.f_rel = 0.05;
    optimizerParameters.x_rel = 0.1;
    optimizerParameters.time_penalty = 500.0;
    optimizerParameters.initial_stepsize_rel = 0.1;
    optimizerParameters.inequality_constraint_tolerance = 0.1;
    optimizerParameters.print_debug_info = true;
    optimizerParameters.print_debug_info_time_allocation = true;
    optimizerParameters.use_soft_constraints = true;
    optimizerParameters.soft_constraint_weight = 100;

    /*  SETUP OPTIMIZER  */
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N10> optimizer(DIM, optimizerParameters);
    optimizer.setupFromVertices(waypoint_vertices, 
                                segment_times, 
                                derivative_to_optimize_);
    // optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION,
    // DEG2RAD(max_angular_acceleration_));
    // optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY,
    // DEG2RAD(max_angular_velocity_));
    optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_linear_velocity_);
    optimizer.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,max_linear_acceleration_);

    auto startTime = ros::Time::now();  // Start the timer
    optimizer.optimize();
    auto endTime = ros::Time::now();  // Stop the timer
    auto elapsedTime = (endTime - startTime).toSec();
    trajectory.clear();
    optimizer.getTrajectory(&trajectory);
    // trajectory.scaleSegmentTimesToMeetConstraints(max_linear_velocity_, max_linear_acceleration_);
    auto traj_length = getTrajectoryLength(trajectory, 0.5);
    std::cout << "Optimization took " << elapsedTime << " seconds" << std::endl;
    std::cout << "Derivative to optimize: " << this->derivative_to_optimize_ << std::endl;
    std::cout << "Max linear velocity: " << this->max_linear_velocity_ << "m/sec" << std::endl;
    std::cout << "Max angular velocity: " << R2D(this->max_angular_velocity_) << "deg/sec" << std::endl;
    std::cout << "Max linear acceleration: " << this->max_linear_acceleration_ << "m/sec^2" << std::endl;
    std::cout << "Max angular acceleration: " << R2D(this->max_angular_acceleration_) << "deg/sec^2" << std::endl;
    std::cout << "Trajectory execution time: " << trajectory.getMaxTime() << " seconds" << std::endl;
    std::cout << "Trajectory segments:" << trajectory.segments().size() << std::endl;
    std::cout << "Trajectory length: " << traj_length << " meters" << std::endl;
    if (trajectory.empty())
    {
      return false;
    }

    return true;
  }

  /**
   * @brief Publish the specified trajectory for RViz visulization.
   * 
   * @param trajectory 
   * @param distance Distance by which to seperate additional markers. Set 0.0 to disable.
   * @param frame_id Published frame_id
   */
  void publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory,
                         double distance = 1.0,
                         std::string frame_id = "world")
  {
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;

    mav_trajectory_generation::drawMavTrajectory(trajectory, 
                                                 distance, 
                                                 frame_id, 
                                                 &markers);
    trajectory_markers_pub_.publish(markers);

    // Publish trajectory to be executed
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, 
                                                                   &msg);
    msg.header.frame_id = frame_id;
    trajectory_pub_.publish(msg);
  }

  /**
   * @brief Function to visualize a given path in RViz
   * 
   * @param path_msg 
   */
  void visualizePath(const nav_msgs::Path& path_msg) {
      if (waypoints_pub_) {
          waypoints_pub_.publish(path_msg);
          ROS_INFO("Path published for RViz visualization.");
      } else {
          ROS_ERROR("Path publisher is not initialized.");
      }
  }

  double getTrajectoryLength(const mav_trajectory_generation::Trajectory& trajectory, 
                             double dt, bool verbose=false)
  {
    double trajectory_length = 0.0;
    // Integrate velocities to get the trajectory length
    for (double t = 0.0; t <= trajectory.getMaxTime(); t += dt)
    {
      auto point = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
      auto velocity = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
      auto acceleration = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);

      trajectory_length += std::hypot(velocity[0], velocity[1]) * dt;  // Assuming a constant step size

      if (verbose){
        // Print debug information
        std::cout << std::fixed 
                  << std::showpos 
                  << std::setprecision(2) 
                  << std::setw(3) 
                  << std::setfill(' ') 
                  << "T: " << t << "\t"
                  << "L: " << trajectory_length << "\t"
                  << "V: " << std::hypot(velocity[0], velocity[1]) << "\t"
                  << "A: " << std::hypot(acceleration[0], acceleration[1]) 
                  << std::endl;
      }
    }

    return trajectory_length;
  }

};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_planner4d");
  auto example_evaluate_plan = MyNode();
  ros::spin();
  return 0;
}
