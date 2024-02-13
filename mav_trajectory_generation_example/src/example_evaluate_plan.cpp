#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

// Write the planne trajector function.
// Provide it to the planner.
// Publish the trajectory for rviz to display.
// implement the controller to generate V/W for diff robot gazebo simulator.
// evaluate the plan and show the results.
// use teleop to mix the inputs.

class MyNode
{
public:
  MyNode() : nh_("~"), loop_rate_(50)
  {
    odom_sub_ = nh_.subscribe("odometry", 10, &MyNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 10, true);

    initialize_edges();
    polygon_vertices_.push_back(Point{ -10, -10 });
    polygon_vertices_.push_back(Point{ 10, -10 });
    polygon_vertices_.push_back(Point{ 10, 10 });
    polygon_vertices_.push_back(Point{ -10, 10 });
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
    for (uint64_t idx = 0; idx < 20; idx++)
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
    std::cout << "Waypoints:" << path.poses.size() << std::endl;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    // Store the odometry message in the member variable
    odom_msg_ = *odom_msg;
  }

  void publishCmdVel()
  {
    while (ros::ok())
    {
      geometry_msgs::Twist cmd_vel_msg;

      cmd_vel_msg.linear.x = odom_msg_.twist.twist.linear.x;
      cmd_vel_msg.angular.z = odom_msg_.twist.twist.angular.z;

      cmd_vel_pub_.publish(cmd_vel_msg);

      loop_rate_.sleep();
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Path path;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_, path_pub_;

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
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_evaluate_plan");
  auto example_evaluate_plan = MyNode();
  example_evaluate_plan.publishCmdVel();
  ros::shutdown();
  return 0;
}
