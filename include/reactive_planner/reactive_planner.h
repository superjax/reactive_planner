#ifndef REACTIVE_PLANNER_H
#define REACTIVE_PLANNER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>


#include <sensor_msgs/PointCloud.h>
#include <fcu_common/ExtendedCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

namespace reactive_planner
{

class reactivePlanner
{

public:

  reactivePlanner();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber scan_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher command_pub_;

  // Parameters
  double c1_, c2_, c3_, c4_;
  double t1_, t2_, s1_, s2_;
  double k0_, kg_;
  double hover_throttle_;

  // Local Variables
  double dg_;
  Eigen::Vector2d goal_;
  Eigen::Vector2d command_;
  nav_msgs::Odometry current_state_;


  // Topic Callback Functions
  void scanCallback(const sensor_msgs::PointCloudConstPtr &msg);
  void stateCallback(const nav_msgs::Odometry msg);
  void goalCallback(const geometry_msgs::Vector3ConstPtr &msg);

  // Helper Functions
  double sign(double x);
  double sigmoid(double x);
};

} // namespace reactive_planner

#endif // reactivePlanner_H
