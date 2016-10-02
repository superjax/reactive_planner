#ifndef REACTIVE_PLANNER_H
#define REACTIVE_PLANNER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>


#include <sensor_msgs/PointCloud.h>
#include <fcu_common/ExtendedCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#include <dynamic_reconfigure/server.h>
#include <reactive_planner/PlannerConfig.h>

#define G 9.80665

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
  double c11_, c21_, c12_, c22_, c3_, c4_;
  double t1_, t2_, s1_, s2_;
  double ko_theta_, ko_phi_, kg_theta_, kg_phi_;
  double hover_throttle_;
  double max_force_;
  double mass_;
  double k_speed_;
  double horizontal_search_range_;
  double vertical_search_range_;
  double nominal_speed_;
  double k_strafe_, max_strafe_;
  double max_yaw_rate_;
  double laser_range_;

  // Local Variables
  Eigen::Vector3d goal_;
  Eigen::Vector3d desired_acceleration_;
  nav_msgs::Odometry current_state_;

  // Topic Callback Functions
  void scanCallback(const sensor_msgs::PointCloudConstPtr &msg);
  void stateCallback(const nav_msgs::Odometry msg);
  void goalCallback(const fcu_common::ExtendedCommandConstPtr &msg);

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<reactive_planner::PlannerConfig> _server;
  dynamic_reconfigure::Server<reactive_planner::PlannerConfig>::CallbackType _func;
  void reconfigure_callback(reactive_planner::PlannerConfig &config, uint32_t level);

  // Helper Functions
  double sign(double x);
  double sigmoid(double x);
  double sat(double x, double max);
};

} // namespace reactive_planner

#endif // reactivePlanner_H

