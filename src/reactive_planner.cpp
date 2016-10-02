#include "reactive_planner/reactive_planner.h"

#include <math.h>
#include <tf/tf.h>
#include <stdio.h>

using namespace std;

namespace reactive_planner
{

reactivePlanner::reactivePlanner() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  nh_private_.param<double>("c11", c11_, 5.0);
  nh_private_.param<double>("c12", c12_, 0.1);
  nh_private_.param<double>("c21", c21_, 5.0);
  nh_private_.param<double>("c22", c22_, 5.0);
  nh_private_.param<double>("c3", c3_, 0.1);
  nh_private_.param<double>("c4", c4_, 0.1);
  nh_private_.param<double>("t1", t1_, 0.1);
  nh_private_.param<double>("t2", t2_, 0.1);
  nh_private_.param<double>("s1", s1_, 0.1);
  nh_private_.param<double>("s2", s2_, 0.1);
  nh_private_.param<double>("ko_theta", ko_theta_, 0.01);
  nh_private_.param<double>("ko_theta", ko_phi_, 0.1);
  nh_private_.param<double>("kg_theta", kg_theta_, 50.0);
  nh_private_.param<double>("kg_phi", kg_phi_, 0.1  );
  nh_private_.param<double>("hover_throttle", hover_throttle_, 0.34);
  nh_private_.param<double>("max_force", max_force_, 60.0);
  nh_private_.param<double>("mass", mass_, 3.81);
  nh_private_.param<double>("k_speed", k_speed_, 0.05);
  nh_private_.param<double>("horizontal_search_range", horizontal_search_range_, 10*M_PI/180.0);
  nh_private_.param<double>("vertical_search_range", vertical_search_range_, 10*M_PI/180.0);
  nh_private_.param<double>("laser_range", laser_range_, 40.0);


  // Setup publishers and subscribers
  scan_sub_ = nh_.subscribe("scan", 1, &reactivePlanner::scanCallback, this);
  state_sub_ = nh_.subscribe("shredder/ground_truth/odometry", 1, &reactivePlanner::stateCallback, this);
  goal_sub_ = nh_.subscribe("waypoint", 1, &reactivePlanner::goalCallback, this);

  command_pub_ = nh_.advertise<fcu_common::ExtendedCommand>("high_level_command", 1);

  // Initialize current state
  current_state_.twist.twist.linear.x = 0;
  current_state_.twist.twist.linear.y = 0;
  current_state_.twist.twist.linear.z = 0;

  _func = boost::bind(&reactivePlanner::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);
}

void reactivePlanner::reconfigure_callback(PlannerConfig &config, uint32_t level)
{
  c11_ = config.c11;
  c12_ = config.c12;
  c21_ = config.c21;
  c22_ = config.c22;
  c3_ = config.c3;
  c4_ = config.c4;
  t1_ = config.t1;
  t2_ = config.t2;
  s1_ = config.s1;
  s2_ = config.s2;
  kg_theta_ = config.kg1;
  kg_phi_ = config.kg2;
  ko_theta_ = config.ko1;
  ko_phi_ = config.ko2;
  nominal_speed_ = config.v0;
  k_speed_ = config.k_speed;
  horizontal_search_range_ = config.horizontal_search;
  vertical_search_range_ = config.vertical_search;
  k_strafe_ = config.k_strafe;
  max_strafe_ = config.max_strafe;
  max_yaw_rate_ = config.max_yaw_rate;
}

void reactivePlanner::scanCallback(const sensor_msgs::PointCloudConstPtr &msg)
{
  static double prev_time = 0;

  if(prev_time = 0)
    return;

  double dt = msg->header.stamp.toSec() - prev_time;
  prev_time = msg->header.stamp.toSec();

  // This is an implementation of the obstacle avoidance algorithm given by
  // S. Sherer et. al in "Flying Fast and Low among Obstacles"

  //======================================================================================
  // Attraction Goal Calculation
  //======================================================================================
  // Calculate attraction vector to goal  (Equation 4 of the paper)
  Eigen::Vector2d goal_spherical;
  Eigen::Vector3d goal_error;
  goal_error << goal_.x() - current_state_.pose.pose.position.x,
      goal_.y() - current_state_.pose.pose.position.y,
      goal_.z() - current_state_.pose.pose.position.z;
  tf::Quaternion current_q;
  tf::quaternionMsgToTF(current_state_.pose.pose.orientation, current_q);
  double phi, theta, psi;
  tf::Matrix3x3(current_q).getEulerYPR(psi, theta, phi);

  // calculate the heading error
  double delta_psi = atan2(goal_error.y(), goal_error.x()) - psi;

  // wrap heading error to within -PI to PI
  while(delta_psi > M_PI)
    delta_psi -= 2*M_PI;
  while(delta_psi <= -M_PI)
    delta_psi += 2*M_PI;

  // create the representation of the goal in spherical coordinates
  goal_spherical << delta_psi,
      atan2(goal_error.z(), goal_error.norm());
  Eigen::Vector2d attract;
  attract << kg_theta_*delta_psi*(exp(-c11_*goal_error.norm() + c21_)),
      kg_phi_*goal_spherical.y()*(exp(-c12_*goal_error.norm() + c22_));

  // For speed control, figure out minimum turn radius
  // We will use this to determine when we would hit obstacles
  /// TODO: Something is wrong in this calculation.  The 10x on the
  /// min_turn_radius is something I added to make the calculations
  /// accurate
  Eigen::Vector3d current_velocity;
  current_velocity << current_state_.twist.twist.linear.x,
                      current_state_.twist.twist.linear.y,
                      current_state_.twist.twist.linear.z;
  double v = current_velocity.norm();
  double max_bank_angle = sat(acos(mass_*G/max_force_), 50.0*M_PI/180.0);
  double max_acceleration = max_force_*sin(max_bank_angle);
  double min_turn_radius = v*v / max_acceleration;

  // Figure out where in our map our most recent command would take us at a distance of our minimum turn radius
  double time_to_collision = min_turn_radius/v;
  Eigen::Vector3d projected_position = current_velocity*time_to_collision;

  // This point in space will be the focus of our search.
  /// I haven't decided that this is the best way to do this.  I think this way
  /// will work, but I can't prove that it is the best way.
  double theta_to_search = atan2(projected_position.y(), projected_position.x());
  double phi_to_search = atan2(projected_position.z(), min_turn_radius);
  double min_distance = 40;

  //  // Find safest direction, by using eq. 5 from the paper
  Eigen::Vector2d repulse;
  repulse.setZero();
  for(int i = 0; i < msg->points.size(); i++)
  {
    geometry_msgs::Point32 point = msg->points[i];
    double distance_to_point = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    if(distance_to_point <= 40)
    {
      // Pointcloud is in NWU
      double theta0 = atan2(-1.0*point.y, point.x); // Azimuth of point
      double d0 = sqrt(point.x*point.x + point.y*point.y); // Cartesian Distance to point in projected onto horizontal plane
      double phi0 = atan2(-1.0*point.z, d0); // Elevation of point (positive is down)

      // Sum up all the repulsion terms from the scan (This is equation 5 of the paper)
      Eigen::Vector2d repulsion_term;
      repulsion_term << -ko_theta_ * sign(theta0)*sigmoid(s1_*(1.0 - std::fabs(phi0)/s2_)) * exp(-c3_*d0) * exp(-c4_*std::fabs(theta0)),
          -ko_phi_* sign(phi0)*sigmoid(s1_*(1.0 - std::fabs(theta0)/s2_)) * exp(-c3_*d0) * exp(-c4_*std::fabs(phi0));
      repulse += repulsion_term;

      // record the minimum distance in the range around our projected control
      if( fabs(theta0 - theta_to_search) < horizontal_search_range_
          && fabs(phi0 - phi_to_search) < vertical_search_range_)
      {
        min_distance = (d0 < min_distance) ? d0 : min_distance;
      }
    }
  }

  // Slow down as we approach the goal, so we can actually hit it
  if(goal_error.norm() < min_distance)
    min_distance = goal_error.norm();

  // calculate the speed we should go so we don't hit that thing.
  double desired_velocity = k_speed_*min_distance/time_to_collision;

  Eigen::Vector2d dodger_output = attract + repulse;

  // Pack Up and Send Command
  fcu_common::ExtendedCommand cmd;
  cmd.mode = fcu_common::ExtendedCommand::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  cmd.F = goal_.z();
  cmd.x = sat(desired_velocity, nominal_speed_);
  cmd.y = sat(k_strafe_*dodger_output.x(), max_strafe_);
  cmd.z = sat(dodger_output.x(), max_yaw_rate_);

  // Find Resulting Command from Dodger  (Equation 8 of the paper)
  cout << "r_t = " << repulse.x() << "\ta_t = " << attract.x() << "\tdpsi = " << delta_psi << endl;
  cout << "min_dist = " << min_distance << " out_speed = " << cmd.x << " t = " << time_to_collision << "unsat_speed = " << desired_velocity << "  \n\n";

  command_pub_.publish(cmd);


}

void reactivePlanner::stateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
}

void reactivePlanner::goalCallback(const fcu_common::ExtendedCommandConstPtr &msg)
{
  ROS_ASSERT(msg->mode == fcu_common::ExtendedCommand::MODE_XPOS_YPOS_YAW_ALTITUDE);
  goal_.x()= msg->x;
  goal_.y() = msg->y;
  goal_.z() = msg->F;
}

double reactivePlanner::sign(double x)
{
  return (x > 0) - (x < 0);
}

double reactivePlanner::sigmoid(double x)
{
  1.0/(1.0 + exp(-1.0*x));
}

double reactivePlanner::sat(double x, double max)
{
  x = (x > max) ? max : x;
  x = (x < -1.0*max) ? -1.0*max : x;
  return x;
}


} // namespace reactive_planner


