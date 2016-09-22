#include "reactive_planner/reactive_planner.h"

#include <math.h>

namespace reactive_planner
{

reactivePlanner::reactivePlanner() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
//  nh_private_.param<double>("param", param_, 0);


  // Setup publishers and subscribers
  scan_sub_ = nh_.subscribe("scan", 1, &reactivePlanner::scanCallback, this);
  scan_sub_ = nh_.subscribe("state", 1, &reactivePlanner::stateCallback, this);
  scan_sub_ = nh_.subscribe("goal", 1, &reactivePlanner::goalCallback, this);

  command_pub_ = nh_.advertise<fcu_common::ExtendedCommand>("extended_command", 1);

}

void reactivePlanner::scanCallback(const sensor_msgs::PointCloudConstPtr &msg)
{
  // This is an implementation of the obstacle avoidance algorithm given by
  // S. Sherer et. al in "Flying Fast and Low among Obstacles"

  // Find safest direction, by using eq. 5 from the paper
  Eigen::Vector2d repulse;
  repulse.setZero();

  for(int i = 0; i < msg->points.size(); i++)
  {
    geometry_msgs::Point32 point = msg->points[i];

    /// TODO: Check coordinate frame of pointcloud.
    /// This assumes the same body-fixed axes as a multirotor
    /// (NED - Note, phi here is positive down!!!)
    double theta0 = atan2(point.y, point.x); // Azimuth of point
    double d0 = sqrt(point.x*point.x + point.y*point.y); // Cartesian Distance to point in projected onto horizontal plane
    double phi0 = atan2(point.z, d0); // Elevation of point

    Eigen::Vector2d term1, term2;
    term1 << sign(theta0)*sigmoid(s1_*(1.0 - std::fabs(phi0)/s2_)),
             sign(phi0)*sigmoid(t1_*(1.0 - std::fabs(theta0)/t2_));
    term2 << exp(-c4_*std::fabs(theta0)),
             exp(-c4_*std::fabs(phi0));

    // Sum up all the repulsion terms from the scan
    repulse += -k0_*term1 * exp(-c3_*d0) * term2;
  }

  // Calculate vector to goal
  Eigen::Vector2d attract = kg_*goal_*(exp(-c1_*dg_ + c2_));

  // Find Resulting Command
  command_ = attract + repulse;

  // Convert command into an actual extendedCommand message



}

void reactivePlanner::stateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
}

void reactivePlanner::goalCallback(const geometry_msgs::Vector3ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
}

double reactivePlanner::sign(double x)
{
  return (x > 0) - (x < 0);
}

double reactivePlanner::sigmoid(double x)
{
  1.0/(1.0 + exp(-1.0*x));
}



} // namespace reactive_planner
