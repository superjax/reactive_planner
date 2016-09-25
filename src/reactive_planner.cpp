#include "reactive_planner/reactive_planner.h"

#include <math.h>

using namespace std;

namespace reactive_planner
{

reactivePlanner::reactivePlanner() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  nh_private_.param<double>("c1", c1_, 0.1);
  nh_private_.param<double>("c2", c2_, 0.1);
  nh_private_.param<double>("c3", c3_, 0.1);
  nh_private_.param<double>("c4", c4_, 0.1);
  nh_private_.param<double>("t1", t1_, 0.1);
  nh_private_.param<double>("t2", t2_, 0.1);
  nh_private_.param<double>("s1", s1_, 0.1);
  nh_private_.param<double>("s2", s2_, 0.1);
  nh_private_.param<double>("k0", k0_, 0.1);
  nh_private_.param<double>("kg", kg_, 0.1);
  nh_private_.param<double>("hover_throttle", hover_throttle_, 0.34);
  nh_private_.param<double>("max_force", max_force_, 109.86);
  nh_private_.param<double>("mass", mass_, 3.81);
  nh_private_.param<double>("k_speed", k_speed_, 0.05);
  nh_private_.param<double>("search_range", search_range_, 30*M_PI/180.0);

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

  // For speed control, figure out minimum turn radius
  // We will use this to determine when we would hit obstacles
  double max_bank_angle = acos(mass_*G/max_force_);
  double max_acceleration = max_force_*sin(max_bank_angle);
  double current_forward_velocity = current_state_.twist.twist.linear.x;
  double min_turn_radius = current_forward_velocity*current_forward_velocity / max_acceleration;

  // Figure out where in our map our most recent command would take us at a distance of our minimum turn radius
  Eigen::Vector3d current_velocity;
  current_velocity << current_state_.twist.twist.linear.x,
      current_state_.twist.twist.linear.y,
      current_state_.twist.twist.linear.z;
  double v = current_velocity.norm();
  double a = desired_acceleration_.norm(); // This assumes that the direction we take this time is similar to last time
  // But allows us to avoid doing two loops over the pointcloud

  // 1/2 a*t^2 + v*t = d <-- solve for t, then use to find final position
  double t = (-v + sqrt(v*v - 4*(0.5*a)*-min_turn_radius))/a; // Quadratic Equation (-b + sqrt(b^2 - 4ac))/2a
  Eigen::Vector3d projected_position = 1/2.0*desired_acceleration_*t*t + current_velocity*t;

  // This point in space will be the focus of our search.
  /// I haven't decided that this is the best way to do this.  I think this way
  /// will work, but I can't prove that it is the best way.
  double theta_to_search = atan2(projected_position.y(), projected_position.x());
  double phi_to_search = atan2(projected_position.z(), min_turn_radius);
  static double min_distance = 10000;

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

    // Sum up all the repulsion terms from the scan (This is equation 5 of the paper)
    Eigen::Vector2d repulsion_term;
    repulsion_term << -k0_ * sign(theta0)*sigmoid(s1_*(1.0 - std::fabs(phi0)/s2_)) * exp(-c3_*d0) * exp(-c4_*std::fabs(theta0)),
                      -k0_ * sign(phi0)*sigmoid(s1_*(1.0 - std::fabs(theta0)/s2_)) * exp(-c3_*d0) * exp(-c4_*std::fabs(phi0));
    repulse += repulsion_term;

    // record the minimum distance in the range around our projected control
    if(    fabs(theta0 - theta_to_search) < search_range_
        && fabs(phi0 - phi_to_search) < search_range_)
    {

      min_distance = (d0 < min_distance) ? d0 : min_distance;
    }
  }

  // Calculate attraction vector to goal  (Equation 4 of the paper)
  Eigen::Vector2d goal_spherical;
  goal_spherical << atan2(goal_.y(), goal_.z()),
                    atan2(goal_.z(), goal_.norm());
  Eigen::Vector2d attract = kg_*goal_spherical*(exp(-c1_*goal_.norm() + c2_));
  // Find Resulting Command from Dodger  (Equation 8 of the paper)
  Eigen::Vector2d dodger_output = attract + repulse;

  // Figure out how much we should accelerate in x (out the nose of the MAV)
  /// This is a hard problem.
  /// I can't decide on a better way to figure out how to slow down and
  /// speed up. It seems to me that there is a better way, but this will probably work
  /// for now.  This method also abstracts unit away by use of the gain k_speed_, which
  /// is handy
  double ax_desired = sat(k_speed_*(min_distance - min_turn_radius), max_acceleration);

  desired_acceleration_ << ax_desired,
                           dodger_output(0),
                           dodger_output(1);

  // Convert desired accelerations into roll, pitch, and throttle commands
  // This is a Model inversion technique (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
  double total_acc_c = sqrt((1.0-desired_acceleration_.z())*(1.0-desired_acceleration_.z())
                            + desired_acceleration_.x()*desired_acceleration_.x()
                            + desired_acceleration_.y()*desired_acceleration_.y()); // (in g's)
  double thrust = total_acc_c*hover_throttle_; // calculate the total thrust in normalized units
  double phi_c = asin(desired_acceleration_.y() / total_acc_c);
  double theta_c = -1.0*asin(desired_acceleration_.x() / total_acc_c);

  // Determine a yaw rate command, which keeps us heading in the direction we will be going
  // at a distance of our minimum turning radius
  Eigen::Vector3d projected_velocity = desired_acceleration_*t;
  double delta_psi = atan2(projected_velocity.y(), projected_velocity.x());
  double yaw_rate = delta_psi/t;

  // Pack up command and send it!
  fcu_common::ExtendedCommand output_command;
  output_command.mode = fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  output_command.x = phi_c;
  output_command.y = theta_c;
  output_command.z = yaw_rate;
  output_command.F = thrust;
  command_pub_.publish(output_command);
}

void reactivePlanner::stateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
}

void reactivePlanner::goalCallback(const geometry_msgs::Vector3ConstPtr &msg)
{
  goal_.x()= msg->x;
  goal_.y() = msg->y;
  goal_.z() = msg->z;
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
  x = (x > max) ? x : max;
  x = (x < max) ? x : -1.0*max;
  return x;
}


} // namespace reactive_planner
