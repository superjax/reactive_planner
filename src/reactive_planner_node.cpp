#include <ros/ros.h>
#include "reactive_planner/reactive_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reactive_planner_node");
  ros::NodeHandle nh;

  reactive_planner::reactivePlanner Thing;

  ros::spin();

  return 0;
}
