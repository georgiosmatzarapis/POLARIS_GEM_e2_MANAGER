// author: georgiosmatzarapis

#include "gem_planner/planner.hpp"

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nodeHandle{};

    gem_planner::Planner planner{nodeHandle, "wps.csv"};
    planner.startPublishing();

    ROS_INFO("Planner node started");

    ros::shutdown();
  } catch (const std::exception& exception) {
    ROS_ERROR("Exception: %s", exception.what());
    return 1;
  }

  return 0;
}
