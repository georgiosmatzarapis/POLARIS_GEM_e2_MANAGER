// author: georgiosmatzarapis

#include "gem_planner/planner.hpp"

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nodeHandle{};

    std::string controllerType{};
    if (!nodeHandle.getParam("/planner_node/controller_type", controllerType)) {
      ROS_ERROR("Missing required parameter 'controller_type'");
      return 1;
    }

    gem_planner::Planner planner{nodeHandle, "wps.csv", controllerType};

    ROS_INFO("Planner node started with controller type: %s",
             controllerType.c_str());

    planner.startPublishing();

    ros::shutdown();
  } catch (const std::exception& exception) {
    ROS_ERROR("Exception: %s", exception.what());
    return 1;
  }

  return 0;
}
