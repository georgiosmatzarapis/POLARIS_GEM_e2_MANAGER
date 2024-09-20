// author: georgiosmatzarapis

#include <fstream>

#include <ros/package.h>

#include "gem_planner/controllers/pure_pursuit.hpp"
#include "gem_planner/controllers/stanley.hpp"
#include "gem_planner/planner.hpp"
#include "ros1_lib/publisher.hpp"
#include "ros1_lib/subscriber.hpp"

namespace gem_planner {

// Waypoint struct

// PUBLIC API

Waypoint::Waypoint(std::string xVal, std::string yVal, std::string yawVal)
    : x{std::move(xVal)},
      y{std::move(yVal)},
      yaw{std::move(yawVal)} {}

// Planner class

// PUBLIC API

Planner::Planner(ros::NodeHandle& nodeHandle, std::string waypointsFile,
                 std::string controllerType)
    : robotStateSubscriber_{std::make_unique<
          ros1_lib::Subscriber<std_msgs::String>>(nodeHandle)},
      waypointPublisher_{
          std::make_unique<ros1_lib::Publisher<gem_manager::WaypointsBatch>>(
              nodeHandle, "gem_manager/waypoints", 10)} {

  robotStateSubscriber_->subscribe(
      "/gem_manager/robot_state", 10,
      [this](const std_msgs::String::ConstPtr& msg) {
        this->robotStateCallback(msg);
      });

  const std::string waypointsFullPath{ros::package::getPath("gem_manager") +
                                      "/gem_planner/waypoints/" +
                                      std::move(waypointsFile)};
  loadWaypoints(waypointsFullPath);

  if (controllerType == "PurePursuit") {
    pathTrackingController_ = std::make_unique<PurePursuit>(waypoints_);
  } else if (controllerType == "Stanley") {
    pathTrackingController_ = std::make_unique<Stanley>(waypoints_);
  } else {
    throw std::logic_error("Unsupported controller type: " + controllerType);
  }

  waypointPublishingRate_ =
      pathTrackingController_->getWaypointPublishingRate();
}

void Planner::startPublishing() {
  while (ros::ok()) {
    publishWaypoints();
    ros::spinOnce();
  }
}

// PRIVATE API

void Planner::robotStateCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "RUNNING") {
    isHealthy_ = true;
  } else if (msg->data == "ERROR") {
    isHealthy_ = false;
    lastErrorTime_ = ros::Time::now();
  }
}

void Planner::loadWaypoints(const std::string& filePath) {
  std::ifstream file{filePath};
  if (!file.is_open()) {
    ROS_ERROR("Error opening file: %s", filePath.c_str());
    return;
  }

  std::string line{};
  while (std::getline(file, line)) {
    std::stringstream lineStream{line};
    std::string field{};
    std::vector<std::string> fields{};

    while (std::getline(lineStream, field, ',')) {
      fields.push_back(field);
    }

    if (fields.size() >= 3) {
      std::string x{fields[0]};
      std::string y{fields[1]};
      std::string yaw{fields[2]};
      waypoints_.emplace_back(x, y, yaw);
    } else {
      ROS_ERROR("Invalid waypoint data");
    }
  }
  file.close();
}

void Planner::publishWaypoints() {
  if (isHealthy_) {
    waypointPublisher_->publish(pathTrackingController_->update());

  } else {
    // Check if the recovery time has elapsed to reset the health status
    if (lastErrorTime_ != ros::Time(0) &&
        ros::Time::now() - lastErrorTime_ > ros::Duration(RECOVERY_TIME)) {
      isHealthy_ = true;
    }
  }

  waypointPublishingRate_.sleep();
}

} // namespace gem_planner
