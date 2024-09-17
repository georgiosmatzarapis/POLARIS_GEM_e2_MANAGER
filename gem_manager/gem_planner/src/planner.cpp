// author: georgiosmatzarapis

#include <fstream>

#include <ros/package.h>

#include <gem_manager/Waypoint.h>

#include "gem_planner/planner.hpp"

namespace gem_planner {

// PUBLIC API

Planner::Planner(ros::NodeHandle& nodeHandle, const std::string& waypointsFile)
    : robotStateSubscriber_{nodeHandle.subscribe(
          "/robot_state", 10, &Planner::robotStateCallback, this)},
      waypointPublisher_{
          nodeHandle.advertise<gem_manager::Waypoint>("waypoints_pp", 10)} {
  const std::string waypointsFullPath{ros::package::getPath("gem_manager") +
                                      "/gem_planner/waypoints/" +
                                      waypointsFile};
  loadWaypoints(waypointsFullPath);
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
    std::vector<double> fields{};

    while (std::getline(lineStream, field, ',')) {
      fields.push_back(std::stod(field));
    }

    if (fields.size() >= 3) {
      double x{fields[0]};
      double y{fields[1]};
      double yaw{fields[2]};
      waypoints_.emplace_back(x, y, yaw);
    } else {
      ROS_ERROR("Invalid waypoint data");
    }
  }
  file.close();

  waypointsSize_ = waypoints_.size();
}

void Planner::publishWaypoints() {
  if (isHealthy_) {
    gem_manager::Waypoint waypointMsg;

    waypointMsg.x = waypoints_[currentLocation_].x;
    waypointMsg.y = waypoints_[currentLocation_].y;
    waypointMsg.yaw = waypoints_[currentLocation_].yaw;

    waypointPublisher_.publish(waypointMsg);

    if (currentLocation_ == waypointsSize_ - 1) {
      currentLocation_ = 0;
    } else {
      ++currentLocation_;
    }

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
