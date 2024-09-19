// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gem_state_manager/callback_handlers.hpp"
#include "gem_state_manager/state_manager.hpp"
#include "gem_state_manager/states/idle.hpp"
#include "gem_state_manager/states/running.hpp"

namespace gem_state_manager {

// PUBLIC API

StateManager::StateManager(ros::NodeHandle& nodeHandle,
                           CallbackHandlers& callbackHandlers)
    : currentState_{std::make_unique<IdleState>()},
      statePublisher_{nodeHandle.advertise<std_msgs::String>(
          "gem_manager/robot_state", 10)} {
  callbackHandlers.initSubscriptions(*this);
}

void StateManager::checkVehicleAndColdStart() {
  publishState();
  ros::Duration(5.0).sleep();
  if (getCurrentState() == "IDLE") {
    setState(std::make_unique<RunningState>());
    publishState();
  }
}

void StateManager::handleEvent(Event event, const std::string& data) {
  ROS_INFO("[StateManager] Handling event in state: %s",
           currentState_->getStateName().c_str());
  currentState_->handleEvent(*this, event, data);
  publishState();
}

void StateManager::setState(
    std::unique_ptr<RobotState> newRobotState) noexcept {
  std::lock_guard<std::mutex> lock(stateMutex_);
  currentState_ = std::move(newRobotState);
}

std::string StateManager::getCurrentState() const noexcept {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return currentState_->getStateName();
}

// PRIVATE API

void StateManager::publishState() const {
  std_msgs::String stateMsg;
  stateMsg.data = getCurrentState();
  statePublisher_.publish(stateMsg);
}

} // namespace gem_state_manager
