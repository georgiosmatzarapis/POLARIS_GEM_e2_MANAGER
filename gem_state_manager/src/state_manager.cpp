// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/callback_handlers.hpp"
#include "gem_state_manager/state_manager.hpp"
#include "gem_state_manager/states/idle.hpp"

namespace gem_state_manager {

// PUBLIC API

StateManager::StateManager(CallbackHandlers& callbackHandlers)
    : currentState_{std::make_unique<IdleState>()} {
  callbackHandlers.initSubscriptions(*this);
}

void StateManager::handleEvent(Event event, const std::string& data) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  ROS_INFO("[StateManager] Handling event in state: %s",
           currentState_->getStateName().c_str());
  currentState_->handleEvent(*this, event, data);
}

void StateManager::setState(
    std::unique_ptr<RobotState> newRobotState) noexcept {
  currentState_ = std::move(newRobotState);
}

std::string StateManager::getCurrentState() const noexcept {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return currentState_->getStateName();
}

} // namespace gem_state_manager
