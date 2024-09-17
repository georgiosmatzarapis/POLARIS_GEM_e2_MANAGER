// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/states/error.hpp"

namespace gem_state_manager {

// PUBLIC API

void ErrorState::handleEvent(StateManager& stateManager, Event event,
                             const std::string& data) {
  ROS_INFO("ErrorState handling event: %s, value: %s",
           eventToString(event).c_str(), data.c_str());

  // Not a concrete action for now - can be extended for future scenarios.
}

std::string ErrorState::getStateName() const noexcept { return "ERROR"; }

} // namespace gem_state_manager
