// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/states/error.hpp"
#include "gem_state_manager/states/running.hpp"

namespace gem_state_manager {

// PUBLIC API

void RunningState::handleEvent(StateManager& stateManager, Event event,
                               const std::string& data) {
  // using enum Event; // C++ 20 onwards

  ROS_INFO("RunningState handling event: %s, value: %s",
           eventToString(event).c_str(), data.c_str());

  if (event == Event::BATTERY_LOW || event == Event::TEMPERATURE_HIGH ||
      event == Event::GPS_INACCURATE || event == Event::SIGNAL_LOST ||
      event == Event::SIGNAL_LOW || event == Event::EMERGENCY) {
    stateManager.setState(std::make_unique<ErrorState>());
  }
}

std::string RunningState::getStateName() const noexcept { return "RUNNING"; }

} // namespace gem_state_manager
