// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/states/error.hpp"
#include "gem_state_manager/states/idle.hpp"
#include "gem_state_manager/states/running.hpp"

namespace gem_state_manager {

// PUBLIC API

void IdleState::handleEvent(StateManager& stateManager, Event event,
                            const std::string& data) {
  // using enum Event; // C++ 20 onwards

  ROS_INFO("IdleState handling event: %s, value: %s",
           eventToString(event).c_str(), data.c_str());

  if (event == Event::BATTERY_LOW || event == Event::TEMPERATURE_HIGH ||
      event == Event::GPS_INACCURATE || event == Event::SIGNAL_LOST ||
      event == Event::SIGNAL_LOW || event == Event::EMERGENCY) {
    stateManager.setState(std::make_unique<ErrorState>());
  } else {
    stateManager.setState(std::make_unique<RunningState>());
  }
}

std::string IdleState::getStateName() const noexcept { return "IDLE"; }

} // namespace gem_state_manager
