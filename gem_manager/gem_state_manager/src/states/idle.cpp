// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/states/error.hpp"
#include "gem_state_manager/states/idle.hpp"
#include "gem_state_manager/states/running.hpp"

namespace gem_state_manager {

// PUBLIC API

void IdleState::handleEvent(StateManager& stateManager, Event event,
                            const std::string& data) {
  using enum Event;

  ROS_INFO("IdleState handling event: %s, value: %s",
           eventToString(event).c_str(), data.c_str());

  if (event == BATTERY_LOW || event == TEMPERATURE_HIGH ||
      event == GPS_INACCURATE || event == SIGNAL_LOST || event == SIGNAL_LOW ||
      event == EMERGENCY) {
    stateManager.setState(std::make_unique<ErrorState>());
  } else {
    stateManager.setState(std::make_unique<RunningState>());
  }
}

std::string IdleState::getStateName() const noexcept { return "IDLE"; }

} // namespace gem_state_manager
