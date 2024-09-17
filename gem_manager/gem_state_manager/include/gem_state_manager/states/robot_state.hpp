// author: georgiosmatzarapis

#pragma once

#include "gem_state_manager/events.hpp"

namespace gem_state_manager {

class StateManager;

class RobotState {
 public:
  virtual void handleEvent(StateManager& stateManager, Event event,
                           const std::string& data) = 0;
  [[nodiscard]] virtual std::string getStateName() const noexcept = 0;
  virtual ~RobotState() = default;
};

} // namespace gem_state_manager
