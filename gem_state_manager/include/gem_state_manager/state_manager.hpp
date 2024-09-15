// author: georgiosmatzarapis

#pragma once

#include "gem_state_manager/states/robot_state.hpp"

namespace gem_state_manager {

class CallbackHandlers;

class StateManager final {
 public:
  explicit StateManager(CallbackHandlers& callbackHandlers);

  void handleEvent(Event event, const std::string& data);
  void setState(std::unique_ptr<RobotState> newRobotState) noexcept;
  [[nodiscard]] std::string getCurrentState() const noexcept;

 private:
  mutable std::mutex stateMutex_;
  std::unique_ptr<RobotState> currentState_;
};

} // namespace gem_state_manager
