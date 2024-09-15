// author: georgiosmatzarapis

#pragma once

#include "gem_state_manager/state_manager.hpp"
#include "gem_state_manager/states/robot_state.hpp"

namespace gem_state_manager {

class ErrorState final : public RobotState {
 public:
  ErrorState(const ErrorState&) = delete;
  ErrorState& operator=(const ErrorState&) = delete;
  ErrorState(ErrorState&&) noexcept = delete;
  ErrorState& operator=(ErrorState&&) noexcept = default;
  ErrorState() = default;
  ~ErrorState() override = default;

  void handleEvent(StateManager& stateManager, Event event,
                   const std::string& data) override;
  std::string getStateName() const noexcept override;
};

} // namespace gem_state_manager
