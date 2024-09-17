// author: georgiosmatzarapis

#pragma once

#include "gem_state_manager/state_manager.hpp"
#include "gem_state_manager/states/robot_state.hpp"

namespace gem_state_manager {

class IdleState final : public RobotState {
 public:
  IdleState(const IdleState&) = delete;
  IdleState& operator=(const IdleState&) = delete;
  IdleState(IdleState&&) noexcept = delete;
  IdleState& operator=(IdleState&&) noexcept = default;
  IdleState() = default;
  ~IdleState() override = default;

  void handleEvent(StateManager& stateManager, Event event,
                   const std::string& data) override;
  std::string getStateName() const noexcept override;
};

} // namespace gem_state_manager
