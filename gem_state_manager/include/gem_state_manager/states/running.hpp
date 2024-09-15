// author: georgiosmatzarapis

#pragma once

#include "gem_state_manager/state_manager.hpp"
#include "gem_state_manager/states/robot_state.hpp"

namespace gem_state_manager {

class RunningState final : public RobotState {
 public:
  RunningState(const RunningState&) = delete;
  RunningState& operator=(const RunningState&) = delete;
  RunningState(RunningState&&) noexcept = delete;
  RunningState& operator=(RunningState&&) noexcept = default;
  RunningState() = default;
  ~RunningState() override = default;

  void handleEvent(StateManager& stateManager, Event event,
                   const std::string& data) override;
  std::string getStateName() const noexcept override;
};

} // namespace gem_state_manager
