// author: georgiosmatzarapis

#pragma once

#include <std_msgs/String.h>

#include "core/publisher.hpp"
#include "gem_state_manager/states/robot_state.hpp"

namespace gem_state_manager {

class CallbackHandlers;

class StateManager final {
 public:
  explicit StateManager(ros::NodeHandle& nodeHandle,
                        CallbackHandlers& callbackHandlers);

  /// @brief Waits for a short duration and then checks the vehicle's state,
  ///        initializing the state to "RUNNING" if the vehicle didn't encounter
  ///        errors.
  void checkVehicleAndColdStart();
  void handleEvent(Event event, const std::string& data);
  void setState(std::unique_ptr<RobotState> newRobotState) noexcept;
  [[nodiscard]] std::string getCurrentState() const noexcept;

 private:
  std::unique_ptr<RobotState> currentState_;
  std::unique_ptr<core::Publisher<std_msgs::String>> statePublisher_;
  mutable std::mutex stateMutex_;

  void publishState() const;
};

} // namespace gem_state_manager
