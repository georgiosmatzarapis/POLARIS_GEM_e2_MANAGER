// author: georgiosmatzarapis

#pragma once

#include <mutex>

namespace gem_state_manager {

class CallbackHandlers;

class StateManager final {
  enum class RobotState { IDLE, RUNNING, ERROR };

 public:
  explicit StateManager(CallbackHandlers& callbackHandlers);

  enum class Event {
    BATTERY_LOW,
    TEMPERATURE_HIGH,
    GPS_INACCURATE,
    SIGNAL_LOST,
    SIGNAL_LOW,
    EMERGENCY
  };

  void handleEvent(Event event, const std::string& data);
  [[nodiscard]] RobotState getCurrentState() const noexcept;

 private:
  mutable std::mutex stateMutex_;
  RobotState currentState_{RobotState::IDLE};

  std::string eventToString(Event event) const noexcept;
  void setState(RobotState state) noexcept;
};

} // namespace gem_state_manager
