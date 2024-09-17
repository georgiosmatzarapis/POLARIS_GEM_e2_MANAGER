// author: georgiosmatzarapis

#pragma once

#include <string>

namespace gem_state_manager {

enum class Event {
  BATTERY_LOW,
  TEMPERATURE_HIGH,
  GPS_INACCURATE,
  SIGNAL_LOST,
  SIGNAL_LOW,
  EMERGENCY
};

inline std::string eventToString(Event event) noexcept {
  using enum Event;

  switch (event) {
    case BATTERY_LOW:
      return "Battery Low";
    case TEMPERATURE_HIGH:
      return "Temperature High";
    case GPS_INACCURATE:
      return "GPS Inaccurate";
    case SIGNAL_LOST:
      return "Signal Lost";
    case SIGNAL_LOW:
      return "Signal Low";
    case EMERGENCY:
      return "Emergency";
    default:
      return "Unknown Event";
  }
}

} // namespace gem_state_manager
