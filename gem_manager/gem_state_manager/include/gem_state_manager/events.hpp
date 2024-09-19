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
  // using enum Event; // C++ 20 onwards

  switch (event) {
    case Event::BATTERY_LOW:
      return "Battery Low";
    case Event::TEMPERATURE_HIGH:
      return "Temperature High";
    case Event::GPS_INACCURATE:
      return "GPS Inaccurate";
    case Event::SIGNAL_LOST:
      return "Signal Lost";
    case Event::SIGNAL_LOW:
      return "Signal Low";
    case Event::EMERGENCY:
      return "Emergency";
    default:
      return "Unknown Event";
  }
}

} // namespace gem_state_manager
