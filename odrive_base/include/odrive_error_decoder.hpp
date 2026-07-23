#ifndef ODRIVE_ERROR_DECODER_HPP
#define ODRIVE_ERROR_DECODER_HPP

// Decoders for ODrive bitfield/enum status values into human-readable names.
//
// ODrive.Error is a bitfield: an error value is the bitwise-OR of the individual
// error flags. To decode, check which bits are set. Both `active_errors` and
// `disarm_reason` (from Get_Error, CAN cmd 0x003) use this same enum.
//
// Reference:
//   https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "odrive_enums.h"

namespace odrive_decode {

struct ErrorBit {
  uint32_t mask;
  const char* name;
};

// ODrive.Error bitfield flags (order = ascending bit value).
inline constexpr ErrorBit kOdriveErrorBits[] = {
    {ODRIVE_ERROR_INITIALIZING, "INITIALIZING"},
    {ODRIVE_ERROR_SYSTEM_LEVEL, "SYSTEM_LEVEL"},
    {ODRIVE_ERROR_TIMING_ERROR, "TIMING_ERROR"},
    {ODRIVE_ERROR_MISSING_ESTIMATE, "MISSING_ESTIMATE"},
    {ODRIVE_ERROR_BAD_CONFIG, "BAD_CONFIG"},
    {ODRIVE_ERROR_DRV_FAULT, "DRV_FAULT"},
    {ODRIVE_ERROR_MISSING_INPUT, "MISSING_INPUT"},
    {ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE, "DC_BUS_OVER_VOLTAGE"},
    {ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE, "DC_BUS_UNDER_VOLTAGE"},
    {ODRIVE_ERROR_DC_BUS_OVER_CURRENT, "DC_BUS_OVER_CURRENT"},
    {ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC_BUS_OVER_REGEN_CURRENT"},
    {ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION, "CURRENT_LIMIT_VIOLATION"},
    {ODRIVE_ERROR_MOTOR_OVER_TEMP, "MOTOR_OVER_TEMP"},
    {ODRIVE_ERROR_INVERTER_OVER_TEMP, "INVERTER_OVER_TEMP"},
    {ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION, "VELOCITY_LIMIT_VIOLATION"},
    {ODRIVE_ERROR_POSITION_LIMIT_VIOLATION, "POSITION_LIMIT_VIOLATION"},
    {ODRIVE_ERROR_REQUESTED_CURRENT_TOO_HIGH, "REQUESTED_CURRENT_TOO_HIGH"},
    {ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED, "WATCHDOG_TIMER_EXPIRED"},
    {ODRIVE_ERROR_ESTOP_REQUESTED, "ESTOP_REQUESTED"},
    {ODRIVE_ERROR_SPINOUT_DETECTED, "SPINOUT_DETECTED"},
    {ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED, "BRAKE_RESISTOR_DISARMED"},
    {ODRIVE_ERROR_THERMISTOR_DISCONNECTED, "THERMISTOR_DISCONNECTED"},
    {ODRIVE_ERROR_CALIBRATION_ERROR, "CALIBRATION_ERROR"},
};

// Decode an ODrive.Error bitfield into the list of set flag names.
// Returns {"NONE"} when no bits are set. Any bits not covered by the table are
// reported as a single "UNKNOWN(0x........)" entry so nothing is silently lost.
inline std::vector<std::string> decode_odrive_error(uint32_t error) {
  std::vector<std::string> names;
  if (error == ODRIVE_ERROR_NONE) {
    names.emplace_back("NONE");
    return names;
  }

  uint32_t matched = 0;
  for (const auto& bit : kOdriveErrorBits) {
    if (error & bit.mask) {
      names.emplace_back(bit.name);
      matched |= bit.mask;
    }
  }

  const uint32_t unknown = error & ~matched;
  if (unknown != 0) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "UNKNOWN(0x%08X)", unknown);
    names.emplace_back(buf);
  }
  return names;
}

// Same as decode_odrive_error() but joined into "A | B | C" (or "NONE").
inline std::string decode_odrive_error_string(uint32_t error) {
  const std::vector<std::string> names = decode_odrive_error(error);
  std::string out;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i != 0) out += " | ";
    out += names[i];
  }
  return out;
}

}  // namespace odrive_decode

#endif  // ODRIVE_ERROR_DECODER_HPP
