#include "ErrorStatus.h"

/**
 * @brief Map a Status value to a human-readable message.
 *
 * Returns a null-terminated C-string describing the provided status. For
 * unrecognized status values the function returns "Unknown error".
 *
 * @param status The status value to convert to a message.
 * @return const char* A human-readable message corresponding to `status`.
 */
const char* ErrorStatus::getStatusMessage(Status status) {
  switch(status) {
    case Status::Success: return "OK";
    case Status::SensorReadError: return "Sensor read failed";
    case Status::BluetoothConnectionLost: return "BT disconnected";
    case Status::InvalidDataReceived: return "Bad data";
    case Status::MotorLimitExceeded: return "Motor limit hit";
    case Status::TimeoutError: return "Timeout";
    default: return "Unknown error";
  }
}

/**
 * @brief Determines whether the provided status represents an error.
 *
 * @param status Status value to evaluate.
 * @return `true` if `status` is not `Status::Success`, `false` otherwise.
 */
bool ErrorStatus::isError(Status status) {
  return status != Status::Success;
}