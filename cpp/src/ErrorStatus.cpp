#include "ErrorStatus.h"

/**
 * @brief Returns a human-readable message for a Status code.
 *
 * @param status Status value to translate into a message.
 * @return const char* Corresponding null-terminated C string:
 * - `Success` -> "OK"
 * - `SensorReadError` -> "Sensor read failed"
 * - `BluetoothConnectionLost` -> "BT disconnected"
 * - `InvalidDataReceived` -> "Bad data"
 * - `MotorLimitExceeded` -> "Motor limit hit"
 * - `TimeoutError` -> "Timeout"
 * - other values -> "Unknown error"
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
 * @brief Determines whether a status value represents an error condition.
 *
 * @param status The status value to evaluate.
 * @return true if `status` is not `Status::Success`, false otherwise.
 */
bool ErrorStatus::isError(Status status) {
  return status != Status::Success;
}