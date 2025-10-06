#ifndef ERROR_STATUS_H
#define ERROR_STATUS_H

#include <Arduino.h>

/**
 * @brief Represents outcome codes for hardware and communication operations.
 *
 * Enumerates possible statuses returned by sensors, communications, and actuators.
 *
 * - Success: Operation completed without error.
 * - SensorReadError: Failure while reading from a sensor.
 * - BluetoothConnectionLost: Bluetooth link was lost or disconnected.
 * - InvalidDataReceived: Received data was malformed or out of expected range.
 * - MotorLimitExceeded: Motor commanded beyond configured limits.
 * - TimeoutError: An operation timed out before completion.
 */
enum class Status : uint8_t {
  Success = 0,
  SensorReadError,
  BluetoothConnectionLost,
  InvalidDataReceived,
  MotorLimitExceeded,
  TimeoutError
};

/**
 * Get a human-readable message for the given status.
 *
 * @param status Status value to describe.
 * @returns A null-terminated string describing the status.
 */
/**
 * Determine whether a status value represents an error condition.
 *
 * @param status Status value to evaluate.
 * @returns `true` if `status` is one of `SensorReadError`, `BluetoothConnectionLost`, `InvalidDataReceived`, `MotorLimitExceeded`, or `TimeoutError`; `false` if `status` is `Success`.
 */
class ErrorStatus {
public:
  static const char* getStatusMessage(Status status);
  static bool isError(Status status);
};

#endif