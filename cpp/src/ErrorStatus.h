#ifndef ERROR_STATUS_H
#define ERROR_STATUS_H

#include <Arduino.h>

/**
 * Represents status codes for device operations.
 *
 * Enumerators:
 *  - Success: Operation completed successfully.
 *  - SensorReadError: Failed to read from a sensor.
 *  - BluetoothConnectionLost: Bluetooth connection was lost.
 *  - InvalidDataReceived: Received data was malformed or invalid.
 *  - MotorLimitExceeded: Motor commanded beyond allowed limits.
 *  - TimeoutError: Operation timed out before completion.
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
 * Return a human-readable, null-terminated message corresponding to the given Status.
 * @param status Status value to describe.
 * @returns Pointer to a statically allocated C string containing the message for `status`.
 */

/**
 * Determine whether the given Status represents an error condition.
 * @param status Status value to evaluate.
 * @returns `true` if `status` indicates an error, `false` otherwise.
 */
class ErrorStatus {
public:
  static const char* getStatusMessage(Status status);
  static bool isError(Status status);
};

#endif