#ifndef ERROR_STATUS_H
#define ERROR_STATUS_H

#include <Arduino.h>

enum class Status : uint8_t {
  Success = 0,
  SensorReadError,
  BluetoothConnectionLost,
  InvalidDataReceived,
  MotorLimitExceeded,
  TimeoutError
};

class ErrorStatus {
public:
  static const char* getStatusMessage(Status status);
  static bool isError(Status status);
};

#endif
