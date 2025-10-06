#include "ErrorStatus.h"

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

bool ErrorStatus::isError(Status status) {
  return status != Status::Success;
}
