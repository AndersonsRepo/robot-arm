/*
 * ESP32 Operator Arm Tracker
 * 
 * Reads 3× MPU-9250 IMU sensors via I2C, fuses data with complementary filter,
 * and transmits operator pose via WiFi UDP or Bluetooth SPP at 100 Hz.
 * 
 * Hardware:
 * - ESP32-WROOM-32
 * - 3× MPU-9250 IMU sensors on I2C
 * - WiFi for UDP transmission OR Bluetooth Classic SPP
 * 
 * Pinout:
 * - SDA: GPIO 21
 * - SCL: GPIO 22
 * - IMU1: I2C address 0x68 (AD0=LOW)
 * - IMU2: I2C address 0x69 (AD0=HIGH)
 * - IMU3: I2C address 0x68 (AD0=LOW, different I2C bus)
 * - MODE_BUTTON: GPIO 0 (optional, for mode selection)
 */

// Mode selection - uncomment ONE of these:
#define USE_BLUETOOTH true
// #define USE_WIFI true

// If neither is defined, default to WiFi
#ifndef USE_BLUETOOTH
#ifndef USE_WIFI
#define USE_WIFI true
#endif
#endif

#include <Wire.h>
#include <MPU9250.h>

#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

#ifdef USE_BLUETOOTH
#include "BluetoothSerial.h"
#endif

// Network configuration
#ifdef USE_WIFI
const char* ssid = "TelearmNetwork";
const char* password = "telearm123";
const int udpPort = 5000;
const char* targetIP = "192.168.1.100";  // Raspberry Pi IP
WiFiUDP udp;
#endif

#ifdef USE_BLUETOOTH
BluetoothSerial SerialBT;
const char* bluetoothDeviceName = "TelearmOperator";
#endif

// IMU sensors
MPU9250 imu1(Wire, 0x68);  // Upper arm
MPU9250 imu2(Wire, 0x69);  // Forearm
MPU9250 imu3(Wire, 0x68);  // Hand (different I2C bus)

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long transmitInterval = 10;  // 100 Hz = 10ms
unsigned long lastFusionTime = 0;
const unsigned long fusionInterval = 5;     // 200 Hz = 5ms

// Sequence counter
uint32_t sequence = 0;

// Complementary filter state
struct FilterState {
  float quat_w = 1.0f;
  float quat_x = 0.0f;
  float quat_y = 0.0f;
  float quat_z = 0.0f;
  float prev_time = 0.0f;
  bool initialized = false;
};

FilterState filters[3];
const float alpha = 0.98f;  // Filter coefficient

// Operator pose data
struct OperatorPose {
  uint32_t sequence;
  float timestamp;
  float joint_angles[3];      // shoulder, elbow, wrist
  float joint_velocities[3];
  float confidence;
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 Operator Tracker Starting...");
  
  // Display communication mode
  #ifdef USE_BLUETOOTH
    Serial.println("Mode: Bluetooth Classic SPP");
  #endif
  #ifdef USE_WIFI
    Serial.println("Mode: WiFi UDP");
  #endif
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA, SCL
  Wire.setClock(400000);  // 400kHz
  
  // Initialize IMU sensors
  if (!initializeIMUs()) {
    Serial.println("Failed to initialize IMUs!");
    while(1) delay(1000);
  }
  
  // Initialize communication
  #ifdef USE_WIFI
    connectToWiFi();
    udp.begin(udpPort);
  #endif
  
  #ifdef USE_BLUETOOTH
    setupBluetooth();
  #endif
  
  Serial.println("Operator tracker ready!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Fusion at 200 Hz
  if (currentTime - lastFusionTime >= fusionInterval) {
    updateIMUFusion();
    lastFusionTime = currentTime;
  }
  
  // Transmission at 100 Hz
  if (currentTime - lastTransmitTime >= transmitInterval) {
    transmitOperatorPose();
    lastTransmitTime = currentTime;
  }
  
  // Small delay to prevent watchdog reset
  delay(1);
}

bool initializeIMUs() {
  Serial.println("Initializing IMU sensors...");
  
  // Initialize IMU1 (upper arm)
  if (imu1.begin() != 0) {
    Serial.println("IMU1 initialization failed");
    return false;
  }
  imu1.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  imu1.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  imu1.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  
  // Initialize IMU2 (forearm)
  if (imu2.begin() != 0) {
    Serial.println("IMU2 initialization failed");
    return false;
  }
  imu2.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  imu2.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  imu2.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  
  // Initialize IMU3 (hand)
  if (imu3.begin() != 0) {
    Serial.println("IMU3 initialization failed");
    return false;
  }
  imu3.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  imu3.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  imu3.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  
  Serial.println("All IMUs initialized successfully");
  return true;
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    while(1) delay(1000);
  }
}

#ifdef USE_BLUETOOTH
void setupBluetooth() {
  Serial.print("Initializing Bluetooth: ");
  Serial.println(bluetoothDeviceName);
  
  // Initialize Bluetooth with fixed RFCOMM channel 1
  SerialBT.begin(bluetoothDeviceName, true);  // true = isMaster=false (slave mode)
  
  Serial.println("Bluetooth device ready for pairing");
  Serial.println("Device name: TelearmOperator");
  Serial.println("RFCOMM channel: 1");
  Serial.println("Waiting for connection...");
}
#endif

void updateIMUFusion() {
  float currentTime = millis() / 1000.0f;
  float dt = currentTime - filters[0].prev_time;
  
  if (dt <= 0) dt = 0.001f;  // Minimum time step
  
  // Read all IMUs
  MPU9250* imus[] = {&imu1, &imu2, &imu3};
  
  for (int i = 0; i < 3; i++) {
    imus[i]->readSensor();
    
    // Get raw data
    float ax = imus[i]->getAccelX_mss();
    float ay = imus[i]->getAccelY_mss();
    float az = imus[i]->getAccelZ_mss();
    float gx = imus[i]->getGyroX_rads();
    float gy = imus[i]->getGyroY_rads();
    float gz = imus[i]->getGyroZ_rads();
    
    // Apply complementary filter
    updateComplementaryFilter(i, ax, ay, az, gx, gy, gz, dt);
  }
  
  // Update previous time
  for (int i = 0; i < 3; i++) {
    filters[i].prev_time = currentTime;
  }
}

void updateComplementaryFilter(int imu_id, float ax, float ay, float az, 
                              float gx, float gy, float gz, float dt) {
  FilterState& filter = filters[imu_id];
  
  // Normalize accelerometer
  float accel_mag = sqrt(ax*ax + ay*ay + az*az);
  if (accel_mag > 0.1f) {  // Avoid division by zero
    ax /= accel_mag;
    ay /= accel_mag;
    az /= accel_mag;
  }
  
  // Calculate tilt from accelerometer
  float roll_acc = atan2(ay, az);
  float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));
  
  // Convert to quaternion
  float cr = cos(roll_acc / 2.0f);
  float sr = sin(roll_acc / 2.0f);
  float cp = cos(pitch_acc / 2.0f);
  float sp = sin(pitch_acc / 2.0f);
  
  float qw_acc = cr * cp;
  float qx_acc = sr * cp;
  float qy_acc = cr * sp;
  float qz_acc = -sr * sp;
  
  // Gyro integration
  float omega = sqrt(gx*gx + gy*gy + gz*gz);
  if (omega > 0.001f) {
    float axis_x = gx / omega;
    float axis_y = gy / omega;
    float axis_z = gz / omega;
    
    float half_angle = omega * dt / 2.0f;
    float s = sin(half_angle);
    float c = cos(half_angle);
    
    float qw_gyro = c;
    float qx_gyro = axis_x * s;
    float qy_gyro = axis_y * s;
    float qz_gyro = axis_z * s;
    
    // Multiply with previous quaternion
    float qw_new = filter.quat_w * qw_gyro - filter.quat_x * qx_gyro - 
                   filter.quat_y * qy_gyro - filter.quat_z * qz_gyro;
    float qx_new = filter.quat_w * qx_gyro + filter.quat_x * qw_gyro + 
                   filter.quat_y * qz_gyro - filter.quat_z * qy_gyro;
    float qy_new = filter.quat_w * qy_gyro - filter.quat_x * qz_gyro + 
                   filter.quat_y * qw_gyro + filter.quat_z * qx_gyro;
    float qz_new = filter.quat_w * qz_gyro + filter.quat_x * qy_gyro - 
                   filter.quat_y * qx_gyro + filter.quat_z * qw_gyro;
    
    // Complementary filter
    filter.quat_w = alpha * qw_acc + (1.0f - alpha) * qw_new;
    filter.quat_x = alpha * qx_acc + (1.0f - alpha) * qx_new;
    filter.quat_y = alpha * qy_acc + (1.0f - alpha) * qy_new;
    filter.quat_z = alpha * qz_acc + (1.0f - alpha) * qz_new;
  } else {
    // No gyro motion, use accelerometer only
    filter.quat_w = alpha * qw_acc + (1.0f - alpha) * filter.quat_w;
    filter.quat_x = alpha * qx_acc + (1.0f - alpha) * filter.quat_x;
    filter.quat_y = alpha * qy_acc + (1.0f - alpha) * filter.quat_y;
    filter.quat_z = alpha * qz_acc + (1.0f - alpha) * filter.quat_z;
  }
  
  // Normalize quaternion
  float norm = sqrt(filter.quat_w*filter.quat_w + filter.quat_x*filter.quat_x + 
                    filter.quat_y*filter.quat_y + filter.quat_z*filter.quat_z);
  if (norm > 0.0f) {
    filter.quat_w /= norm;
    filter.quat_x /= norm;
    filter.quat_y /= norm;
    filter.quat_z /= norm;
  }
  
  filter.initialized = true;
}

void transmitOperatorPose() {
  if (!filters[0].initialized || !filters[1].initialized || !filters[2].initialized) {
    return;  // Wait for all filters to initialize
  }
  
  // Estimate joint angles from IMU orientations
  float joint_angles[3];
  float joint_velocities[3];
  
  // Simplified joint angle estimation
  // In practice, this would use more sophisticated kinematic analysis
  
  // Shoulder angle (from upper arm IMU)
  float shoulder_roll = atan2(2.0f * (filters[0].quat_w * filters[0].quat_x + 
                                      filters[0].quat_y * filters[0].quat_z),
                              1.0f - 2.0f * (filters[0].quat_x * filters[0].quat_x + 
                                           filters[0].quat_y * filters[0].quat_y));
  joint_angles[0] = shoulder_roll;
  
  // Elbow angle (relative rotation between upper arm and forearm)
  // Simplified: use pitch difference
  float upper_pitch = atan2(2.0f * (filters[0].quat_w * filters[0].quat_y - 
                                    filters[0].quat_z * filters[0].quat_x),
                            1.0f - 2.0f * (filters[0].quat_y * filters[0].quat_y + 
                                         filters[0].quat_z * filters[0].quat_z));
  float forearm_pitch = atan2(2.0f * (filters[1].quat_w * filters[1].quat_y - 
                                      filters[1].quat_z * filters[1].quat_x),
                              1.0f - 2.0f * (filters[1].quat_y * filters[1].quat_y + 
                                           filters[1].quat_z * filters[1].quat_z));
  joint_angles[1] = forearm_pitch - upper_pitch;
  
  // Wrist angle (relative rotation between forearm and hand)
  float forearm_roll = atan2(2.0f * (filters[1].quat_w * filters[1].quat_x + 
                                     filters[1].quat_y * filters[1].quat_z),
                             1.0f - 2.0f * (filters[1].quat_x * filters[1].quat_x + 
                                          filters[1].quat_y * filters[1].quat_y));
  float hand_roll = atan2(2.0f * (filters[2].quat_w * filters[2].quat_x + 
                                  filters[2].quat_y * filters[2].quat_z),
                          1.0f - 2.0f * (filters[2].quat_x * filters[2].quat_x + 
                                       filters[2].quat_y * filters[2].quat_y));
  joint_angles[2] = hand_roll - forearm_roll;
  
  // Clamp angles to reasonable ranges
  joint_angles[0] = constrain(joint_angles[0], -PI/2, PI/2);  // shoulder
  joint_angles[1] = constrain(joint_angles[1], -PI, PI);      // elbow
  joint_angles[2] = constrain(joint_angles[2], -PI/2, PI/2);  // wrist
  
  // Estimate velocities (simplified - in practice would use proper differentiation)
  static float prev_joint_angles[3] = {0, 0, 0};
  static unsigned long prev_time = 0;
  unsigned long current_time = millis();
  
  if (prev_time > 0) {
    float dt = (current_time - prev_time) / 1000.0f;
    if (dt > 0) {
      for (int i = 0; i < 3; i++) {
        joint_velocities[i] = (joint_angles[i] - prev_joint_angles[i]) / dt;
        // Simple velocity limiting
        joint_velocities[i] = constrain(joint_velocities[i], -2.0*pI, 2.0*PI);
      }
    }
  }
  
  // Store current angles for next iteration
  for (int i = 0; i < 3; i++) {
    prev_joint_angles[i] = joint_angles[i];
  }
  prev_time = current_time;
  
  // Create packet
  OperatorPose pose;
  pose.sequence = sequence++;
  pose.timestamp = current_time / 1000.0f;
  for (int i = 0; i < 3; i++) {
    pose.joint_angles[i] = joint_angles[i];
    pose.joint_velocities[i] = joint_velocities[i];
  }
  pose.confidence = 0.9f;  // High confidence for now
  
  // Send packet via selected communication method
  #ifdef USE_WIFI
    // Send UDP packet
    udp.beginPacket(targetIP, udpPort);
    udp.write((uint8_t*)&pose, sizeof(pose));
    udp.endPacket();
  #endif
  
  #ifdef USE_BLUETOOTH
    // Send Bluetooth packet
    if (SerialBT.hasClient()) {
      SerialBT.write((uint8_t*)&pose, sizeof(pose));
    }
  #endif
  
  // Debug output every 50 packets
  if (sequence % 50 == 0) {
    Serial.printf("Sent packet %u: angles=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]\n",
                  sequence, joint_angles[0], joint_angles[1], joint_angles[2],
                  joint_velocities[0], joint_velocities[1], joint_velocities[2]);
    
    #ifdef USE_BLUETOOTH
      if (SerialBT.hasClient()) {
        Serial.println("Bluetooth: Connected");
      } else {
        Serial.println("Bluetooth: No client connected");
      }
    #endif
  }
}
