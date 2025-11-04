/*
 * ESP32 Arm Driver for TeleArm
 * 
 * Receives joint angle targets via USB serial (binary protocol)
 * Outputs 50 Hz PWM to 6 servos (5 joints + gripper) using LEDC hardware timers
 * Enforces joint limits, rate limits, and timeout failsafe
 * Handles limit switches for homing and safety
 * 
 * Protocol:
 * - Binary packets: [0xAA sync] [TYPE] [PAYLOAD] [CRC16]
 * - Message types: JOINT_TARGET=0x01, HOME=0x02, PARK=0x03, GRIPPER=0x04
 * - JOINT_TARGET: 5× int16 (degrees×100), optional gripper uint8
 * - Watchdog: Must receive commands within 300ms or enter safe mode
 * 
 * Hardware:
 * - ESP32-WROOM-32
 * - 5× MG995 servos (joints) + 1× gripper servo
 * - 5× limit switches (one per joint)
 * - LEDC hardware PWM for stable 50 Hz output
 * 
 * Configuration (match config/pins.yaml):
 * - Servo pins: [2, 4, 5, 18, 19] for joints, pin 21 for gripper
 * - Limit switch pins: [22, 23, 25, 26, 27]
 * - PWM: 50 Hz, 500-2500 microseconds pulse width
 */

#include <Arduino.h>

// Configuration - match config/pins.yaml
constexpr int NUM_JOINTS = 5;
constexpr int SERVO_PINS[NUM_JOINTS] = {2, 4, 5, 18, 19};
constexpr int GRIPPER_PIN = 21;
constexpr int LIMIT_SWITCH_PINS[NUM_JOINTS] = {22, 23, 25, 26, 27};

// PWM configuration
constexpr int PWM_FREQ = 50;  // 50 Hz
constexpr int PWM_RESOLUTION = 16;  // 16-bit resolution
constexpr int PWM_MAX = 65535;

// Servo calibration - match config/pins.yaml
constexpr int PWM_US_MIN = 500;
constexpr int PWM_US_MAX = 2500;
constexpr float ANGLE_MIN_RAD[NUM_JOINTS] = {
  -3.1416, -1.5708, -2.3562, -2.3562, -3.1416
};
constexpr float ANGLE_MAX_RAD[NUM_JOINTS] = {
   3.1416,  1.5708,  2.3562,  2.3562,  3.1416
};

// Rate limits (deg/s) - match config/robot.yaml max_vel
constexpr float MAX_VEL_DEG_PER_SEC[NUM_JOINTS] = {
  859.4, 859.4, 859.4, 859.4, 1145.9
};

// Safety and timing
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 300;
constexpr unsigned long CONTROL_LOOP_PERIOD_MS = 20;  // 50 Hz = 20ms

// Protocol constants
constexpr uint8_t SYNC_BYTE = 0xAA;
enum MessageType {
  JOINT_TARGET = 0x01,
  HOME = 0x02,
  PARK = 0x03,
  GRIPPER = 0x04,
  STATUS_REQUEST = 0x05
};

// State
float current_joints_deg[NUM_JOINTS] = {0, 0, 0, 0, 0};
float target_joints_deg[NUM_JOINTS] = {0, 0, 0, 0, 0};
uint8_t gripper_value = 128;  // Default mid position
unsigned long last_command_time = 0;
bool servos_enabled = false;
bool homing_in_progress = false;
bool limit_switch_states[NUM_JOINTS] = {false, false, false, false, false};

// Homing state
bool joints_homed[NUM_JOINTS] = {false, false, false, false, false};
constexpr float HOMING_BACKOFF_DEG = 5.0;
constexpr float HOMING_SPEED_DEG_PER_SEC = 30.0;

// LEDC channels for servos
constexpr int LEDC_CHANNELS[NUM_JOINTS] = {0, 1, 2, 3, 4};
constexpr int LEDC_GRIPPER_CHANNEL = 5;

// Serial receive buffer
uint8_t rx_buffer[64];
int rx_index = 0;

// Helper functions
int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float clamp_float(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int deg_to_pulse_us(int joint_idx, float angle_deg) {
  // Clamp angle to limits
  float angle_rad = angle_deg * PI / 180.0;
  angle_rad = clamp_float(angle_rad, ANGLE_MIN_RAD[joint_idx], ANGLE_MAX_RAD[joint_idx]);
  
  // Map angle to pulse width
  float t = (angle_rad - ANGLE_MIN_RAD[joint_idx]) / 
            (ANGLE_MAX_RAD[joint_idx] - ANGLE_MIN_RAD[joint_idx]);
  int pulse_us = PWM_US_MIN + t * (PWM_US_MAX - PWM_US_MIN);
  
  return clamp_int(pulse_us, PWM_US_MIN, PWM_US_MAX);
}

int pulse_us_to_pwm_value(int pulse_us) {
  // Convert microseconds to PWM duty cycle
  // For 50 Hz: period = 20ms = 20000us
  // Duty cycle = pulse_us / 20000
  int duty = (pulse_us * PWM_MAX) / 20000;
  return clamp_int(duty, 0, PWM_MAX);
}

void set_servo_pwm(int joint_idx, float angle_deg) {
  if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return;
  if (!servos_enabled) return;
  
  int pulse_us = deg_to_pulse_us(joint_idx, angle_deg);
  int duty = pulse_us_to_pwm_value(pulse_us);
  
  ledcWrite(LEDC_CHANNELS[joint_idx], duty);
}

void set_gripper_pwm(uint8_t value) {
  if (!servos_enabled) return;
  
  // Map 0-255 to pulse width range
  int pulse_us = PWM_US_MIN + (value * (PWM_US_MAX - PWM_US_MIN)) / 255;
  int duty = pulse_us_to_pwm_value(pulse_us);
  
  ledcWrite(LEDC_GRIPPER_CHANNEL, duty);
}

void read_limit_switches() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    // Limit switches are INPUT_PULLUP, active LOW
    limit_switch_states[i] = (digitalRead(LIMIT_SWITCH_PINS[i]) == LOW);
  }
}

void disable_servos() {
  // Set all servos to neutral (mid pulse width)
  int neutral_pulse = (PWM_US_MIN + PWM_US_MAX) / 2;
  int neutral_duty = pulse_us_to_pwm_value(neutral_pulse);
  
  for (int i = 0; i < NUM_JOINTS; i++) {
    ledcWrite(LEDC_CHANNELS[i], neutral_duty);
  }
  ledcWrite(LEDC_GRIPPER_CHANNEL, neutral_duty);
  
  servos_enabled = false;
}

void enable_servos() {
  servos_enabled = true;
  last_command_time = millis();
}

uint16_t calculate_crc16(uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
      crc &= 0xFFFF;
    }
  }
  return crc;
}

bool process_joint_target_packet(uint8_t* data, int len) {
  // Expected: [SYNC] [TYPE] [JOINT0: int16] ... [JOINT4: int16] [GRIPPER?: uint8] [CRC16: uint16]
  if (len < 2 + NUM_JOINTS * 2 + 2) return false;  // Minimum: header + 5 joints + CRC
  
  // Check sync byte
  if (data[0] != SYNC_BYTE) return false;
  if (data[1] != JOINT_TARGET) return false;
  
  // Unpack joint angles (int16, big-endian, degrees × 100)
  for (int i = 0; i < NUM_JOINTS; i++) {
    int16_t centidegrees = (data[2 + i*2] << 8) | data[3 + i*2];
    target_joints_deg[i] = centidegrees / 100.0;
  }
  
  // Optional gripper (if packet is long enough)
  int gripper_offset = 2 + NUM_JOINTS * 2;
  if (len >= gripper_offset + 1 + 2) {  // gripper + CRC
    gripper_value = data[gripper_offset];
  }
  
  // Verify CRC
  int crc_offset = len - 2;
  uint16_t received_crc = (data[crc_offset] << 8) | data[crc_offset + 1];
  uint16_t calculated_crc = calculate_crc16(data, crc_offset);
  
  if (received_crc != calculated_crc) {
    Serial.println("CRC error");
    return false;
  }
  
  // Enforce rate limits
  for (int i = 0; i < NUM_JOINTS; i++) {
    float delta_deg = target_joints_deg[i] - current_joints_deg[i];
    float max_delta_per_cycle = (MAX_VEL_DEG_PER_SEC[i] * CONTROL_LOOP_PERIOD_MS) / 1000.0;
    
    if (abs(delta_deg) > max_delta_per_cycle) {
      if (delta_deg > 0) {
        target_joints_deg[i] = current_joints_deg[i] + max_delta_per_cycle;
      } else {
        target_joints_deg[i] = current_joints_deg[i] - max_delta_per_cycle;
      }
    }
  }
  
  // Clamp to joint limits
  for (int i = 0; i < NUM_JOINTS; i++) {
    float angle_rad = target_joints_deg[i] * PI / 180.0;
    angle_rad = clamp_float(angle_rad, ANGLE_MIN_RAD[i], ANGLE_MAX_RAD[i]);
    target_joints_deg[i] = angle_rad * 180.0 / PI;
  }
  
  enable_servos();
  return true;
}

void execute_homing() {
  Serial.println("Starting homing sequence...");
  homing_in_progress = true;
  
  // For each joint, move slowly toward limit switch until it trips
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print("Homing joint ");
    Serial.println(i);
    
    // Move toward minimum limit (assuming limit switch is at min position)
    float homing_angle = target_joints_deg[i];
    float homing_step_deg = (HOMING_SPEED_DEG_PER_SEC * CONTROL_LOOP_PERIOD_MS) / 1000.0;
    
    while (!limit_switch_states[i] && homing_angle > ANGLE_MIN_RAD[i] * 180.0 / PI) {
      homing_angle -= homing_step_deg;
      set_servo_pwm(i, homing_angle);
      
      read_limit_switches();
      delay(CONTROL_LOOP_PERIOD_MS);
    }
    
    // Limit switch tripped - set as zero and back off
    if (limit_switch_states[i]) {
      Serial.print("Limit switch hit for joint ");
      Serial.println(i);
      
      // Back off by configured amount
      target_joints_deg[i] = homing_angle + HOMING_BACKOFF_DEG;
      joints_homed[i] = true;
      
      set_servo_pwm(i, target_joints_deg[i]);
      delay(500);
    }
  }
  
  homing_in_progress = false;
  Serial.println("Homing complete");
}

void execute_park() {
  Serial.println("Parking arm...");
  // Move to safe park position (all joints to mid-range)
  for (int i = 0; i < NUM_JOINTS; i++) {
    target_joints_deg[i] = (ANGLE_MIN_RAD[i] + ANGLE_MAX_RAD[i]) * 180.0 / PI / 2.0;
  }
  enable_servos();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 Arm Driver Starting...");
  
  // Initialize limit switch pins
  for (int i = 0; i < NUM_JOINTS; i++) {
    pinMode(LIMIT_SWITCH_PINS[i], INPUT_PULLUP);
  }
  
  // Initialize LEDC for servos
  for (int i = 0; i < NUM_JOINTS; i++) {
    ledcSetup(LEDC_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(SERVO_PINS[i], LEDC_CHANNELS[i]);
  }
  
  // Initialize gripper LEDC
  ledcSetup(LEDC_GRIPPER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(GRIPPER_PIN, LEDC_GRIPPER_CHANNEL);
  
  // Initialize servos to neutral
  disable_servos();
  
  // Read initial limit switch states
  read_limit_switches();
  
  Serial.println("ESP32 Arm Driver Ready");
  Serial.println("Waiting for commands...");
}

void loop() {
  unsigned long now = millis();
  
  // Read limit switches
  read_limit_switches();
  
  // Check watchdog timeout
  if (servos_enabled && (now - last_command_time > WATCHDOG_TIMEOUT_MS)) {
    Serial.println("Watchdog timeout - entering safe mode");
    disable_servos();
    // Optionally execute park instead of just disabling
    // execute_park();
  }
  
  // Process serial input
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    // Reset buffer on sync byte
    if (byte == SYNC_BYTE) {
      rx_index = 0;
      rx_buffer[rx_index++] = byte;
      continue;
    }
    
    // Accumulate bytes
    if (rx_index < sizeof(rx_buffer)) {
      rx_buffer[rx_index++] = byte;
    }
    
    // Check if we have a complete packet
    // Minimum: SYNC + TYPE + 5 joints (10 bytes) + CRC (2 bytes) = 14 bytes
    if (rx_index >= 14) {
      uint8_t msg_type = rx_buffer[1];
      
      bool packet_ok = false;
      if (msg_type == JOINT_TARGET) {
        packet_ok = process_joint_target_packet(rx_buffer, rx_index);
      } else if (msg_type == HOME) {
        execute_homing();
        packet_ok = true;
      } else if (msg_type == PARK) {
        execute_park();
        packet_ok = true;
      } else if (msg_type == GRIPPER) {
        if (rx_index >= 5) {  // SYNC + TYPE + gripper + CRC
          gripper_value = rx_buffer[2];
          set_gripper_pwm(gripper_value);
          packet_ok = true;
        }
      }
      
      if (packet_ok) {
        Serial.print("OK\n");
      } else {
        Serial.print("ERR\n");
      }
      
      // Reset buffer
      rx_index = 0;
    }
  }
  
  // Update servos if enabled
  if (servos_enabled && !homing_in_progress) {
    // Smooth interpolation toward target
    for (int i = 0; i < NUM_JOINTS; i++) {
      float alpha = 0.1;  // Smoothing factor
      current_joints_deg[i] = alpha * target_joints_deg[i] + (1 - alpha) * current_joints_deg[i];
      set_servo_pwm(i, current_joints_deg[i]);
    }
    
    set_gripper_pwm(gripper_value);
  }
  
  // Control loop rate (50 Hz)
  delay(CONTROL_LOOP_PERIOD_MS);
}


