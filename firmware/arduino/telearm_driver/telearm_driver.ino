/*
 * Telearm Arduino Driver with E-stop and Watchdog
 * 
 * This firmware controls 5 hobby servos for the Telearm robot with safety features.
 * Configuration parameters should match config/pins.yaml and config/teleop.yaml.
 * 
 * Protocol:
 * - Send: "M,<joint_index>,<angle_radians>\n"
 * - Receive: "OK" or "ERR"
 * - Watchdog: Must receive commands within 200ms or servos will be disabled
 * - E-stop: Pin 13 (LOW = emergency stop, servos disabled)
 * 
 * Hardware configuration (from config/pins.yaml):
 * - Pins: [3, 5, 6, 9, 10]
 * - Baud rate: 115200
 * - PWM range: 500-2500 microseconds
 * - E-stop pin: 13
 * 
 * Safety features:
 * - Watchdog timer (200ms timeout from config/teleop.yaml)
 * - Emergency stop button on pin 13
 * - Automatic servo disable on safety violation
 */

#include <Servo.h>

constexpr int NUM_JOINTS = 5;
int PINS[NUM_JOINTS] = {3, 5, 6, 9, 10};
Servo servos[NUM_JOINTS];

// Servo configuration - match config/pins.yaml
int   us_min[NUM_JOINTS] = {500, 500, 500, 500, 500};
int   us_max[NUM_JOINTS] = {2500,2500,2500,2500,2500};
float rad_min[NUM_JOINTS]= {-3.1416, -1.5708, -2.3562, -2.3562, -3.1416};
float rad_max[NUM_JOINTS]= { 3.1416,  1.5708,  2.3562,  2.3562,  3.1416};

// Safety configuration - match config/teleop.yaml
const int ESTOP_PIN = 13;  // Emergency stop pin
const unsigned long WATCHDOG_TIMEOUT = 200;  // milliseconds
unsigned long lastCommandTime = 0;
bool servosEnabled = true;
bool emergencyStop = false;

String buf;

int clampi(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }
float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

int radToUs(int i, float rad){
  rad = clampf(rad, rad_min[i], rad_max[i]);
  float t = (rad - rad_min[i]) / (rad_max[i] - rad_min[i]); // 0..1
  int us = (int)(us_min[i] + t * (us_max[i] - us_min[i]));
  return clampi(us, us_min[i], us_max[i]);
}

void setup(){
  Serial.begin(115200);
  
  // Initialize E-stop pin
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  
  // Initialize servos
  for(int i=0;i<NUM_JOINTS;i++){
    servos[i].attach(PINS[i]);
    servos[i].writeMicroseconds((us_min[i]+us_max[i])/2); // mid
  }
  
  lastCommandTime = millis();
  Serial.println("READY");
}

void loop(){
  // Check E-stop button
  if (digitalRead(ESTOP_PIN) == LOW) {
    emergencyStop = true;
    disableServos();
    Serial.println("ESTOP");
    return;
  }
  
  // Check watchdog timeout
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (servosEnabled) {
      disableServos();
      Serial.println("WATCHDOG");
    }
  }
  
  // Process serial commands
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c == '\n'){
      if (buf.length() > 0 && buf.charAt(0) == 'M'){
        int p1 = buf.indexOf(',');
        int p2 = buf.indexOf(',', p1+1);
        int p3 = buf.indexOf(',', p2+1);
        int idx = buf.substring(p1+1, p2).toInt();
        float ang = buf.substring(p2+1, (p3==-1?buf.length():p3)).toFloat();
        
        if (idx >=0 && idx < NUM_JOINTS && servosEnabled && !emergencyStop){
          int us = radToUs(idx, ang);
          servos[idx].writeMicroseconds(us);
          lastCommandTime = millis();  // Update watchdog
          Serial.println("OK");
        } else if (!servosEnabled) {
          Serial.println("DISABLED");
        } else if (emergencyStop) {
          Serial.println("ESTOP");
        } else {
          Serial.println("ERR");
        }
      }
      buf = "";
    } else if (c != '\r'){
      buf += c;
    }
  }
}

void disableServos(){
  // Stop all servos by writing neutral position
  for(int i=0;i<NUM_JOINTS;i++){
    servos[i].writeMicroseconds((us_min[i]+us_max[i])/2);
  }
  servosEnabled = false;
}

void enableServos(){
  servosEnabled = true;
  emergencyStop = false;
  lastCommandTime = millis();
}