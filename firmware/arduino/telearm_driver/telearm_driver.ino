/*
#include <Servo.h>

constexpr int NUM_JOINTS = 5;
int PINS[NUM_JOINTS] = {3, 5, 6, 9, 10};
Servo servos[NUM_JOINTS];

// Tune these per joint/servo
int   us_min[NUM_JOINTS] = {500, 500, 500, 500, 500};
int   us_max[NUM_JOINTS] = {2500,2500,2500,2500,2500};
float rad_min[NUM_JOINTS]= {-3.1416, -1.5708, -2.3562, -2.3562, -3.1416};
float rad_max[NUM_JOINTS]= { 3.1416,  1.5708,  2.3562,  2.3562,  3.1416};

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
  for(int i=0;i<NUM_JOINTS;i++){
    servos[i].attach(PINS[i]);
    servos[i].writeMicroseconds((us_min[i]+us_max[i])/2); // mid
  }
  Serial.println("READY");
}

void loop(){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c == '
'){
      if (buf.length() > 0 && buf.charAt(0) == 'M'){
        int p1 = buf.indexOf(',');
        int p2 = buf.indexOf(',', p1+1);
        int p3 = buf.indexOf(',', p2+1);
        int idx = buf.substring(p1+1, p2).toInt();
        float ang = buf.substring(p2+1, (p3==-1?buf.length():p3)).toFloat();
        if (idx >=0 && idx < NUM_JOINTS){
          int us = radToUs(idx, ang);
          servos[idx].writeMicroseconds(us);
          Serial.println("OK");
        } else {
          Serial.println("ERR");
        }
      }
      buf = "";
    } else if (c != '
'){
      buf += c;
    }
  }
}
*/
