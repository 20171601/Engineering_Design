#include <Servo.h>

//Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

//Framework setting
#define _DIST_MIN 100
#define _DIST_MAX 410

//Servo speed control
#define _SERVO_ANGLE 20.0
#define _SERVO_SPEED 100.0

//Servo range
#define _DUTY_MIN 1320
#define _DUTY_NEU 1400
#define _DUTY_MAX 1480

#define INTERVAL 10.0

//Servo instance
Servo myservo;
int a, b;

int duty_curr;
int duty_chg_per_interval;

void setup(){
//attach servo
  myservo.attach(PIN_SERVO);

//move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;

//initialize serial port
  Serial.begin(57600);

  a = 85;
  b = 383;
  
  //convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / _SERVO_ANGLE) * (float(INTERVAL)/1000.0);  

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop(){
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:100, max:410, dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  
  if(dist_cali > 255) {
    duty_curr += duty_chg_per_interval;
  }
  else {
    duty_curr -= duty_chg_per_interval;
  }
  
  myservo.writeMicroseconds(duty_curr);
  delay(20);
}
