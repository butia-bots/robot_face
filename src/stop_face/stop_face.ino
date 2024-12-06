#include <Servo.h> 


Servo EYEBROW_HEIGHT_RIGHT;
Servo EYEBROW_HEIGHT_LEFT;
Servo EYEBROW_ANGLE_RIGHT;
Servo EYEBROW_ANGLE_LEFT;
Servo EYELID_RIGHT;
Servo EYELID_LEFT;
Servo EYE_HORIZONTAL;
Servo EYE_VERTICAL;
Servo JAW_LEFT;
Servo JAW_RIGHT;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  EYEBROW_HEIGHT_RIGHT.attach(4);
  EYEBROW_HEIGHT_LEFT.attach(3);
  EYEBROW_ANGLE_RIGHT.attach(5);
  EYEBROW_ANGLE_LEFT.attach(6);
  EYELID_RIGHT.attach(8);
  EYELID_LEFT.attach(10);
  EYE_HORIZONTAL.attach(2);
  EYE_VERTICAL.attach(7);
  JAW_LEFT.attach(11);
  JAW_RIGHT.attach(12); 

  EYEBROW_HEIGHT_RIGHT.write(70);
  EYEBROW_HEIGHT_LEFT.write(70);
  EYEBROW_ANGLE_RIGHT.write(130);
  EYEBROW_ANGLE_LEFT.write(140);
  EYELID_RIGHT.write(0);
  EYELID_LEFT.write(20);
  EYE_HORIZONTAL.write(60);
  EYE_VERTICAL.write(110);
  JAW_LEFT.write(125);
  JAW_RIGHT.write(80);
}

void loop() {
     
}
