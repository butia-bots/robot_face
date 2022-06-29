#include "motor.h"

int Motor::counter = 2;

void Motor::setMotorDefinitions(unsigned char startAngle, unsigned char beginLimit, unsigned char endLimit, int alternado = 0){
  servo.attach(counter++);
  if (alternado == 0) {
    limitAngle[0] = beginLimit;
    limitAngle[1] = endLimit;
  } else {
    limitAngle[0] = endLimit;
    limitAngle[1] = beginLimit;
  }
  servo.write(checkRange(startAngle));
}

void Motor::goTo(unsigned char angle){
  int new_angle = map(angle, 0, 100, limitAngle[0], limitAngle[1])
  servo.write(new_angle);
}

unsigned char Motor::checkRange(unsigned char targetPos){
 if (targetPos < limitAngle[0]){
   return limitAngle[0];
 }
 else if (targetPos > limitAngle[1]){
   return limitAngle[1];
 }
 else return targetPos;
}

void Motor::returnCount(){
  Serial.println(counter);
}
