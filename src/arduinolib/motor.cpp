#include "motor.h"

void Motor::setMotorDefinitions(unsigned char startAngle, unsigned char beginLimit,  unsigned char endLimit, unsigned char attachPin){
  servo.attach(attachPin);
  limitAngle[0] = beginLimit;
  limitAngle[1] = endLimit;
  goTo(startAngle);
}

void Motor::fadeWrite(Servo objServo, int finalAngle, int delayms){

  int currentAngle = objServo.read();

  int increment = (currentAngle < finalAngle) ? 1 : -1;

  for(; (increment == 1) ? (currentAngle < finalAngle) : (currentAngle > finalAngle); currentAngle += increment){
    objServo.write(currentAngle);
    delay(delayms);
  }

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

//motors using percentual values (0-100)
percentualMotor::percentualMotor() {}

void percentualMotor::goTo(unsigned char angle){
  
  int new_angle = map(angle, 0, 100, reversed==0 ? limitAngle[0] : limitAngle[1], reversed==0 ? limitAngle[1] : limitAngle[0]);
  new_angle = checkRange(new_angle);

  Serial.print("ANGLE TO GO: ");
  Serial.println(new_angle);
  //servo.write(new_angle);
  fadeWrite(servo, new_angle, 1);
}



//motors using raw angle values (0-180)
rawangleMotor::rawangleMotor() {}

void rawangleMotor::goTo(unsigned char angle){
  int new_angle=checkRange(angle);
  Serial.print("ANGLE TO GO: ");
  Serial.println(new_angle);
  //servo.write(new_angle);
  fadeWrite(servo, new_angle, 1);
}
