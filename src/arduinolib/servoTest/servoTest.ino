#include "motor.h"

#include <DynamixelProtocol.h>
#include <Servo.h>

#define ID 128
#define BAUDRATE 1000000

Servo testServo;

Motor* motor[1];
DynamixelProtocol dxl(BAUDRATE, ID);
unsigned char adress;
unsigned char angle;

void setup() {
  //dxl.init();
  Serial.begin(9600); //BAUDRATE
  motor[0] = new rawangleMotor();
  motor[0]->setMotorDefinitions(170, 15, 170, 4); 

  //testServo.attach(4);
 // Serial.println(testServo.read());
}

int angles2;

void loop() {
  //if(Serial.available()){
  //  angles2 = Serial.parseInt();
  //  testServo.write(angles2);
  //}; 
  //motor[0]->goTo(angles2);
  //Serial.print("Input: ");
  //Serial.println(angles2);
 }
 
