#include "motor.h"

#include <DynamixelProtocol.h>
#include <Servo.h>

#define ID 128
#define BAUDRATE 1000000

enum motors : byte {
  EYEBROW_HEIGHT_RIGHT, EYEBROW_HEIGHT_LEFT, 
  EYEBROW_ANGLE_RIGHT, EYEBROW_ANGLE_LEFT,
  EYELID_RIGHT, EYELID_LEFT,
  EYE_HORIZONTAL, EYE_VERTICAL,
  JAW_ROTATE, JAW_HORIZONTAL
};

Motor* motor[10];
DynamixelProtocol dxl(BAUDRATE, ID);
unsigned char adress;
unsigned char angle;

void setup() {
  dxl.init();
  //Serial.begin(9600); //BAUDRATE
  //Serial.flush();           //**setMotorDefinitions(start, beginlimit, endlimit, pin)
  
  motor[EYEBROW_HEIGHT_RIGHT] = new percentualMotor();
  motor[EYEBROW_HEIGHT_RIGHT]->reversed=1;
  motor[EYEBROW_HEIGHT_LEFT] = new percentualMotor();
  motor[EYEBROW_HEIGHT_RIGHT]->setMotorDefinitions(50, 70, 155, 4); //EDITAR PINS CONFORME NECESS�RIO
  motor[EYEBROW_HEIGHT_LEFT]->setMotorDefinitions(50, 15, 115, 8); 

  motor[EYEBROW_ANGLE_RIGHT] = new rawangleMotor();
  motor[EYEBROW_ANGLE_LEFT] = new rawangleMotor();
  motor[EYEBROW_ANGLE_RIGHT]->setMotorDefinitions(50, 10, 170, 5); 
  motor[EYEBROW_ANGLE_LEFT]->setMotorDefinitions(120, 10, 170, 9); 


  motor[EYELID_RIGHT] = new percentualMotor();
  motor[EYELID_RIGHT]->reversed=1;
  motor[EYELID_LEFT] = new percentualMotor();
  motor[EYELID_RIGHT]->setMotorDefinitions(50, 60, 140, 3);
  motor[EYELID_LEFT]->setMotorDefinitions(50, 60, 120, 2);

  motor[EYE_HORIZONTAL] = new rawangleMotor();
  motor[EYE_VERTICAL] = new rawangleMotor();
  motor[EYE_HORIZONTAL]->setMotorDefinitions(95, 10, 140, 10); 
  motor[EYE_VERTICAL]->setMotorDefinitions(135, 120, 155, 11);
  
  motor[JAW_ROTATE] = new percentualMotor();
  motor[JAW_HORIZONTAL] = new percentualMotor();
  motor[JAW_ROTATE]->setMotorDefinitions(125, 90, 130, 12);//*a definir
  motor[JAW_HORIZONTAL]->setMotorDefinitions(130, 90, 140, 13);//*a definir
}

int angles2;

void loop() {
  
  // angles2 = Serial.parseInt();
  // Serial.print("Input: ");
  // Serial.println(angles2);
  // motor[EYELID_RIGHT]->goTo(angles2);
 // }
//  dxl.checkMessages();
//  
  dxl.checkMessages();
 
  if(dxl.instruction != DXL_NO_DATA && dxl.id == ID) {
    switch(dxl.instruction) {
      case DXL_WRITE_DATA:
        adress = dxl.parameters[0];
        angle = dxl.parameters[1];
        switch(adress) {
          case EYEBROW_HEIGHT_RIGHT:
            motor[EYEBROW_HEIGHT_RIGHT]->goTo(angle);
            break;
          case EYEBROW_HEIGHT_LEFT:
            motor[EYEBROW_HEIGHT_LEFT]->goTo(angle);
            break;
          case EYEBROW_ANGLE_RIGHT:
            motor[EYEBROW_ANGLE_RIGHT]->goTo(angle);
            break;
          case EYEBROW_ANGLE_LEFT:
            motor[EYEBROW_ANGLE_LEFT]->goTo(angle);
            break;
          case EYELID_RIGHT:
            motor[EYELID_RIGHT]->goTo(angle);
            break;
          case EYELID_LEFT:
            motor[EYELID_LEFT]->goTo(angle);
            break;
          case EYE_HORIZONTAL:
            motor[EYE_HORIZONTAL]->goTo(angle);
            break;
          case EYE_VERTICAL:
            motor[EYE_VERTICAL]->goTo(angle);
            break;
          case JAW_ROTATE:
            motor[JAW_ROTATE]->goTo(angle);
            motor[JAW_HORIZONTAL]->goTo(angle);   
            break;
          default:
            break;
        }
      default:
        break;
    }   
  }
}
