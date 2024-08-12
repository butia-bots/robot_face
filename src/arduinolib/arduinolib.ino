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
  JAW_LEFT, JAW_RIGHT
};

Motor* motor[9];
DynamixelProtocol dxl(BAUDRATE, ID);
unsigned char adress;
unsigned char angle;

void setup() {
  dxl.init();
  //Serial.begin(9600); //BAUDRATE
  //Serial.flush();          

   //**setMotorDefinitions(start, beginlimitendlimit, pin)
  motor[EYEBROW_HEIGHT_RIGHT] = new percentualMotor();
 // motor[EYEBROW_HEIGHT_RIGHT]->reversed=1;
  motor[EYEBROW_HEIGHT_LEFT] = new percentualMotor();
  motor[EYEBROW_HEIGHT_RIGHT]->setMotorDefinitions(50, 15, 140, 4); //EDITAR PINS CONFORME NECESS�RIO
  motor[EYEBROW_HEIGHT_LEFT]->setMotorDefinitions(50, 30, 125, 8); 

  motor[EYEBROW_ANGLE_RIGHT] = new rawangleMotor();
  motor[EYEBROW_ANGLE_LEFT] = new rawangleMotor();
  motor[EYEBROW_ANGLE_RIGHT]->setMotorDefinitions(90, 10, 170, 5); 
  motor[EYEBROW_ANGLE_LEFT]->setMotorDefinitions(90, 10, 170, 9); 


  motor[EYELID_RIGHT] = new percentualMotor();
  motor[EYELID_RIGHT]->reversed=1;
  motor[EYELID_LEFT] = new percentualMotor();
  motor[EYELID_RIGHT]->setMotorDefinitions(100, 20, 140, 3);
  //motor[EYELID_LEFT]->setMotorDefinitions(50, 63, 135, 2);

  motor[EYE_HORIZONTAL] = new percentualMotor(); // precisam ser percentuais pela logica do controle da direção do olhar
  motor[EYE_VERTICAL] = new percentualMotor();
  motor[EYE_HORIZONTAL]->setMotorDefinitions(65, 10, 140, 10); 
  motor[EYE_VERTICAL]->setMotorDefinitions(43, 120, 155, 11);
  
  motor[JAW_LEFT] = new rawangleMotor();
  motor[JAW_RIGHT] = new rawangleMotor();
  motor[JAW_LEFT]->setMotorDefinitions(108, 118, 118, 12);//
  motor[JAW_RIGHT]->setMotorDefinitions(116, 126, 126, 13);//
}

int angles2;

void loop() {
  
  // angles2 = Serial.parseInt();
  // Serial.print("Input: ");
  // Serial.println(angles2);
 // }
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
          case JAW_LEFT: //8
            motor[JAW_LEFT]->goTo(angle); 
            break;
          case JAW_RIGHT: //8
            motor[JAW_RIGHT]->goTo(angle); 
            break;
          default:
            break;
        }
      default:
        break;
    }   
  }
}
