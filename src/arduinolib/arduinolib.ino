#include "motor.h"
#include "thread.h"

#include <DynamixelProtocol.h>
#include <Servo.h>

#define ID 128
#define BAUDRATE 1000000

enum motors : byte {
  EYEBROW_HEIGHT_RIGHT, EYEBROW_HEIGHT_LEFT, EYEBROW_ANGLE_RIGHT, EYEBROW_ANGLE_LEFT,
  EYELID_RIGHT, EYELID_LEFT,
  EYE_HORIZONTAL, EYE_VERTICAL,
  JAW_ROTATE_RIGHT, JAW_ROTATE_LEFT//, 
//  JAW_HORIZONTAL_RIGHT, JAW_HORIZONTAL_LEFT
};

Motor motor[10];
DynamixelProtocol dxl(BAUDRATE, ID);
unsigned char adress;
unsigned char angle;

void setup() {
  dxl.init();
  //Serial.begin(9600); //BAUDRATE
  //Serial.flush();           //**setMotorDefinitions(start, beginlimit, endlimit, pin)
  motor[EYEBROW_HEIGHT_RIGHT].setMotorDefinitions(80, 70, 155, 4); //EDITAR PINS CONFORME NECESS�RIO
  motor[EYEBROW_HEIGHT_LEFT].setMotorDefinitions(80, 15, 115, 8); 
  motor[EYEBROW_ANGLE_RIGHT].setMotorDefinitions(50, 10, 170, 5); 
  motor[EYEBROW_ANGLE_LEFT].setMotorDefinitions(120, 10, 170, 9); 

  motor[EYELID_RIGHT].setMotorDefinitions(70, 60, 140, 3);
  motor[EYELID_LEFT].setMotorDefinitions(115, 60, 120, 2);

  motor[EYE_HORIZONTAL].setMotorDefinitions(95, 10, 170, 10); 
  motor[EYE_VERTICAL].setMotorDefinitions(135, 110, 150, 11);
  
  motor[JAW_ROTATE_RIGHT].setMotorDefinitions(125, 90, 130, 12);//*a definir
  motor[JAW_ROTATE_LEFT].setMotorDefinitions(105, 125, 140, 6);//*a definir

  // motor[JAW_HORIZONTAL_RIGHT].setMotorDefinitions(130, 90, 140, 13);//*a definir
  // motor[JAW_HORIZONTAL_LEFT].setMotorDefinitions(0, 10, 60, 7);//*a definir
}

int angles2;

void loop() {
  
  // angles2 = Serial.parseInt();
  // Serial.print("Input: ");
  // Serial.println(angles2);
  // motor[EYELID_RIGHT].goTo(angles2);
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
            motor[EYEBROW_HEIGHT_RIGHT].goTo(angle);
            break;
          case EYEBROW_HEIGHT_LEFT:
            motor[EYEBROW_HEIGHT_LEFT].goTo(angle);
            break;
          case EYEBROW_ANGLE_RIGHT:
            motor[EYEBROW_ANGLE_RIGHT].goTo(angle);
            break;
          case EYEBROW_ANGLE_LEFT:
            motor[EYEBROW_ANGLE_LEFT].goTo(angle);
            break;
          case EYELID_RIGHT:
            motor[EYELID_RIGHT].goTo(angle);
            break;
          case EYELID_LEFT:
            motor[EYELID_LEFT].goTo(angle);
            break;
          case EYE_HORIZONTAL:
            motor[EYE_HORIZONTAL].goTo(angle);
            break;
          case EYE_VERTICAL:
            motor[EYE_VERTICAL].goTo(angle);
            break;
          case JAW_ROTATE_RIGHT:
            motor[JAW_ROTATE_RIGHT].goTo(angle);
            break;
          case JAW_ROTATE_LEFT:
            motor[JAW_ROTATE_LEFT].goTo(angle);
            break;
          // case JAW_HORIZONTAL_RIGHT:
          //   motor[JAW_HORIZONTAL_RIGHT].goTo(angle);
          //   break;
          // case JAW_HORIZONTAL_LEFT:
          //   motor[JAW_HORIZONTAL_LEFT].goTo(angle);   
          //   break;
          default:
            break;
        }
      default:
        break;
    }   
  }
}
