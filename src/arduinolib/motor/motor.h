#include <Arduino.h>
#include <Servo.h>

#ifndef Motor_h
#define Motor_h

class Motor{  
  public:
    void setMotorDefinitions(uint8_t startAngle, uint8_t beginLimit,  uint8_t endLimit, uint8_t attachPin);
    void goTo(unsigned char angle);
    static void returnCount();
        
  private:
    static int counter;
    unsigned char checkRange(unsigned char targetPos);
    unsigned char limitAngle[2] = {5,175};
    unsigned char invert = 0;
    Servo servo;
    void fadeWrite(Servo objServo, int finalAngle, int delayms);
};

#endif

