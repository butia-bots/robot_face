#include <Arduino.h>
#include <Servo.h>

#ifndef Motor_h
#define Motor_h

class Motor{  
  public:
    void setMotorDefinitions(unsigned char startAngle, unsigned char beginLimit,  unsigned char endLimit, unsigned char attachPin);
    virtual void goTo(unsigned char angle) = 0;
    unsigned char reversed = 0;
    
  protected:
    unsigned char checkRange(unsigned char targetPos);
    unsigned char limitAngle[2] = {5,175};
    void fadeWrite(Servo objServo, int finalAngle, int delayms);
    Servo servo;  
};

class percentualMotor : public Motor {
  public:
    percentualMotor();
    void goTo(unsigned char angle) override;
};

class rawangleMotor : public Motor {
  public:
    rawangleMotor();
    void goTo(unsigned char angle) override;
};
#endif

