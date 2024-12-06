#include <Servo.h>

Servo JAW_RIGHT;
Servo JAW_LEFT;
Servo EYELID_RIGHT;
Servo EYELID_LEFT;

int close_EL = 100;
int open_EL = 20;
int close_ER = 100;
int open_ER = 0;

int close_L = 94; //MAX 100
int open_L = 85; // diminui p/ abrir +
int close_R = 92; //
int open_R = 105; // aumenta  p/ abrir+

  void fakejaw(){

    JAW_RIGHT.write(open_R);
    JAW_LEFT.write(open_L);

    delay(400);

    JAW_RIGHT.write(close_R);
    JAW_LEFT.write(close_L);

    delay(400);
  }
  void blink(){

    delay(1500);

    EYELID_RIGHT.write(close_ER);
    EYELID_LEFT.write(close_EL);

    delay(200);
    
    EYELID_RIGHT.write(open_ER);
    EYELID_LEFT.write(open_EL);


  }


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  JAW_RIGHT.attach(11);
  JAW_LEFT.attach(12);
  EYELID_RIGHT.attach(8);
  EYELID_LEFT.attach(10);
}

void loop() {

  fakejaw();
  blink();
     
}
