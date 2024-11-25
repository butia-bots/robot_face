#include <Servo.h>

Servo EYELID_RIGHT;
Servo EYELID_LEFT;

int close_L = 100;
int open_L = 20;
int close_R = 100;
int open_R = 0;

  void blink(){

    EYELID_RIGHT.write(close_R);
    EYELID_LEFT.write(close_L);

    delay(200);
    
    EYELID_RIGHT.write(open_R);
    EYELID_LEFT.write(open_L);

    delay(200);


  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  EYELID_RIGHT.attach(8);
  EYELID_LEFT.attach(10);
}

void loop() {

  delay(1500);

  blink();

     
}
