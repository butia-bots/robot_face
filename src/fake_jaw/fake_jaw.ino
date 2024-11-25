#import <Servo.h>

Servo JAW_RIGHT;
Servo JAW_LEFT;

int close_L = 97;
int open_L = 125;
int close_R = 100;
int open_R = 80;

  void fake_jaw(){

    JAW_RIGHT.write(open_R);
    JAW_LEFT.write(open_L);

    delay(400);

    JAW_RIGHT.write(close_R);
    JAW_LEFT.write(close_L);

    delay(400);
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  EYELID_RIGHT.attach(11);
  EYELID_LEFT.attach(12);
}

void loop() {

  blink();
     
}