#import <Servo.h>

Servo JAW_RIGHT;
Servo JAW_LEFT;
Servo EYELID_RIGHT;
Servo EYELID_LEFT;

int close_L = 100;
int open_L = 20;
int close_R = 100;
int open_R = 0;

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
  void blink(){

    delay(1500);

    EYELID_RIGHT.write(close_R);
    EYELID_LEFT.write(close_L);

    delay(200);
    
    EYELID_RIGHT.write(open_R);
    EYELID_LEFT.write(open_L);


  }


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  EYELID_RIGHT.attach(11);
  EYELID_LEFT.attach(12);
}

void loop() {

  fake_jaw();
  blink();
     
}