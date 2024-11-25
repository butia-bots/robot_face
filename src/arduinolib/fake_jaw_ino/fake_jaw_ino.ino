// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo left;  // create servo object to control a servo 
Servo right;             // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  left.attach(11);  
  right.attach(12);// attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{ 
                                 // in steps of 1 degree 
    left.write(125);
    right.write(80);      // tell servo to go to position in variable 'pos' 
    delay(200);
    
    left.write(97);
    right.write(100);      // tell servo to go to position in variable 'pos' 
    delay(200);    // waits 15ms for the servo to reach the position 

} 
