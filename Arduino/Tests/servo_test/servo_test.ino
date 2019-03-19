#include <Servo.h>


Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
myservo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
myservo.writeMicroseconds(1490);


/*
 * Servo1:
 * 
 * 0 degrees        760    NOT REALLY 0 degrees  
 * 90 degrees       1490       
 * 180 degrees      2255   NOT REALLY 180 degrees
 *  
 * 
 * Servo2:
 * 
 * 0 degrees      765       NOT REALLY 0 degrees  constrain to 
 * 90 degrees     1465
 * 180 degrees*   2255      NOT REALLY 180 degrees constrain to
 * 
 * 
 * Servo3:
 *  
 * 0 degrees      :900  constrain to 900
 * 90 degrees     :1410
 * 180 degrees    :1920 constrain to 1900
 */


}
