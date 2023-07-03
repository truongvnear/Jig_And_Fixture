#include <Arduino.h>
#include <Servo.h>

Servo servoLeft, servoRight;  // create servo object to control a servo
// twelve servo objects can be created on most boards

#define pos_Min 0
#define pos_Max 90
#define speedTime 20

int pos = 0;    // variable to store the servo position
unsigned int counter = 0;

void setup() {
  servoLeft.attach(3);  // attaches the servo on pin 9 to the servo object
  servoRight.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  
  digitalWrite(LED_BUILTIN, HIGH);
  for (pos = pos_Min; pos <= pos_Max; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
    servoRight.write(pos);              // tell servo to go to position in variable 'pos'
    delay(speedTime);                       // waits 15ms for the servo to reach the position
  }
  counter++;
  delay(100);
  Serial.print("Counter: ");
  Serial.println(counter);

  digitalWrite(LED_BUILTIN, LOW);
  for (pos = pos_Max; pos >= pos_Min; pos -= 1) { // goes from 180 degrees to 0 degrees
    servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
    servoRight.write(pos);              // tell servo to go to position in variable 'pos'
    delay(speedTime);                       // waits 15ms for the servo to reach the position
  }
  counter++;
  Serial.print("Counter: ");
  Serial.println(counter);
}