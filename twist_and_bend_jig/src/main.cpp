#include <Arduino.h>
#include <Servo.h>

Servo servoLeft, servoRight;  // create servo object to control a servo
// twelve servo objects can be created on most boards

#define pos_Min 0
#define pos_Max 180
#define pos_Init 90
#define angle_test 60  // angle to test twist

#define COUNT_TARGET 17500

#define speedTime 10   // delay when increase 1 degree
/**
 * 1 cycle degree =  target_Min -> target_MAX -> target_MIN = 240 degree
 * 1 cycle time = 1 cycle degree * speedTime  + 2 * delay time = 240 * 10 + 2 *100 = 2600 ms
 * Angle Speed = 1 degree / 10 ms = 100 degree / second = 1.745 rad/s
 */

int pos = 0;    // variable to store the servo position
unsigned int counter = 0;
int pos_target_min = pos_Init - angle_test;
int pos_target_max = pos_Init + angle_test;

void setup() {
  servoLeft.attach(3);  // attaches the servo on pin 9 to the servo object
  servoRight.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  servoLeft.write(pos_Init);              // tell servo to go to position in variable 'pos'
  servoRight.write(pos_Init);              // tell servo to go to position in variable 'pos'
  Serial.println("Set motor at init position done.");
  if ( (pos_Init + angle_test) > pos_Max)
  {
    Serial.println("Error: Angle test over angle Max!!!");
    while (1);
  }
  delay(5000);                       // waits 15ms for the servo to reach the position
  for (pos = pos_Init;  pos >= pos_target_min; pos -= 1) 
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
    servoRight.write(pos);              // tell servo to go to position in variable 'pos'
    delay(speedTime);                       // waits 15ms for the servo to reach the position
  }
  delay(100);
}

void loop() {
  
  digitalWrite(LED_BUILTIN, HIGH);
  for (pos = pos_target_min; pos <= pos_target_max; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
    servoRight.write(pos);              // tell servo to go to position in variable 'pos'
    delay(speedTime);                       // waits 15ms for the servo to reach the position
  }
  // counter++;
  // Serial.print("Counter: ");
  // Serial.println(counter);
  delay(100);

  digitalWrite(LED_BUILTIN, LOW);
  for (pos = pos_target_max; pos >= pos_target_min; pos -= 1) { // goes from 180 degrees to 0 degrees
    servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
    servoRight.write(pos);              // tell servo to go to position in variable 'pos'
    delay(speedTime);                       // waits 15ms for the servo to reach the position
  }
  counter++;
  Serial.print("Counter: ");
  Serial.println(counter);
  delay(100);
  if ( counter >= COUNT_TARGET)
  {
    for (pos = pos_target_min;  pos <= pos_Init; pos += 1) 
    { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servoLeft.write(pos);              // tell servo to go to position in variable 'pos'
      servoRight.write(pos);              // tell servo to go to position in variable 'pos'
      delay(speedTime);                       // waits 15ms for the servo to reach the position
    }
    while (1)
    {
      Serial.println("Test twist complete!");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
    
  }
}