#include <Arduino.h>

// Declare IO pin
#define CLAIM_COIL_PIN 2

// Define status
#define CLAIM_ON HIGH
#define CLAIM_OFF LOW

// Define parameter
#define TIME_IN   2000  // time move in (ms)
#define TIME_WAIT 2000  // time waiting (ms)
#define TIME_OUT  2000  // time move out (ms)

#define COUNT_TARGET 17520

unsigned int counter = 0;

// Function declare
void led_blink(unsigned int count, unsigned int delay_ms);
void print_counter();
void check_finish();
void jig_process_action();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CLAIM_COIL_PIN, OUTPUT);
  digitalWrite(CLAIM_COIL_PIN, CLAIM_OFF);
  led_blink(3, 100);
  Serial.begin(9600);
  Serial.println("Start claim test: ");
}

void loop() {
  
  jig_process_action();
  counter++;
  print_counter();
  check_finish();
  
}

// Function implement
void jig_process_action()
{
  // Step 1: press, move in
  digitalWrite(CLAIM_COIL_PIN, CLAIM_ON);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(TIME_IN);
  // Step 2: waiting press
  delay(TIME_WAIT);
  // Step 3: unpress, move out
  digitalWrite(CLAIM_COIL_PIN, CLAIM_OFF);
  digitalWrite(LED_BUILTIN, LOW);
  delay(TIME_OUT);

}

void print_counter()
{
  Serial.print("Counter: ");
  Serial.println(counter);
}

void check_finish()
{
  if ( counter >= COUNT_TARGET)
  {
    while (1)
    {
      Serial.println("Test twist complete!");
      led_blink(1, 50);
    }
  }
}

void led_blink(unsigned int count, unsigned int delay_ms)
{
  for (unsigned int i = 0; i < count; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_ms);
  }
  
}