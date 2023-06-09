/*******************************************************************************
 * Title                 :   PPG and Buffer jig firmware
 * Filename              :   main.cpp
 * Author                :   Truong VN
 * Origin Date           :   08/06/2023
 * Notes                 :   MCU is STM32F103C8
 *******************************************************************************/

/*******************************************************************************
 * INCLUDE
 *******************************************************************************/
#include <Arduino.h>
// #include <MAX30105.h>
#include <Wire.h>

/******************************************************************************
 * PREPROCESSOR CONSTANTS
 *******************************************************************************/
#define BUTTON_ON LOW
#define BUTTON_OFF HIGH
#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define BUFFER_ON HIGH
#define BUFFER_OFF LOW
#define LED_ON HIGH
#define LED_OFF LOW

/******************************************************************************
 * IO DEFINE
 *******************************************************************************/
// Input
#define BT_START PB11
#define BUFFER1 PA8
#define BUFFER2 PB8
#define BUFFER3 PB9
#define BUFFER4 PA11
#define BUFFER5 PA12
// Output
#define LED_PWR PC13
#define LED_PASS PC14
#define LED_FAIL PC15
#define LED_RUN PB10
#define RL_OUT1 PA6
#define RL_OUT2 PA5
#define RL_OUT3 PA4
#define RL_OUT4 PA3
#define RL_OUT5 PA2
#define RL_OUT6 PA1
#define RL_OUT7 PA0

// I2C pin
#define PIN_SDA PB9
#define PIN_SCL PB8

/******************************************************************************
 * VARIABLE DEFINITIONS
 *******************************************************************************/

/******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************/
void setAllRelayOutput(bool status);
void setAllLedOutput(bool status);
bool checkBufferResult(bool status);
void setAllOutput(bool status);
bool testBufferSequence();
void i2cScanner();

/******************************************************************************
 * SETUP FUNCTION
 *******************************************************************************/
void setup()
{
  // Set mode for ouput pin
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_PASS, OUTPUT);
  pinMode(LED_FAIL, OUTPUT);
  pinMode(LED_RUN, OUTPUT);
  pinMode(RL_OUT1, OUTPUT);
  pinMode(RL_OUT2, OUTPUT);
  pinMode(RL_OUT3, OUTPUT);
  pinMode(RL_OUT4, OUTPUT);
  pinMode(RL_OUT5, OUTPUT);
  pinMode(RL_OUT6, OUTPUT);
  pinMode(RL_OUT7, OUTPUT);

  // Set mode for input pin
  pinMode(BT_START, INPUT_PULLUP);
  pinMode(BUFFER1, INPUT_PULLUP);
  pinMode(BUFFER2, INPUT_PULLUP);
  pinMode(BUFFER3, INPUT_PULLUP);
  pinMode(BUFFER4, INPUT_PULLUP);
  pinMode(BUFFER5, INPUT_PULLUP);

  // Set init pin state
  setAllOutput(HIGH);
  delay(1000);
  setAllOutput(LOW);
  delay(1000);
  digitalWrite(LED_PWR, HIGH);

  // // Setup I2C port
  Wire.setSCL(PIN_SCL);
  Wire.setSDA(PIN_SDA);
  Wire.begin(); 

  // On serial port
  Serial1.begin(9600);
  Serial1.println("Board test ready.");
}

/******************************************************************************
 * LOOP FUNCTION
 *******************************************************************************/
void loop()
{
  bool checkResult = false;
  i2cScanner();
  // Button start is press
  if (digitalRead(BT_START) == BUTTON_ON)
  {

    Serial1.println("Button start ON, Start running.");
    digitalWrite(LED_PASS, LED_OFF);
    digitalWrite(LED_FAIL, LED_OFF);
    digitalWrite(LED_RUN, LED_ON);

    if (testBufferSequence())
    {
      // if test buffer pass
      Serial1.println("Buffer test PASS");
      digitalWrite(LED_PASS, LED_ON);
      digitalWrite(LED_FAIL, LED_OFF);
      digitalWrite(LED_RUN, LED_OFF);
    }
    else
    {
      // if test buffer fail
      Serial1.println("Buffer test FAIL");
      digitalWrite(LED_PASS, LED_OFF);
      digitalWrite(LED_FAIL, LED_ON);
      digitalWrite(LED_RUN, LED_OFF);
    }

    // Wait start button release
    while (digitalRead(BT_START) == BUTTON_ON)
    {
      Serial1.println("Please release START button.");
      delay(100);
    }
  }
}

/******************************************************************************
 * FUNCTION DEFINE
 *******************************************************************************/

/**
 * @brief Set status level for all Relay Output
 *
 * @param status LOW or HIGH
 */
void setAllRelayOutput(bool status)
{
  digitalWrite(RL_OUT1, status);
  digitalWrite(RL_OUT2, status);
  digitalWrite(RL_OUT3, status);
  digitalWrite(RL_OUT4, status);
  digitalWrite(RL_OUT5, status);
  digitalWrite(RL_OUT6, status);
  digitalWrite(RL_OUT7, status);
}

/**
 * @brief Set status level for all LED Output
 *
 * @param status LOW or HIGH
 */
void setAllLedOutput(bool status)
{
  digitalWrite(LED_PWR, status);
  digitalWrite(LED_PASS, status);
  digitalWrite(LED_FAIL, status);
  digitalWrite(LED_RUN,  status);
}

/**
 * @brief Read buffer input pins and compare with status
 *
 * @param status logic level (LOW or HIGH) to compare
 * @return true if match,
 * @return false if not match.
 */
bool checkBufferResult(bool status)
{
  bool result = true;
  result &= (digitalRead(BUFFER1) == status);
  result &= (digitalRead(BUFFER2) == status);
  result &= (digitalRead(BUFFER3) == status);
  result &= (digitalRead(BUFFER4) == status);
  result &= (digitalRead(BUFFER5) == status);
  return result;
}

/**
 * @brief Set status level for All Output pin
 *
 * @param status
 */
void setAllOutput(bool status)
{
  if (status)
  {
    setAllLedOutput(LED_ON);
    setAllRelayOutput(RELAY_ON);
  }
  else
  {
    setAllLedOutput(LED_OFF);
    setAllRelayOutput(RELAY_OFF);
  }
}

bool testBufferSequence()
{
  // Test with HIGH signal
  setAllRelayOutput(RELAY_ON);
  delay(1000);
  // If check fail
  if (!checkBufferResult(BUFFER_ON))
  {
    setAllRelayOutput(RELAY_OFF);
    return false;
  }
  setAllRelayOutput(RELAY_OFF);
  delay(1000);
  // If check fail
  if (!checkBufferResult(BUFFER_OFF))
  {
    return false;
  }
  return true;
}

void i2cScanner()
{
  int nDevices = 0;

  Serial1.println("Scanning...");

  for (byte address = 1; address < 127; ++address)
  {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      Serial1.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial1.print("0");
      }
      Serial1.print(address, HEX);
      Serial1.println("  !");

      ++nDevices;
    }
    else if (error == 4)
    {
      Serial1.print("Unknown error at address 0x");
      if (address < 16)
      {
        Serial1.print("0");
      }
      Serial1.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial1.println("No I2C devices found\n");
  }
  else
  {
    Serial1.println("done\n");
  }
  delay(3000); // Wait 5 seconds for next scan
}


/*************** END OF FILES *************************************************/