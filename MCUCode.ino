// #CAN-NODE NAME: G1
// #MOTOR NODE: ?
// #CAN ID BEING SENT: 0x100 (SENSOR), 0x200 (ENCODER)
// #DATA LENGTH: ? BYTES
// #ACCEPTS RASPI JOY CAN ID: x500
// #E-STOP ID: 0x600

#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include "AS5600.h"

AS5600L as5600;

// # PINOUT #

//CS pin for ESP32s
MCP2515 mcp2515(5);
//MOSI     - D23
//SCK     - D18
//MISO     - D19
//CS     - D5
//GND    - GND
//5V     - VIN
//3V3     - 3V3

//Motor
#define ENA 33;
#define DIR 32;
#define PUL 25;
//Encoder
#define SCL 22;
#define SDA 21;
//Sensor
#define CURR 26;

// # VARIABLES #

//External Data
int data1 = 0; //Current sensor data
float data2 = 0; //Encoder data

//Data to be sent
int steps = 0; //Amount of steps to move

//Boolean valules for checking
bool changeS; //Sensor data changed
bool changeE; //Encoder data changed

//Encoder angles
int magnetStat = 0;
int lowbyt; 
word highbyt;
int rawAngle; 
float degAngle;
float correctedAngle = 0;
float startAngle = 0; 
float totalAngle = 0;
float stepAngle = 1.8; //Degrees per step
float enc = 0;

//Sensor data
int curr = 0;


// ## FUNCTIONS AND CODE ##

void setup() {
  //Start serial output
  Serial.begin(115200);
  while(!Serial); //Wait for serial to open

  //Set LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Onboard LED Indicator Initialized.")

  //Declare Encoder pins
  Wire.begin(21, 22); // SDA, SCL
  as5600.begin(); 
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.println("Encoder Initialized.");
  delay(100);

  // Declare motor pins as output:
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, HIGH);
  digitalWrite(enablePin, LOW);
  Serial.println("Motor Initialized.");
  delay(100);

  //Initialize SPI
  SPI.begin();
  Serial.println("SPI Initialized.");
  delay(100);

  //Initialize CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  Serial.println("MCP Initialized.");


  //Final print statements for startup
  Serial.println("System fully operational, Boss. CAN is a-go.");
  Serial.println("--------------------------------------------");

  delay(100);
}

void loop() {

  //Check data from peripherals
  encData();
  sensorData();

  //Check for CAN messages
  recieveCAN();

  //If sensor data has updated, send it
  if(changeS)
  {
    sendCAN(0x100, data2); //CAN Sensor ID is 0x100
    changeS = false; //Sensor data sent, reset the change
  }

  //If encoder data has updated, send it
  if(changeE)
  {
    sendCAN(0x200, data2); //CAN Encoder ID is 0x200
    changeE = false; //Encoder data sent, reset the change
  }

  delay(1000); // Send every second
}

//Send CAN frame
void sendCAN(can_id, data)
{
  struct can_frame canMsg;
  canMsg.can_id = can_id;    // CAN ID
  canMsg.can_dlc = 2;       // Data length (1 byte)
  canMsg.data[0] = data;   // Set data byte to current value

  // Send the message
  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id = data1)
    {
      Serial.print("Sent Sensor Data: ");
      Serial.println(data);
    }
    if(canMsg.can_id = data2)
    {
      Serial.print("Sent Encoder Data: ");
      Serial.println(data);
    }

    delay(500); //Delay to not overload system
  }
  else 
  {
    Serial.println("Failed to send message!");
    blink();  //Blink LED 10 times to signal SEND error without serial print
  }

}

//Read CAN frame
void recieveCAN()
{
  struct can_frame canMsg;

  // Check if a new message is available
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    //Joystick CAN ID
    if (canMsg.can_id == 0x500) { // Check CAN ID
      int receivedValue = canMsg.data[0];
      Serial.print("Received: ");
      Serial.println(receivedValue);

      drive();
    }
    //E-stop CAN ID
    if(canMsg.can_id == 0x600)
    {
      freeze();
    }
  }
}

//Current sensor data
void sensorData()
{
  //Record original current data
  curr = data1;

  //Read from sensor data
  data1 = ;
  //If previous data is not equivalent to current data, there has been a change
  if(curr != data1){
    changeS = true; //Sensor data has updated
  }
}

//Encoder data
void encData()
{
  //Record original encoder data
  enc = data2;
  // Read the angle in degrees
  data2 = as5600.readAngle();

  if(enc != data2)
  {
    // Print the angle to the Serial Monitor
    Serial.print("Angle: ");
    Serial.println(data2);
    changeE = true; //Encoder data has updated
  }
      
  delay(500); // Delay for half a second
}

//Drive motor
void drive(int val)
{
  //Motor driving behaviour here. May need library to automate this.

}

//Freeze all MCUs (Emergency Stop)
void freeze()
{
  struct can_frame canMsg;
  bool reset = false;

  while(!reset)
  {
    //If button is re-pressed, unfreeze 
    if(mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
      //E-stop CAN ID
      if(canMsg.can_id == 0x600) {
        reset = true;
      }
    }
  }
  //Reset motors back to default state! This will need to record where the motors are. Reset them to 0? We'll have to record data.
}

//Blink LED 10 times to signal send error
void blink()
{
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}
