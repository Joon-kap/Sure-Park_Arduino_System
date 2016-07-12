/****************************************************************
* File: LEDTest
* Project: LG Exec Ed Program
* Copyright: Copyright (c) 2016 Anthony J. Lattanze
* Versions:
* 1.0 May 2016 - Initial version
*
* Description:
*
* This program simply turns on and off all the LEDs on the parking
* lot game board, Note that the entry and exit LEDs can be both
* green and red (blue as well, but the blue pin bas been cliped).
*
* Parameters: None
*
* Internal Methods: None
***************************************************************/

#include <Servo.h> 

#define EntryGateGreenLED 26
#define EntryGateRedLED   27
#define ExitGateGreenLED  28
#define ExitGateRedLED    29
#define ParkingStall1LED  22
#define ParkingStall2LED  23
#define ParkingStall3LED  24
#define ParkingStall4LED  25

int delayvalue = 1000;

void setup() 
{ 
  pinMode(EntryGateGreenLED, OUTPUT);    // This section makes all the LED pins outputs.
  pinMode(EntryGateRedLED, OUTPUT);
  pinMode(ExitGateGreenLED, OUTPUT);
  pinMode(ExitGateRedLED, OUTPUT);
  pinMode(ParkingStall1LED, OUTPUT);
  pinMode(ParkingStall2LED, OUTPUT);
  pinMode(ParkingStall3LED, OUTPUT);
  pinMode(ParkingStall4LED, OUTPUT);

  digitalWrite(EntryGateGreenLED, HIGH);  // The gate LEDs are turned off by setting their pins
  digitalWrite(EntryGateRedLED, HIGH);    // high. The reason for this is that they are
  digitalWrite(ExitGateGreenLED, HIGH);   // 3 color LEDs with a common annode (+). So setting
  digitalWrite(ExitGateRedLED, HIGH);     // any of the other 3 legs low turns on the LED.
  
  digitalWrite(ParkingStall1LED, LOW);    // Standard LEDs are used for the parking stall
  digitalWrite(ParkingStall2LED, LOW);    // LEDs. Set the pin high and they light.
  digitalWrite(ParkingStall3LED, LOW);
  digitalWrite(ParkingStall4LED, LOW);

  Serial.begin(9600);
} 

void loop() 
{
  Serial.println( "Turn on entry red LED" );
  digitalWrite(EntryGateRedLED, LOW);
  delay( delayvalue );
  digitalWrite(EntryGateRedLED, HIGH);

  Serial.println( "Turn on entry green LED" );
  digitalWrite(EntryGateGreenLED, LOW);
  delay( delayvalue );
  digitalWrite(EntryGateGreenLED, HIGH);
  
  Serial.println( "Turn on stall 1 LED" );
  digitalWrite(ParkingStall1LED, HIGH);
  delay( delayvalue );
  digitalWrite(ParkingStall1LED, LOW);

  Serial.println( "Turn on stall 2 LED" );
  digitalWrite(ParkingStall2LED, HIGH);
  delay( delayvalue );
  digitalWrite(ParkingStall2LED, LOW);
  
  Serial.println( "Turn on stall 3 LED" );
  digitalWrite(ParkingStall3LED, HIGH);
  delay( delayvalue );
  digitalWrite(ParkingStall3LED, LOW);
  
  Serial.println( "Turn on stall 4 LED" );
  digitalWrite(ParkingStall4LED, HIGH);
  delay( delayvalue );
  digitalWrite(ParkingStall4LED, LOW);
  
  Serial.println( "Turn on exit red LED" );
  digitalWrite(ExitGateRedLED, LOW);
  delay( delayvalue );
  digitalWrite(ExitGateRedLED, HIGH);

  Serial.println( "Turn on exit green LED" );
  digitalWrite(ExitGateGreenLED, LOW);
  delay( delayvalue );
  digitalWrite(ExitGateGreenLED, HIGH);

} 
