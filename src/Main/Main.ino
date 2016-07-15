#include <Servo.h> 

#define EntryGateServoPin 5
#define ExitGateServoPin 6
#define Open  90
#define Close 0

#define EntryBeamRcvr  34 
#define ExitBeamRcvr   35

#define EntryGateGreenLED 26
#define EntryGateRedLED   27
#define ExitGateGreenLED  28
#define ExitGateRedLED    29
#define ParkingStall1LED  22
#define ParkingStall2LED  23
#define ParkingStall3LED  24
#define ParkingStall4LED  25

#define Stall1SensorPin 30
#define Stall2SensorPin 31
#define Stall3SensorPin 32
#define Stall4SensorPin 33

int EntryBeamState;
int ExitBeamState;

int delayvalue = 1000;
Servo EntryGateServo;
Servo ExitGateServo;

long  Stall1SensorVal;
long  Stall2SensorVal;
long  Stall3SensorVal;
long  Stall4SensorVal;    

void setup() {
pinMode(EntryBeamRcvr, INPUT);     // Make entry IR rcvr an input
digitalWrite(EntryBeamRcvr, HIGH); // enable the built-in pullup

pinMode(ExitBeamRcvr, INPUT);      // Make exit IR rcvr an input
digitalWrite(ExitBeamRcvr, HIGH);  // enable the built-in pullup

EntryGateServo.attach(EntryGateServoPin);
EntryGateServo.write(Close);
ExitGateServo.attach(ExitGateServoPin);
ExitGateServo.write(Close);  

pinMode(EntryGateGreenLED, OUTPUT);    // This section makes all the LED pins outputs.
pinMode(EntryGateRedLED, OUTPUT);
pinMode(ExitGateGreenLED, OUTPUT);
pinMode(ExitGateRedLED, OUTPUT);
pinMode(ParkingStall1LED, OUTPUT);
pinMode(ParkingStall2LED, OUTPUT);
pinMode(ParkingStall3LED, OUTPUT);
pinMode(ParkingStall4LED, OUTPUT);

analogWrite(EntryGateGreenLED, 255);  // The gate LEDs are turned off by setting their pins
analogWrite(EntryGateRedLED, 0);    // high. The reason for this is that they are
analogWrite(ExitGateGreenLED, 255);   // 3 color LEDs with a common annode (+). So setting
analogWrite(ExitGateRedLED, 0);     // any of the other 3 legs low turns on the LED.

digitalWrite(ParkingStall1LED, HIGH);    // Standard LEDs are used for the parking stall
digitalWrite(ParkingStall2LED, HIGH);    // LEDs. Set the pin high and they light.
digitalWrite(ParkingStall3LED, HIGH);
digitalWrite(ParkingStall4LED, HIGH);

Serial.begin(9600);
}

void loop() {
  GateOpen();
  GateClose();
  CheckSpot();
}

long GateOpen() {
  EntryBeamState = digitalRead(EntryBeamRcvr);
  if (EntryBeamState == LOW)
  {
    Serial.println( "Open Entry Gate" );   //Here we open the entry gate
    EntryGateServo.write(Open);
    analogWrite(EntryGateGreenLED, 0);
    analogWrite(EntryGateRedLED, 255);
    delay( delayvalue );
  }

  ExitBeamState = digitalRead(ExitBeamRcvr);
  if (ExitBeamState == LOW)
  {
    Serial.println( "Open Exit Gate" );   //Here we open the exit gate
    ExitGateServo.write(Open);
    analogWrite(ExitGateGreenLED, 0);
    analogWrite(ExitGateRedLED, 255);
    delay( delayvalue );
  }
}

long GateClose() {
  EntryBeamState = digitalRead(EntryBeamRcvr);
  if (EntryBeamState == HIGH)
  {
    Serial.println( "Close Entry Gate" );   //Here we close the exit gate
    EntryGateServo.write(Close);
    analogWrite(EntryGateGreenLED, 255);
    analogWrite(EntryGateRedLED, 0);
    delay( delayvalue );
  }

  ExitBeamState = digitalRead(ExitBeamRcvr);
  if (ExitBeamState == HIGH)
  {
    Serial.println( "Close Exit Gate" );   //Here we close the exit gate
    ExitGateServo.write(Close);
    analogWrite(ExitGateGreenLED, 255);
    analogWrite(ExitGateRedLED, 0);
    delay( delayvalue );
  }
}

long ProximityVal(int Pin)
{
    long duration = 0;
    pinMode(Pin, OUTPUT);         // Sets pin as OUTPUT
    digitalWrite(Pin, HIGH);      // Pin HIGH
    delay(1);                     // Wait for the capacitor to stabilize

    pinMode(Pin, INPUT);          // Sets pin as INPUT
    digitalWrite(Pin, LOW);       // Pin LOW
    while(digitalRead(Pin))       // Count until the pin goes
    {                             // LOW (cap discharges)
       duration++;                
    }   
    return duration;              // Returns the duration of the pulse
}

long CheckSpot() {
  Stall1SensorVal = ProximityVal(Stall1SensorPin); //Check parking space 1
  Serial.print("  Stall 1 = ");
  Serial.print(Stall1SensorVal);
  if (Stall1SensorVal < 50)
  {
    Serial.println( "Turn off stall 1 LED" );
    digitalWrite(ParkingStall1LED, LOW);
  }
  else
  {
    Serial.println( "Turn on stall 1 LED" );
    digitalWrite(ParkingStall1LED, HIGH);
  }

  Stall2SensorVal = ProximityVal(Stall2SensorPin); //Check parking space 2
  Serial.print("  Stall 2 = ");
  Serial.print(Stall2SensorVal);
  if (Stall2SensorVal < 50)
  {
    Serial.println( "Turn off stall 2 LED" );
    digitalWrite(ParkingStall2LED, LOW);
  }
  else
  {
    Serial.println( "Turn on stall 2 LED" );
    digitalWrite(ParkingStall2LED, HIGH);
  }

  Stall3SensorVal = ProximityVal(Stall3SensorPin); //Check parking space 3
  Serial.print("  Stall 3 = ");
  Serial.print(Stall3SensorVal);
  if (Stall3SensorVal < 50)
  {
    Serial.println( "Turn off stall 3 LED" );
    digitalWrite(ParkingStall3LED, LOW);
  }
  else
  {
    Serial.println( "Turn on stall 3 LED" );
    digitalWrite(ParkingStall3LED, HIGH);
  }

  Stall4SensorVal =  ProximityVal(Stall4SensorPin); //Check parking space 4
  Serial.print("  Stall 4 = ");
  Serial.println(Stall4SensorVal);
  if (Stall4SensorVal < 50)
  {
    Serial.println( "Turn off stall 4 LED" );
    digitalWrite(ParkingStall4LED, LOW);
  }
  else
  {
    Serial.println( "Turn on stall 4 LED" );
    digitalWrite(ParkingStall4LED, HIGH);
  }
}

