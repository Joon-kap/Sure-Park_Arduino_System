#include <Servo.h> 

#include <SPI.h>
#include <WiFi.h>
#include <WebSocketClient.h>

#define PORTID  550               // IP socket port ID

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

char ssid[] = "ASUS_Guest2";              // The network SSID for CMU unsecure network
char c;                           // Character read from server
int status = WL_IDLE_STATUS;      // Network connection status
IPAddress ipserver(192,168,1,169);  // The server's IP address
char chserver[] = "192.168.1.124";
// WiFiClient client;                // The client (our) socket
IPAddress ip;                     // The IP address of the shield
IPAddress subnet;                 // The IP address of the shield
long rssi;                        // Wifi shield signal strength
byte mac[6];                      // Wifi shield MAC address
WebSocketClient sokClient;
char pword[] = "16swarchitect";
String GateData;

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

//Serial.println("Attempting to connect to network...");
//Serial.print("SSID: ");
//Serial.println(ssid);
while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = sokClient.wifiBegin(ssid, pword);
    if (sokClient.connect(ipserver, PORTID)!= true)
    {
      Serial.println( "Connection retry..." );
      status = sokClient.wifiBegin(ssid, pword);
      delay(delayvalue);
    }
    else
    {
      sokClient.setDataArrivedDelegate(dataArrived);
      Serial.println( "\n----------------------------------------" );
      printConnectionStatus();  // Print the basic connection and network information
      Serial.println( "\n----------------------------------------\n" );
    }
  }
}
void loop() {
  sokClient.monitor();
  GateOpen();
  GateClose();
  CheckSpot();
}

void dataArrived(WebSocketClient sokClient, String data)
{
  Serial.println("Data Arrived: " + data);
  GateData = data;
}

void printConnectionStatus()
{
  // Print the basic connection and network information: Network, IP, and Subnet mask
  ip = WiFi.localIP();
  Serial.print("Connected to ");
  Serial.print(ssid);
  Serial.print(" IP Address:: ");
  Serial.println(ip);
  subnet = WiFi.subnetMask();
  Serial.print("Netmask: ");
  Serial.println(subnet);
   
  // Print our MAC address.
  WiFi.macAddress(mac);
  Serial.print("WiFi Shield MAC address: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
   
  // Print the wireless signal strength:
  rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  
} // printConnectionStatus

long GateOpen() {
  EntryBeamState = digitalRead(EntryBeamRcvr);
  if (EntryBeamState == LOW && GateData == "open")
  {
    //Serial.println( "Open Entry Gate" );   //Here we open the entry gate
    EntryGateServo.write(Open);
    analogWrite(EntryGateGreenLED, 0);
    analogWrite(EntryGateRedLED, 255);
    delay( delayvalue );
  }

  ExitBeamState = digitalRead(ExitBeamRcvr);
  if (ExitBeamState == LOW)
  {
    //Serial.println( "Open Exit Gate" );   //Here we open the exit gate
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
    //Serial.println( "Close Entry Gate" );   //Here we close the exit gate
    EntryGateServo.write(Close);
    analogWrite(EntryGateGreenLED, 255);
    analogWrite(EntryGateRedLED, 0);
    GateData = "close";
    delay( delayvalue );
  }

  ExitBeamState = digitalRead(ExitBeamRcvr);
  if (ExitBeamState == HIGH)
  {
    //Serial.println( "Close Exit Gate" );   //Here we close the exit gate
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
  //Serial.print("  Stall 1 = ");
  //Serial.print(Stall1SensorVal);
  if (Stall1SensorVal < 50)
  {
    //Serial.println( "Turn off stall 1 LED" );
    digitalWrite(ParkingStall1LED, LOW);
  }
  else
  {
    //Serial.println( "Turn on stall 1 LED" );
    digitalWrite(ParkingStall1LED, HIGH);
  }

  Stall2SensorVal = ProximityVal(Stall2SensorPin); //Check parking space 2
  //Serial.print("  Stall 2 = ");
  //Serial.print(Stall2SensorVal);
  if (Stall2SensorVal < 50)
  {
    //Serial.println( "Turn off stall 2 LED" );
    digitalWrite(ParkingStall2LED, LOW);
  }
  else
  {
    //Serial.println( "Turn on stall 2 LED" );
    digitalWrite(ParkingStall2LED, HIGH);
  }

  Stall3SensorVal = ProximityVal(Stall3SensorPin); //Check parking space 3
  //Serial.print("  Stall 3 = ");
  //Serial.print(Stall3SensorVal);
  if (Stall3SensorVal < 50)
  {
    //Serial.println( "Turn off stall 3 LED" );
    digitalWrite(ParkingStall3LED, LOW);
  }
  else
  {
    //Serial.println( "Turn on stall 3 LED" );
    digitalWrite(ParkingStall3LED, HIGH);
  }

  Stall4SensorVal =  ProximityVal(Stall4SensorPin); //Check parking space 4
  //Serial.print("  Stall 4 = ");
  //Serial.println(Stall4SensorVal);
  if (Stall4SensorVal < 50)
  {
    //Serial.println( "Turn off stall 4 LED" );
    digitalWrite(ParkingStall4LED, LOW);
  }
  else
  {
    //Serial.println( "Turn on stall 4 LED" );
    digitalWrite(ParkingStall4LED, HIGH);
  }
}

