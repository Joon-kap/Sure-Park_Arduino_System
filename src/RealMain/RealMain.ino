#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <Servo.h>

#include <SPI.h>
#include <WiFi.h>
#include <Ethernet.h>
#include <WebSocketClient.h>

//#define PORTID  8080               // IP socket port ID
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


#define LED_BRIGHTNESS 255

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
IPAddress ipserver(192, 168, 1, 197); // The server's IP address  //EUNSANG
//IPAddress ipserver(192, 168, 1, 6); // The server's IP address  //JUNGAP


//char serverName[] = "192.168.1.197";

// WiFiClient client;                // The client (our) socket
IPAddress ip;                     // The IP address of the shield
IPAddress subnet;                 // The IP address of the shield
long rssi;                        // Wifi shield signal strength
byte mac[6];                      // Wifi shield MAC address
WebSocketClient sokClient;
WiFiClient getClient;
char pword[] = "16swarchitect";
String GateData;


#define MAX_SENSOR_NUMBER 6
char SENSOR_STATE[MAX_SENSOR_NUMBER+1];
char OLD_SENSOR_STATE[MAX_SENSOR_NUMBER+1];
char SERVER_STATE[MAX_SENSOR_NUMBER+1];

void setup() {
  int status = WL_IDLE_STATUS;      // Network connection status

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

  analogWrite(EntryGateGreenLED, LED_BRIGHTNESS);  // The gate LEDs are turned off by setting their pins
  analogWrite(EntryGateRedLED, 0);    // high. The reason for this is that they are
  analogWrite(ExitGateGreenLED, LED_BRIGHTNESS);   // 3 color LEDs with a common annode (+). So setting
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
    if ((sokClient.connect(ipserver, PORTID) != true)&&(status<=0))
    {
      Serial.println( "Connection retry..." );
      //status = sokClient.wifiBegin(ssid, pword);
      delay(delayvalue);
      continue;
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
#define LOOP_DELAY (100)
int loop_delay=0;
void loop() {
  loop_delay++;
  if(loop_delay == LOOP_DELAY) {
    loop_delay=0;
    if(UpdateSensorState()) {
      getPage(ipserver, "SENSORUPDATE", SENSOR_STATE);
    }
    UpdateActuator();
    delay(10);
  }
  sokClient.monitor();

}

void dataArrived(WebSocketClient sokClient, String data)
{
  Serial.println("Data Arrived: " + data);
  GateData = data;
  int strIndex;
  strIndex=GateData.indexOf("SERVERREQ_OPEN_EXIT");
  if(strIndex>=0) {
    Serial.print("SERVERREQ_OPEN_EXIT>>");
    ExitGateServo.write(Open);
  }
  strIndex=GateData.indexOf("SERVERREQ_OPEN_ENTRY");
  if(strIndex>=0) {
    Serial.print("SERVERREQ_OPEN_ENTRY>>");
    EntryGateServo.write(Open);
  }
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
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

  // Print the wireless signal strength:
  rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

} // printConnectionStatus



byte getPage(IPAddress ipBuf, char page[], char sensorstat[])
{
  //WiFiClient client = sokClient.getClient(); 
  int inChar;
  char outBuf[128];
  char strBuf[128];
  int status = 0;
  int timeout = 100;
#if 1  
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (getClient.connect(ipserver, 8080)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    
    snprintf(strBuf, 128, "GET /surepark_server/rev/test.do?%s=%s HTTP/1.1", page, sensorstat);
    getClient.println(strBuf);
    Serial.println(strBuf);
    //getClient.println("GET /surepark_server/rev/test.do?go=7 HTTP/1.1");

    getClient.println("Host: www.google.com");
    getClient.println("Connection: close");
    getClient.println();
    Serial.println("Respond:"); 
  
    timeout = 3000;
    String data = "";

    while(timeout--) {
      while (getClient.available()) {
        char c = getClient.read();
        data+=c;
        if((c=='\n')||(c=='}')) {
          int strindex = data.indexOf("RETUPDATE");
          if(strindex>0) {
            //Serial.println(data+strindex);
            Serial.println(data);
          }
          data = "";
        }
        //Serial.print(c);
      }
      delay(1);
    }
          String RETUPDATE = "RETUPDATE";
          int strindex = data.indexOf(RETUPDATE);
          if(strindex>=0) {
            //Serial.println(data+strindex);
            //data.substring(strindex);
            for(int i=0; i<MAX_SENSOR_NUMBER; i++) {
              SERVER_STATE[i] = data.charAt(strindex+3+i+RETUPDATE.length());
            }
            SERVER_STATE[MAX_SENSOR_NUMBER] = '\0';
            Serial.println(SERVER_STATE);
          }
   Serial.println("END!");
   getClient.flush();
   getClient.stop();
  }
  else
  {
    Serial.println(F("Connection failed"));
    getClient.flush();
    getClient.stop();
  }
#endif
}


bool UpdateSensorState() {
  bool ret = false;
  //LOW == Car exist, HIGH == No Car
  SENSOR_STATE[0] = (digitalRead(EntryBeamRcvr)==LOW)?'0':'1';
  SENSOR_STATE[1] = (digitalRead(ExitBeamRcvr)==LOW)?'0':'1';
  SENSOR_STATE[2] = (ProximityVal(Stall1SensorPin)<50)?'0':'1';
  SENSOR_STATE[3] = (ProximityVal(Stall2SensorPin)<50)?'0':'1';
  SENSOR_STATE[4] = (ProximityVal(Stall3SensorPin)<50)?'0':'1';
  SENSOR_STATE[5] = (ProximityVal(Stall4SensorPin)<50)?'0':'1';
  SENSOR_STATE[MAX_SENSOR_NUMBER] = '\0';
  OLD_SENSOR_STATE[MAX_SENSOR_NUMBER] = '\0';
  
  ret = false;
  //Compare with old sensors' value
  for(int i=0; i<MAX_SENSOR_NUMBER; i++) {
    if(OLD_SENSOR_STATE[i] != SENSOR_STATE[i]) {
      ret = true;
      break; 
    }
  }

  //Store to old value
  for(int i=0; i<MAX_SENSOR_NUMBER; i++) {
    OLD_SENSOR_STATE[i] = SENSOR_STATE[i];
  }
  return ret;
}

bool UpdateActuator() {
  bool ret = false;
  if(SENSOR_STATE[0]=='0') {
 //   EntryGateServo.write(Open);
    analogWrite(EntryGateGreenLED, 0);
    analogWrite(EntryGateRedLED, LED_BRIGHTNESS);
    delay( delayvalue );
  } else {
    EntryGateServo.write(Close);
    analogWrite(EntryGateGreenLED, LED_BRIGHTNESS);
    analogWrite(EntryGateRedLED, 0);
    delay( delayvalue );
  }
  if(SENSOR_STATE[1]=='0') {
//    ExitGateServo.write(Open);
    analogWrite(ExitGateGreenLED, 0);
    analogWrite(ExitGateRedLED, LED_BRIGHTNESS);
    delay( delayvalue );
  } else {
    ExitGateServo.write(Close);
    analogWrite(ExitGateGreenLED, LED_BRIGHTNESS);
    analogWrite(ExitGateRedLED, 0);
    delay( delayvalue );
  }
  if(SENSOR_STATE[2]=='0') {
    digitalWrite(ParkingStall1LED, LOW);
  } else {
    digitalWrite(ParkingStall1LED, HIGH);
  }
    if(SENSOR_STATE[3]=='0') {
    digitalWrite(ParkingStall2LED, LOW);
  } else {
    digitalWrite(ParkingStall2LED, HIGH);
  }
    if(SENSOR_STATE[4]=='0') {
    digitalWrite(ParkingStall3LED, LOW);
  } else {
    digitalWrite(ParkingStall3LED, HIGH);
  }
    if(SENSOR_STATE[5]=='0') {
    digitalWrite(ParkingStall4LED, LOW);
  } else {
    digitalWrite(ParkingStall4LED, HIGH);
  }
  return ret;
}

long ProximityVal(int Pin)
{
  long duration = 0;
  pinMode(Pin, OUTPUT);         // Sets pin as OUTPUT
  digitalWrite(Pin, HIGH);      // Pin HIGH
  delay(1);                     // Wait for the capacitor to stabilize

  pinMode(Pin, INPUT);          // Sets pin as INPUT
  digitalWrite(Pin, LOW);       // Pin LOW
  while (digitalRead(Pin))      // Count until the pin goes
  { // LOW (cap discharges)
    duration++;
  }
  return duration;              // Returns the duration of the pulse
}

