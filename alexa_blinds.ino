/* Example sketch to control a 28BYJ-48 stepper motor with ULN2003 driver board, AccelStepper and Arduino UNO: continuous rotation. More info: https://www.makerguides.com */

#include <SPI.h>
#include <WiFiNINA.h>
#include <AccelStepper.h>

#include <Espalexa.h>

#include "wifi_secrets.h"

#define SERIAL_BAUDRATE 9600

// Motor pin definitions:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode: 4096 steps / rotation
#define MotorInterfaceType 8

#define BLINDS_MAX_TARGET 2048

Espalexa espalexa;

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper1 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// wifi
char ssid[] = WIFI_SSID;    // your network SSID (name)
char pass[] = WIFI_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();
  
  // Wi-Fi connection
  wifiCheck();
  wifiSetup();
  printWiFiStatus();
  
  // Set the maximum steps per second:
  stepper1.setMaxSpeed(1000);

  alexaDeviceSetup();
}

void loop() {
 espalexa.loop();

//  blockingRunSpeedToPosition(8192);
//  delay(1000);
//
//  blockingRunSpeedToPosition(0);
//  delay(1000);
}

void wifiCheck() {
  // check if the WiFi module works
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi check failed");
    // don't continue:
    while (true);
  }
}

// Wi-Fi Connection
void wifiSetup() {
  while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void alexaDeviceSetup() {
  espalexa.addDevice("Blinds 1", blindOneHandler);
    
  espalexa.begin();
}

void blindOneHandler(uint8_t brightness)
{
    Serial.print("Blinds 1 changed to ");

    long thePosition = BLINDS_MAX_TARGET * espalexa.toPercent(brightness);

    if (brightness) {
      Serial.print("ON, brightness ");
      Serial.println(brightness);
      blockingRunSpeedToPosition(stepper1, thePosition);
    }
    else  {
      Serial.println("OFF");
      blockingRunSpeedToPosition(stepper1, thePosition);
    }
    
    Serial.print("Blinds 1 Complete");
}


void blockingRunSpeedToPosition(AccelStepper* stepper, long position)
{
    stepper.moveTo(position);
    stepper.setSpeed(500);
    while (stepper.distanceToGo() != 0)
      stepper.runSpeedToPosition();
}
