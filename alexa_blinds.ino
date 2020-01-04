/* Example sketch to control a 28BYJ-48 stepper motor with ULN2003 driver board, AccelStepper and Arduino UNO: continuous rotation. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>

// Motor pin definitions:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode: 4096 steps / rotation
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper1 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

void setup() {
  // Set the maximum steps per second:
  stepper1.setMaxSpeed(1000);
}

void blockingRunSpeedToPosition(long position)
{
    stepper1.moveTo(position);
    stepper1.setSpeed(500);
    while (stepper1.distanceToGo() != 0)
      stepper1.runSpeedToPosition();
}

void loop() {
  blockingRunSpeedToPosition(8192);
  delay(1000);

  blockingRunSpeedToPosition(0);
  delay(1000);
}
