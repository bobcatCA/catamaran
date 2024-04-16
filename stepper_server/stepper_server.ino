// Acknowledgement of some code borrowed from Brian Schmalz Example5: http://www.schmalzhaus.com/EasyDriver/EasyDriverExamples.html
// Acknowledgement of some code borrowed from Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp

#include <AccelStepper.h>
#include <PID_v1.h>
#include <SPI.h>

// Define the stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);

// Define our analog pot input pin
#define  SAIL_SENSOR  0
#define  COMMAND_INPUT  2
#define  SPEED_PIN 5

// Define our maximum and minimum speed in steps per second
#define  MAX_SPEED 10000
#define  MIN_SPEED 0

unsigned long previousTimeAnalog = millis();
long analogUpdateInterval = 200;
double setpoint, input, output;
double Kp = 2, Ki = 5, Kd = 1;
PID sailPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.print("Serial initialized - Stepper Server");

  // The only AccelStepper value we have to set here is the max speeed, which is higher than we'll ever go
  stepper1.setMaxSpeed(10000.0);

  // Initialize linked variables
  input = analogRead(SAIL_SENSOR);
  setpoint = analogRead(COMMAND_INPUT);
  sailPID.SetMode(AUTOMATIC);
  sailPID.SetOutputLimits(-255, 255);  // Bi-directional
}

void loop() {
  
  if (millis() - previousTimeAnalog > analogUpdateInterval)  {
    // PID control compute
    input = analogRead(SAIL_SENSOR);
    setpoint = analogRead(COMMAND_INPUT);
    sailPID.Compute();
    previousTimeAnalog = millis();  // Update the last time analogs were read
    /*Serial.print("setpoint is: ");
    Serial.println(setpoint);
    Serial.print("input is: ");
    Serial.println(input);
    Serial.print("output is: ");
    Serial.println(output); */
  }
  
  // scale the current speed to the desired range and send to stepper motor driver
  int current_speed = ((output / 255) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
  // Update the stepper to run at this new speed
  stepper1.setSpeed(current_speed);
  stepper1.runSpeed();
}
