// Acknowledgement of some code borrowed from Brian Schmalz Example5: http://www.schmalzhaus.com/EasyDriver/EasyDriverExamples.html
// Acknowledgement of some code borrowed from Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
// Date: 05/26/2024

// Librairies
#include <AccelStepper.h>
#include <CircularBuffer.hpp>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

// Define our digital pin in/out
#define  dirPin  3
#define  enPin  7
#define  stepPin  2

// Define our maximum and minimum speed in steps per second
#define  errorThreshold -20
#define  maximumSpeed 10000
#define  minimumSpeed 0
#define  pauseTimeCheck 500
#define  trimBand 5
#define  trimCenter 5

// Globals initialization
bool onTarget = false;
bool onMiddle = true;
bool sheetSlack = true;
CircularBuffer<int, 10> posErrors;
int currentSpeed = 0;
int errorsAverage = 0;
double setpoint, input, output;  // PID variables
double Kp = 1, Ki = 0, Kd = 0;  // PID constants (to be tuned)
unsigned long inputUpdateInterval = 80;  // Frequency of analog reads, in milliseconds
unsigned long previousTimeAnalog;  // Tracks when the last analog read occured

// Declare Stepper and PID
AccelStepper stepper1(AccelStepper::DRIVER, stepPin, dirPin);
PID sailPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Serial communication TODO: delete later once debugged
  Serial.begin(9600);
  Wire.begin(4);  // Join I2C bus, address 1 assuming this is master for now?
  Wire.onReceive(receiveI2C);

  // The only AccelStepper value we have to set here is the max speeed, which is higher than we'll ever go
  stepper1.setMaxSpeed(maximumSpeed);
  stepper1.setEnablePin(enPin);  // Note: The enable pin is inverted with a NOT gate to match driver input
  stepper1.enableOutputs();  // Set enable to TRUE (disabled if input close to setpoint)

  // Initialize linked variables
  previousTimeAnalog = millis();
  sailPID.SetMode(AUTOMATIC);
  sailPID.SetOutputLimits(-255, 255);  // Bi-directional for motor CW/CCW
}

void loop() {

  // Update analog readings if the interval of time has passed
  if (millis() - previousTimeAnalog > inputUpdateInterval)  {

    // Compute new PID output given the new setpoint/input. scale the speed to appropriate limits.
    sailPID.Compute();
    int errorPID = input - setpoint;  // Compute error for this step
    posErrors.push(errorPID);  // Push the error on this step to the top of the buffer
    errorsAverage = arrayAverage();

    // Compute the new speed setpoint and send to stepper
    currentSpeed = ((output / 255) * (maximumSpeed - minimumSpeed)) + minimumSpeed;
    previousTimeAnalog = millis();  // Update the last time analogs were read

    // Compute booleans
    if (abs(setpoint - input) < trimBand)  {
      onTarget = true;
    } else {
      onTarget = false;
    }

    if (currentSpeed < 0 && abs(input) < trimCenter)  {
      onMiddle = true;
    } else {
      onMiddle = false;
    }

    if (setpoint > input && errorsAverage < errorThreshold)  {
      sheetSlack = true;
    } else {
      sheetSlack = false;
    }
    
  }

  Serial.println(errorsAverage);
  // While the input and setpoint are close, shut down the motor and wait until they diverge again
  if (onTarget || onMiddle || sheetSlack)  {
    pauseSailControl();
  }
  stepper1.enableOutputs();
  stepper1.setSpeed(currentSpeed);

  // Run the stepper at the desired speed
  stepper1.runSpeed();
}


// Function that is called when an I2C message is received.
void receiveI2C(int numBytes)  {
  // Bit shift to get sail trim
  int sailTrim  = Wire.read() << 8;
  stepper1.runSpeed();  // Additional call for smoothness
  sailTrim |= Wire.read();
  input = sailTrim;
  stepper1.runSpeed();  // Additional call for smoothness

  // Bit shift to get input rudder command
  int sailCommand  = Wire.read() << 8;
  stepper1.runSpeed();  // Additional call for smoothness
  sailCommand |= Wire.read();
  setpoint = sailCommand;
}


// Function to hold in place if the sail position is in the middle (prevent over-winding)
void pauseSailControl()  {
  stepper1.disableOutputs();  // Stop the stepper motor when close to the target/setpoint
  int startTime = millis();
  while (millis() - startTime < pauseTimeCheck)  {
    delay(100);
    // Loiter here, then go back and check analogs
  }
}


int arrayAverage()  {
  int arrLength = 10 - posErrors.available();
  int sum = 0;
  for (int i = 0; i < arrLength; i++)  {
    sum = sum + posErrors[i];
  }
  int average = sum / 10;
  return average;
}
