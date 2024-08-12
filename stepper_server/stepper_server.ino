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
#define  errorTimeout 3000  // After which the sheet is assumed to be slack, sail control pauses
#define  maximumSpeed 10000
#define  minimumSpeed 0
#define  pauseTimeCheck 500  // Time to pause until re-calculating status booleans
#define  inputUpdateInterval 50  // Frequency of updating the PID calculation, ms
#define  trimBand 5  // Acceptable error band, within which sail control pauses
#define  trimCenter 5  // Center trim range where sail control pauses to avoid over-tension

// Globals initialization
bool onTarget = false;
bool onMiddle = true;
bool sheetSlack = true;
bool statusMain = false;
CircularBuffer<int, 10> posErrors;
int currentSpeed = 0;
int errorsAverage = 0;
double setpoint, input, output;  // PID variables
double Kp = 3, Ki = 0, Kd = 0;  // PID constants (to be tuned)
unsigned long previousTimeInput;  // Tracks when the last sensor read/PID compute was done
unsigned long previousTimeTaught;  // Stores the last time the sheet line was known to be taught

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
  previousTimeInput = millis();
  sailPID.SetMode(AUTOMATIC);
  sailPID.SetOutputLimits(-255, 255);  // Bi-directional for motor CW/CCW
}

void loop() {

  // Update analog readings if the interval of time has passed
  if (millis() - previousTimeInput > inputUpdateInterval)  {

    // Compute new PID output given the new setpoint/input. scale the speed to appropriate limits.
    sailPID.Compute();
    int errorPID = input - setpoint;  // Compute error for this step
    posErrors.push(errorPID);  // Push the error on this step to the top of the buffer
    errorsAverage = arrayAverage();

    // Compute the new speed setpoint and send to stepper
    currentSpeed = ((output / 255) * (maximumSpeed - minimumSpeed)) + minimumSpeed;
    previousTimeInput = millis();  // Update the last time analogs were read
  }

  // While the input and setpoint are close, shut down the motor and wait until they diverge again
  if (onTarget || onMiddle || sheetSlack)  {
    statusMain = false;
    pauseSailControl();
  } else {
    statusMain = true;
  }

  if (statusMain)  {
    // Run the stepper at the desired speed if no fault conditions are detected
    stepper1.setSpeed(currentSpeed);
    stepper1.runSpeed();
  }
}


// Function that is called when an I2C message is received.
void receiveI2C(int numBytes)  {
  // Bit shift to get sail trim
  int sailTrim  = Wire.read() << 8;
  sailTrim |= Wire.read();
  input = sailTrim;

  // Bit shift to get input rudder command
  int sailCommand  = Wire.read() << 8;
  sailCommand |= Wire.read();
  setpoint = sailCommand;

  // Compute booleans
  if (abs(setpoint - input) < trimBand)  {
    onTarget = true;
  } else {
    onTarget = false;
  }

  if (setpoint < input && abs(input) < trimCenter)  {
    onMiddle = true;
  } else {
    onMiddle = false;
  }

  if (errorsAverage > -1 * trimBand)  {
    sheetSlack = false;
    previousTimeTaught = millis();
  } else {
    if (millis() - previousTimeTaught > errorTimeout)  {
      sheetSlack = true;
    }  else {} // Return until the timeout condition is reached
  }
}


// Function to hold in place if the sail position is in the middle (prevent over-winding)
void pauseSailControl()  {
  //stepper1.disableOutputs();  // Stop the stepper motor when close to the target/setpoint
  stepper1.stop();  // Stop the stepper when close to target/setpoint
  delay(pauseTimeCheck);
  // Loiter here, then go back and check analogs
}


// Function for calculating the average of the Circular Buffer. Couldn't find a way to pass the buffer
// in as an argument, so just doing it as a Global.
int arrayAverage()  {
  int arrLength = 10 - posErrors.available();
  int sum = 0;
  for (int i = 0; i < arrLength; i++)  {
    sum = sum + posErrors[i];
  }
  int average = sum / 10;
  return average;
}
