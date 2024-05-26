// Acknowledgement of some code borrowed from Brian Schmalz Example5: http://www.schmalzhaus.com/EasyDriver/EasyDriverExamples.html
// Acknowledgement of some code borrowed from Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
// Date: 05/26/2024

// Librairies
#include <AccelStepper.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

// Define our digital pin in/out
#define  dirPin  3
#define  enPin  7
#define  stepPin  2

// Define our maximum and minimum speed in steps per second
#define  maximumSpeed 10000
#define  minimumSpeed 0

// Globals initialization
double setpoint, input, output;  // PID variables
double Kp = 1, Ki = 0, Kd = 0;  // PID constants (to be tuned)
int operatingBand = 30;  // Margin within which the motor is inactive, to save power consumption
unsigned long analogUpdateInterval = 800;  // Frequency of analog reads, in milliseconds
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
  if (millis() - previousTimeAnalog > analogUpdateInterval)  {   
    
    // Compute new PID output given the new setpoint/input. scale the speed to appropriate limits.
    sailPID.Compute();
    int current_speed = ((output / 255) * (maximumSpeed - minimumSpeed)) + minimumSpeed;
    stepper1.setSpeed(current_speed);
    
    previousTimeAnalog = millis();  // Update the last time analogs were read
    /*
    Serial.print("setpoint is: ");
    Serial.println(setpoint);
    Serial.print("input is: ");
    Serial.println(input);
    Serial.print("output is: ");
    Serial.println(output);
    */
  }
  
  stepper1.runSpeed();

  // While the input and setpoint are close, shut down the motor and wait until they diverge again
  while (abs(setpoint - input) < operatingBand)  {
    stepper1.disableOutputs();
    delay(25);

    if (abs(setpoint - input) >= operatingBand)  {
      stepper1.enableOutputs();  // Enable the stepper if the operating condition is outside the dead band
      }
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
  }
