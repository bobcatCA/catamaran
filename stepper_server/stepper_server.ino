// Acknowledgement of some code borrowed from Brian Schmalz Example5: http://www.schmalzhaus.com/EasyDriver/EasyDriverExamples.html
// Acknowledgement of some code borrowed from Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp

// Librairies
#include <AccelStepper.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

// Define our digital pin in/out
#define  DIR_PIN  3
#define  ENABLE_PIN  7
#define  STEP_PIN  2

// Define our analog pin in/out
#define  COMMAND_INPUT  2
#define  SAIL_SENSOR  0
#define  SPEED_PIN 5

// Define our maximum and minimum speed in steps per second
#define  MAX_SPEED 10000
#define  MIN_SPEED 0

// Globals initialization
int i2cMessage;
double setpoint, input, output;  // PID variables
double Kp = 1, Ki = 1, Kd = 1;  // PID constants (to be tuned)
int operatingBand = 30;  // Margin within which the motor is inactive
long analogUpdateInterval = 800;  // Frequency of analog reads, in milliseconds
unsigned long previousTimeAnalog;  // Tracks when the last analog read occured

// Declare Stepper and PID
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
PID sailPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Serial communication TODO: delete later once debugged
  Serial.begin(9600);
  Wire.begin(4);  // Join I2C bus, address 1 assuming this is master for now?
  Wire.onReceive(receiveI2C);
  //Serial.print("Serial initialized - Stepper Server");

  // The only AccelStepper value we have to set here is the max speeed, which is higher than we'll ever go
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setEnablePin(ENABLE_PIN);  // Note: The enable pin is inverted with a NOT gate to match driver input
  stepper1.enableOutputs();  // Set enable to TRUE (disabled if input close to setpoint)
  
  // Initialize linked variables
  //input = analogRead(SAIL_SENSOR);
  //setpoint = analogRead(COMMAND_INPUT);
  previousTimeAnalog = millis();
  sailPID.SetMode(AUTOMATIC);
  sailPID.SetOutputLimits(-255, 255);  // Bi-directional
}

void loop() {

  // Update analog readings if the interval of time has passed
  if (millis() - previousTimeAnalog > analogUpdateInterval)  {   
    //setpoint = analogRead(COMMAND_INPUT);
    //input = analogRead(SAIL_SENSOR);

    // Compute new PID output given the new setpoint/input. scale the speed to appropriate limits.
    sailPID.Compute();
    int current_speed = ((output / 255) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
    stepper1.setSpeed(current_speed);
    
    previousTimeAnalog = millis();  // Update the last time analogs were read
    Serial.print("setpoint is: ");
    Serial.println(setpoint);
    Serial.print("input is: ");
    Serial.println(input);
    Serial.print("output is: ");
    Serial.println(output);
  }
  
  stepper1.runSpeed();

  // While the input and setpoint are close, shut down the motor and wait until they diverge again
  while (abs(setpoint - input) < operatingBand)  {
    stepper1.disableOutputs();
    //setpoint = analogRead(COMMAND_INPUT);
    //input = analogRead(SAIL_SENSOR);
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
