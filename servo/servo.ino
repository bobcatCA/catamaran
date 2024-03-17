#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 800, 2200);     // scale it to use it with the servo (value between 1ms and 2ms) 
  Serial.println(val);
  myservo.writeMicroseconds(val);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
}
