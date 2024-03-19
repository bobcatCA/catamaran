#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>

// Global variables
int trimPin = A0;  //  Pin to take in analog position/potentiometer voltage
int sailTrim = 0;
int servoPosition = 1520;  // Initialize rudder position to neutral
long servoUpdateInterval = 15;  // Servo updates every 15 ms (smooth-ish)
unsigned long previousTimeServo = millis();
long radioUpdateInterval = 500;  // Interval for updating LoRa radio communication;
unsigned long previousTimeRadio = millis();

Servo rudderServo;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait unitl serial starts

  if (!LoRa.begin(915600000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSyncWord(0x34);  // TODO: what's this function for?
  rudderServo.attach(5);  // Attaches servo to pin (6) on the servo object
}

void loop() {
  unsigned long currentTime = millis();  // Take current time for "multithreading"

  receive_message();  // Receive a message over LoRa

  // Update servo position if the update interval has elapsed
  if (currentTime - previousTimeServo > servoUpdateInterval) {

    // Servo control
    rudderServo.writeMicroseconds(servoPosition);

    // Update previous update time to current time
    previousTimeServo = millis();
  }

  // Send new packet if the radio update interval has elapsed
  if (currentTime - previousTimeRadio > radioUpdateInterval) {
    send_message();
  }
}


void receive_message() {
  // Receive packet via LoRa client (remote control)
  int packetSize = LoRa.parsePacket();  // TODO: investigate this function
  char msg[packetSize];  // Create a character array equal to the size of the incoming packet
  if (packetSize) {
    Serial.println("Received packet");
    int i = 0;

    // Read packet
    while (LoRa.available()) {
      char character = LoRa.read();

      msg[i] = character;
      i++;
    }
    const char *ptr_msg = msg;
    servoPosition = atoi(ptr_msg);
    Serial.println(msg);
    Serial.println("packet complete");  
    // servoPosition = atoi(msg);
    Serial.println(servoPosition);
  }
}


void send_message() {
    sailTrim = analogRead(trimPin);  // Get the sail trim angle
    Serial.println("Sending packet: ");  // TODO: delete once debugging complete
    Serial.println(sailTrim);

    // send packet
    LoRa.beginPacket();
    LoRa.print("voltage: ");
    LoRa.print(sailTrim);
    LoRa.endPacket();  // TODO: Does this add a header or something?
    previousTimeRadio = millis();  // Update previous time for radio send
  }
