// Acknowledgement of code borrowed from Nichlolas Zambetti [http://www.zambetti.com](http://www.zambetti.com)

// Libraries
#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// Global variables for command/control
int sailTrim = 0;
int rudderCommand;  // incoming command for rudder position
int rudderTest = 858;  // TODO: delete once the bitwise wire send is working
int sailCommand;  // incoming command for sail position
int servoPosition;  // Initialize rudder position to neutral
long servoUpdateInterval = 15;  // Servo updates every 15 ms (smooth-ish)
unsigned long previousTimeServo = millis();

// Globals for pin in/out
const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin
int trimPin = A0;  //  Pin to take in analog position/potentiometer voltage

// Globals for message incoming/outgoing via LoRa
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
int sendInterval = 2000;          // interval between sends
long lastSendTime = 0;        // last send time
String incoming;              // incoming message
String outgoing;              // outgoing message

// Special Globals
Servo rudderServo;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait unitl serial starts
  Wire.begin();  // Initialize I2C bus
  Serial.println("LoRa Server starting");

  if (!LoRa.begin(915600000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSyncWord(0x34);  // TODO: what's this function for?
  rudderServo.attach(5);  // Attaches servo to pin (6) on the servo object
}

void loop() {
  int expectedPacketLength = 32;  // Number of characters of normal incoming message
  incoming = onReceive(LoRa.parsePacket());

  // Parse the incoming packet. If the length is correct, parse the various commands from the incoming packet
  if (incoming.length() == expectedPacketLength)  {
    rudderCommand = incoming.substring(13, 17).toInt();
    sailCommand = incoming.substring(28, 33).toInt();
    Serial.println(incoming);
  }

  // Read the sensor values to be sent back to the controller node
  sailTrim = analogRead(trimPin);

  // Update servo position if the update interval has elapsed
  if (millis() - previousTimeServo > servoUpdateInterval) {

    // Servo control only if the setpoint is well off of the real position
    if (abs(rudderCommand - servoPosition) > 5)  {
      servoPosition = servoPosition + (rudderCommand - servoPosition) / 5;  // Increment slowly until the position responds (P-control of sorts)
      rudderServo.writeMicroseconds(servoPosition);
    }

    // Update previous update time to current time
    previousTimeServo = millis();
  }

  // Send an outgoing packet to the controller note if enough time has elapsed.
  if (millis() - lastSendTime > sendInterval) {
    String message = "sailtrim" + String(sailTrim);   // send a message
    sendMessageLoRa(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    sendInterval = random(sendInterval) + 1000;    // 2-3 seconds

    // Wire I2C transmission
    Wire.beginTransmission(4);  // Send to address 4 (stepper server)
    Wire.write(sailTrim >> 8);  // Shift 8 bits
    Wire.write(sailTrim & 255);  // Pad so the message is always 16-bit length
    Wire.write(sailCommand >> 8);  // Shift 8 bits
    Wire.write(sailCommand & 255);
    Wire.endTransmission();
  }
}


// Function for receiving packet.
String onReceive(int packetSize) {
  if (packetSize == 0) return "NO PACKET";  // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incomingMessage = "";

  while (LoRa.available()) {
    incomingMessage += (char)LoRa.read();
  }

  if (incomingLength != incomingMessage.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return "ERROR";                     // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return "ERROR";                     // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  /*Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incomingMessage);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();*/
  return incomingMessage;
}


// Function for sending packet.
void sendMessageLoRa(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
