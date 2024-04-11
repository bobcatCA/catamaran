#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>

// Global variables
int trimPin = A0;  //  Pin to take in analog position/potentiometer voltage
int sailTrim = 0;
int servoPosition = 1520;  // Initialize rudder position to neutral
long servoUpdateInterval = 15;  // Servo updates every 15 ms (smooth-ish)
unsigned long previousTimeServo = millis();

// Globals for new Duplex communication
const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

String incoming;              // incoming message
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

// Start rudder servo
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
  // receive_message();  // Receive a message over LoRa
  incoming = onReceive(LoRa.parsePacket());
  if (incoming.length() == 30)  {
    servoPosition = incoming.substring(13, 16).toInt();
    servoPosition = map(servoPosition, 0, 1023, 1000, 2000);
    Serial.println(servoPosition);
    }  

  sailTrim = analogRead(trimPin);

  // Update servo position if the update interval has elapsed
  if (millis() - previousTimeServo > servoUpdateInterval) {

    // Servo control
    rudderServo.writeMicroseconds(servoPosition);

    // Update previous update time to current time
    previousTimeServo = millis();
  }

  if (millis() - lastSendTime > interval) {
    String message = "sailtrim" + String(sailTrim);   // send a message
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;    // 2-3 seconds
    }
}


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


void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
