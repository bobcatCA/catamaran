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

// Globals for new Duplex communication
const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

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
  unsigned long currentTime = millis();  // Take current time for "multithreading"

  // receive_message();  // Receive a message over LoRa
  onReceive(LoRa.parsePacket());

  sailTrim = analogRead(trimPin);

  // Update servo position if the update interval has elapsed
  if (currentTime - previousTimeServo > servoUpdateInterval) {

    // Servo control
    rudderServo.writeMicroseconds(servoPosition);

    // Update previous update time to current time
    previousTimeServo = millis();
  }

  // Send new packet if the radio update interval has elapsed
  if (currentTime - previousTimeRadio > radioUpdateInterval) {
    //send_message();
    //delay(300);
    
  }
  if (millis() - lastSendTime > interval) {
    String message = "sailtrim" + String(sailTrim);   // send a message
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;    // 2-3 seconds
    }
}

/*
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
}*/


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


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  /*Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));*/
  Serial.println("Message: " + incoming);
  /*Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();*/
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
