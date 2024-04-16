#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal.h>

// Global variables for command/control
const int rs = 7, en = 6, d4 = 5, d5 = 4,  d6 = 3, d7 = 2;  // Set pins for LCD. Some are used by the LoRa board but not sure which?
int rudderVoltage = 0;
int sailVoltage = 0;

// Globals for pin in/out
const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin
int sailControlPin = A3;
int rudderControlPin = A2;

// Globals for messages incoming/outging via LoRa
String incoming;              // incoming message
String outgoing;              // outgoing message
String lcdStatus;
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int sendInterval = 50;          // interval between sends

// Start LCD display
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
  lcd.begin(16, 2);  // Start LCD for 16 x 2 grid
  Serial.begin(9600);  // Start serial, 9600 baud rate
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915600000)) {  // Start LoRa, 915 MHz
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSyncWord(0x34);  // TODO: investigate this function
  lcd.clear();
}


void loop() {
  lcd.clear();
  incoming = onReceive(LoRa.parsePacket());
  if (incoming.length() == 11)  {
    Serial.println("Message: " + incoming);
    lcdStatus = incoming;  
    }
    
  rudderVoltage = analogRead(rudderControlPin);  // Get the sensor voltage and convert to 10-bit value
  rudderVoltage = map(rudderVoltage, 0, 1023, 1000, 2000);  // scale it to use it with the servo (value between 800 and 2200 micro-seconds)
  sailVoltage = analogRead(sailControlPin);  // Get the sensor voltage and convert to 10-bit value
  sailVoltage = map(sailVoltage, 0, 1023, 1000, 2000);

  // Display controller values on lcd screen
  lcd.setCursor(0, 0);
  lcd.print(rudderVoltage);
  lcd.setCursor(0, 1);
  lcd.print(sailVoltage);    
  lcd.setCursor(5, 1);
  lcd.print(lcdStatus);
  delay(10);

  if (millis() - lastSendTime > sendInterval) {
    String message = "ruddercommand" + String(rudderVoltage) + "sailcommand" + String(sailVoltage);
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    sendInterval = random(sendInterval) + 1000;    // 2-3 seconds
    }
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
