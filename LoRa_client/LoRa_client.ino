#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal.h>

// Define addresses for pin in/out (LCD)
#define rsPin 7
#define enPin 6
#define d4Pin 5
#define d5Pin 4
#define d6Pin 3
#define d7Pin 2

// Define addresses for pin in/out (LoRa and Control)
#define csPin 7
#define irqPin 1
#define resetPin 6
#define rudderControlPin A0
#define sailControlPin A2

// Global variables for command/control
int rudderVoltage = 0;
int sailVoltage = 0;

// Globals for messages incoming/outging via LoRa
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
int sendInterval = 200;          // interval between sends
int sailTrim;
long lastSendTime = 0;        // last send time

// Start LCD display
LiquidCrystal lcd(rsPin, enPin, d4Pin, d5Pin, d6Pin, d7Pin);

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
  // Clear LCD and get an incoming data packet from LoRa. TODO: fix onReceive return value.
  lcd.clear();
  String incoming = onReceive(LoRa.parsePacket());

  // Get sensor readings for controller pots
  rudderVoltage = analogRead(rudderControlPin);  // Get the sensor voltage and convert to 10-bit value
  int rudderCommandDegrees = map(rudderVoltage, 0, 1023, 0, 270);  // scale it to use it with the servo (value between 800 and 2200 micro-seconds)
  sailVoltage = analogRead(sailControlPin);  // Get the sensor voltage and convert to 10-bit value
  int sailCommandDegrees = map(sailVoltage, 0, 1023, -90, 90);
  int sailTrimDegrees = map(sailTrim, 0, 1023, -90, 90);

  // Display controller values on lcd screen (1st line - rudder control)
  lcd.setCursor(0, 0);
  lcd.print("rudd ");
  lcd.setCursor(6, 0);
  lcd.print(rudderCommandDegrees);

  // 2nd line (sail control and reading)
  lcd.setCursor(0,1);
  lcd.print("sail ");
  lcd.setCursor(6, 1);
  lcd.print(sailCommandDegrees);
  lcd.setCursor(11, 1);
  lcd.print(sailTrimDegrees);
  delay(10);

  if (millis() - lastSendTime > sendInterval) {
    sendMessage(rudderVoltage, sailVoltage);
    lastSendTime = millis();            // timestamp the message
    sendInterval = random(sendInterval) + 100;    // 200-300 milliseconds
  }
}


// Function for sending packet.
void sendMessage(int sensorReading1, int sensorReading2) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(sensorReading1 & 255);  // Pad so the message is always 16-bit length
  LoRa.write((sensorReading1 >> 8) & 0xff);  // Shift 8 bits
  LoRa.write(sensorReading2 & 255);  // Pad so the message is always 16-bit length
  LoRa.write((sensorReading2 >> 8) & 0xff);  // Shift 8 bits
  LoRa.endPacket();                     // finish packet and send it
}

String onReceive(int packetSize) {
  if (packetSize == 0) return "NO PACKET";  // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  int sensorReading = LoRa.read();
  sensorReading = LoRa.read()<<8 | sensorReading;
  sailTrim = sensorReading;

  String incomingMessage = "";

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return "ERROR";                     // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  /*
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  */
  return incomingMessage;
}
