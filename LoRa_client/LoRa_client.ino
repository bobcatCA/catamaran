#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal.h>

// Global variables
char msg;
const int rs = 7, en = 6, d4 = 5, d5 = 4,  d6 = 3, d7 = 2;  // Set pins for LCD. Some are used by the LoRa board but not sure which?
int sensorPin = A2;
int potVoltage = 0;
long transmitUpdateInterval = 60;  // Send a radio transmit via LoRa every # milliseconds
unsigned long previousTransmit = millis();

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
  unsigned long currentTime = millis();  // Get the current time for "multithreading"

  controller_read_packet();  // Look for and read an incoming packet, if available
  
  potVoltage = analogRead(sensorPin);  // Get the sensor voltave
  potVoltage = map(potVoltage, 0, 1023, 1000, 2000);  // scale it to use it with the servo (value between 800 and 2200 micro-seconds)


  // Transmit a packet if the time interval has elapsed.
  if (currentTime - previousTransmit > transmitUpdateInterval)  {
    controller_send_packet();
  }
}


void controller_read_packet() {

  // try to parse packet, if available
  int packetSize = LoRa.parsePacket();  // TODO: investigate this function
  if (packetSize) {
    // received a packet
    lcd.setCursor(0, 1);

    // read packet
    while (LoRa.available()) {
      msg = LoRa.read();
      Serial.print(msg);
      lcd.print(msg);
    }
  }
}

// Send a data packet
void controller_send_packet() {
  Serial.println(potVoltage);
  LoRa.beginPacket();
  LoRa.print(potVoltage);
  LoRa.endPacket();
  previousTransmit = millis();
}
