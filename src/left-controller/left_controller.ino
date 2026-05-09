// Left Controller — ESP32 receives face-detection commands from Python over serial and relays them to the firing system and robot via NRF24L01; MPU6050 gesture confirms fire

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <TinyMPU6050.h>

RF24 radio(4, 5); // CE, CSN pins
MPU6050 mpu(Wire);

const byte addressFiringSystem[6] = "00001";
const byte addressRobot[6] = "00002";
const byte addressController[6] = "00003";

const int RED_LED = 32;
const int GREEN_LED = 33;
const int BLUE_LED = 14;
const int WHITE_LED = 27;

bool waitingForFiringSystemResponse = false;
unsigned long lastSendTimeFiringSystem = 0;
const unsigned long resendInterval = 2000;

bool faceDetected = false;
bool fireReady = false;
bool waitingForHandMovement = false;

struct RadioPacket {
    uint8_t command;
    int16_t data;
};

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);

  mpu.Initialize();
  Serial.println("MPU6050 initialized");

  initializeRadioForFiringSystem();

  digitalWrite(GREEN_LED, HIGH);
  Serial.println("Left Controller ready");
}

void loop() {
  if (Serial.available()) {
    handleSerialInput();
  }

  if (radio.available()) {
    handleRadioInput();
  }

  if (waitingForHandMovement && fireReady) {
    checkHandMovement();
  }

  if (waitingForFiringSystemResponse && (millis() - lastSendTimeFiringSystem > resendInterval)) {
    communicateWithFiringSystem("Resend");
  }

  updateLEDs();
}

void handleSerialInput() {
  String input = Serial.readStringUntil('\n');

  if (input == "D0") {
    handleFaceDetected();
  } else if (input == "F0") {
    fireReady = true;
    waitingForHandMovement = true;
    digitalWrite(WHITE_LED, HIGH);
    Serial.println("Waiting for hand movement to confirm fire");
  } else if (input == "R0") {
    resetSystem();
  } else {
    communicateWithFiringSystem(input);
  }
}

void checkHandMovement() {
  mpu.Execute();
  if (mpu.GetRawAccX() <= -8000) {
    // Forward movement detected — confirm fire
    communicateWithFiringSystem("F0");
    waitingForHandMovement = false;
    fireReady = false;
    Serial.println("Hand movement detected, sending fire command");
  }
}

void handleFaceDetected() {
  faceDetected = true;
  digitalWrite(BLUE_LED, HIGH);

  communicateWithFiringSystem("D0");

  // Send stop command to robot
  radio.stopListening();
  radio.setChannel(76);  // Channel for robot
  radio.openWritingPipe(addressRobot);

  RadioPacket packet;
  packet.command = 2;  // Command 2 = face-detected (D0) signal
  packet.data = 0;

  for (int i = 0; i < 3; i++) {
    radio.write(&packet, sizeof(RadioPacket));
    delay(10);
  }

  initializeRadioForFiringSystem();
}

void communicateWithFiringSystem(String message) {
  radio.stopListening();
  delay(50);

  if (radio.write(message.c_str(), message.length() + 1)) {
    Serial.print("Sent to Firing System: ");
    Serial.println(message);
    waitingForFiringSystemResponse = true;
    lastSendTimeFiringSystem = millis();
  } else {
    Serial.println("Failed to send to Firing System");
    digitalWrite(RED_LED, HIGH);
  }

  radio.startListening();
  delay(50);
}

void initializeRadioForFiringSystem() {
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    digitalWrite(RED_LED, HIGH);
    while (1) {}
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(21);  // Channel for firing system
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(32);

  radio.openWritingPipe(addressFiringSystem);
  radio.openReadingPipe(1, addressController);
  radio.startListening();

  Serial.println("Radio initialized for Firing System");
}

void handleRadioInput() {
  char receivedMessage[32] = "";
  radio.read(&receivedMessage, sizeof(receivedMessage));
  Serial.print("Received from Firing System: ");
  Serial.println(receivedMessage);
  waitingForFiringSystemResponse = false;
}

void resetSystem() {
  faceDetected = false;
  fireReady = false;
  waitingForHandMovement = false;
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(WHITE_LED, LOW);
  Serial.println("System reset");
}

void updateLEDs() {
  digitalWrite(BLUE_LED, faceDetected ? HIGH : LOW);
  digitalWrite(WHITE_LED, fireReady ? HIGH : LOW);
}
