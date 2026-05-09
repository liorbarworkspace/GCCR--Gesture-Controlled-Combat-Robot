// Right Controller — Arduino UNO reads MPU6050 hand gestures and transmits movement commands to the robot via NRF24L01

#include <Wire.h>
#include <TinyMPU6050.h>
#include <SPI.h>
#include <RF24.h>

// Radio pin definitions
const int CE_PIN = 7;
const int CSN_PIN = 8;

// Address definitions
const byte addressRobot[6] = "00002";

// Object creation
MPU6050 mpu(Wire);
RF24 radio(CE_PIN, CSN_PIN);

// Data structure for transmission
struct RadioPacket {
    uint8_t command;
    int16_t data;
};

RadioPacket packet;

void setup() {
    Serial.begin(115200);

    // Initialize MPU6050
    mpu.Initialize();

    // Initialize radio
    if (!radio.begin()) {
        Serial.println("Radio hardware not responding!");
        while (1);
    }

    radio.openWritingPipe(addressRobot);
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(76);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(32);
    radio.stopListening();  // Transmit-only mode

    Serial.println("Right Controller ready");
}

void loop() {
    mpu.Execute();

    int message = 0;
    if (mpu.GetRawAccX() <= -8000) {
        message = 1;  // Forward
        Serial.println("Forward");
    } else if (mpu.GetRawAccX() >= 8000) {
        message = 2;  // Backward
        Serial.println("Backward");
    } else if (mpu.GetRawAccY() <= -8000) {
        message = 3;  // Left
        Serial.println("Left");
    } else if (mpu.GetRawAccY() >= 8000) {
        message = 4;  // Right
        Serial.println("Right");
    } else {
        message = 0;  // Stop
        Serial.println("Stop");
    }

    // Send data packet
    packet.command = 1;  // Command 1 = movement command
    packet.data = message;

    Serial.print("Sending: Command = ");
    Serial.print(packet.command);
    Serial.print(", Data = ");
    Serial.println(packet.data);

    if (radio.write(&packet, sizeof(RadioPacket))) {
        Serial.println("Sent successfully");
    } else {
        Serial.println("Failed to send");
    }

    delay(100);  // Send rate: 10 times per second
}
