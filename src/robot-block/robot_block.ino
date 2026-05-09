// Robot Block — Arduino MEGA receives movement commands from the right controller via NRF24L01 and drives DC motors via L298N; halts on face-detected signal

#include <SPI.h>
#include <RF24.h>

// Motor pin definitions
//motor A
#define ENA 9
#define IN1 6
#define IN2 5
//motor B
#define ENB 2
#define IN3 4
#define IN4 3

// Radio pin definitions
const int CE_PIN = 7;
const int CSN_PIN = 8;

// Address definitions
const byte addressRobot[6] = "00002";

// Create radio object
RF24 radio(CE_PIN, CSN_PIN);

// Data structure for reception
struct RadioPacket {
    uint8_t command;
    int16_t data;
};

RadioPacket packet;

unsigned long lastCommunicationTime = 0;
const unsigned long communicationTimeout = 1500; // Max wait time without communication (ms)
bool faceDetected = false; // Flag for face detection state

void setup() {
    Serial.begin(115200);

    // Configure motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Turn off motors - Initial state
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // Set initial motor speed
    analogWrite(ENA, 122);
    analogWrite(ENB, 134);

    stop_all()
    // Initialize radio
    if (!radio.begin()) {
        Serial.println("Radio hardware not responding!");
        while (1);
    }
    radio.openReadingPipe(1, addressRobot);
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(76);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(32);
    radio.startListening();

    Serial.println("Robot ready");
}

void loop() {
    unsigned long currentTime = millis();

    if (radio.available()) {
        lastCommunicationTime = currentTime;
        Serial.println("Radio available");

        RadioPacket receivedPacket;
        radio.read(&receivedPacket, sizeof(RadioPacket));

        Serial.print("Received: Command = ");
        Serial.print(receivedPacket.command);
        Serial.print(", Data = ");
        Serial.println(receivedPacket.data);

        if (receivedPacket.command == 1) {
            if (!faceDetected) {
                moveAccordingly(receivedPacket.data);
            } else {
                Serial.println("Face detected, ignoring movement command");
            }
        } else if (receivedPacket.command == 2) {  // Command 2 = face-detected (D0) signal
            faceDetected = true;
            stop_all();
            Serial.println("FACE DETECTED");
        }
    }

    if (currentTime - lastCommunicationTime > communicationTimeout) {
        stop_all();
        Serial.println("Communication timeout, stopping all motors");
    }
}

void moveAccordingly(int directionOfMovement) {
    Serial.print("Moving according to direction: ");
    Serial.println(directionOfMovement);

    switch (directionOfMovement) {
        case 1:
            go_forward();
            Serial.println("go_forward");
            break;
        case 2:
            go_backwards();
            Serial.println("go_backwards");
            break;
        case 3:
            go_left();
            Serial.println("go_left");
            break;
        case 4:
            go_right();
            Serial.println("go_right");
            break;
        case 0:
            stop_all();
            Serial.println("stop_all");
            break;
        default:
            Serial.println("Unknown direction");
            break;
    }
}

void go_forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void go_backwards() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void go_right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void go_left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop_all() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
