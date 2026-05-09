// Firing System — Arduino UNO + Adafruit Motor Shield controls stepper XY aiming and relay trigger; receives MX/MY/S0/F0/D0/R0 commands from the left controller via NRF24L01

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_MotorShield.h>

RF24 radio(7, 8); // CE, CSN
const byte addressFiringSystem[6] = "00001";
const byte addressController[6] = "00003";

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepperX = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *stepperY = AFMS.getStepper(200, 1);

const int RELAY_PIN_1 = 6;
const int RELAY_PIN_2 = 5;

const int RED_LED = 4;
const int GREEN_LED = 2;
const int BLUE_LED = 3;

const int MAX_STEPS_X = 5;
const int MAX_STEPS_Y = 20;

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  stepperX->setSpeed(10);
  stepperY->setSpeed(10);

  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    digitalWrite(RED_LED, HIGH);
    while (1);
  }
  radio.openReadingPipe(1, addressFiringSystem);
  radio.openWritingPipe(addressController);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(21);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(32);
  radio.startListening();

  digitalWrite(GREEN_LED, HIGH);
  Serial.println("Firing system ready");
}

void loop() {
  if (radio.available()) {
    char receivedMessage[32] = "";
    radio.read(&receivedMessage, sizeof(receivedMessage));
    Serial.print("Received from Left Controller: ");
    Serial.println(receivedMessage);

    handleCommand(String(receivedMessage));

    radio.stopListening();
    delay(50);
    String response = "ACK";
    if (radio.write(response.c_str(), response.length() + 1)) {
      Serial.println("Sent ACK to Left Controller");
    } else {
      Serial.println("Failed to send ACK");
    }
    radio.startListening();
  }
}

void handleCommand(String command) {
  digitalWrite(BLUE_LED, HIGH);

  if (command.startsWith("MX")) {
    int steps = command.substring(2).toInt();
    moveMotorX(steps);
  } else if (command.startsWith("MY")) {
    int steps = command.substring(2).toInt();
    moveMotorY(steps);
  } else if (command == "S0") {
    stopMotorsWithHold();  // Stop with hold (stepper remains energized)
  } else if (command == "F0") {
     Serial.println("FIRE READY");
    fire();
  } else if (command == "D0") {
     Serial.println("FACE_DETECTED");
     } else if (command == "R0") {
    releaseMotors();
  } else {
    Serial.println("Unknown command");
  }

  digitalWrite(BLUE_LED, LOW);
}

void moveMotorX(int steps) {
    steps = constrain(steps, -MAX_STEPS_X, MAX_STEPS_X);
    stepperX->step(abs(steps), steps > 0 ? FORWARD : BACKWARD, MICROSTEP);
    // Motor stays energized after move
}

void moveMotorY(int steps) {
    steps = constrain(steps, -MAX_STEPS_Y, MAX_STEPS_Y);
    stepperY->step(abs(steps), steps > 0 ? FORWARD : BACKWARD, MICROSTEP);
    // Motor stays energized after move
}

void stopMotorsWithHold() {
    // Nothing to do — motors already holding position
    Serial.println("Motors stopped with hold");
}

void releaseMotors() {
    stepperX->release();
    stepperY->release();
    Serial.println("All motors released");
}

void fire() {

  moveMotorY(int steps)
    // Stop motors before firing
  stopMotorsWithHold();

    // Flash all LEDs 3 times before firing
    for (int i = 0; i < 3; i++) {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(BLUE_LED, HIGH);
        delay(500);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, LOW);
        delay(500);
    }

    digitalWrite(RELAY_PIN_2, LOW);
    digitalWrite(RELAY_PIN_1, LOW);
    delay(5000);
    digitalWrite(RELAY_PIN_1, HIGH);
    digitalWrite(RELAY_PIN_2, HIGH);

    Serial.println("Fire command executed");
}
