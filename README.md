# GCCR — Gesture Controlled Combat Robot

A combat robot controlled by natural hand gestures, with an autonomous face-detection and auto-aiming firing system. Built as an electrical engineering graduation project.

---

## Demo

> Robot driven by hand tilt gestures · Turret auto-tracks and locks on a detected face · Fire confirmed by a second hand gesture

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        LAPTOP (Python)                          │
│  OpenCV face detection → serial commands → Left Controller      │
└──────────────────────────────┬──────────────────────────────────┘
                               │ USB Serial
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│              LEFT CONTROLLER  (ESP32)                            │
│  Receives D0/F0/R0 from Python · MPU6050 gesture confirms fire   │
│  Broadcasts over two NRF channels simultaneously                 │
└────────────┬─────────────────────────────────────┬──────────────┘
             │ NRF ch.21 (firing system)            │ NRF ch.76 (robot)
             ▼                                      ▼
┌────────────────────────┐             ┌────────────────────────────┐
│   FIRING SYSTEM        │             │   ROBOT BLOCK              │
│   Arduino UNO          │             │   Arduino MEGA             │
│   Adafruit MotorShield │             │   L298N Motor Driver       │
│   Stepper X/Y aim      │             │   4× DC motors             │
│   Relay trigger        │             │   Stops on face-detected   │
└────────────────────────┘             └────────────────────────────┘
                                                    ▲
                                                    │ NRF ch.76
                                       ┌────────────────────────────┐
                                       │   RIGHT CONTROLLER         │
                                       │   Arduino UNO              │
                                       │   MPU6050 (hand tilt)      │
                                       │   Sends movement commands  │
                                       └────────────────────────────┘
```

---

## How It Works

### Movement control
The right controller reads raw accelerometer data from the MPU6050 at 10 Hz. Tilting the hand past a threshold (±8000 raw units on X or Y axis) maps to forward / backward / left / right commands, sent as structured radio packets to the robot over NRF channel 76.

### Autonomous face detection and auto-aim
The Python script streams MJPEG video from an IP camera and runs OpenCV's Haar cascade face detector every 5 frames. When a face appears:
1. A `D0` command is sent over serial to the left controller.
2. The robot stops immediately and ignores further movement input.
3. The firing system receives `D0` and begins the alignment sequence.

Alignment uses a closeness percentage algorithm `(100 − 100·|dx| / (width/2))` smoothed over a 10-frame rolling window. The turret moves in coarse steps until closeness ≥ 92%, switches to fine micro-steps until ≥ 95%, then holds position with the stepper energized. After a stabilization delay and a 3-second visual countdown, the Y-axis stepper elevates to firing angle.

### Gesture-confirmed fire
Once the turret is aligned, the left controller waits for the operator to physically confirm with a forward wrist flick (MPU6050 AccX ≤ −8000). Only then is the `F0` fire command relayed to the firing system. The relay activates for 5 seconds, then the system resets.

### Safety design
- Communication timeout: if the robot receives no radio packet for 1500 ms, all motors stop automatically.
- Relay is normally-HIGH (de-energized). It activates only on an explicit fire command — power loss = safe state.
- ACK/retry loop: the left controller retransmits to the firing system every 2 seconds until it receives an acknowledgement.
- The robot ignores all movement commands while a face is detected.

---

## Hardware

| Block | MCU | Key Components |
|---|---|---|
| Right Controller | Arduino UNO | MPU6050 (I²C), NRF24L01 (SPI) |
| Left Controller | ESP32 | MPU6050 (I²C), NRF24L01 (SPI), 4× LEDs |
| Robot Block | Arduino MEGA | L298N dual H-bridge, 4× DC motors, NRF24L01 |
| Firing System | Arduino UNO | Adafruit Motor Shield V2, 2× NEMA stepper motors, 2× relay, NRF24L01 |
| Face Detection | Laptop | IP camera (MJPEG/WiFi), USB–serial to ESP32 |

---

## Wireless Communication

Two independent NRF24L01 links run in parallel:

| Channel | Link | Data |
|---|---|---|
| 76 | Right Controller → Robot | Structured packet: `{command, data}` at 250 kbps |
| 21 | Left Controller ↔ Firing System | String commands: `D0`, `F0`, `R0`, `MX±n`, `MY±n`, `S0`, `ACK` |

The left controller dynamically switches between the two channels to relay robot-stop signals on channel 76 and firing commands on channel 21.

---

## Command Protocol

| Command | Direction | Meaning |
|---|---|---|
| `D0` | Python → Left → Robot + Firing | Face detected — stop robot, start aim |
| `F0` | Left → Firing | Fire confirmed by hand gesture |
| `R0` | Left → Robot + Firing | Target lost — reset, release motors |
| `MX±n` | Left → Firing | Move stepper X by n micro-steps |
| `MY±n` | Left → Firing | Move stepper Y by n micro-steps |
| `S0` | Left → Firing | Stop steppers with hold (stay energized) |
| `ACK` | Firing → Left | Acknowledgement received |

---

## Running the Project

### Requirements
```
Python 3.x
opencv-python
numpy
pyserial
```

### Arduino libraries
- `RF24`
- `TinyMPU6050`
- `Adafruit_MotorShield`

### Steps
1. Flash `src/right-controller/right_controller.ino` to the right-hand Arduino UNO.
2. Flash `src/left-controller/left_controller.ino` to the ESP32.
3. Flash `src/robot-block/robot_block.ino` to the Arduino MEGA.
4. Flash `src/firing-system/firing_system.ino` to the firing-system Arduino UNO.
5. Set the correct serial port in `face_detection.py` (`SERIAL_PORT = 'COMx'`).
6. Set the IP camera URL in `face_detection.py` to match your camera's stream address.
7. Run:
```bash
python src/face-detection/face_detection.py
```
8. Press `c` to manually calibrate turret center, `s` to start autonomous detection, `q` to quit.

---

## Docs

Block diagrams and system flow charts are in [`docs/`](docs/).

---

## Authors

**Lior Bar** — Electrical & Electronics Engineering  
**Raziel Natanya** — Electrical & Electronics Engineering
