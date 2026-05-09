# Face Detection — reads IP camera stream, detects faces with Haar cascade, aligns stepper turret via serial commands to left controller, and triggers fire sequence

import cv2
import numpy as np
import serial
import time
from datetime import datetime
from collections import deque

# Camera settings
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 720
FPS = 30

# Serial settings
SERIAL_PORT = 'COM13'  # Change to correct port
BAUD_RATE = 115200

# Detection thresholds and timing
CENTER_THRESHOLD = 20
FIRE_THRESHOLD = 95
FINE_ADJUSTMENT_THRESHOLD = 92
MAX_STEPS_X = 5
MAX_STEPS_X_FINE = 1
MIN_STEPS = 1
MAX_STEPS_Y = 20
COMMAND_DELAY = 0.5
FACE_DETECTION_INTERVAL = 5
COMMAND_INTERVAL = 2
COUNTDOWN_TIME = 3
STABILIZATION_DELAY = 1.5
ALIGNMENT_FRAMES = 10  # Reduced required alignment frames
MOVEMENT_COOLDOWN = 3
FINAL_DX_THRESHOLD = 9
MAX_FINE_ADJUSTMENTS = 3  # Max fine-adjustment attempts

# Initialize camera
cap = cv2.VideoCapture("http://10.0.0.144:8080/video")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

# Initialize face detector
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

center_x, center_y = CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2
font = cv2.FONT_HERSHEY_SIMPLEX
last_command_time = 0
last_movement_time = 0

# Smoothing buffer
smoothing_window = deque(maxlen=10)

def send_command(command, data=0):
    global last_command_time
    current_time = time.time()
    if current_time - last_command_time < COMMAND_DELAY:
        time.sleep(COMMAND_DELAY - (current_time - last_command_time))

    packet = f"{command}{data}\n"
    ser.write(packet.encode())
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"{timestamp} - Sent: {packet.strip()}")
    last_command_time = current_time
    return True

def calculate_steps(dx, max_steps):
    steps = int(dx / 20)
    if abs(steps) < MIN_STEPS and dx != 0:
        steps = MIN_STEPS if dx > 0 else -MIN_STEPS
    return np.clip(steps, -max_steps, max_steps)


def draw_text(frame, text, pos, size=0.6, color=(255, 0, 0), thickness=2):
    cv2.putText(frame, text, pos, font, size, color, thickness, cv2.LINE_AA)

def draw_crosshair(frame, x, y, size=10, color=(0, 255, 0), thickness=2):
    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)

def calculate_closeness(dx):
    # Calculate horizontal closeness to center as percentage
    closeness = 100 - (100 * abs(dx) / (CAMERA_WIDTH // 2))
    return max(0, min(closeness, 100))  # Clamp to 0–100

def smooth_closeness(closeness):
    # Smooth closeness values
    smoothing_window.append(closeness)
    return sum(smoothing_window) / len(smoothing_window)

def manual_calibration():
    print("Manual Calibration Mode")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        draw_crosshair(frame, center_x, center_y)
        draw_text(frame, "Calibration: WASD to adjust, Q to quit", (10, 20))
        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            send_command("S")  # Stop motors with hold
            break
        elif key == ord('w'):
            send_command("MY", 5)
        elif key == ord('s'):
            send_command("MY", -5)
        elif key == ord('a'):
            send_command("MX", -5)
        elif key == ord('d'):
            send_command("MX", 5)

    cv2.destroyWindow("Calibration")
    print("Calibration complete.")

def main():
    global last_movement_time
    face_detected = False
    x_aligned = False
    y_moved = False
    fire_ready_sent = False
    frame_count = 0
    stabilization_start = 0
    aligned_frames = 0
    countdown_started = False
    countdown_start = 0
    last_face_detection_time = 0
    smoothed_closeness = 0
    fine_adjustment_count = 0  # Fine-adjustment attempt counter

    print("Press 'c' to calibrate, 's' to start detection, or 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            continue

        draw_crosshair(frame, center_x, center_y)
        draw_text(frame, "Press 'c' to calibrate, 's' to start, 'q' to quit", (10, 20))
        cv2.imshow("Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            manual_calibration()
        elif key == ord('s'):
            break
        elif key == ord('q'):
            send_command("R")  # Release motor coils on exit
            cap.release()
            cv2.destroyAllWindows()
            ser.close()
            return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            continue

        frame_count += 1
        current_time = time.time()

        if frame_count % FACE_DETECTION_INTERVAL == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(30, 30))

            if len(faces) > 0:
                if not face_detected:
                    if current_time - last_command_time >= COMMAND_INTERVAL:
                        send_command("D")  # Face Detected
                    face_detected = True
                last_face_detection_time = current_time

                (x, y, w, h) = faces[0]
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                dx = face_center_x - center_x
                dy = face_center_y - center_y

                closeness_x = calculate_closeness(dx)
                smoothed_closeness = smooth_closeness(closeness_x)

                print(f"Raw Closeness: {int(closeness_x)}%, Smoothed Closeness: {int(smoothed_closeness)}%, dx: {dx}")

                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                if not x_aligned:
                    if smoothed_closeness >= FIRE_THRESHOLD and (
                            abs(dx) <= FINAL_DX_THRESHOLD or fine_adjustment_count >= MAX_FINE_ADJUSTMENTS):
                        aligned_frames += 1
                        if aligned_frames >= ALIGNMENT_FRAMES:
                            if current_time - last_command_time >= COMMAND_INTERVAL:
                                send_command("S")  # Stop with hold on X axis
                                print(f"X aligned, stopped with hold. Smoothed Closeness: {int(smoothed_closeness)}%")
                                x_aligned = True
                                stabilization_start = current_time
                    elif smoothed_closeness >= FINE_ADJUSTMENT_THRESHOLD:
                        # Fine adjustment
                        if fine_adjustment_count < MAX_FINE_ADJUSTMENTS:
                            steps_x = calculate_steps(dx, MAX_STEPS_X_FINE)
                            if abs(steps_x) > 0 and current_time - last_movement_time >= MOVEMENT_COOLDOWN:
                                if current_time - last_command_time >= COMMAND_INTERVAL:
                                    send_command("MX", steps_x)
                                    print(f"Sent fine X command: {steps_x}")
                                    last_movement_time = current_time
                                    fine_adjustment_count += 1
                        else:
                            print("Max fine adjustments reached. Considering aligned.")
                            x_aligned = True
                            stabilization_start = current_time
                    else:
                        # Regular movement
                        steps_x = calculate_steps(dx, MAX_STEPS_X)
                        if abs(steps_x) > 0 and current_time - last_movement_time >= MOVEMENT_COOLDOWN:
                            if current_time - last_command_time >= COMMAND_INTERVAL:
                                send_command("MX", steps_x)
                                print(f"Sent X command: {steps_x}")
                                last_movement_time = current_time
                                fine_adjustment_count = 0  # Reset fine-adjustment counter
                elif not y_moved:
                    if smoothed_closeness < FINE_ADJUSTMENT_THRESHOLD:
                        x_aligned = False
                        fine_adjustment_count = 0
                        countdown_started = False
                    elif not countdown_started:
                        if current_time - stabilization_start >= STABILIZATION_DELAY:
                            countdown_started = True
                            countdown_start = current_time
                    elif current_time - countdown_start < COUNTDOWN_TIME:
                        countdown = int(COUNTDOWN_TIME - (current_time - countdown_start))
                        draw_text(frame, str(countdown), (center_x - 50, center_y + 50), size=5, color=(0, 0, 255),
                                  thickness=5)
                    else:
                        if current_time - last_movement_time >= MOVEMENT_COOLDOWN:
                            if current_time - last_command_time >= COMMAND_INTERVAL:
                                steps_y = min(1, MAX_STEPS_Y)  # Capped at MAX_STEPS_Y
                                send_command("MY", steps_y)  # Move up
                                print(f"Sent Y command: {steps_y}")
                                y_moved = True
                                last_movement_time = current_time

                if x_aligned and y_moved and not fire_ready_sent:
                    if current_time - last_command_time >= COMMAND_INTERVAL:
                        send_command("F")  # Fire ready
                        print("Fire ready!")
                        fire_ready_sent = True

            elif current_time - last_face_detection_time > 1:  # No face detected for more than 1 second
                if face_detected:
                    face_detected = False
                    x_aligned = False
                  #q  y_moved = False
                    fire_ready_sent = False
                    aligned_frames = 0
                    countdown_started = False
                    if current_time - last_command_time >= COMMAND_INTERVAL:
                        send_command("R")  # Release motors
                        print("Target lost, RESTART")

        # On-screen status overlay
        draw_text(frame, f"Face Detected: {'Yes' if face_detected else 'No'}", (10, 40))
        draw_text(frame, f"X Aligned: {'Yes' if x_aligned else 'No'}", (10, 60))
        draw_text(frame, f"Y Moved: {'Yes' if y_moved else 'No'}", (10, 80))
        draw_text(frame, f"Fire Ready: {'Yes' if fire_ready_sent else 'No'}", (10, 100))
        draw_text(frame, f"Closeness: {int(smoothed_closeness)}%", (10, 120))

        # Draw center target rectangle
        cv2.rectangle(frame,
                      (center_x - CENTER_THRESHOLD, center_y - CENTER_THRESHOLD),
                      (center_x + CENTER_THRESHOLD, center_y + CENTER_THRESHOLD),
                      (0, 255, 0), 2)

        draw_crosshair(frame, center_x, center_y)
        cv2.imshow("Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            send_command("R")  # Release motor coils on exit
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
