# Webcam Live — simple IP camera viewer used to verify the camera feed before running face detection

import cv2

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# Initialize camera
cap = cv2.VideoCapture("http://10.0.0.150:8080/video")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

def main():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow("Live View", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
