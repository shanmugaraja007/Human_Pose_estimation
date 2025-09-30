

from ultralytics import YOLO
import cv2
import imutils
import time


print("[INFO] Loading YOLOv8-Pose model...")
model = YOLO('yolov8n-pose.pt') 


keypoint_labels = [
    "Nose", "Left Eye", "Right Eye", "Left Ear", "Right Ear",
    "Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow",
    "Left Wrist", "Right Wrist", "Left Hip", "Right Hip",
    "Left Knee", "Right Knee", "Left Ankle", "Right Ankle"
]


print("[INFO] Starting video stream...")
vs = cv2.VideoCapture(0)
time.sleep(2.0)


vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


while True:
    ret, frame = vs.read()
    if not ret:
        break


    frame = imutils.resize(frame, width=400)


    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


    results = model(frame_rgb)


    annotated_frame = frame.copy()

    # Loop through all detected people
    for result in results:
        if result.keypoints is not None:
            for person_kp in result.keypoints.xy:
                for i, coord in enumerate(person_kp):
                    x, y = int(coord[0]), int(coord[1])
                    # Draw keypoint circle
                    cv2.circle(annotated_frame, (x, y), 5, (0, 255, 0), -1)
                    # Draw keypoint label
                    cv2.putText(annotated_frame, keypoint_labels[i], (x + 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

    # Display the annotated frame
    cv2.imshow("YOLOv8-Pose Live Webcam", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
vs.release()
cv2.destroyAllWindows()
