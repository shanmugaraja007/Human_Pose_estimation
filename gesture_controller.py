import numpy as np


def detect_gesture(keypoints):
    """
    Detect gesture from YOLOv8 pose keypoints.
    One arm up → 'move'
    Two arms up → 'stop'
    """

    if keypoints is None or len(keypoints.shape) < 3:
        return None

    keypoints = keypoints.cpu().numpy()[0]  # Shape: [17, 3] for COCO

    # Example keypoint indices for COCO format:
    LEFT_WRIST = 9
    RIGHT_WRIST = 10
    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6

    left_wrist_y = keypoints[LEFT_WRIST][1]
    right_wrist_y = keypoints[RIGHT_WRIST][1]
    left_shoulder_y = keypoints[LEFT_SHOULDER][1]
    right_shoulder_y = keypoints[RIGHT_SHOULDER][1]

    arms_up = 0

    # Check if left arm is raised
    if left_wrist_y < left_shoulder_y:
        arms_up += 1

    # Check if right arm is raised
    if right_wrist_y < right_shoulder_y:
        arms_up += 1

    if arms_up == 1:
        return 'move'
    elif arms_up == 2:
        return 'stop'
    return None
