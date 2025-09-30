#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from gesture_controller import detect_gesture


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt")  # Lightweight YOLOv8 pose model

        self.subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.get_logger().info(f"Subscribed to {self.camera_topic}, publishing to {self.cmd_vel_topic}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        results = self.model(frame)

        if len(results) == 0 or not hasattr(results[0], "keypoints"):
            return

        keypoints = results[0].keypoints

        # Detect gesture
        gesture = detect_gesture(keypoints)

        # Publish command based on gesture
        twist = Twist()
        if gesture == 'move':
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif gesture == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

        # Show annotated frame
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Pose Estimation", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
