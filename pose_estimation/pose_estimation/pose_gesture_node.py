#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from geometry_msgs.msg import Point,Twist


class PoseGestureNode(Node):
    def __init__(self):
        super().__init__("pose_gesture_node")
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt")

        
        self.create_subscription(Image, "/image_raw", self.image_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_img = self.create_publisher(Image, "/pose_estimation/image", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/pose_estimation/markers", 10)
        
        
        self.skeleton_pairs = [
            (5, 7), (7, 9),   
            (6, 8), (8, 10),   
            (11, 13), (13, 15), 
            (12, 14), (14, 16), 
            (5, 6), (11, 12),   
            (5, 11), (6, 12)    
        ]

        



    def image_callback(self, msg):
       
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)
        result = results[0]

       
        annotated = result.plot()

        
        cv2.imshow("Pose Estimation", annotated)
        cv2.waitKey(1)

        
        img_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)
        
        
        if result.keypoints is not None:
            kpts = result.keypoints.xy[0].cpu().numpy()
            marker_array = MarkerArray()
            marker_id = 0

           
            for (x, y) in kpts:
                m = Marker()
                m.header = msg.header
                m.id = marker_id; marker_id += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.scale.x = m.scale.y = m.scale.z = 0.05
                m.color.g = 1.0; m.color.a = 1.0
                m.pose.orientation.w = 1.0
                m.pose.position.x = float(x) / 100.0
                m.pose.position.y = float(y) / 100.0
                m.pose.position.z = 0.0
                marker_array.markers.append(m)

            
            for (i1, i2) in self.skeleton_pairs:
                if i1 < len(kpts) and i2 < len(kpts):
                    m = Marker()
                    m.header = msg.header
                    m.id = marker_id; marker_id += 1
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.scale.x = 0.02
                    m.color.r = 1.0; m.color.a = 1.0
                    m.pose.orientation.w = 1.0

                    p1, p2 = Point(), Point()
                    p1.x, p1.y, p1.z = float(kpts[i1][0]) / 100.0, float(kpts[i1][1]) / 100.0, 0.0
                    p2.x, p2.y, p2.z = float(kpts[i2][0]) / 100.0, float(kpts[i2][1]) / 100.0, 0.0
                    m.points.extend([p1, p2])

                    marker_array.markers.append(m)

            self.pub_markers.publish(marker_array)
        gesture = self.detect_gesture(kpts)
        self.execute_gesture(gesture)

    def detect_gesture(self, kpts):
        
        NOSE, LEFT_WRIST, RIGHT_WRIST = 0, 9, 10

        if LEFT_WRIST >= len(kpts) or RIGHT_WRIST >= len(kpts) or NOSE >= len(kpts):
            return "NONE"

       
        if kpts[LEFT_WRIST][1] < kpts[NOSE][1] and kpts[RIGHT_WRIST][1] < kpts[NOSE][1]:
            return "STOP"
        elif kpts[RIGHT_WRIST][1] < kpts[NOSE][1]:
            return "FORWARD"
        else:
            return "NONE"

    def execute_gesture(self, gesture):
        twist = Twist()
        if gesture == "FORWARD":
            twist.linear.x = 0.2
            self.get_logger().info(" Gesture: FORWARD -> Moving robot")
        elif gesture == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Gesture: STOP -> Stopping robot")
        else:
            twist.linear.x = 0.0
        self.pub_cmd.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = PoseGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
