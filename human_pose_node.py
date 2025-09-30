#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from tf_transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point


class PoseMarkerNode(Node):
    def __init__(self):
        super().__init__("pose_marker_node")
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt")

        
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

       
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

        
        self.spawn_human()

    def spawn_human(self):
        """Spawn a human SDF model in front of the robot"""
        client = self.create_client(SpawnEntity, "/spawn_entity")
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /spawn_entity service...")

        
        sdf_path = "/home/user/ros2_ws/src/pose_estimation/models/casual_female/model.sdf"
        if not os.path.exists(sdf_path):
            self.get_logger().error(f"Human SDF not found at {sdf_path}")
            return

        with open(sdf_path, "r") as f:
            sdf_xml = f.read()

        pose = Pose()
        pose.position.x = 4.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, -1.492767)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        req = SpawnEntity.Request()
        req.name = "human"
        req.xml = sdf_xml
        req.robot_namespace = "human_ns"
        req.initial_pose = pose

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Human spawned successfully in Gazebo!")
        else:
            self.get_logger().error("Failed to spawn human in Gazebo.")

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


def main(args=None):
    rclpy.init(args=args)
    node = PoseMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
