#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
import numpy as np
import csv
import time
from datetime import datetime
import os

MODEL = "yolov8m.pt"


class CarDetectionNode(Node):
    def __init__(self):
        super().__init__("car_detection_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscribe to vehicle local position
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.position_callback,
            qos,
        )
        
        # Subscribe to RGB image
        self.rgb_sub = self.create_subscription(
            Image,
            "/world/car/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.image_callback,
            qos,
        )
        
        # Publisher for bounding box center
        self.bbox_center_pub = self.create_publisher(
            Float32MultiArray, "/car_bbox_center", 10
        )
        
        # Initialize YOLO model and CV bridge
        self.model = YOLO(MODEL)
        self.br = CvBridge()
        self.cx = 960.0
        self.cy = 540.0
        self.window_name = "Car Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 360)
        
        # Variables to store latest position and bounding box data
        self.latest_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'timestamp': 0}
        self.latest_bbox = [0.0, 0.0, 0.0, 0.0]
        
        # CSV file setup with custom path
        log_dir = "/home/hild/Documents/Logs"
        os.makedirs(log_dir, exist_ok=True)  # Create directory if it doesn't exist
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(log_dir, f'drone_data_{timestamp}.csv')
        self.get_logger().info(f"CSV file will be saved at: {csv_path}")
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Drone_X_m', 'Drone_Y_m', 'Drone_Z_m', 
                                'Bbox_Center_X_px', 'Bbox_Center_Y_px', 'Bbox_Width_px', 'Bbox_Height_px'])
        
        # Timer for logging every 250ms
        self.timer = self.create_timer(0.25, self.log_data_callback)

    def position_callback(self, msg):
        """Store the latest vehicle position."""
        self.latest_position = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'timestamp': msg.timestamp
        }

    def image_callback(self, rgb_msg):
        """Process image and update bounding box data."""
        try:
            rgb_image = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
            if rgb_image.shape[:2] != (1080, 1920):
                return
            results = self.model.predict(rgb_image, classes=[2], verbose=False)
            target_found = False
            bbox_msg = Float32MultiArray()
            
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes.xywh.cpu().numpy()
                confidences = results[0].boxes.conf.cpu().numpy()

                best_idx = np.argmax(confidences)
                center_x, center_y, width, height = boxes[best_idx]
                conf = confidences[best_idx]

                area = width * height
                self.get_logger().info(f"Detected bounding box area (w * h): {area:.2f}")

                if 0 <= center_x <= 1920 and 0 <= center_y <= 1080:
                    self.latest_bbox = [float(center_x), float(center_y), float(width), float(height)]
                    bbox_msg.data = self.latest_bbox
                    self.bbox_center_pub.publish(bbox_msg)

                    target_found = True
                    x1 = int(center_x - width / 2)
                    y1 = int(center_y - height / 2)
                    x2 = int(center_x + width / 2)
                    y2 = int(center_y + height / 2)

                    cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.circle(
                        rgb_image, (int(center_x), int(center_y)), 3, (0, 255, 0), -1
                    )
                    cv2.putText(
                        rgb_image,
                        f"({center_x:.1f}, {center_y:.1f}), w={width:.1f}, h={height:.1f}",
                        (int(center_x), int(center_y) + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 255, 0),
                        3,
                    )

            if not target_found:
                self.latest_bbox = [0.0, 0.0, 0.0, 0.0]
                bbox_msg.data = self.latest_bbox
                self.bbox_center_pub.publish(bbox_msg)

            display_image = cv2.resize(rgb_image, (640, 360))
            cv2.imshow(self.window_name, display_image)
            cv2.waitKey(1)

        except Exception:
            pass

    def log_data_callback(self):
        """Log drone position and bounding box data to CSV every 250ms."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.csv_writer.writerow([
            current_time,
            f"{self.latest_position['x']:.3f}",
            f"{self.latest_position['y']:.3f}",
            f"{self.latest_position['z']:.3f}",
            f"{self.latest_bbox[0]:.1f}",
            f"{self.latest_bbox[1]:.1f}",
            f"{self.latest_bbox[2]:.1f}",
            f"{self.latest_bbox[3]:.1f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        """Clean up resources."""
        self.csv_file.close()
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()