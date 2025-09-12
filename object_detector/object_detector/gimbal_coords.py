#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
import numpy as np

MODEL = "yolov8m.pt"


class CarDetectionNode(Node):
    def __init__(self):
        super().__init__("car_detection_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.rgb_sub = self.create_subscription(
            Image,
            "/world/car/model/x500_gimbal_0/link/camera_link/sensor/camera/image",
            self.image_callback,
            qos,
        )
        self.bbox_center_pub = self.create_publisher(
            Float32MultiArray, "/car_bbox_center", 10
        )
        self.model = YOLO(MODEL)
        self.br = CvBridge()
        self.cx = 640.0
        self.cy = 360.0
        self.window_name = "Car Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 360)

    def image_callback(self, rgb_msg):
        try:
            rgb_image = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
            if rgb_image.shape[:2] != (720, 1280):
                return
            results = self.model.predict(rgb_image, classes=[2], verbose=False)
            target_found = False
            if results[0].boxes is not None and len(results[0].boxes) > 0:

                boxes = results[0].boxes.xywh.cpu().numpy()
                confidences = results[0].boxes.conf.cpu().numpy()

                best_idx = np.argmax(confidences)
                center_x, center_y, width, height = boxes[best_idx]
                conf = confidences[best_idx]

                if 0 <= center_x <= 720 and 0 <= center_y <= 1280:

                    bbox_msg = Float32MultiArray()
                    bbox_msg.data = [float(center_x), float(center_y), float(width), float(height)]
                    x_error = self.cx - center_x 
                    y_error = self.cy - center_y  
                    self.get_logger().info(f"X Error: {x_error:.2f}, Y Error: {y_error:.2f}")
                    
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

                bbox_msg = Float32MultiArray()
                bbox_msg.data = []
                self.bbox_center_pub.publish(bbox_msg)

            display_image = cv2.resize(rgb_image, (640, 360))
            cv2.imshow(self.window_name, display_image)
            cv2.waitKey(1)

        except Exception:
            pass

    def destroy_node(self):
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