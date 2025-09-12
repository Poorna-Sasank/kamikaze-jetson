import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import cv2
from cv_bridge import CvBridge
import numpy as np

from ultralytics import YOLO

MODEL = "yolov8m.pt"

class PID:
    def __init__(self, kp=0.1, ki=0.0, kd=0.05, output_limits=(-90.0, 90.0), integral_limits=(-0.1, 0.1)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt=1.0):
        self.integral = np.clip(self.integral + error * dt, self.integral_min, self.integral_max)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(output, self.output_min, self.output_max)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__("gimbal_control_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.img_subscriber_ = self.create_subscription(
            Image,
            "/world/car/model/x500_gimbal_0/link/camera_link/sensor/camera/image",
            self.image_callback,
            qos,
        )

        self.gimbal_pitch_publisher_ = self.create_publisher(
            Float64, "/model/x500_gimbal_0/command/gimbal_pitch", 400
        )

        self.gimbal_yaw_publisher_ = self.create_publisher(
            Float64, "/model/x500_gimbal_0/command/gimbal_yaw", 400
        )

        self.model = YOLO(MODEL)
        self.bridge = CvBridge()
        self.pid = PID(kp=0.00001, ki=0.00002, kd=0.002)

        self.window_name = "Gimbal Control"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 360)

        self.cx = 640.0
        self.cy = 360.0
        self.prev_time = self.get_clock().now()

    def image_callback(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            if img.shape[:2] != (720, 1280):
                return

            results = self.model.predict(img, classes=[2], verbose=False)
            current_time = self.get_clock().now()
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            self.prev_time = current_time

            pitch_msg = Float64()
            yaw_msg = Float64()

            if results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes.xywh.cpu().numpy()
                confidences = results[0].boxes.conf.cpu().numpy()

                best_idx = np.argmax(confidences)
                center_x, center_y, width, height = boxes[best_idx]
                conf = confidences[best_idx]

                err_x = (center_x - self.cx) / self.cx
                err_y = (center_y - self.cy) / self.cy

                yaw_angle = -self.pid.compute(err_x, dt)
                yaw_msg.data = yaw_angle
                self.gimbal_yaw_publisher_.publish(yaw_msg)

                pitch_angle = -self.pid.compute(err_y, dt)
                pitch_msg.data = pitch_angle
                self.gimbal_pitch_publisher_.publish(pitch_msg)

                cv2.rectangle(img, (int(center_x - width / 2), int(center_y - height / 2)),
                              (int(center_x + width / 2), int(center_y + height / 2)), (255, 0, 0), 2)
                cv2.putText(img, f"Conf: {conf:.2f}", (int(center_x), int(center_y - height / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                self.pid.reset()

            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)

        except Exception:
            pass

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()