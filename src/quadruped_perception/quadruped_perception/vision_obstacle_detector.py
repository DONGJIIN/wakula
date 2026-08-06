"""OpenCV color-feature front end for competition obstacle perception.

The node deliberately publishes geometric color features instead of claiming
an obstacle class. Depth/point-cloud geometry and the competition state
machine can fuse these measurements later without coupling to OpenCV details.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class VisionObstacleDetector(Node):
    """Segment orange obstacles and blue height-bar markings in HSV space."""

    def __init__(self):
        super().__init__("vision_obstacle_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("debug_mask_topic", "/vision/color_mask")
        self.declare_parameter("min_area_px", 300.0)
        self.declare_parameter("morphology_size", 5)
        self.declare_parameter("orange_hsv_lower", [5, 80, 70])
        self.declare_parameter("orange_hsv_upper", [25, 255, 255])
        self.declare_parameter("blue_hsv_lower", [90, 70, 50])
        self.declare_parameter("blue_hsv_upper", [135, 255, 255])

        self.image_topic = str(self.get_parameter("image_topic").value)
        debug_topic = str(self.get_parameter("debug_mask_topic").value)
        self.min_area = float(self.get_parameter("min_area_px").value)
        kernel_size = max(1, int(self.get_parameter("morphology_size").value))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self.orange_lower = np.array(
            self.get_parameter("orange_hsv_lower").value, dtype=np.uint8
        )
        self.orange_upper = np.array(
            self.get_parameter("orange_hsv_upper").value, dtype=np.uint8
        )
        self.blue_lower = np.array(
            self.get_parameter("blue_hsv_lower").value, dtype=np.uint8
        )
        self.blue_upper = np.array(
            self.get_parameter("blue_hsv_upper").value, dtype=np.uint8
        )

        self.bridge = CvBridge()
        self.feature_pub = self.create_publisher(
            Float32MultiArray, "/vision/color_features", 10
        )
        self.mask_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"OpenCV vision front end listening on {self.image_topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return
        if bgr.size == 0:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        orange_mask = self._clean_mask(
            cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        )
        blue_mask = self._clean_mask(cv2.inRange(hsv, self.blue_lower, self.blue_upper))
        orange = self._largest_feature(orange_mask)
        blue = self._largest_feature(blue_mask)
        image_area = float(bgr.shape[0] * bgr.shape[1])

        features = Float32MultiArray()
        features.data = [
            orange[0] / image_area,
            orange[1],
            orange[2],
            orange[3],
            orange[4],
            blue[0] / image_area,
            blue[1],
            blue[2],
            blue[3],
            blue[4],
        ]
        self.feature_pub.publish(features)

        debug_mask = cv2.merge((blue_mask, np.zeros_like(blue_mask), orange_mask))
        debug_msg = self.bridge.cv2_to_imgmsg(debug_mask, encoding="bgr8")
        debug_msg.header = msg.header
        self.mask_pub.publish(debug_msg)

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    def _largest_feature(self, mask: np.ndarray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        x, y, width, height = cv2.boundingRect(contour)
        image_height, image_width = mask.shape[:2]
        center_x = (x + width / 2.0) / image_width
        center_y = (y + height / 2.0) / image_height
        return area, center_x, center_y, width / image_width, height / image_height


def main(args=None):
    rclpy.init(args=args)
    node = VisionObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
