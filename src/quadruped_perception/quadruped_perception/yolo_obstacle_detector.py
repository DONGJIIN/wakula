"""Optional, resource-bounded YOLO ONNX front end for future deployment."""

from pathlib import Path
from time import perf_counter
from typing import List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


DecodedDetection = Tuple[int, float, float, float, float, float]


def decode_yolo_output(
    output: np.ndarray,
    image_width: int,
    image_height: int,
    input_width: int,
    input_height: int,
    confidence_threshold: float,
    nms_threshold: float,
    output_has_objectness: bool,
    max_detections: int,
) -> List[DecodedDetection]:
    """Decode common YOLOv5/v8/v11 ONNX output into pixel boxes."""
    predictions = np.asarray(output).squeeze()
    if predictions.ndim != 2:
        return []
    # Recent Ultralytics exports use [channels, candidates]; older exports
    # commonly use [candidates, channels].
    if predictions.shape[0] <= 256 and predictions.shape[1] > predictions.shape[0]:
        predictions = predictions.T

    class_offset = 5 if output_has_objectness else 4
    if predictions.shape[1] <= class_offset:
        return []

    x_scale = image_width / float(input_width)
    y_scale = image_height / float(input_height)
    boxes = []
    scores = []
    class_ids = []
    for row in predictions:
        class_scores = row[class_offset:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        if output_has_objectness:
            score *= float(row[4])
        if not np.isfinite(score) or score < confidence_threshold:
            continue
        center_x, center_y, width, height = (float(value) for value in row[:4])
        if not all(np.isfinite(value) for value in (center_x, center_y, width, height)):
            continue
        left = (center_x - width / 2.0) * x_scale
        top = (center_y - height / 2.0) * y_scale
        boxes.append([left, top, width * x_scale, height * y_scale])
        scores.append(score)
        class_ids.append(class_id)

    if not boxes:
        return []
    kept = cv2.dnn.NMSBoxes(boxes, scores, confidence_threshold, nms_threshold)
    indices = np.asarray(kept).reshape(-1).tolist() if len(kept) else []
    decoded = []
    for index in indices[:max_detections]:
        left, top, width, height = boxes[index]
        left = max(0.0, min(float(image_width), left))
        top = max(0.0, min(float(image_height), top))
        width = max(0.0, min(float(image_width) - left, width))
        height = max(0.0, min(float(image_height) - top, height))
        decoded.append(
            (
                class_ids[index],
                scores[index],
                left + width / 2.0,
                top + height / 2.0,
                width,
                height,
            )
        )
    return decoded


class YoloObstacleDetector(Node):
    """Run optional throttled YOLO inference without blocking camera callbacks."""

    def __init__(self):
        super().__init__("yolo_obstacle_detector")
        self.declare_parameter("enabled", False)
        self.declare_parameter("model_path", "")
        self.declare_parameter("labels_path", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/vision/yolo/detections")
        self.declare_parameter("debug_image_topic", "/vision/yolo/debug_image")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("input_width", 320)
        self.declare_parameter("input_height", 320)
        self.declare_parameter("inference_hz", 5.0)
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("output_has_objectness", False)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("opencv_threads", 2)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.latest_image = None
        self.last_processed_stamp = None
        self.bridge = CvBridge()
        if not self.enabled:
            self.get_logger().info("YOLO is disabled; no model or camera resources allocated")
            return

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_file():
            self.get_logger().error(
                f"YOLO requested but model_path is not a file: {model_path}"
            )
            self.enabled = False
            return

        requested_width = max(32, int(self.get_parameter("input_width").value))
        requested_height = max(32, int(self.get_parameter("input_height").value))
        self.input_width = ((requested_width + 31) // 32) * 32
        self.input_height = ((requested_height + 31) // 32) * 32
        self.confidence = min(
            1.0, max(0.0, float(self.get_parameter("confidence_threshold").value))
        )
        self.nms = min(1.0, max(0.0, float(self.get_parameter("nms_threshold").value)))
        self.has_objectness = bool(
            self.get_parameter("output_has_objectness").value
        )
        self.max_detections = max(
            1, int(self.get_parameter("max_detections").value)
        )
        self.publish_debug = bool(
            self.get_parameter("publish_debug_image").value
        )
        thread_count = min(
            4, max(1, int(self.get_parameter("opencv_threads").value))
        )
        cv2.setNumThreads(thread_count)
        try:
            self.network = cv2.dnn.readNetFromONNX(str(model_path))
        except cv2.error as exc:
            self.get_logger().error(f"Failed to load YOLO ONNX model: {exc}")
            self.enabled = False
            return
        self.network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.labels = self._load_labels(
            str(self.get_parameter("labels_path").value)
        )

        detections_topic = str(self.get_parameter("detections_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.detections_pub = self.create_publisher(
            Detection2DArray, detections_topic, 10
        )
        self.inference_time_pub = self.create_publisher(
            Float32, "/vision/yolo/inference_ms", 10
        )
        self.debug_pub = (
            self.create_publisher(Image, debug_topic, 1)
            if self.publish_debug
            else None
        )
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        inference_hz = min(
            10.0, max(0.2, float(self.get_parameter("inference_hz").value))
        )
        self.create_timer(1.0 / inference_hz, self.inference_callback)
        self.get_logger().info(
            f"YOLO enabled: {model_path}, {self.input_width}x{self.input_height}, "
            f"{inference_hz:.1f} Hz, {thread_count} OpenCV threads"
        )

    def _load_labels(self, labels_path: str) -> List[str]:
        path = Path(labels_path).expanduser()
        if not labels_path or not path.is_file():
            return []
        return [line.strip() for line in path.read_text().splitlines() if line.strip()]

    def image_callback(self, msg: Image) -> None:
        """Keep only the newest image; the timer bounds inference frequency."""
        self.latest_image = msg

    def inference_callback(self) -> None:
        """Run one inference when a new frame is available."""
        msg = self.latest_image
        if msg is None or msg.header.stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = msg.header.stamp
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"YOLO image conversion failed: {exc}")
            return

        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1.0 / 255.0,
            size=(self.input_width, self.input_height),
            swapRB=True,
            crop=False,
        )
        started = perf_counter()
        self.network.setInput(blob)
        output = self.network.forward()
        elapsed_ms = (perf_counter() - started) * 1000.0
        self.inference_time_pub.publish(Float32(data=float(elapsed_ms)))
        detections = decode_yolo_output(
            output,
            image.shape[1],
            image.shape[0],
            self.input_width,
            self.input_height,
            self.confidence,
            self.nms,
            self.has_objectness,
            self.max_detections,
        )
        self._publish_detections(msg, image, detections)

    def _publish_detections(
        self, msg: Image, image: np.ndarray, detections: Sequence[DecodedDetection]
    ) -> None:
        output = Detection2DArray()
        output.header = msg.header
        for class_id, score, center_x, center_y, width, height in detections:
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = center_x
            detection.bbox.center.position.y = center_y
            detection.bbox.size_x = width
            detection.bbox.size_y = height
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = self._class_name(class_id)
            result.hypothesis.score = score
            detection.results.append(result)
            output.detections.append(detection)
            if self.publish_debug:
                self._draw_detection(
                    image, class_id, score, center_x, center_y, width, height
                )
        self.detections_pub.publish(output)
        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.labels):
            return self.labels[class_id]
        return str(class_id)

    def _draw_detection(
        self,
        image: np.ndarray,
        class_id: int,
        score: float,
        center_x: float,
        center_y: float,
        width: float,
        height: float,
    ) -> None:
        left = int(center_x - width / 2.0)
        top = int(center_y - height / 2.0)
        right = int(center_x + width / 2.0)
        bottom = int(center_y + height / 2.0)
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 255), 2)
        label = f"{self._class_name(class_id)} {score:.2f}"
        cv2.putText(
            image,
            label,
            (left, max(15, top - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


def main(args=None):
    """Run the optional YOLO detector node."""
    rclpy.init(args=args)
    node = YoloObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
