"""Tests for lightweight YOLO ONNX output decoding."""

import numpy as np

from quadruped_perception.yolo_obstacle_detector import decode_yolo_output


def decode(output, has_objectness=False):
    """Decode a synthetic 320-square model output for a 640x480 image."""
    return decode_yolo_output(
        output,
        image_width=640,
        image_height=480,
        input_width=320,
        input_height=320,
        confidence_threshold=0.45,
        nms_threshold=0.45,
        output_has_objectness=has_objectness,
        max_detections=20,
    )


def test_decodes_channel_first_yolov8_output():
    """YOLOv8/v11 channel-first output is transposed and scaled correctly."""
    output = np.zeros((1, 6, 10), dtype=np.float32)
    output[0, :, 0] = [160.0, 160.0, 100.0, 80.0, 0.90, 0.10]

    detections = decode(output)

    assert len(detections) == 1
    class_id, score, center_x, center_y, width, height = detections[0]
    assert class_id == 0
    assert abs(score - 0.90) < 1e-5
    assert (center_x, center_y, width, height) == (320.0, 240.0, 200.0, 120.0)


def test_supports_objectness_style_output():
    """YOLOv5 objectness is multiplied by the selected class score."""
    output = np.zeros((1, 7, 10), dtype=np.float32)
    output[0, :, 0] = [160.0, 160.0, 40.0, 40.0, 0.80, 0.90, 0.10]

    detections = decode(output, has_objectness=True)

    assert len(detections) == 1
    assert detections[0][0] == 0
    assert abs(detections[0][1] - 0.72) < 1e-5


def test_rejects_low_confidence_and_invalid_shapes():
    """Invalid and weak model output produces no detections."""
    weak = np.zeros((1, 6, 10), dtype=np.float32)
    weak[0, :, 0] = [160.0, 160.0, 40.0, 40.0, 0.20, 0.10]

    assert decode(weak) == []
    assert decode(np.zeros((1, 2, 3, 4), dtype=np.float32)) == []
