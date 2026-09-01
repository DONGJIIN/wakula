"""Structural sensor-message tests for safe multi-topic source arbitration."""

from copy import deepcopy

from sensor_msgs.msg import Image, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from quadruped_perception.sensor_contracts import (
    image_message_contract_valid,
    point_cloud_message_contract_valid,
    source_stamp_strictly_advances,
    source_stamp_is_plausible,
)


def _header():
    """Return a non-zero stamped sensor header suitable for offline contract tests."""
    header = Header()
    header.stamp.sec = 1
    header.frame_id = "sensor_link"
    return header


def _image():
    """Return a small valid padded-row-free BGR image."""
    message = Image()
    message.header = _header()
    message.width = 4
    message.height = 3
    message.encoding = "bgr8"
    message.step = message.width * 3
    message.data = bytes(message.step * message.height)
    return message


def test_image_contract_rejects_empty_stale_and_inconsistent_buffers():
    """Malformed primary camera traffic must not suppress a healthy fallback topic."""
    message = _image()
    assert image_message_contract_valid(message)

    empty = deepcopy(message)
    empty.data = b""
    assert not image_message_contract_valid(empty)
    zero_stamp = deepcopy(message)
    zero_stamp.header.stamp.sec = 0
    assert not image_message_contract_valid(zero_stamp)
    short_step = deepcopy(message)
    short_step.step = 2
    assert not image_message_contract_valid(short_step)
    no_encoding = deepcopy(message)
    no_encoding.encoding = ""
    assert not image_message_contract_valid(no_encoding)
    slash_frame = deepcopy(message)
    slash_frame.header.frame_id = "/camera_link"
    assert not image_message_contract_valid(slash_frame)

    # Metadata validation in the camera callback supplies bgr8's three bytes per pixel.  A row
    # containing only one byte per pixel used to lock source ownership until CvBridge failed later.
    short_bgr_row = deepcopy(message)
    short_bgr_row.step = short_bgr_row.width
    short_bgr_row.data = bytes(short_bgr_row.step * short_bgr_row.height)
    assert not image_message_contract_valid(short_bgr_row, minimum_bytes_per_pixel=3)

    trailing_bytes = deepcopy(message)
    trailing_bytes.data = bytes(trailing_bytes.data) + b"\x00"
    assert not image_message_contract_valid(trailing_bytes, minimum_bytes_per_pixel=3)


def test_point_cloud_contract_requires_consistent_float_xyz_layout():
    """Reject missing fields, truncated buffers, and nonstandard integer XYZ coordinates."""
    message = point_cloud2.create_cloud_xyz32(_header(), [(1.0, 0.0, 0.0)])
    assert point_cloud_message_contract_valid(message)

    missing_z = deepcopy(message)
    missing_z.fields = missing_z.fields[:2]
    assert not point_cloud_message_contract_valid(missing_z)
    truncated = deepcopy(message)
    truncated.data = truncated.data[:-1]
    assert not point_cloud_message_contract_valid(truncated)
    integer_x = deepcopy(message)
    integer_x.fields[0].datatype = PointField.UINT16
    assert not point_cloud_message_contract_valid(integer_x)
    overflowing_z = deepcopy(message)
    overflowing_z.fields[2].offset = overflowing_z.point_step - 2
    assert not point_cloud_message_contract_valid(overflowing_z)
    zero_stamp = deepcopy(message)
    zero_stamp.header.stamp.sec = 0
    assert not point_cloud_message_contract_valid(zero_stamp)

    duplicate_x = deepcopy(message)
    duplicate_x.fields.insert(0, deepcopy(duplicate_x.fields[0]))
    assert not point_cloud_message_contract_valid(duplicate_x)

    overlapping = deepcopy(message)
    overlapping.fields[1].offset = overlapping.fields[0].offset
    assert not point_cloud_message_contract_valid(overlapping)

    mixed_precision = deepcopy(message)
    mixed_precision.point_step = 16
    mixed_precision.row_step = 16
    mixed_precision.data = bytes(16)
    mixed_precision.fields[2].offset = 8
    mixed_precision.fields[2].datatype = PointField.FLOAT64
    assert not point_cloud_message_contract_valid(mixed_precision)

    trailing_bytes = deepcopy(message)
    trailing_bytes.data = bytes(trailing_bytes.data) + b"\x00"
    assert not point_cloud_message_contract_valid(trailing_bytes)


def test_source_stamp_rejects_replay_and_unreasonable_future_data():
    """High-rate stale traffic must not hold the preferred source lock forever."""
    header = _header()
    assert source_stamp_is_plausible(header, 1.5, 1.0)
    assert not source_stamp_is_plausible(header, 3.0, 1.0)
    assert not source_stamp_is_plausible(header, 0.5, 1.0, future_tolerance=0.25)
    header.stamp.sec = 10
    header.stamp.nanosec = 100_000_000
    assert source_stamp_is_plausible(header, 10.0, 1.0)
    header.stamp.nanosec = 100_000_001
    assert not source_stamp_is_plausible(header, 10.0, 1.0)


def test_source_stamp_requires_strict_progress_within_one_sensor_session():
    """Duplicate and delayed packets cannot become extra camera/cloud votes."""
    header = _header()
    assert source_stamp_strictly_advances(header, None)
    assert not source_stamp_strictly_advances(header, 1.0)
    assert not source_stamp_strictly_advances(header, 1.1)
    header.stamp.nanosec = 100_000_000
    assert source_stamp_strictly_advances(header, 1.0)
