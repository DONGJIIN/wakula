"""Cheap structural checks performed before a candidate sensor owns the pipeline.

The perception nodes subscribe to several common default topics so a new camera or depth
sensor can usually be connected without editing code.  DDS discovery alone cannot tell whether
a publisher is useful: an accidentally bridged empty Image or PointCloud2 may publish forever.
If such a stream becomes the active source before validation, every healthy fallback topic is
suppressed by the source-switch timeout.

These checks deliberately inspect only metadata and buffer sizes; expensive image conversion,
TF lookup, and NumPy point decoding remain in the rate-limited processing timer.  They also have
no rclpy dependency, which makes the ownership rule straightforward to test offline.
"""

from __future__ import annotations

from math import isfinite

from sensor_msgs.msg import Image, PointCloud2, PointField


def header_contract_valid(header) -> bool:
    """Require a usable source frame and a positive, normalized ROS timestamp."""
    frame = str(header.frame_id).strip()
    seconds = int(header.stamp.sec)
    nanoseconds = int(header.stamp.nanosec)
    return (
        bool(frame)
        and not frame.startswith("/")
        and not frame.endswith("/")
        and not any(character.isspace() for character in frame)
        and seconds >= 0
        and 0 <= nanoseconds < 1_000_000_000
        and (seconds > 0 or nanoseconds > 0)
    )


def source_stamp_is_plausible(
    header, now_seconds: float, maximum_age: float, future_tolerance: float = 0.10
) -> bool:
    """Reject replayed/future samples so stale traffic cannot refresh source ownership.

    The 100 ms defensive default matches the navigation safety consumer.  A producer accepted by
    source arbitration must not later be rejected solely because perception used a wider future
    window; otherwise that unusable high-rate source can keep a healthy backup locked out.
    Online nodes pass their validated YAML value explicitly so the contract remains auditable.
    """
    stamp = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
    values = (stamp, now_seconds, maximum_age, future_tolerance)
    if not header_contract_valid(header) or not all(isfinite(value) for value in values):
        return False
    age = float(now_seconds) - stamp
    return max(0.0, float(maximum_age)) >= age >= -max(0.0, float(future_tolerance))


def source_stamp_strictly_advances(header, previous_seconds) -> bool:
    """Require one sensor source to advance its Header timestamp monotonically.

    DDS can redeliver a sample and some USB/Ethernet drivers flush an older buffered
    frame after a newer one.  Treating either packet as a new observation would let a
    single physical image/cloud contribute several votes to obstacle confirmation.
    ``previous_seconds=None`` starts a new source session; callers must reset it when
    the ROS clock rewinds or a different candidate topic takes ownership.
    """
    if not header_contract_valid(header):
        return False
    current = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
    if not isfinite(current):
        return False
    if previous_seconds is None:
        return True
    try:
        previous = float(previous_seconds)
    except (TypeError, ValueError):
        return False
    return isfinite(previous) and current > previous + 1e-9


def image_message_contract_valid(
    message: Image, minimum_bytes_per_pixel: int = 1
) -> bool:
    """Reject empty/inconsistent raw Image buffers before selecting their topic.

    ``step`` may include row padding, therefore the contract allows more than the minimum
    payload.  Exact channel/bit-depth interpretation remains CvBridge's responsibility.
    """
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    bytes_per_pixel = max(1, int(minimum_bytes_per_pixel))
    return (
        header_contract_valid(message.header)
        and width > 0
        and height > 0
        and step >= width * bytes_per_pixel
        and bool(str(message.encoding).strip())
        # ROS Image defines exactly ``step * height`` bytes.  Extra trailing bytes are not row
        # padding (padding is already included in step) and usually indicate a stale width/step
        # metadata bug; accepting them lets CvBridge reinterpret a corrupt frame differently.
        and len(message.data) == step * height
    )


def point_cloud_message_contract_valid(message: PointCloud2) -> bool:
    """Require a non-empty, consistent-layout cloud with conventional floating XYZ fields."""
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if (
        not header_contract_valid(message.header)
        or width <= 0
        or height <= 0
        or point_step <= 0
        or row_step < point_step * width
        # PointCloud2 row padding is represented by row_step itself, so the serialized payload must
        # match it exactly.  A larger buffer with stale dimensions can make different decoders read
        # different physical points and is not a portable sensor contract.
        or len(message.data) != row_step * height
    ):
        return False
    field_names = [str(field.name) for field in message.fields]
    # ``read_points_numpy`` rejects duplicate names even when the requested XYZ names themselves
    # look valid.  Reject them before source arbitration so a malformed preferred topic cannot own
    # the pipeline and starve a healthy backup sensor.
    if len(field_names) != len(set(field_names)):
        return False
    fields = {field.name: field for field in message.fields}
    xyz_fields = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        scalar_size = (
            4 if field is not None and int(field.datatype) == PointField.FLOAT32 else 8
        )
        if (
            field is None
            or int(field.count) != 1
            or int(field.datatype) not in (PointField.FLOAT32, PointField.FLOAT64)
            or int(field.offset) < 0
            or int(field.offset) + scalar_size > point_step
        ):
            return False
        xyz_fields.append(
            (
                int(field.offset),
                int(field.offset) + scalar_size,
                int(field.datatype),
            )
        )
    # Jazzy's vectorized decoder requires all selected fields to have one datatype.  Distinct byte
    # ranges are also part of a conventional XYZ layout; overlapping fields otherwise decode three
    # copies of one coordinate and can create plausible but completely false terrain geometry.
    if len({item[2] for item in xyz_fields}) != 1:
        return False
    intervals = sorted((start, end) for start, end, _ in xyz_fields)
    if any(
        next_start < end
        for (_, end), (next_start, _) in zip(intervals, intervals[1:])
    ):
        return False
    return True
