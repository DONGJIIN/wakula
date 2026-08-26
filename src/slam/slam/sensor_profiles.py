"""读取并校验可替换传感器的话题 profile。

profile 只解决厂商默认话题名不同的问题，不允许改变消息类型、坐标系语义或算法参数。
因此换设备时可以只改 YAML/remap，而不会悄悄破坏 SLAM、Nav2 和感知节点间的合同。
"""

from pathlib import Path
from typing import Dict, Mapping

import yaml


TOPIC_KEYS = (
    "scan_topic",
    "odom_topic",
    "camera_topic",
    "point_cloud_topic",
)


def _validated_topic_name(
    value: object,
    *,
    field: str,
    allow_empty: bool,
) -> str:
    """Return one conservative, fully-qualified ROS 2 topic name.

    The launch files deliberately use absolute names so a copied algorithm stack has
    exactly the same public contract regardless of the node namespace chosen by the
    hardware team.  This is not intended to reimplement all of ``rcl`` name
    validation; it catches the integration mistakes that are both common and hard to
    diagnose later: whitespace, relative names, repeated separators and substitutions.

    Camera and point-cloud inputs may be empty because those sensors are optional.
    LaserScan and odometry are mandatory and therefore pass ``allow_empty=False``.
    """
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    topic = value.strip()
    if not topic:
        if allow_empty:
            return ""
        raise ValueError(f"{field} must not be empty")
    if not topic.startswith("/"):
        raise ValueError(f"{field} must be an absolute ROS topic: {topic!r}")
    if topic != "/" and topic.endswith("/"):
        raise ValueError(f"{field} must not end with '/': {topic!r}")
    if "//" in topic or any(character.isspace() for character in topic):
        raise ValueError(f"{field} contains whitespace or an empty namespace token")
    if any(character in topic for character in "~{}"):
        raise ValueError(f"{field} must not contain ROS substitutions: {topic!r}")
    return topic


def load_sensor_profiles(path: str) -> Dict[str, Dict[str, str]]:
    """从一个 YAML 文件返回经过结构和必填项校验的话题 profile。

    profile 只能包含名称；消息类型与 TF 合同保持固定，防止换设备时无意改变算法接口。
    """
    with Path(path).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    raw_profiles = document.get("profiles")
    if not isinstance(raw_profiles, dict) or not raw_profiles:
        raise ValueError(
            "sensor profile file must contain a non-empty 'profiles' map"
        )

    profiles: Dict[str, Dict[str, str]] = {}
    for profile_name, raw_topics in raw_profiles.items():
        if not isinstance(profile_name, str) or not profile_name.strip():
            raise ValueError(
                "sensor profile names must be non-empty strings"
            )
        if not isinstance(raw_topics, dict):
            raise ValueError(f"sensor profile '{profile_name}' must be a map")
        missing = [key for key in TOPIC_KEYS if key not in raw_topics]
        if missing:
            missing_names = ", ".join(missing)
            raise ValueError(
                f"sensor profile '{profile_name}' is missing: {missing_names}"
            )
        topics = {}
        for key in TOPIC_KEYS:
            topics[key] = _validated_topic_name(
                raw_topics[key],
                field=f"sensor profile '{profile_name}.{key}'",
                allow_empty=key in ("camera_topic", "point_cloud_topic"),
            )
        profiles[profile_name] = topics
    return profiles


def resolve_sensor_topics(
    profiles: Mapping[str, Mapping[str, str]],
    profile_name: str,
    overrides: Mapping[str, str],
) -> Dict[str, str]:
    """解析指定 profile，并用经过同等校验的非空启动参数覆盖话题。

    覆盖值通常来自 launch 命令行。若只校验 YAML 而信任覆盖值，一个漏写的 ``/`` 会让
    节点悄悄订阅私有命名空间，最终表现为“算法已启动但一直没有数据”。因此两条入口必须
    使用完全相同的规则。
    """
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles))
        raise ValueError(
            f"unknown sensor_profile '{profile_name}'; available: {available}"
        )
    resolved = dict(profiles[profile_name])
    for key in TOPIC_KEYS:
        value = overrides.get(key, "")
        if value is None:
            continue
        if not isinstance(value, str):
            raise ValueError(f"topic override '{key}' must be a string")
        if value.strip():
            resolved[key] = _validated_topic_name(
                value,
                field=f"topic override '{key}'",
                allow_empty=False,
            )
    return resolved
