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
            value = raw_topics[key]
            if not isinstance(value, str):
                raise ValueError(
                    f"sensor profile '{profile_name}.{key}' must be a string"
                )
            topics[key] = value.strip()
        if not topics["scan_topic"] or not topics["odom_topic"]:
            raise ValueError(
                f"sensor profile '{profile_name}' requires scan_topic and "
                "odom_topic"
            )
        profiles[profile_name] = topics
    return profiles


def resolve_sensor_topics(
    profiles: Mapping[str, Mapping[str, str]],
    profile_name: str,
    overrides: Mapping[str, str],
) -> Dict[str, str]:
    """解析指定 profile，并用非空命令行参数覆盖相应话题。"""
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles))
        raise ValueError(
            f"unknown sensor_profile '{profile_name}'; available: {available}"
        )
    resolved = dict(profiles[profile_name])
    for key in TOPIC_KEYS:
        value = overrides.get(key, "")
        if value and value.strip():
            resolved[key] = value.strip()
    return resolved
