"""从 ROS 2 rosbag 离线评估并标定 Wakula 感知阈值。

工具只读取低带宽结果话题，将人工真值按时间戳匹配后计算分类指标。达到最低样本量时，
较早数据用于搜索参数，最新数据留作独立验证，防止“同一数据既调参又评分”的泄漏。
输出建议 YAML 不会覆盖正式配置；是否采用仍需结合各类别召回率和真机安全测试判断。
"""

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import yaml

from quadruped_planning.obstacle_crossing_manager import select_terrain_decision


VISION_CODES = {
    0: "none",
    1: "poles",
    2: "height_bar",
    3: "wall",
    4: "colored_obstacle",
}
TERRAIN_LABELS = {"WALK", "STEP", "CLIMB", "STOP"}


@dataclass(frozen=True)
class TimedRecord:
    """A decoded topic sample with its rosbag receive timestamp."""

    stamp_ns: int
    data: Tuple[float, ...]


@dataclass(frozen=True)
class GroundTruth:
    """Human label aligned to the rosbag clock."""

    stamp_ns: int
    vision_label: str = ""
    terrain_label: str = ""


def classification_metrics(
    expected: Sequence[str], predicted: Sequence[str]
) -> Dict[str, object]:
    """计算 accuracy、Wilson 95% 区间、混淆矩阵和逐类指标。

    使用 macro-F1 作为主要调参目标，可避免样本多数为 WALK/none 时仅靠“总猜平地”
    获得看似很高的 accuracy。Wilson 区间在小样本下比普通正态近似更可靠。
    """
    if len(expected) != len(predicted):
        raise ValueError("expected and predicted lengths differ")
    labels = sorted(set(expected) | set(predicted))
    confusion = {
        actual: {guess: 0 for guess in labels}
        for actual in labels
    }
    for actual, guess in zip(expected, predicted):
        confusion[actual][guess] += 1
    per_class = {}
    for label in labels:
        true_positive = confusion[label][label]
        false_positive = sum(
            confusion[actual][label] for actual in labels if actual != label
        )
        false_negative = sum(
            confusion[label][guess] for guess in labels if guess != label
        )
        precision = true_positive / max(1, true_positive + false_positive)
        recall = true_positive / max(1, true_positive + false_negative)
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall
            else 0.0
        )
        per_class[label] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "support": sum(confusion[label].values()),
        }
    count = len(expected)
    correct = sum(actual == guess for actual, guess in zip(expected, predicted))
    if count:
        z_value = 1.96
        proportion = correct / count
        denominator = 1.0 + z_value * z_value / count
        center = (proportion + z_value * z_value / (2.0 * count)) / denominator
        margin = (
            z_value
            * math.sqrt(
                proportion * (1.0 - proportion) / count
                + z_value * z_value / (4.0 * count * count)
            )
            / denominator
        )
        confidence_interval = [max(0.0, center - margin), min(1.0, center + margin)]
    else:
        confidence_interval = [0.0, 1.0]
    return {
        "samples": count,
        "accuracy": correct / max(1, count),
        "accuracy_ci95": confidence_interval,
        "macro_f1": (
            sum(item["f1"] for item in per_class.values()) / max(1, len(labels))
        ),
        "labels": labels,
        "confusion_matrix": confusion,
        "per_class": per_class,
    }


def vision_prediction(record: TimedRecord, confidence_threshold: float) -> str:
    """Decode one atomic visual-evidence record at a candidate threshold."""
    if len(record.data) < 2:
        return "none"
    code = int(round(record.data[0]))
    confidence = float(record.data[1])
    return VISION_CODES.get(code, "none") if confidence >= confidence_threshold else "none"


def terrain_prediction(
    record: TimedRecord,
    step: float,
    climb: float,
    stop: float,
    min_points: int = 30,
    max_slope: float = 0.45,
    max_roughness: float = 0.06,
) -> str:
    """Re-run the production terrain decision against one feature record."""
    data = record.data
    if len(data) < 4:
        return "STOP"
    height = float(data[6]) if len(data) > 6 else float(data[2])
    points = float(data[3])
    slope = abs(float(data[4])) if len(data) > 4 else 0.0
    roughness = float(data[5]) if len(data) > 5 else 0.0
    return select_terrain_decision(
        height,
        points,
        slope,
        roughness,
        min_points,
        step,
        climb,
        stop,
        max_slope,
        max_roughness,
    )[0]


def nearest_record(
    records: Sequence[TimedRecord],
    stamp_ns: int,
    tolerance_ns: int,
    stamps: Sequence[int] | None = None,
) -> Tuple[TimedRecord, int] | Tuple[None, None]:
    """用二分查找匹配容差内最近记录，避免对大型 rosbag 反复线性扫描。"""
    if not records:
        return None, None
    stamps = stamps or [item.stamp_ns for item in records]
    index = bisect.bisect_left(stamps, stamp_ns)
    candidates = records[max(0, index - 1) : min(len(records), index + 1)]
    record = min(candidates, key=lambda item: abs(item.stamp_ns - stamp_ns))
    delta = abs(record.stamp_ns - stamp_ns)
    return (record, delta) if delta <= tolerance_ns else (None, None)


def optimize_vision_threshold(
    samples: Sequence[Tuple[TimedRecord, str]],
) -> Tuple[float, Dict[str, object]]:
    """网格搜索视觉阈值：先最大化 macro-F1，再比较 accuracy 和保守高阈值。"""
    best = None
    for integer_threshold in range(30, 91, 5):
        threshold = integer_threshold / 100.0
        expected = [label for _, label in samples]
        predicted = [vision_prediction(record, threshold) for record, _ in samples]
        metrics = classification_metrics(expected, predicted)
        score = (metrics["macro_f1"], metrics["accuracy"], threshold)
        if best is None or score > best[0]:
            best = score, threshold, metrics
    if best is None:
        raise ValueError("no labeled vision samples")
    return best[1], best[2]


def optimize_terrain_thresholds(
    samples: Sequence[Tuple[TimedRecord, str]],
) -> Tuple[Tuple[float, float, float], Dict[str, object]]:
    """用生产决策函数搜索严格递增的米制高度阈值。

    分数相同时优先接近保守默认配置，避免有限样本把参数推到搜索边界。
    """
    step_values = [value / 100.0 for value in range(4, 17, 2)]
    climb_values = [value / 100.0 for value in range(12, 29, 2)]
    stop_values = [value / 100.0 for value in range(24, 45, 2)]
    expected = [label for _, label in samples]
    best = None
    for step in step_values:
        for climb in climb_values:
            for stop in stop_values:
                if not step < climb < stop:
                    continue
                predicted = [
                    terrain_prediction(record, step, climb, stop)
                    for record, _ in samples
                ]
                metrics = classification_metrics(expected, predicted)
                # Prefer a high score, then thresholds near conservative defaults.
                distance = abs(step - 0.08) + abs(climb - 0.18) + abs(stop - 0.32)
                score = (metrics["macro_f1"], metrics["accuracy"], -distance)
                if best is None or score > best[0]:
                    best = score, (step, climb, stop), metrics
    if best is None:
        raise ValueError("no labeled terrain samples")
    return best[1], best[2]


def chronological_split(samples, validation_fraction: float):
    """按时间保留最新样本验证，更贴近部署面对“未来数据”的真实情况。"""
    ordered = sorted(samples, key=lambda item: item[0].stamp_ns)
    fraction = max(0.0, min(0.5, float(validation_fraction)))
    if len(ordered) < 4 or fraction == 0.0:
        return ordered, []
    validation_count = max(1, int(round(len(ordered) * fraction)))
    validation_count = min(validation_count, len(ordered) - 2)
    return ordered[:-validation_count], ordered[-validation_count:]


def load_labels(path: Path) -> List[GroundTruth]:
    """Load the documented CSV label format and validate class names."""
    labels = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            vision = row.get("vision_label", "").strip().lower()
            terrain = row.get("terrain_label", "").strip().upper()
            if vision and vision not in VISION_CODES.values():
                raise ValueError(f"unknown vision label: {vision}")
            if terrain and terrain not in TERRAIN_LABELS:
                raise ValueError(f"unknown terrain label: {terrain}")
            labels.append(
                GroundTruth(int(row["timestamp_ns"]), vision, terrain)
            )
    return sorted(labels, key=lambda item: item.stamp_ns)


def read_rosbag(
    bag_path: Path, vision_topic: str, terrain_topic: str
) -> Dict[str, List[TimedRecord]]:
    """Decode only the two low-bandwidth result topics from a rosbag2 bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    wanted = {vision_topic, terrain_topic}
    missing = wanted - type_map.keys()
    if missing:
        raise ValueError(f"bag is missing topics: {', '.join(sorted(missing))}")
    message_types = {topic: get_message(type_map[topic]) for topic in wanted}
    records = {vision_topic: [], terrain_topic: []}
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(serialized, message_types[topic])
        records[topic].append(
            TimedRecord(int(stamp_ns), tuple(float(value) for value in msg.data))
        )
    return records


def write_label_template(
    path: Path,
    vision_records: Sequence[TimedRecord],
    terrain_records: Sequence[TimedRecord],
    period_s: float,
) -> int:
    """按固定时间间隔生成稀疏标注表，不依据预测结果挑选样本。"""
    all_stamps = sorted(
        {item.stamp_ns for item in vision_records}
        | {item.stamp_ns for item in terrain_records}
    )
    if not all_stamps:
        raise ValueError("bag contains no perception records")
    period_ns = max(1, int(period_s * 1e9))
    selected = []
    next_stamp = all_stamps[0]
    for stamp in all_stamps:
        if stamp >= next_stamp:
            selected.append(stamp)
            next_stamp = stamp + period_ns
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "timestamp_ns",
                "elapsed_s",
                "vision_label",
                "terrain_label",
                "notes",
            ]
        )
        origin = selected[0]
        for stamp in selected:
            writer.writerow([stamp, f"{(stamp - origin) / 1e9:.3f}", "", "", ""])
    return len(selected)


def evaluate(
    labels: Sequence[GroundTruth],
    vision_records: Sequence[TimedRecord],
    terrain_records: Sequence[TimedRecord],
    tolerance_s: float,
    validation_fraction: float = 0.30,
    minimum_samples: int = 20,
) -> Tuple[Dict[str, object], Dict[str, object]]:
    """对齐真值、检查样本门槛、训练/验证，并返回报告及 YAML 建议值。"""
    tolerance_ns = int(max(0.0, tolerance_s) * 1e9)
    vision_samples = []
    terrain_samples = []
    deltas = []
    vision_stamps = [item.stamp_ns for item in vision_records]
    terrain_stamps = [item.stamp_ns for item in terrain_records]
    for label in labels:
        if label.vision_label:
            record, delta = nearest_record(
                vision_records, label.stamp_ns, tolerance_ns, vision_stamps
            )
            if record is not None:
                vision_samples.append((record, label.vision_label))
                deltas.append(delta)
        if label.terrain_label:
            record, delta = nearest_record(
                terrain_records, label.stamp_ns, tolerance_ns, terrain_stamps
            )
            if record is not None:
                terrain_samples.append((record, label.terrain_label))
                deltas.append(delta)
    report = {
        "labeled_rows": len(labels),
        "matched_vision_samples": len(vision_samples),
        "matched_terrain_samples": len(terrain_samples),
        "mean_alignment_ms": (
            sum(deltas) / max(1, len(deltas)) / 1e6
        ),
        "validation_fraction": max(0.0, min(0.5, validation_fraction)),
        "warnings": [],
    }
    suggestions = {}
    if vision_samples:
        if len(vision_samples) < minimum_samples:
            report["warnings"].append(
                f"vision has {len(vision_samples)} samples; need {minimum_samples} "
                "before accepting calibrated parameters"
            )
            expected = [label for _, label in vision_samples]
            predicted = [vision_prediction(record, 0.55) for record, _ in vision_samples]
            report["vision"] = {
                "status": "insufficient_samples",
                "baseline_threshold": 0.55,
                "all_samples": classification_metrics(expected, predicted),
            }
        else:
            training, validation = chronological_split(
                vision_samples, validation_fraction
            )
            threshold, training_metrics = optimize_vision_threshold(training)
            vision_report = {
                "status": "calibrated",
                "selected_threshold": threshold,
                "training": training_metrics,
            }
            if validation:
                expected = [label for _, label in validation]
                predicted = [
                    vision_prediction(record, threshold) for record, _ in validation
                ]
                vision_report["validation"] = classification_metrics(
                    expected, predicted
                )
            else:
                report["warnings"].append(
                    "vision validation holdout unavailable; collect more samples"
                )
            report["vision"] = vision_report
            suggestions["vision_min_confidence"] = threshold
    if terrain_samples:
        if len(terrain_samples) < minimum_samples:
            report["warnings"].append(
                f"terrain has {len(terrain_samples)} samples; need {minimum_samples} "
                "before accepting calibrated parameters"
            )
            expected = [label for _, label in terrain_samples]
            predicted = [
                terrain_prediction(record, 0.08, 0.18, 0.32)
                for record, _ in terrain_samples
            ]
            report["terrain"] = {
                "status": "insufficient_samples",
                "baseline_thresholds": [0.08, 0.18, 0.32],
                "all_samples": classification_metrics(expected, predicted),
            }
        else:
            training, validation = chronological_split(
                terrain_samples, validation_fraction
            )
            thresholds, training_metrics = optimize_terrain_thresholds(training)
            terrain_report = {
                "status": "calibrated",
                "selected_thresholds": list(thresholds),
                "training": training_metrics,
            }
            if validation:
                expected = [label for _, label in validation]
                predicted = [
                    terrain_prediction(record, *thresholds)
                    for record, _ in validation
                ]
                terrain_report["validation"] = classification_metrics(
                    expected, predicted
                )
            else:
                report["warnings"].append(
                    "terrain validation holdout unavailable; collect more samples"
                )
            report["terrain"] = terrain_report
            suggestions.update(
                {
                    "step_threshold": thresholds[0],
                    "climb_threshold": thresholds[1],
                    "stop_threshold": thresholds[2],
                }
            )
    if not vision_samples and not terrain_samples:
        raise ValueError("no labels matched bag records within tolerance")
    return report, suggestions


def build_argument_parser() -> argparse.ArgumentParser:
    """定义稳定 CLI；路径参数保持 Path 类型，避免调用方手动转换。"""
    parser = argparse.ArgumentParser(
        description="Create labels or evaluate Wakula perception from rosbag2."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--labels", type=Path)
    parser.add_argument("--write-label-template", type=Path)
    parser.add_argument("--report", type=Path, default=Path("perception_report.json"))
    parser.add_argument(
        "--suggestions", type=Path, default=Path("calibration_suggestions.yaml")
    )
    parser.add_argument("--vision-topic", default="/vision/obstacle_evidence")
    parser.add_argument("--terrain-topic", default="/terrain/features")
    parser.add_argument("--tolerance", type=float, default=0.15)
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--validation-fraction", type=float, default=0.30)
    parser.add_argument("--minimum-samples", type=int, default=20)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    records = read_rosbag(args.bag, args.vision_topic, args.terrain_topic)
    vision_records = records[args.vision_topic]
    terrain_records = records[args.terrain_topic]
    if args.write_label_template:
        count = write_label_template(
            args.write_label_template,
            vision_records,
            terrain_records,
            args.sample_period,
        )
        print(f"Wrote {count} label rows to {args.write_label_template}")
        return 0
    if args.labels is None:
        raise SystemExit("--labels or --write-label-template is required")
    report, suggestions = evaluate(
        load_labels(args.labels),
        vision_records,
        terrain_records,
        args.tolerance,
        args.validation_fraction,
        max(1, args.minimum_samples),
    )
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.suggestions.write_text(
        yaml.safe_dump(
            {"obstacle_crossing_manager": {"ros__parameters": suggestions}},
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Suggested overrides: {args.suggestions}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
