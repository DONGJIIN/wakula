"""Tests for rosbag-independent calibration and metric functions."""

from quadruped_tools.perception_bag_evaluator import (
    GroundTruth,
    TimedRecord,
    classification_metrics,
    evaluate,
    load_labels,
    optimize_terrain_thresholds,
    optimize_vision_threshold,
    write_label_template,
)


def test_classification_metrics_include_confusion_and_macro_f1():
    metrics = classification_metrics(
        ["poles", "poles", "none"], ["poles", "none", "none"]
    )
    assert abs(metrics["accuracy"] - 2 / 3) < 1e-6
    assert metrics["confusion_matrix"]["poles"]["none"] == 1
    assert 0.0 < metrics["macro_f1"] < 1.0


def test_optimizers_recover_separable_vision_and_terrain_samples():
    vision = [
        (TimedRecord(1, (1.0, 0.80)), "poles"),
        (TimedRecord(2, (1.0, 0.70)), "poles"),
        (TimedRecord(3, (1.0, 0.35)), "none"),
    ]
    confidence, vision_metrics = optimize_vision_threshold(vision)
    assert 0.35 < confidence <= 0.70
    assert vision_metrics["accuracy"] == 1.0

    terrain = [
        (TimedRecord(1, (0, 0, 0.04, 100, 0, 0, 0.04)), "WALK"),
        (TimedRecord(2, (0, 0, 0.10, 100, 0, 0, 0.10)), "STEP"),
        (TimedRecord(3, (0, 0, 0.20, 100, 0, 0, 0.20)), "CLIMB"),
        (TimedRecord(4, (0, 0, 0.36, 100, 0, 0, 0.36)), "STOP"),
    ]
    thresholds, terrain_metrics = optimize_terrain_thresholds(terrain)
    assert thresholds[0] < thresholds[1] < thresholds[2]
    assert terrain_metrics["accuracy"] == 1.0


def test_evaluate_aligns_labels_and_returns_yaml_parameters():
    labels = [
        GroundTruth(1_000_000_000, "poles", "STEP"),
        GroundTruth(2_000_000_000, "none", "WALK"),
    ]
    vision = [
        TimedRecord(1_010_000_000, (1.0, 0.80)),
        TimedRecord(2_010_000_000, (1.0, 0.30)),
    ]
    terrain = [
        TimedRecord(1_020_000_000, (0, 0, 0.10, 100, 0, 0, 0.10)),
        TimedRecord(2_020_000_000, (0, 0, 0.04, 100, 0, 0, 0.04)),
    ]
    report, suggestions = evaluate(labels, vision, terrain, 0.05)
    assert report["matched_vision_samples"] == 2
    assert report["matched_terrain_samples"] == 2
    assert "vision_min_confidence" in suggestions
    assert suggestions["step_threshold"] < suggestions["climb_threshold"]


def test_label_template_and_csv_validation(tmp_path):
    output = tmp_path / "labels.csv"
    records = [TimedRecord(1_000_000_000, ()), TimedRecord(2_000_000_000, ())]
    assert write_label_template(output, records, [], 0.5) == 2
    labels = load_labels(output)
    assert [item.stamp_ns for item in labels] == [1_000_000_000, 2_000_000_000]
