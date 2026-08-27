#!/usr/bin/env python3
"""用棋盘格照片生成 ROS CameraInfo 内参，并给出可量化质量结论。

这是离线工具，不常驻算法进程。建议用最终安装位置、分辨率和对焦状态采集 15～30 张
棋盘格照片；棋盘应覆盖画面中心、四角、远近和倾斜姿态。参数 board-cols/board-rows 指
棋盘格的“内角点数”，不是黑白格数量。
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import yaml


def reprojection_errors(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    rotation_vectors: Sequence[np.ndarray],
    translation_vectors: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> List[float]:
    """计算每张图的像素 RMS，避免只看一个被平均后的全局数字。"""
    errors = []
    for object_set, image_set, rotation, translation in zip(
        object_points, image_points, rotation_vectors, translation_vectors
    ):
        projected, _ = cv2.projectPoints(
            object_set, rotation, translation, camera_matrix, distortion
        )
        difference = image_set.reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(difference * difference, axis=1)))))
    return errors


def robust_keep_indices(errors: Sequence[float], maximum_error: float) -> List[int]:
    """以 median + 2.5*MAD 剔除明显坏帧，同时服从用户设定的绝对像素上限。"""
    values = np.asarray(errors, dtype=np.float64)
    if values.size == 0:
        return []
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_limit = median + max(0.15, 2.5 * 1.4826 * mad)
    limit = min(max(0.05, float(maximum_error)), robust_limit)
    return [index for index, value in enumerate(values) if float(value) <= limit]


def calibration_quality(
    rms: float,
    per_view_errors: Sequence[float],
    view_centers: Sequence[Tuple[float, float]],
    accepted_views: int,
    *,
    maximum_rms: float = 0.8,
) -> Tuple[bool, List[str]]:
    """给标定结果做最低验收，而不是“能生成 YAML 就算成功”。"""
    warnings: List[str] = []
    if accepted_views < 10:
        warnings.append("有效棋盘格少于 10 张；建议采集 15～30 张")
    if not np.isfinite(rms) or float(rms) > float(maximum_rms):
        warnings.append(
            f"全局重投影 RMS={float(rms):.3f}px，高于 {float(maximum_rms):.3f}px"
        )
    if per_view_errors and max(per_view_errors) > max(1.2, float(maximum_rms) * 1.5):
        warnings.append("仍有单张图片重投影误差过大")
    if view_centers:
        xs = [point[0] for point in view_centers]
        ys = [point[1] for point in view_centers]
        if max(xs) - min(xs) < 0.35 or max(ys) - min(ys) < 0.25:
            warnings.append("棋盘中心覆盖范围不足；需要覆盖画面四角和边缘")
    return not warnings, warnings


def detect_observations(
    image_paths: Sequence[str], board_size: Tuple[int, int], square_size: float
) -> Tuple[List[np.ndarray], List[np.ndarray], List[str], Tuple[int, int], List[Tuple[float, float]]]:
    """读取图片并提取亚像素角点；所有图片必须保持同一分辨率。"""
    template = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    template[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    template *= float(square_size)
    object_points, image_points, accepted, centers = [], [], [], []
    image_size = None
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            raise ValueError(
                f"图片分辨率不一致：期望 {image_size}，{path} 为 {current_size}"
            )
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = False
        corners = None
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray,
                board_size,
                flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
            )
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray,
                board_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
                )
        if not found or corners is None:
            continue
        object_points.append(template.copy())
        image_points.append(np.asarray(corners, dtype=np.float32))
        accepted.append(str(path))
        mean = np.mean(corners.reshape(-1, 2), axis=0)
        centers.append((float(mean[0] / image_size[0]), float(mean[1] / image_size[1])))
    if image_size is None:
        raise ValueError("没有可读取的图片")
    return object_points, image_points, accepted, image_size, centers


def calibrate(
    image_paths: Sequence[str],
    board_size: Tuple[int, int],
    square_size: float,
    maximum_view_error: float,
) -> Dict:
    """完成初次拟合、坏帧剔除和最终拟合，返回可序列化结果。"""
    object_points, image_points, accepted, image_size, centers = detect_observations(
        image_paths, board_size, square_size
    )
    if len(accepted) < 6:
        raise ValueError(
            f"只检测到 {len(accepted)} 张有效棋盘格，至少需要 6 张，推荐 15～30 张"
        )

    def run(indices):
        selected_object = [object_points[index] for index in indices]
        selected_image = [image_points[index] for index in indices]
        rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
            selected_object, selected_image, image_size, None, None
        )
        errors = reprojection_errors(
            selected_object, selected_image, rotations, translations, matrix, distortion
        )
        return float(rms), matrix, distortion, errors

    all_indices = list(range(len(accepted)))
    initial_rms, _matrix, _distortion, initial_errors = run(all_indices)
    relative_keep = robust_keep_indices(initial_errors, maximum_view_error)
    kept = [all_indices[index] for index in relative_keep]
    # 坏帧剔除不能把观测数量削到不再可用；这种情况应保留原始集合并明确由质量门判失败。
    if len(kept) < 6:
        kept = all_indices
    rms, matrix, distortion, errors = run(kept)
    return {
        "input_images": len(image_paths),
        "detected_images": len(accepted),
        "image_size": image_size,
        "rms": rms,
        "camera_matrix": matrix,
        "distortion": distortion.reshape(-1),
        "per_view_errors": errors,
        "accepted_images": [accepted[index] for index in kept],
        "rejected_images": [accepted[index] for index in all_indices if index not in kept],
        "view_centers": [centers[index] for index in kept],
        "initial_rms": initial_rms,
    }


def ros_camera_yaml(result: Dict, camera_name: str) -> Dict:
    """转换为 ROS camera_info_manager 可直接加载的 YAML 结构。"""
    width, height = result["image_size"]
    matrix = np.asarray(result["camera_matrix"], dtype=float)
    distortion = np.asarray(result["distortion"], dtype=float).reshape(-1)
    projection = np.zeros((3, 4), dtype=float)
    projection[:, :3] = matrix
    return {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": str(camera_name),
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.reshape(-1).tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": int(distortion.size), "data": distortion.tolist()
        },
        "rectification_matrix": {
            "rows": 3, "cols": 3, "data": np.eye(3).reshape(-1).tolist()
        },
        "projection_matrix": {
            "rows": 3, "cols": 4, "data": projection.reshape(-1).tolist()
        },
        "calibration_report": {
            "rms_pixels": round(float(result["rms"]), 6),
            "initial_rms_pixels": round(float(result["initial_rms"]), 6),
            "accepted_views": len(result["accepted_images"]),
            "rejected_views": len(result["rejected_images"]),
            "undetected_views": int(result["input_images"] - result["detected_images"]),
            "maximum_view_error_pixels": round(max(result["per_view_errors"]), 6),
        },
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Wakula OpenCV 棋盘格相机内参标定")
    parser.add_argument("--images", required=True, help="图片 glob，例如 calibration/*.png")
    parser.add_argument("--board-cols", type=int, default=9, help="横向内角点数")
    parser.add_argument("--board-rows", type=int, default=6, help="纵向内角点数")
    parser.add_argument("--square-size", type=float, required=True, help="棋盘格边长，单位 m")
    parser.add_argument("--camera-name", default="camera")
    parser.add_argument("--output", default="camera_intrinsics.yaml")
    parser.add_argument("--maximum-view-error", type=float, default=1.2)
    parser.add_argument("--maximum-rms", type=float, default=0.8)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    options = parse_args(argv)
    paths = sorted(glob.glob(options.images, recursive=True))
    if not paths:
        raise SystemExit(f"没有匹配图片：{options.images}")
    if options.board_cols < 3 or options.board_rows < 3 or options.square_size <= 0.0:
        raise SystemExit("内角点行列必须 >=3，square-size 必须 >0")
    try:
        result = calibrate(
            paths,
            (options.board_cols, options.board_rows),
            options.square_size,
            options.maximum_view_error,
        )
    except (ValueError, cv2.error) as exc:
        raise SystemExit(f"标定失败：{exc}") from exc
    passed, warnings = calibration_quality(
        result["rms"],
        result["per_view_errors"],
        result["view_centers"],
        len(result["accepted_images"]),
        maximum_rms=options.maximum_rms,
    )
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            ros_camera_yaml(result, options.camera_name),
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    print(
        f"读取 {len(paths)} 张，检测到 {result['detected_images']} 张，"
        f"采用 {len(result['accepted_images'])} 张，误差剔除 {len(result['rejected_images'])} 张"
    )
    print(f"重投影 RMS: {result['rms']:.4f} px；输出: {output.resolve()}")
    if passed:
        print("标定质量：通过。仍需单独完成相机—雷达/机身外参标定。")
        return 0
    print("标定质量：未通过：")
    for warning in warnings:
        print(f"  - {warning}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
