#!/usr/bin/env python3
"""
Standalone camera extrinsic calibration utility.

This tool reproduces the T-DT radar 5-point manual calibration flow without ROS2.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual camera extrinsic calibration with solvePnP.")
    parser.add_argument(
        "--input",
        default="0",
        help="Input source: image/video file path, or camera index (default: 0).",
    )
    parser.add_argument(
        "--camera-params",
        default="config/camera_params.yaml",
        help="Path to camera intrinsics file (OpenCV YAML with camera_matrix/dist_coeffs).",
    )
    parser.add_argument(
        "--world-points",
        default="config/world_points.yaml",
        help="Path to world points yaml file.",
    )
    parser.add_argument(
        "--output",
        default="config/out_matrix.yaml",
        help="Output file path for world_rvec/world_tvec (OpenCV YAML).",
    )
    parser.add_argument("--display-width", type=int, default=1536, help="Display width.")
    parser.add_argument("--display-height", type=int, default=1125, help="Display height.")
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only validate configs and exit without opening windows.",
    )
    return parser.parse_args()


def load_camera_params(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open camera params file: {path}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("dist_coeffs").mat()
    fs.release()

    if camera_matrix is None or dist_coeffs is None:
        raise RuntimeError("camera_matrix or dist_coeffs is missing in camera params file.")
    return camera_matrix, dist_coeffs


def load_world_points(path: Path) -> Tuple[List[str], np.ndarray]:
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    points = raw.get("points", [])
    if len(points) < 4:
        raise RuntimeError("At least 4 world points are required.")

    labels: List[str] = []
    values: List[List[float]] = []
    for idx, p in enumerate(points):
        for key in ("x", "y", "z"):
            if key not in p:
                raise RuntimeError(f"Point index {idx} missing key '{key}'.")
        labels.append(p.get("name", f"point_{idx}"))
        values.append([float(p["x"]), float(p["y"]), float(p["z"])])
    return labels, np.asarray(values, dtype=np.float32)


def parse_source(source: str):
    if source.isdigit():
        return int(source)
    return source


def solve_and_save(
    world_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    output_path: Path,
) -> Tuple[np.ndarray, np.ndarray]:
    ok, rvec, tvec = cv2.solvePnP(
        world_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok:
        raise RuntimeError("solvePnP failed.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(output_path), cv2.FILE_STORAGE_WRITE)
    fs.write("world_rvec", rvec)
    fs.write("world_tvec", tvec)
    fs.release()
    return rvec, tvec


def draw_text(frame: np.ndarray, lines: List[str]) -> None:
    for idx, line in enumerate(lines):
        y = 50 + idx * 45
        cv2.putText(
            frame,
            line,
            (30, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )


def extract_roi(display_frame: np.ndarray, x: int, y: int, radius: int = 50) -> np.ndarray | None:
    h, w = display_frame.shape[:2]
    if x < radius or y < radius or x >= w - radius or y >= h - radius:
        return None
    roi = display_frame[y - radius : y + radius, x - radius : x + radius]
    zoom = cv2.resize(roi, (400, 400))
    cv2.line(zoom, (200, 100), (200, 300), (0, 0, 255), 1)
    cv2.line(zoom, (100, 200), (300, 200), (0, 0, 255), 1)
    return zoom


def main() -> int:
    args = parse_args()
    camera_path = Path(args.camera_params)
    world_points_path = Path(args.world_points)
    output_path = Path(args.output)

    camera_matrix, dist_coeffs = load_camera_params(camera_path)
    labels, world_points = load_world_points(world_points_path)

    print("Loaded camera params from:", camera_path)
    print("Loaded", len(world_points), "world points from:", world_points_path)
    for i, (name, point) in enumerate(zip(labels, world_points.tolist())):
        print(f"  {i+1}. {name}: {point}")

    if args.check_only:
        print("Config check passed.")
        return 0

    source = parse_source(args.input)
    capture = None
    static_frame = None
    if isinstance(source, int):
        capture = cv2.VideoCapture(source)
    else:
        input_path = Path(source)
        if input_path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".webp"}:
            static_frame = cv2.imread(str(input_path))
            if static_frame is None:
                raise RuntimeError(f"Failed to read image: {input_path}")
        else:
            capture = cv2.VideoCapture(str(input_path))

    if capture is not None and not capture.isOpened():
        raise RuntimeError(f"Cannot open input source: {source}")

    cv2.namedWindow("calibrate", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("ROI", cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow("ROI", 400, 400)

    is_calibrating = False
    image_points: List[List[float]] = []
    cursor_x, cursor_y = 0, 0
    adjust_mode = False
    current_display = None
    current_original = None

    def on_mouse(event, x, y, _flags, _userdata):
        nonlocal cursor_x, cursor_y, adjust_mode
        if current_display is None:
            return
        cursor_x, cursor_y = x, y
        if event == cv2.EVENT_MOUSEMOVE:
            roi = extract_roi(current_display, x, y)
            if roi is not None:
                cv2.imshow("ROI", roi)
        elif event == cv2.EVENT_LBUTTONDOWN and is_calibrating:
            adjust_mode = True
            cursor_x, cursor_y = x, y

    cv2.setMouseCallback("calibrate", on_mouse)

    while True:
        if static_frame is not None:
            frame = static_frame.copy()
        else:
            assert capture is not None
            ok, frame = capture.read()
            if not ok:
                break

        current_original = frame
        display = cv2.resize(frame, (args.display_width, args.display_height))
        current_display = display

        if is_calibrating:
            remaining = len(world_points) - len(image_points)
            next_name = labels[len(image_points)] if remaining > 0 else "none"
            draw_text(
                display,
                [
                    f"picked: {len(image_points)}/{len(world_points)}",
                    f"next point: {next_name}",
                    "click point, use WASD to fine tune, press n to confirm",
                    "press r to reset, q to quit",
                ],
            )
        else:
            draw_text(display, ["press ENTER to start calibration", "press q to quit"])

        if adjust_mode:
            while True:
                roi = extract_roi(display, cursor_x, cursor_y)
                if roi is not None:
                    cv2.imshow("ROI", roi)
                preview = display.copy()
                cv2.drawMarker(
                    preview,
                    (cursor_x, cursor_y),
                    (0, 255, 255),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )
                cv2.imshow("calibrate", preview)
                key = cv2.waitKey(20) & 0xFF
                if key == ord("w"):
                    cursor_y = max(50, cursor_y - 1)
                elif key == ord("s"):
                    cursor_y = min(args.display_height - 51, cursor_y + 1)
                elif key == ord("a"):
                    cursor_x = max(50, cursor_x - 1)
                elif key == ord("d"):
                    cursor_x = min(args.display_width - 51, cursor_x + 1)
                elif key == ord("n"):
                    h0, w0 = current_original.shape[:2]
                    x0 = cursor_x * (w0 / args.display_width)
                    y0 = cursor_y * (h0 / args.display_height)
                    image_points.append([x0, y0])
                    print(
                        f"saved point {len(image_points)}: display=({cursor_x},{cursor_y}) "
                        f"original=({x0:.2f},{y0:.2f})",
                    )
                    adjust_mode = False
                    break
                elif key in (ord("q"), 27):
                    adjust_mode = False
                    break

        for idx, p in enumerate(image_points):
            px = int(p[0] * args.display_width / current_original.shape[1])
            py = int(p[1] * args.display_height / current_original.shape[0])
            cv2.circle(display, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(
                display,
                str(idx + 1),
                (px + 10, py - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        cv2.imshow("calibrate", display)
        key = cv2.waitKey(10) & 0xFF

        if key in (ord("q"), 27):
            break
        if key == 13:
            is_calibrating = True
        elif key == ord("r"):
            image_points.clear()
            is_calibrating = False
            print("Calibration state reset.")

        if len(image_points) == len(world_points):
            image_np = np.asarray(image_points, dtype=np.float32)
            rvec, tvec = solve_and_save(world_points, image_np, camera_matrix, dist_coeffs, output_path)
            print("Calibration complete.")
            print("rvec:\n", rvec)
            print("tvec:\n", tvec)
            print("Saved to:", output_path)
            image_points.clear()
            is_calibrating = False

    if capture is not None:
        capture.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pylint: disable=broad-exception-caught
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
