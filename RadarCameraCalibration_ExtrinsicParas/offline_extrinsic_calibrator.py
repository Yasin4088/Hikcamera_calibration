#!/usr/bin/env python3
"""Offline extrinsic calibration using local images.

This script reuses:
  - config/camera_params.yaml
  - config/world_points.yaml

Workflow:
  1) Iterate images under a directory.
  2) Manually click image points in world_points order.
  3) Solve camera extrinsic by solvePnP.
  4) Save per-image extrinsics and write best result to out_matrix.yaml.
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


SUPPORTED_IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


@dataclass
class SolveResult:
    image_path: Path
    image_points: List[Tuple[float, float]]
    rvec: np.ndarray
    tvec: np.ndarray
    mean_error: float
    rms_error: float
    max_error: float
    inlier_count: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline image-based extrinsic calibration.")
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=Path("images"),
        help="Directory containing calibration images.",
    )
    parser.add_argument(
        "--camera-params",
        type=Path,
        default=Path("config/camera_params.yaml"),
        help="Path to camera intrinsics yaml.",
    )
    parser.add_argument(
        "--world-points",
        type=Path,
        default=Path("config/world_points.yaml"),
        help="Path to world points yaml.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/offline_extrinsics.yaml"),
        help="Path to save per-image extrinsic results.",
    )
    parser.add_argument(
        "--write-best-to",
        type=Path,
        default=Path("config/out_matrix.yaml"),
        help="Write best (lowest RMS) extrinsic to OpenCV yaml.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=1600,
        help="Max preview width.",
    )
    parser.add_argument(
        "--max-height",
        type=int,
        default=1000,
        help="Max preview height.",
    )
    parser.add_argument(
        "--single-image",
        type=Path,
        default=None,
        help="Only calibrate one image file path.",
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
        raise RuntimeError(f"camera_matrix or dist_coeffs missing in: {path}")
    return camera_matrix.astype(np.float64), dist_coeffs.astype(np.float64)


def _strip_comment(line: str) -> str:
    if "#" in line:
        return line.split("#", 1)[0]
    return line


def _parse_value(raw: str) -> str:
    value = raw.strip()
    if value.startswith(("'", '"')) and value.endswith(("'", '"')) and len(value) >= 2:
        value = value[1:-1]
    return value


def load_world_points(path: Path) -> Tuple[List[str], np.ndarray]:
    lines = path.read_text(encoding="utf-8").splitlines()
    labels: List[str] = []
    points: List[Tuple[float, float, float]] = []

    current = {}

    def flush_current() -> None:
        nonlocal current
        if not current:
            return
        if "x" not in current or "y" not in current or "z" not in current:
            raise RuntimeError(f"world point missing x/y/z in file: {path}")
        name = str(current.get("name", f"point_{len(points)}"))
        labels.append(name)
        points.append((float(current["x"]), float(current["y"]), float(current["z"])))
        current = {}

    for raw in lines:
        line = _strip_comment(raw).strip()
        if not line or line == "points:":
            continue

        if line.startswith("- "):
            flush_current()
            line = line[2:].strip()
            if not line:
                continue
            if ":" in line:
                key, val = line.split(":", 1)
                current[key.strip()] = _parse_value(val)
            continue

        if ":" in line:
            key, val = line.split(":", 1)
            current[key.strip()] = _parse_value(val)

    flush_current()

    if len(points) < 4:
        raise RuntimeError("At least 4 world points are required.")

    return labels, np.asarray(points, dtype=np.float32)


def list_images(image_dir: Path) -> List[Path]:
    files = [p for p in image_dir.iterdir() if p.is_file() and p.suffix.lower() in SUPPORTED_IMAGE_EXTS]

    def sort_key(p: Path) -> Tuple[str, int, str]:
        m = re.search(r"(\d+)", p.stem)
        idx = int(m.group(1)) if m else math.inf
        return (p.stem, idx, p.name)

    return sorted(files, key=sort_key)


class PointPicker:
    def __init__(
        self,
        image: np.ndarray,
        labels: Sequence[str],
        max_width: int,
        max_height: int,
        window_name: str,
    ) -> None:
        self.image = image
        self.labels = list(labels)
        self.window_name = window_name

        h, w = image.shape[:2]
        scale_w = max_width / float(w)
        scale_h = max_height / float(h)
        self.scale = min(scale_w, scale_h, 1.0)
        self.display_size = (max(1, int(round(w * self.scale))), max(1, int(round(h * self.scale))))

        self.confirmed_points: List[Tuple[float, float]] = []
        self.pending_point: Optional[Tuple[float, float]] = None
        self.roi_window_name = f"{self.window_name}_roi"
        self.roi_radius = 50
        self.roi_zoom_size = 400

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self.roi_window_name, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(self.roi_window_name, self.roi_zoom_size, self.roi_zoom_size)
        cv2.setMouseCallback(self.window_name, self._on_mouse)

    def _to_display(self, pt: Tuple[float, float]) -> Tuple[int, int]:
        x, y = pt
        return int(round(x * self.scale)), int(round(y * self.scale))

    def _to_original(self, x_disp: int, y_disp: int) -> Tuple[float, float]:
        x = float(x_disp) / self.scale
        y = float(y_disp) / self.scale
        return x, y

    def _on_mouse(self, event: int, x: int, y: int, _flags: int, _userdata: object) -> None:
        self._show_roi_from_display(x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(self.confirmed_points) < len(self.labels):
            self.pending_point = self._to_original(x, y)

    def _show_roi_from_original(self, x_orig: float, y_orig: float) -> None:
        h, w = self.image.shape[:2]
        x = int(round(x_orig))
        y = int(round(y_orig))

        if x < self.roi_radius or y < self.roi_radius:
            return
        if x >= w - self.roi_radius or y >= h - self.roi_radius:
            return

        roi = self.image[
            y - self.roi_radius : y + self.roi_radius,
            x - self.roi_radius : x + self.roi_radius,
        ].copy()
        zoom = cv2.resize(roi, (self.roi_zoom_size, self.roi_zoom_size), interpolation=cv2.INTER_NEAREST)
        center = self.roi_zoom_size // 2
        cv2.line(zoom, (center, center - 100), (center, center + 100), (0, 0, 255), 1)
        cv2.line(zoom, (center - 100, center), (center + 100, center), (0, 0, 255), 1)
        cv2.imshow(self.roi_window_name, zoom)

    def _show_roi_from_display(self, x_disp: int, y_disp: int) -> None:
        x_orig, y_orig = self._to_original(x_disp, y_disp)
        self._show_roi_from_original(x_orig, y_orig)

    def _draw(self) -> np.ndarray:
        canvas = cv2.resize(self.image, self.display_size, interpolation=cv2.INTER_AREA)

        for idx, pt in enumerate(self.confirmed_points):
            px, py = self._to_display(pt)
            cv2.circle(canvas, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(
                canvas,
                f"{idx + 1}:{self.labels[idx]}",
                (px + 10, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        if self.pending_point is not None:
            px, py = self._to_display(self.pending_point)
            cv2.drawMarker(canvas, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)

        idx = len(self.confirmed_points)
        next_name = self.labels[idx] if idx < len(self.labels) else "done"

        infos = [
            f"picked: {len(self.confirmed_points)}/{len(self.labels)}",
            f"next: {next_name}",
            "LMB select, WASD fine tune, n confirm",
            "u undo, r reset, x skip image, q quit",
        ]
        for i, text in enumerate(infos):
            cv2.putText(
                canvas,
                text,
                (20, 35 + i * 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return canvas

    def pick(self) -> Optional[List[Tuple[float, float]]]:
        while True:
            canvas = self._draw()
            cv2.imshow(self.window_name, canvas)
            if self.pending_point is not None:
                self._show_roi_from_original(self.pending_point[0], self.pending_point[1])
            key = cv2.waitKey(20) & 0xFF

            if key == ord("q") or key == 27:
                raise KeyboardInterrupt("User requested exit.")
            if key == ord("x"):
                return None
            if key == ord("u"):
                if self.confirmed_points:
                    self.confirmed_points.pop()
                self.pending_point = None
            elif key == ord("r"):
                self.confirmed_points.clear()
                self.pending_point = None
            elif key in (ord("w"), ord("a"), ord("s"), ord("d")):
                if self.pending_point is None:
                    continue
                x, y = self.pending_point
                if key == ord("w"):
                    y -= 1.0
                elif key == ord("s"):
                    y += 1.0
                elif key == ord("a"):
                    x -= 1.0
                elif key == ord("d"):
                    x += 1.0
                h, w = self.image.shape[:2]
                x = float(np.clip(x, 0, w - 1))
                y = float(np.clip(y, 0, h - 1))
                self.pending_point = (x, y)
            elif key == ord("n"):
                if self.pending_point is not None and len(self.confirmed_points) < len(self.labels):
                    self.confirmed_points.append(self.pending_point)
                    self.pending_point = None
                    if len(self.confirmed_points) == len(self.labels):
                        return self.confirmed_points


def solve_extrinsic(
    object_points: np.ndarray,
    image_points: Sequence[Tuple[float, float]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, int]:
    image_points_np = np.asarray(image_points, dtype=np.float32)

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points_np,
        camera_matrix,
        dist_coeffs,
        iterationsCount=200,
        reprojectionError=4.0,
        confidence=0.99,
        flags=cv2.SOLVEPNP_EPNP,
    )
    used_ransac = ok and inliers is not None and len(inliers) >= 4

    if used_ransac:
        inlier_idx = inliers.reshape(-1)
        object_points_inliers = object_points[inlier_idx]
        image_points_inliers = image_points_np[inlier_idx]
        inlier_count = int(len(inlier_idx))
    else:
        # Fallback: avoid hard failure on difficult manual picks; refine with all points.
        ok_init, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points_np,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ok_init:
            raise RuntimeError("solvePnPRansac failed and solvePnP(EPNP) fallback also failed.")
        object_points_inliers = object_points
        image_points_inliers = image_points_np
        inlier_count = int(len(image_points_np))

    ok_iter, rvec, tvec = cv2.solvePnP(
        object_points_inliers,
        image_points_inliers,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok_iter:
        raise RuntimeError("solvePnP refinement (ITERATIVE) failed.")

    if not used_ransac:
        print("[WARN] RANSAC failed or inliers < 4, fallback to all-points iterative refinement.")
    return rvec.astype(np.float64), tvec.astype(np.float64), inlier_count


def compute_reprojection_error(
    object_points: np.ndarray,
    image_points: Sequence[Tuple[float, float]],
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[float, float, float]:
    image_points_np = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points_np, axis=1)
    mean_err = float(np.mean(errors))
    rms_err = float(np.sqrt(np.mean(errors ** 2)))
    max_err = float(np.max(errors))
    return mean_err, rms_err, max_err


def save_best_to_opencv_yaml(path: Path, rvec: np.ndarray, tvec: np.ndarray) -> None:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot write output file: {path}")
    fs.write("world_rvec", rvec)
    fs.write("world_tvec", tvec)
    fs.release()


def save_results_yaml(path: Path, results: Sequence[SolveResult], best_idx: int) -> None:
    lines: List[str] = []
    lines.append("offline_extrinsics:")
    for i, res in enumerate(results):
        lines.append("  - image: " + str(res.image_path))
        lines.append("    is_best: " + ("true" if i == best_idx else "false"))
        lines.append("    mean_error: {:.6f}".format(res.mean_error))
        lines.append("    rms_error: {:.6f}".format(res.rms_error))
        lines.append("    max_error: {:.6f}".format(res.max_error))
        lines.append(f"    inlier_count: {res.inlier_count}")
        lines.append(
            "    rvec: [{:.12f}, {:.12f}, {:.12f}]".format(
                float(res.rvec[0, 0]), float(res.rvec[1, 0]), float(res.rvec[2, 0])
            )
        )
        lines.append(
            "    tvec: [{:.12f}, {:.12f}, {:.12f}]".format(
                float(res.tvec[0, 0]), float(res.tvec[1, 0]), float(res.tvec[2, 0])
            )
        )
        lines.append("    image_points:")
        for px, py in res.image_points:
            lines.append("      - [{:.3f}, {:.3f}]".format(px, py))

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()

    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    labels, object_points = load_world_points(args.world_points)

    if args.single_image is not None:
        image_paths = [args.single_image]
    else:
        image_paths = list_images(args.image_dir)
    if not image_paths:
        raise RuntimeError(f"No images found. dir={args.image_dir}")

    print(f"Loaded camera params: {args.camera_params}")
    print(f"Loaded {len(labels)} world points from: {args.world_points}")
    for i, (name, point) in enumerate(zip(labels, object_points), start=1):
        print(f"  {i}. {name}: [{point[0]}, {point[1]}, {point[2]}]")
    print(f"Found {len(image_paths)} image(s).")

    results: List[SolveResult] = []
    window_name = "offline_extrinsic_calibration"

    try:
        for image_path in image_paths:
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                print(f"[WARN] Cannot read image, skip: {image_path}")
                continue

            print(f"\n=== Calibrating {image_path} ===")
            picker = PointPicker(
                image=image,
                labels=labels,
                max_width=args.max_width,
                max_height=args.max_height,
                window_name=window_name,
            )
            selected = picker.pick()
            if selected is None:
                print(f"[INFO] Skipped image: {image_path}")
                continue

            rvec, tvec, inlier_count = solve_extrinsic(object_points, selected, camera_matrix, dist_coeffs)
            mean_err, rms_err, max_err = compute_reprojection_error(
                object_points, selected, rvec, tvec, camera_matrix, dist_coeffs
            )

            print("rvec =", rvec.reshape(-1).tolist())
            print("tvec =", tvec.reshape(-1).tolist())
            print(f"inliers: {inlier_count}/{len(selected)}")
            print(f"reprojection error: mean={mean_err:.4f}px rms={rms_err:.4f}px max={max_err:.4f}px")

            results.append(
                SolveResult(
                    image_path=image_path,
                    image_points=selected,
                    rvec=rvec,
                    tvec=tvec,
                    mean_error=mean_err,
                    rms_error=rms_err,
                    max_error=max_err,
                    inlier_count=inlier_count,
                )
            )
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted calibration.")
    finally:
        cv2.destroyAllWindows()

    if not results:
        raise RuntimeError("No valid calibration result produced.")

    best_idx = int(np.argmin([r.rms_error for r in results]))
    best = results[best_idx]

    save_results_yaml(args.output, results, best_idx)
    save_best_to_opencv_yaml(args.write_best_to, best.rvec, best.tvec)

    print("\n=== Summary ===")
    for i, r in enumerate(results):
        tag = " (BEST)" if i == best_idx else ""
        print(f"{r.image_path}: rms={r.rms_error:.4f}px, mean={r.mean_error:.4f}px, max={r.max_error:.4f}px{tag}")
    print(f"Saved per-image results to: {args.output}")
    print(f"Saved best result to: {args.write_best_to}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
