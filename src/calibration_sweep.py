"""Benchmark độ nhạy của phép chiếu LiDAR-camera với calibration yaw drift.

Script dùng cùng một tập điểm cho mọi mức yaw, xuất CSV, biểu đồ tổng hợp,
ảnh so sánh và một failure case. Không dùng random nên kết quả tái lập được.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import dataset_type, list_frames, load_frame
from starter.projection import draw_box2d, overlay_points, perturb_extrinsic, project_velo_to_image


def points_in_boxes(uv: np.ndarray, boxes: list[np.ndarray]) -> np.ndarray:
    """Mask điểm ảnh nằm trong ít nhất một box 2D."""
    hit = np.zeros(len(uv), dtype=bool)
    for x1, y1, x2, y2 in boxes:
        hit |= (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
    return hit


def edge_alignment(image: np.ndarray, uv: np.ndarray, tolerance_px: float = 3.0) -> float:
    """Phần trăm điểm chiếu cách image edge không quá ``tolerance_px`` pixel."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 80, 160)
    distance = cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 3)
    xy = np.rint(uv).astype(int)
    xy[:, 0] = np.clip(xy[:, 0], 0, image.shape[1] - 1)
    xy[:, 1] = np.clip(xy[:, 1], 0, image.shape[0] - 1)
    return float(100.0 * np.mean(distance[xy[:, 1], xy[:, 0]] <= tolerance_px)) if len(xy) else 0.0


def evaluate_frame(frame: dict, yaw_deg: float = 0.0, ty_m: float = 0.0) -> dict[str, float]:
    points = frame["points"]
    base_uv, _, base_mask = project_velo_to_image(points, frame["calib"], frame["image"].shape)
    test_calib = perturb_extrinsic(frame["calib"], yaw_deg=yaw_deg, t_xyz_m=(0, ty_m, 0))
    uv, _, mask = project_velo_to_image(points, test_calib, frame["image"].shape)
    finite_count = int(np.isfinite(points[:, :3]).all(axis=1).sum())

    boxes = [obj.bbox for obj in frame["labels"]]
    box_hit = points_in_boxes(uv, boxes)
    common = base_mask & mask
    if np.any(common):
        base_by_id = np.full((len(points), 2), np.nan)
        test_by_id = np.full((len(points), 2), np.nan)
        base_by_id[base_mask] = base_uv
        test_by_id[mask] = uv
        displacement = np.linalg.norm(test_by_id[common] - base_by_id[common], axis=1)
        median_shift = float(np.median(displacement))
        p95_shift = float(np.percentile(displacement, 95))
    else:
        median_shift = p95_shift = float("nan")

    return {
        "point_count": float(len(points)),
        "finite_count": float(finite_count),
        "inside_count": float(mask.sum()),
        "inside_fov_pct": 100.0 * float(mask.sum()) / max(finite_count, 1),
        "box_hit_pct": 100.0 * float(box_hit.mean()) if len(box_hit) else 0.0,
        "edge_alignment_pct": edge_alignment(frame["image"], uv),
        "median_pixel_shift": median_shift,
        "p95_pixel_shift": p95_shift,
    }


def save_comparison(frame: dict, yaws: list[float], out_path: Path) -> None:
    panels = []
    for yaw in yaws:
        calib = perturb_extrinsic(frame["calib"], yaw_deg=yaw)
        uv, depth, _ = project_velo_to_image(frame["points"], calib, frame["image"].shape)
        panel = overlay_points(frame["image"], uv, depth, radius=2)
        for obj in frame["labels"]:
            panel = draw_box2d(panel, obj.bbox, label=obj.type)
        cv2.putText(panel, f"yaw={yaw:g} deg", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
        cv2.putText(panel, f"yaw={yaw:g} deg", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 1)
        panels.append(panel)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), np.concatenate(panels, axis=1))


def plot_summary(rows: list[dict[str, object]], x_key: str, x_label: str, title: str, out_path: Path) -> None:
    datasets = sorted({str(row["dataset"]) for row in rows})
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    metrics = [
        ("inside_fov_pct", "Inside FOV (%)"),
        ("box_hit_pct", "Inside 2D boxes (%)"),
        ("median_pixel_shift", "Median shift (px)"),
    ]
    for dataset in datasets:
        selected = [row for row in rows if row["dataset"] == dataset]
        xs = [float(row[x_key]) for row in selected]
        for ax, (key, label) in zip(axes, metrics):
            ax.plot(xs, [float(row[key]) for row in selected], marker="o", label=dataset)
            ax.set_xlabel(x_label)
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
    axes[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-roots", nargs="+", default=["data/kitti_mini", "data/nuscenes_mini_subset"])
    parser.add_argument("--yaw-deg", nargs="+", type=float, default=[-3, -2, -1, -0.5, 0, 0.5, 1, 2, 3])
    parser.add_argument("--translation-cm", nargs="+", type=float, default=[-10, -5, -2, 0, 2, 5, 10])
    parser.add_argument("--max-frames", type=int, default=20, help="số frame đầu tiên mỗi dataset; 0 nghĩa là tất cả")
    parser.add_argument("--out-dir", default="results")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    detail_rows: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    translation_detail_rows: list[dict[str, object]] = []
    translation_summary_rows: list[dict[str, object]] = []
    loaded: dict[str, list[dict]] = {}
    for root in args.data_roots:
        name = Path(root).name
        ids = list_frames(root)
        if args.max_frames > 0:
            ids = ids[: args.max_frames]
        kwargs = {"use_ego_motion": True} if dataset_type(root) == "nuscenes" else {}
        frames = [load_frame(root, frame_id, **kwargs) for frame_id in ids]
        loaded[name] = frames
        for yaw in args.yaw_deg:
            frame_metrics = []
            for frame in frames:
                metrics = evaluate_frame(frame, yaw)
                detail_rows.append({"dataset": name, "frame_id": frame["frame_id"], "yaw_deg": yaw, **metrics})
                frame_metrics.append(metrics)
            summary = {key: float(np.mean([m[key] for m in frame_metrics])) for key in frame_metrics[0]}
            summary_rows.append({"dataset": name, "frames": len(frames), "yaw_deg": yaw, **summary})
        for translation_cm in args.translation_cm:
            frame_metrics = []
            for frame in frames:
                metrics = evaluate_frame(frame, ty_m=translation_cm / 100.0)
                translation_detail_rows.append(
                    {"dataset": name, "frame_id": frame["frame_id"], "translation_y_cm": translation_cm, **metrics}
                )
                frame_metrics.append(metrics)
            summary = {key: float(np.mean([m[key] for m in frame_metrics])) for key in frame_metrics[0]}
            translation_summary_rows.append(
                {"dataset": name, "frames": len(frames), "translation_y_cm": translation_cm, **summary}
            )

    out_dir.mkdir(parents=True, exist_ok=True)
    outputs = (
        ("yaw_perturb_detail.csv", detail_rows),
        ("yaw_perturb_sweep.csv", summary_rows),
        ("translation_perturb_detail.csv", translation_detail_rows),
        ("translation_perturb_sweep.csv", translation_summary_rows),
    )
    for filename, rows in outputs:
        with (out_dir / filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    plot_summary(
        summary_rows,
        "yaw_deg",
        "Yaw drift (degree)",
        "LiDAR-camera projection sensitivity to yaw drift",
        out_dir / "figures" / "yaw_sweep_metrics.png",
    )
    plot_summary(
        translation_summary_rows,
        "translation_y_cm",
        "Lateral translation drift (cm)",
        "LiDAR-camera projection sensitivity to translation drift",
        out_dir / "figures" / "translation_sweep_metrics.png",
    )
    kitti_frames = loaded.get("kitti_mini", next(iter(loaded.values())))
    # Chọn ba frame theo khoảng cách trung vị của object để evidence có near/mid/far thật.
    ranked = sorted(
        kitti_frames,
        key=lambda frame: float(np.median([obj.location[2] for obj in frame["labels"]]))
        if frame["labels"] else float("inf"),
    )
    demo_frames = [ranked[0], ranked[len(ranked) // 2], ranked[-1]]
    for distance_name, frame in zip(("near", "mid", "far"), demo_frames):
        save_comparison(frame, [0], out_dir / "figures" / f"demo_{distance_name}_{frame['frame_id']}_baseline.png")

    # Failure của metric box-hit: chọn frame mà score tăng nhiều nhất dù yaw đã lệch 3 độ.
    false_improvements = []
    for frame in kitti_frames:
        delta = evaluate_frame(frame, 3)["box_hit_pct"] - evaluate_frame(frame, 0)["box_hit_pct"]
        false_improvements.append((delta, frame))
    failure_frame = max(false_improvements, key=lambda item: item[0])[1]
    save_comparison(failure_frame, [0, 1, 3], out_dir / "figures" / "fail_01_yaw_drift_metric.png")

    print(
        f"Wrote {len(detail_rows) + len(translation_detail_rows)} detail measurements and "
        f"{len(summary_rows) + len(translation_summary_rows)} summary rows to {out_dir}"
    )
    for row in summary_rows:
        print(
            f"{row['dataset']:24s} yaw={float(row['yaw_deg']):>3.1f} deg "
            f"FOV={float(row['inside_fov_pct']):5.2f}% box={float(row['box_hit_pct']):5.2f}% "
            f"shift={float(row['median_pixel_shift']):6.2f}px edge={float(row['edge_alignment_pct']):5.2f}%"
        )
    for row in translation_summary_rows:
        print(
            f"{row['dataset']:24s} ty={float(row['translation_y_cm']):>5.1f} cm "
            f"FOV={float(row['inside_fov_pct']):5.2f}% box={float(row['box_hit_pct']):5.2f}% "
            f"shift={float(row['median_pixel_shift']):6.2f}px edge={float(row['edge_alignment_pct']):5.2f}%"
        )


if __name__ == "__main__":
    main()
