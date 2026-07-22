#!/usr/bin/env python3
"""
Full pipeline runner for stereo video -> COLMAP sparse reconstruction.

Steps:
  1) Extract images from left/right videos (extract_images)
  2) Sync image pairs by timestamp (match_images_by_ns)
  3) (Optional) Generate masks with SAM3 (create_masks) - requires prior
     `pixi run -e sam hf auth login` (interactive, not automated here)
  4) COLMAP feature_extractor -> rig_configurator -> sequential_matcher -> mapper
  5) colmap2nerf: sparse/0 + images -> transforms.json
  6) nerfstudio setup (clone + post-install, skipped if already done) + ns-train splatfacto
  7) ns-export gaussian-splat -> .ply
  8) transform_splats: invert nerfstudio's coordinate normalization to recover metric scale

All parameters are read from a yaml config file.

Usage:
    pixi run python scripts/run_full_reconstruction.py --config configs/reconstruction.yaml
"""
import argparse
import subprocess
import sys
from pathlib import Path

import yaml


def run(cmd: list[str]) -> None:
    print(f"\n$ {' '.join(cmd)}\n", flush=True)
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"[ERROR] command failed (exit={result.returncode}): {' '.join(cmd)}")
        sys.exit(result.returncode)


def trim_video(video_path: str, start: str, end: str, work_dir: Path) -> str:
    """Cut [start, end) out of video_path via ffmpeg, return path to the trimmed copy."""
    work_dir.mkdir(parents=True, exist_ok=True)
    suffix = Path(video_path).suffix
    trimmed_path = work_dir / f"{Path(video_path).stem}_trimmed{suffix}"

    cmd = ["ffmpeg", "-y"]
    if start:
        cmd += ["-ss", str(start)]
    cmd += ["-i", video_path]
    if end:
        cmd += ["-to", str(end)]
    cmd += ["-c:v", "libx264", "-preset", "fast", "-crf", "18", "-c:a", "copy", str(trimmed_path)]

    run(cmd)
    return str(trimmed_path)


def resolve_video(cfg: dict, side: str) -> str:
    """Return the video path to actually extract from - trimmed copy if trim is enabled."""
    videos_cfg = cfg["videos"]
    video_path = videos_cfg[side]

    trim_cfg = videos_cfg.get("trim", {})
    if not trim_cfg.get("enabled", False):
        return video_path

    side_trim = trim_cfg.get(side, {})
    start, end = side_trim.get("start", ""), side_trim.get("end", "")
    if not start and not end:
        return video_path

    work_dir = Path(cfg["output_dir"]) / "trimmed_videos"
    return trim_video(video_path, start, end, work_dir)


def extract_images(cfg: dict) -> dict:
    extract_cfg = cfg["extract"]
    output_dir = Path(cfg["output_dir"])
    left_out = output_dir / "monkey_images_left"
    right_out = output_dir / "monkey_images_right"

    pairs = [
        (resolve_video(cfg, "left"), left_out),
        (resolve_video(cfg, "right"), right_out),
    ]
    for video, output in pairs:
        cmd = [
            "pixi", "run", "-e", "default", "extract_images",
            "--video", video,
            "--output", str(output),
            "--resolution", str(extract_cfg["resolution"]),
        ]
        if extract_cfg.get("gray", False):
            cmd.append("--gray")
        if extract_cfg.get("skip") not in (None, ""):
            cmd += ["--skip", str(extract_cfg["skip"])]
        if extract_cfg.get("max_frames") not in (None, "", 0, "0"):
            cmd += ["--max_frames", str(extract_cfg["max_frames"])]
        run(cmd)

    return {"left": left_out, "right": right_out}


def sync_images(cfg: dict, extracted: dict) -> dict:
    sync_cfg = cfg.get("sync", {})
    output_dir = Path(cfg["output_dir"])
    colmap_cfg = cfg["colmap_images"]

    colmap_left = output_dir / "colmap_images" / colmap_cfg["rig_name"] / colmap_cfg["camera_left_name"]
    colmap_right = output_dir / "colmap_images" / colmap_cfg["rig_name"] / colmap_cfg["camera_right_name"]

    cmd = [
        "pixi", "run", "-e", "default", "match_images_by_ns",
        "--images_folder_left", str(extracted["left"]),
        "--images_folder_right", str(extracted["right"]),
        "--colmap_folder_left", str(colmap_left),
        "--colmap_folder_right", str(colmap_right),
        "--threshold-ns", str(sync_cfg.get("threshold_ns", 5_000_000)),
    ]
    if sync_cfg.get("sample_step") not in (None, ""):
        cmd += ["--sample_step", str(sync_cfg["sample_step"])]
    run(cmd)

    return {"colmap_images_root": output_dir / "colmap_images"}


def generate_masks(cfg: dict) -> None:
    if not cfg.get("masks", {}).get("enabled", False):
        return
    print("(assuming `pixi run -e sam hf auth login` was already done interactively)")
    run(["pixi", "run", "-e", "sam", "create_masks"])


def run_colmap_pipeline(cfg: dict) -> None:
    output_dir = Path(cfg["output_dir"])
    colmap_cfg = cfg["colmap"]

    colmap_images_dir = output_dir / "colmap_images"
    database_path = colmap_cfg.get("database_path") or str(output_dir / "database.db")
    sparse_output = colmap_cfg.get("sparse_output") or str(output_dir / "sparse")

    # --- feature_extractor ---
    cmd = [
        "pixi", "run", "-e", "colmap", "colmap", "feature_extractor",
        "--image_path", str(colmap_images_dir),
        "--database_path", database_path,
        "--ImageReader.single_camera", str(colmap_cfg.get("single_camera", 1)),
        "--ImageReader.single_camera_per_folder", str(colmap_cfg.get("single_camera_per_folder", 1)),
    ]
    if colmap_cfg.get("use_masks", False):
        cmd += ["--ImageReader.mask_path", str(output_dir / "masks")]
    cmd += ["--ImageReader.single_camera_per_image", str(colmap_cfg.get("single_camera_per_image", 0))]
    run(cmd)

    # --- rig_configurator ---
    run([
        "pixi", "run", "-e", "colmap", "colmap", "rig_configurator",
        "--database_path", database_path,
        "--rig_config_path", colmap_cfg["rig_config_path"],
    ])

    # --- sequential_matcher ---
    run([
        "pixi", "run", "-e", "colmap", "colmap", "sequential_matcher",
        "--database_path", database_path,
    ])

    # --- mapper ---
    Path(sparse_output).mkdir(parents=True, exist_ok=True)
    cmd = [
        "pixi", "run", "-e", "colmap", "colmap", "mapper",
        "--database_path", database_path,
    ]
    if colmap_cfg.get("ba_refine_sensor_from_rig", 0):
        cmd += ["--Mapper.ba_refine_sensor_from_rig", str(colmap_cfg["ba_refine_sensor_from_rig"])]
    cmd += [
        "--Mapper.ba_refine_focal_length", str(colmap_cfg.get("ba_refine_focal_length", 0)),
        "--Mapper.ba_refine_extra_params", str(colmap_cfg.get("ba_refine_extra_params", 0)),
        "--image_path", str(colmap_images_dir),
        "--output_path", sparse_output,
        "--Mapper.ba_use_gpu", str(colmap_cfg.get("ba_use_gpu", 1)),
    ]
    run(cmd)


def run_colmap2nerf(cfg: dict) -> None:
    c2n_cfg = cfg.get("colmap2nerf", {})
    if not c2n_cfg.get("enabled", False):
        return

    output_dir = Path(cfg["output_dir"])
    sparse_input = c2n_cfg.get("sparse_input") or str(output_dir / "sparse" / "0")
    images_input = c2n_cfg.get("images_input") or str(output_dir / "colmap_images")
    output = c2n_cfg.get("output") or str(output_dir / "transforms.json")

    cmd = [
        "pixi", "run", "-e", "default", "colmap2nerf",
        "--text", sparse_input,
        "--images", images_input,
        "--out", output,
    ]
    if c2n_cfg.get("keep_colmap_coords", True):
        cmd.append("--keep_colmap_coords")
    run(cmd)


def ensure_nerfstudio_setup(cfg: dict) -> None:
    ns_cfg = cfg.get("nerfstudio", {})
    marker = Path(ns_cfg.get("setup_marker", "nerfstudio/.post_install_done"))
    if marker.exists():
        print(f"(nerfstudio already set up, found {marker})")
        return

    run(["pixi", "run", "nerfstudio-clone"])
    run(["pixi", "run", "nerfstudio-post-install"])

    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()


def run_nerfstudio_train(cfg: dict) -> None:
    ns_cfg = cfg.get("nerfstudio", {})
    if not ns_cfg.get("enabled", False):
        return

    ensure_nerfstudio_setup(cfg)

    output_dir = Path(cfg["output_dir"]).resolve()
    cmd = [
        "pixi", "run", "-e", "nerfstudio", "ns-train", "splatfacto",
        "--data", str(output_dir),
        "--vis", ns_cfg.get("vis", "viewer"),
    ]
    if ns_cfg.get("masked", False):
        cmd += ["--pipeline.model.background_color", "random"]
    if ns_cfg.get("quit_on_train_completion", True):
        cmd += ["--viewer.quit-on-train-completion", "True"]
    run(cmd)


def find_latest_run(cfg: dict) -> Path | None:
    """Locate the most recent ns-train output dir: outputs/{data_folder_name}/splatfacto/{timestamp}/"""
    data_folder_name = Path(cfg["output_dir"]).name
    candidates = sorted(Path("outputs", data_folder_name, "splatfacto").glob("*"))
    return candidates[-1] if candidates else None


def print_viewer_hint(cfg: dict) -> None:
    latest = find_latest_run(cfg)
    if latest is None:
        return
    config_yml = latest / "config.yml"
    print("\nTo inspect the trained result interactively, run:")
    print(f"  pixi run -e nerfstudio ns-viewer --load-config {config_yml}")


def export_splat(cfg: dict) -> Path | None:
    export_cfg = cfg.get("export_splat", {})
    if not export_cfg.get("enabled", False):
        return None

    latest = find_latest_run(cfg)
    if latest is None:
        print("[ERROR] Could not auto-locate a trained splatfacto run to export from.")
        sys.exit(1)

    config_yml = latest / "config.yml"
    output_dir = Path(export_cfg.get("output_dir") or Path(cfg["output_dir"]) / "exports" / "splat")

    run([
        "pixi", "run", "-e", "nerfstudio", "ns-export", "gaussian-splat",
        "--load-config", str(config_yml),
        "--output-dir", str(output_dir),
    ])
    return output_dir


def find_exported_ply(export_dir: Path) -> Path | None:
    candidates = sorted(export_dir.glob("*.ply"))
    return candidates[0] if candidates else None


def run_transform_splats(cfg: dict, exported_dir: Path | None) -> None:
    ts_cfg = cfg.get("transform_splats", {})
    if not ts_cfg.get("enabled", False):
        return

    splat_path = ts_cfg.get("splat_path")
    if not splat_path:
        if exported_dir is None:
            print("[ERROR] transform_splats.splat_path not set and no export_splat output to fall back on.")
            sys.exit(1)
        found = find_exported_ply(exported_dir)
        if found is None:
            print(f"[ERROR] No .ply found in {exported_dir}. Set transform_splats.splat_path explicitly.")
            sys.exit(1)
        splat_path = str(found)

    output_dir = Path(cfg["output_dir"])
    transform_path = ts_cfg.get("transform_path")
    if not transform_path:
        latest = find_latest_run(cfg)
        if latest is None:
            print("[ERROR] Could not auto-locate dataparser_transforms.json (no ns-train run found).")
            sys.exit(1)
        transform_path = str(latest / "dataparser_transforms.json")

    output_path = ts_cfg.get("output_path") or str(output_dir / "splat_original.ply")

    run([
        "pixi", "run", "-e", "default", "transform_splats",
        "--splat_path", splat_path,
        "--transform_path", transform_path,
        "--output_path", output_path,
    ])


def main() -> None:
    parser = argparse.ArgumentParser(description="Full stereo-video-to-COLMAP-reconstruction pipeline runner")
    parser.add_argument("--config", default="reconstruction_config.yaml", help="Path to the config yaml")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)

    cfg = yaml.safe_load(config_path.read_text())
    Path(cfg["output_dir"]).mkdir(parents=True, exist_ok=True)

    print("=== 1/4: Extracting images ===")
    extracted = extract_images(cfg)

    print("=== 2/4: Syncing images by timestamp ===")
    sync_images(cfg, extracted)

    print("=== 3/4: Generating masks (optional) ===")
    generate_masks(cfg)

    print("=== 4/7: Running COLMAP reconstruction ===")
    run_colmap_pipeline(cfg)

    print("=== 5/7: Converting COLMAP output to NeRF transforms.json ===")
    run_colmap2nerf(cfg)

    print("=== 6/8: Training splatfacto (nerfstudio) ===")
    run_nerfstudio_train(cfg)
    print_viewer_hint(cfg)

    print("=== 7/8: Exporting gaussian splat (.ply) ===")
    exported_dir = export_splat(cfg)

    print("=== 8/8: Recovering metric scale (transform_splats) ===")
    run_transform_splats(cfg, exported_dir)

    output_dir = cfg["output_dir"]
    print("\nDone. Outputs:")
    print(f"  {output_dir}/database.db")
    print(f"  {output_dir}/sparse/")
    print(f"  {output_dir}/transforms.json")
    print(f"  outputs/{Path(output_dir).name}/splatfacto/<timestamp>/")
    if exported_dir is not None:
        print(f"  {exported_dir}/*.ply")
    if cfg.get("transform_splats", {}).get("enabled", False):
        out = cfg["transform_splats"].get("output_path") or f"{output_dir}/splat_original.ply"
        print(f"  {out}")


if __name__ == "__main__":
    main()