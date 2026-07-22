#!/usr/bin/env python3
"""
Full pipeline runner for Kalibr stereo calibration.

Usage:
    pixi run -e kalibr calibrate configs/calibration.yaml
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml


def run(cmd: list[str], env: dict | None = None, input_text: str | None = None) -> None:
    print(f"\n$ {' '.join(cmd)}\n", flush=True)
    result = subprocess.run(cmd, env=env, input=input_text, text=True if input_text is not None else None)
    if result.returncode != 0:
        print(f"[ERROR] command failed (exit={result.returncode}): {' '.join(cmd)}")
        sys.exit(result.returncode)


def get_camera_list(cfg: dict) -> list[dict]:
    cameras = cfg.get("cameras") or []
    if not cameras:
        print("[ERROR] Config must have a non-empty `cameras:` list.")
        sys.exit(2)
    return cameras


def extract_images(cfg: dict) -> None:
    extract_cfg = cfg["extract"]
    calib_dir = Path(cfg["calibration_dir"])
    cameras = get_camera_list(cfg)

    for idx, cam in enumerate(cameras):
        video = cam["video"]
        output = calib_dir / f"cam{idx}"

        cmd = [
            "pixi", "run", "-e", "default", "extract_images",
            "--video", video,
            "--output", str(output),
            "--resolution", str(extract_cfg["resolution"]),
        ]
        if extract_cfg.get("gray", True):
            cmd.append("--gray")
        if extract_cfg.get("skip") not in (None, ""):
            cmd += ["--skip", str(extract_cfg["skip"])]
        if extract_cfg.get("max_frames") not in (None, "", 0, "0"):
            cmd += ["--max_frames", str(extract_cfg["max_frames"])]

        run(cmd)


def compute_image_dirs(cfg: dict) -> dict:
    calib_dir = Path(cfg["calibration_dir"])
    cameras = get_camera_list(cfg)
    num_cameras = len(cameras)

    raw_dirs = {i: calib_dir / f"cam{i}" for i in range(num_cameras)}

    if num_cameras == 1:
        return raw_dirs

    synced_dir = calib_dir / "syncd"
    return {i: synced_dir / f"cam{i}" for i in range(num_cameras)}


def sync_images(cfg: dict) -> dict:
    sync_cfg = cfg.get("sync", {})
    calib_dir = Path(cfg["calibration_dir"])
    cameras = get_camera_list(cfg)
    num_cameras = len(cameras)

    raw_dirs = {i: calib_dir / f"cam{i}" for i in range(num_cameras)}

    if num_cameras == 1:
        # Nothing to sync with a single camera.
        return raw_dirs

    # 2+ cameras: match_images_by_ns.py
    synced_dir = calib_dir / "syncd"
    cmd = ["python", "scripts/match_images_by_ns.py"]
    for i in range(num_cameras):
        cmd += [f"--images_folder_{i}", str(raw_dirs[i])]
        cmd += [f"--colmap_folder_{i}", str(synced_dir / f"cam{i}")]
    cmd += [
        "--threshold-ns", str(sync_cfg.get("threshold_ns", 5_000_000)),
        "--sample_step", str(sync_cfg.get("sample_step", 1)),
    ]
    run(cmd)
    return {i: synced_dir / f"cam{i}" for i in range(num_cameras)}


def calibrate(cfg: dict, image_dirs: dict) -> None:
    calib_cfg = cfg["calibrate"]
    num_cameras = len(image_dirs)

    focal_length_guesses = calib_cfg.get("focal_length_guesses") or []
    manual_init = bool(focal_length_guesses) or str(calib_cfg.get("manual_focal_length_init", 0)) == "1"

    if focal_length_guesses and len(focal_length_guesses) != num_cameras:
        print(f"[ERROR] focal_length_guesses has {len(focal_length_guesses)} values "
              f"but there are {num_cameras} cameras - they must match.")
        sys.exit(2)

    env = os.environ.copy()

    if manual_init:
        env["KALIBR_MANUAL_FOCAL_LENGTH_INIT"] = "1"
    else:
        env.pop("KALIBR_MANUAL_FOCAL_LENGTH_INIT", None)

    input_text = None
    if manual_init and focal_length_guesses:
        input_text = "\n".join(str(v) for v in focal_length_guesses) + "\n"

    cmd = ["pixi", "run", "-e", "kalibr", "./scripts/calibrate_multi_rig.sh"]
    for i in range(num_cameras):
        cmd.append(f"images_folder_{i}={image_dirs[i]}")
    cmd += [
        f"output_folder={cfg['calibration_dir']}",
        f"target={calib_cfg['target']}",
        f"freq={calib_cfg['freq']}",
        f"verbose={calib_cfg.get('verbose', 0)}",
        f"create_bag={calib_cfg.get('create_bag', 1)}",
        f"camera_model={calib_cfg.get('camera_model', 'pinhole-radtan')}",
    ]

    run(cmd, env=env, input_text=input_text)


def postprocess(cfg: dict) -> None:
    post_cfg = cfg.get("postprocess", {})
    if not post_cfg.get("enabled", False):
        return

    calib_dir = Path(cfg["calibration_dir"])
    camchain = calib_dir / "calibration-camchain.yaml"
    output = post_cfg.get("output") or str(calib_dir / "calibration.yaml")

    cmd = [
        "python", "scripts/generate_calibration_yaml.py",
        "--camchain", str(camchain),
        "--april", post_cfg["april"],
        "--housing", post_cfg["housing"],
        "--output", str(output),
        "--camera", post_cfg.get("camera", "cam0"),
    ]
    run(cmd)


def generate_rig_config(cfg: dict, num_cameras: int) -> None:
    rig_cfg = cfg.get("rig_config", {})
    if not rig_cfg.get("enabled", False):
        return

    if num_cameras == 1:
        print("(single camera - no rig/relative pose to generate, skipping rig_config.json)")
        return

    calib_dir = Path(cfg["calibration_dir"])
    camchain = calib_dir / "calibration-camchain.yaml"
    output = rig_cfg.get("output") or str(calib_dir / "rig_config.json")

    cmd = ["pixi", "run", "-e", "default", "get_rig_config_json", str(camchain), str(output)]
    run(cmd)


DEFAULT_STEPS = ["extract", "sync", "calibrate", "postprocess", "rig_config"]


def resolve_steps(cfg: dict, cli_steps: str | None) -> set[str]:
    """Decide which steps actually run. CLI --steps overrides yaml `steps:`,
    which in turn overrides the "run everything" default."""
    if cli_steps:
        requested = {s.strip() for s in cli_steps.split(",") if s.strip()}
        unknown = requested - set(DEFAULT_STEPS)
        if unknown:
            print(f"[ERROR] Unknown step(s) in --steps: {unknown}. Valid steps: {DEFAULT_STEPS}")
            sys.exit(2)
        return requested

    steps_cfg = cfg.get("steps")
    if not steps_cfg:
        return set(DEFAULT_STEPS)

    return {name for name in DEFAULT_STEPS if steps_cfg.get(name, True)}


def main() -> None:
    parser = argparse.ArgumentParser(description="Kalibr full pipeline runner")
    parser.add_argument("--config", default="calibration_config.yaml", help="Path to the config yaml")
    parser.add_argument("--steps", default=None,
                         help=f"Comma-separated steps to run (default: all). Choices: {','.join(DEFAULT_STEPS)}. "
     "Overrides the yaml `steps:` section.")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)

    cfg = yaml.safe_load(config_path.read_text())
    Path(cfg["calibration_dir"]).mkdir(parents=True, exist_ok=True)

    active_steps = resolve_steps(cfg, args.steps)
    print(f"Active steps: {sorted(active_steps, key=DEFAULT_STEPS.index)}\n")

    num_cameras = len(get_camera_list(cfg))

    if "extract" in active_steps:
        print("=== 1/5: Extracting images ===")
        extract_images(cfg)
    else:
        print("=== 1/5: Extracting images (skipped) ===")

    if "sync" in active_steps:
        print("=== 2/5: Syncing images ===")
        image_dirs = sync_images(cfg)
    else:
        print("=== 2/5: Syncing images (skipped, using expected existing paths) ===")
        image_dirs = compute_image_dirs(cfg)

    if "calibrate" in active_steps:
        print("=== 3/5: Running calibration ===")
        calibrate(cfg, image_dirs)
    else:
        print("=== 3/5: Running calibration (skipped) ===")

    if "postprocess" in active_steps:
        print("=== 4/5: Generating final calibration.yaml (housing correction) ===")
        postprocess(cfg)
    else:
        print("=== 4/5: Generating final calibration.yaml (skipped) ===")

    if "rig_config" in active_steps:
        print("=== 5/5: Generating rig_config.json ===")
        generate_rig_config(cfg, num_cameras)
    else:
        print("=== 5/5: Generating rig_config.json (skipped) ===")

    print("\nDone. Outputs:")
    print(f"  {cfg['calibration_dir']}/calibration-camchain.yaml")
    print(f"  {cfg['calibration_dir']}/calibration-results-cam.txt")
    print(f"  {cfg['calibration_dir']}/calibration-report-cam.pdf")
    if cfg.get("postprocess", {}).get("enabled", False):
        out = cfg["postprocess"].get("output") or f"{cfg['calibration_dir']}/calibration.yaml"
        print(f"  {out}  (final OPENCV+FLATPORT format)")
    if cfg.get("rig_config", {}).get("enabled", False) and num_cameras > 1:
        out = cfg["rig_config"].get("output") or f"{cfg['calibration_dir']}/rig_config.json"
        print(f"  {out}")


if __name__ == "__main__":
    main()