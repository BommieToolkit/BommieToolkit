#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import yaml 


def rotmat_to_quat(R):
    """
    Convert 3x3 rotation matrix to quaternion [w, x, y, z].
    R is a list-of-lists [[r00, r01, r02], ...]
    """
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]

    trace = r00 + r11 + r22

    if trace > 0.0:
        S = math.sqrt(trace + 1.0) * 2.0  # S=4*qw
        qw = 0.25 * S
        qx = (r21 - r12) / S
        qy = (r02 - r20) / S
        qz = (r10 - r01) / S
    elif (r00 > r11) and (r00 > r22):
        S = math.sqrt(1.0 + r00 - r11 - r22) * 2.0  # S=4*qx
        qw = (r21 - r12) / S
        qx = 0.25 * S
        qy = (r01 + r10) / S
        qz = (r02 + r20) / S
    elif r11 > r22:
        S = math.sqrt(1.0 + r11 - r00 - r22) * 2.0  # S=4*qy
        qw = (r02 - r20) / S
        qx = (r01 + r10) / S
        qy = 0.25 * S
        qz = (r12 + r21) / S
    else:
        S = math.sqrt(1.0 + r22 - r00 - r11) * 2.0  # S=4*qz
        qw = (r10 - r01) / S
        qx = (r02 + r20) / S
        qy = (r12 + r21) / S
        qz = 0.25 * S

    return [qw, qx, qy, qz]


def topic_to_prefix(topic: str) -> str:
    """
    Turn a ROS topic like '/cam0/image_raw' into an image_prefix like 'cam0/'.
    Adjust this if you want 'rig1/camera1/' etc.
    """
    # e.g. '/cam0/image_raw' -> 'cam0/'
    parts = topic.strip("/").split("/")
    if not parts:
        return ""
    return parts[0] + "/"


def yaml_to_rig_config(yaml_path: Path, json_path: Path):
    with yaml_path.open("r") as f:
        calib = yaml.safe_load(f)

    # Sort camera keys like cam0, cam1, ...
    cam_keys = sorted(
        (k for k in calib.keys() if k.startswith("cam")),
        key=lambda k: int(k[3:])  # assumes 'cam<number>'
    )

    cameras_out = []

    for idx, cam_key in enumerate(cam_keys):
        cam_data = calib[cam_key]

        intrinsics = cam_data["intrinsics"]  # [fx, fy, cx, cy]
        dist = cam_data["distortion_coeffs"]  # [k1, k2, p1, p2] for radtan

        # Map to OPENCV-style [fx, fy, cx, cy, k1, k2, p1, p2]
        camera_params = list(intrinsics) + list(dist)

        # Basic camera dict
        cam_out = {
            "image_prefix": f"rig1/camera{idx+1}/",
            "camera_model_name": "OPENCV",
            "camera_params": camera_params,
        }

        # First camera is reference sensor
        if idx == 0:
            cam_out["ref_sensor"] = True
        else:
            # If extrinsics are present (Kalibr style T_cn_cnm1), convert
            T = cam_data.get("T_cn_cnm1", None)
            if T is not None:
                # T is 4x4; take 3x3 rotation and translation
                R = [row[:3] for row in T[:3]]
                t = [T[0][3], T[1][3], T[2][3]]

                cam_out["cam_from_rig_rotation"] = rotmat_to_quat(R)
                cam_out["cam_from_rig_translation"] = t

        cameras_out.append(cam_out)

    rig_config = [{"cameras": cameras_out}]

    with json_path.open("w") as f:
        json.dump(rig_config, f, indent=2)


def main():
    ap = argparse.ArgumentParser(
        description="Convert Kalibr camchain.yaml to rig_config.json format."
    )
    ap.add_argument(
        "yaml_in",
        type=Path,
        help="Input calibration-camchain.yaml from Kalibr",
    )
    ap.add_argument(
        "json_out",
        type=Path,
        help="Output rig_config.json path",
    )
    args = ap.parse_args()

    yaml_to_rig_config(args.yaml_in, args.json_out)


if __name__ == "__main__":
    main()
