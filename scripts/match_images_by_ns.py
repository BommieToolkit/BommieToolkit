#!/usr/bin/env python3
import argparse
import re
import shutil
from pathlib import Path
from bisect import bisect_left

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp", ".gif", ".webp"}

def extract_ns(p: Path):
    """Extract the longest run of digits from the filename as an int nanosecond timestamp."""
    m = re.findall(r'(\d+)', p.name)
    if not m:
        return None
    # pick the longest run of digits; if tie, pick the first
    ts = max(m, key=len)
    try:
        return int(ts)
    except ValueError:
        return None

def list_images_with_ts(folder: Path):
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    items = []
    for p in files:
        ts = extract_ns(p)
        if ts is not None:
            items.append((ts, p))
    return items

def build_sorted_b(b_items):
    """Return sorted list of timestamps and parallel list of Paths for B."""
    b_items_sorted = sorted(b_items, key=lambda x: x[0])
    b_ts = [t for t, _ in b_items_sorted]
    b_paths = [p for _, p in b_items_sorted]
    return b_ts, b_paths

def closest_index(sorted_list, x):
    """Index of closest value to x in sorted_list (ties resolve to the left)."""
    pos = bisect_left(sorted_list, x)
    if pos == 0:
        return 0
    if pos == len(sorted_list):
        return len(sorted_list) - 1
    before = pos - 1
    after = pos
    if abs(sorted_list[after] - x) < abs(x - sorted_list[before]):
        return after
    else:
        return before

def main():
    ap = argparse.ArgumentParser(description="Match images across two folders by nanosecond timestamps.")

    ap.add_argument("--images_folder_left", type=Path, required=True, help="Path to images folder left")
    ap.add_argument("--images_folder_right", type=Path, required=True, help="Path to images folder right")
    ap.add_argument("--colmap_folder_left", type=Path, help="Output folder for copies from left")
    ap.add_argument("--colmap_folder_right", type=Path, help="Output folder for copies from right")
    ap.add_argument("--sample_step", type=int, default=10, help="")
    ap.add_argument("--threshold-ns", type=int, required=True,
                    help="Max allowed absolute timestamp difference (in nanoseconds)")
    args = ap.parse_args()

    a_items = list_images_with_ts(args.images_folder_left)
    b_items = list_images_with_ts(args.images_folder_right)

    if not a_items:
        print("No timestamped images found in folder A.")
        return
    if not b_items:
        print("No timestamped images found in folder B.")
        return

    if args.colmap_folder_left.exists() and args.colmap_folder_left.is_dir():
        shutil.rmtree(args.colmap_folder_left)
    args.colmap_folder_left.mkdir(parents=True, exist_ok=True)

    if args.colmap_folder_right.exists() and args.colmap_folder_right.is_dir():
        shutil.rmtree(args.colmap_folder_right)
    args.colmap_folder_right.mkdir(parents=True, exist_ok=True)

    b_ts, b_paths = build_sorted_b(b_items)

    matches = 0
    skipped = 0

    subsample = args.sample_step
    for ts_a, path_a in sorted(a_items, key=lambda x: x[0]):
        idx = closest_index(b_ts, ts_a)
        ts_b = b_ts[idx]
        path_b = b_paths[idx]
        diff = abs(ts_a - ts_b)

        if diff <= args.threshold_ns and matches % subsample == 0:
            # Copy A -> colmap_folder_left with original filename
            dest_a = args.colmap_folder_left / path_a.name
            shutil.copy2(path_a, dest_a)
            #dest_a.symlink_to(path_a.resolve())

            # Copy B -> colmap_folder_right but use A's filename
            dest_b = args.colmap_folder_right / path_a.name
            shutil.copy2(path_b, dest_b)
            #dest_b.symlink_to(path_b.resolve())
            
            matches += 1
        else:
            skipped += 1

    print(f"Done. Matches copied: {matches}. A images skipped (no close match): {skipped}.")

if __name__ == "__main__":
    main()
