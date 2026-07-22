#!/usr/bin/env python3
import argparse
import re
import shutil
from pathlib import Path
from bisect import bisect_left
import sys

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

def build_sorted(items):
    """Return sorted list of timestamps and parallel list of Paths."""
    items_sorted = sorted(items, key=lambda x: x[0])
    ts = [t for t, _ in items_sorted]
    paths = [p for _, p in items_sorted]
    return ts, paths

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
    
def parse_numbered_flag(argv: list[str], prefix: str) -> dict[int, str]:
    """Pull out --{prefix}_N value pairs from argv."""
    result = {}
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg.startswith(f"--{prefix}_"):
            idx_str = arg[len(f"--{prefix}_"):]
            if idx_str.isdigit() and i + 1 < len(argv):
                result[int(idx_str)] = argv[i + 1]
        i += 1
    return result


def main():
    ap = argparse.ArgumentParser(description="Match images across N folders by nanosecond timestamps.")

    ap.add_argument("--sample_step", type=int, default=10, help="")
    ap.add_argument("--threshold-ns", type=int, required=True,
                    help="Max allowed absolute timestamp difference (in nanoseconds)")
    
    args, _unknown = ap.parse_known_args()
 
    images_folder = parse_numbered_flag(sys.argv[1:], "images_folder")
    colmap_folder = parse_numbered_flag(sys.argv[1:], "colmap_folder")

    if not images_folder:
        print("No --images_folder_N arguments given.")
        return
 
    num_cameras = max(images_folder) + 1
    for i in range(num_cameras):
        if i not in images_folder:
            print(f"Missing --images_folder_{i} - camera indices must be contiguous starting at 0.")
            return
        if i not in colmap_folder:
            print(f"Missing --colmap_folder_{i} - every images_folder_N needs a matching colmap_folder_N.")
            return

    in_paths = {i: Path(images_folder[i]) for i in range(num_cameras)}
    out_paths = {i: Path(colmap_folder[i]) for i in range(num_cameras)}
 
    items = {i: list_images_with_ts(in_paths[i]) for i in range(num_cameras)}
    for i, its in items.items():
        if not its:
            print(f"No timestamped images found in folder cam{i}.")
            return
 
    for folder in out_paths.values():
        if folder.exists() and folder.is_dir():
            shutil.rmtree(folder)
        folder.mkdir(parents=True, exist_ok=True)
 
    sorted_ts, sorted_paths = {}, {}
    for i, its in items.items():
        ts, paths = build_sorted(its)
        sorted_ts[i] = ts
        sorted_paths[i] = paths
 
    # cam0 is always the reference/anchor
    ref_ts, ref_paths = sorted_ts[0], sorted_paths[0]
 
    matches = 0
    skipped = 0
    counter = 0
    subsample = args.sample_step
 
    for ts_a, path_a in zip(ref_ts, ref_paths):
        best = {0: path_a}
        ok = True
        for i in range(1, num_cameras):
            idx = closest_index(sorted_ts[i], ts_a)
            ts_b = sorted_ts[i][idx]
            if abs(ts_a - ts_b) > args.threshold_ns:
                ok = False
                break
            best[i] = sorted_paths[i][idx]
 
        if ok and counter % subsample == 0:
            for i, src_path in best.items():
                # all cameras' output files share cam0's filename
                dest = out_paths[i] / path_a.name
                shutil.copy2(src_path, dest)
                # dest.symlink_to(src_path.resolve())
            matches += 1
        else:
            skipped += 1
 
        counter += 1
 
    print(f"Done. Matches copied: {matches}. cam0 images skipped (no close match / subsampled): {skipped}.")
 
 
if __name__ == "__main__":
    main()
