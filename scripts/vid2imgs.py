import cv2
import argparse
import os
from tqdm import tqdm
import subprocess
from typing import Optional
from fractions import Fraction
from argparse import BooleanOptionalAction
from pathlib import Path
import shutil

def probe_timecode(path: str) -> Optional[str]:
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "d:0",
        "-show_entries", "stream_tags=timecode",
        "-of", "default=nw=1:nk=1",
        path,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=True)
        val = out.stdout.strip()
        return val or None
    except subprocess.CalledProcessError as e:
        print("ffprobe error:", e.stderr.strip())
        return None
    
def timecode_to_ns(tc: str, fps) -> int:
    """
    Convert 'HH:MM:SS:FF' to nanoseconds.
    - fps can be a float (e.g., 25.0, 29.97) or a Fraction (e.g., Fraction(30000,1001)).
    - Non–drop-frame math (i.e., straight frame counting).
    """
    if isinstance(fps, (int, float)):
        # snap common fractional frame rates to exact rationals
        if abs(fps - 29.97) < 1e-6: fps = Fraction(30000, 1001)
        elif abs(fps - 59.94) < 1e-6: fps = Fraction(60000, 1001)
        else: fps = Fraction(str(fps))  # exact rational from decimal string

    hh, mm, ss, ff = map(int, tc.split(":"))
    total_seconds = Fraction(hh*3600 + mm*60 + ss, 1) + Fraction(ff, 1) / fps
    return int(round(total_seconds * 1_000_000_000))

def sharpness(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = cv2.Laplacian(gray, cv2.CV_64F).var()
    return fm

def main():

    print('Converting video to images...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--video', type=str, help='Path to video file')
    parser.add_argument('--output', type=str, help='Path to output directory')
    parser.add_argument('--fps', type=int, default=30, help='Frame rate of the video')
    parser.add_argument('--skip', type=int, default=0, help='Number of frames to skip')
    parser.add_argument('--max_frames', type=int, default=2000, help='Maximum number of frames to extract')
    parser.add_argument('--vis', type=bool, default=False, help='Visualize the video')
    parser.add_argument('--scale', action=BooleanOptionalAction, default=False, help='Auto-scale frames to ~640x480 total pixels while preserving aspect ratio')
    parser.add_argument('--gray', action='store_true', help='Process frames in grayscale')
    parser.add_argument('--format', type=str, default='png', help='Image format')

    args = parser.parse_args()
    path_video = args.video
    path_output = args.output
    skip = args.skip
    format = args.format
    fps = args.fps
    max_frames = args.max_frames
    vis = args.vis

    # If output folder exists, remove it and everything inside
    folder_output = Path(path_output)
    if folder_output.exists() and folder_output.is_dir():
        shutil.rmtree(folder_output)
    folder_output.mkdir(parents=True, exist_ok=True)

    # Resolution scaling
    TARGET_PIXELS = 640 * 480  # 307,200
    factor = None  # we'll infer this on the first frame if scale=True

    # Time synchronization via timecode
    tc = probe_timecode(path_video)
    time_ns = 1000000000000000000 + timecode_to_ns(tc, fps)
    delta = int(1.0/fps * 1e9)

    if format not in ['png', 'jpg', 'jpeg', 'bmp', 'tiff']:
        print('Invalid image format!!!')
        return

    cap = cv2.VideoCapture(path_video)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    counter = 0
    for i in tqdm(range(frame_count), desc ="Extracting"):
        ret, img = cap.read()
        time_ns += delta

        if factor is None:
            if args.scale:
                h0, w0 = img.shape[:2]
                factor = (TARGET_PIXELS / float(w0 * h0)) ** 0.5
                factor = min(1.0, factor)

        if ret and (skip == 0 or i % skip == 0):
            image_timestamp = time_ns
            width = int(img.shape[1] *  factor)
            height = int(img.shape[0] * factor)
            res_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

            # Check sharpness
            fm = sharpness(res_img)
            if fm < 30:
                continue
            
            if vis:
                cv2.imshow('Frame', res_img)
                cv2.waitKey(30)
            
            # Change the image to grayscale
            image_timestamp_str = f"{image_timestamp:019d}"

            if args.gray:
                gray = cv2.cvtColor(res_img, cv2.COLOR_BGR2GRAY)
                cv2.imwrite(os.path.join(path_output, f'{image_timestamp_str}.{format}'), gray)
            else:
                cv2.imwrite(os.path.join(path_output, f'{image_timestamp_str}.{format}'), res_img)

            counter += 1

        if counter >= max_frames:
            break
    
    cap.release()
    if vis: cv2.destroyAllWindows()

    return



if __name__ == '__main__':
    main()