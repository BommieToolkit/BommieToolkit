```bash
pixi run -e kalibr kalibr-calibrate-stereo-rig  video_left=videos/left.MP4 video_right=videos/right.MP4 target=files/april_10x6.yaml freq=30 output_folder=calibration_output
```

```bash
python match_images_by_ns.py /home/alejandro/kalibr_bommie/calibration_output/cam0 /home/alejandro/kalibr_bommie/calibration_output/cam1 /home/alejandro/colmap_bommie/reconstruction_output/images/rig1/camera1 /home/alejandro/colmap_bommie/reconstruction_output/images/rig1/camera2 --threshold-ns 5000000

```