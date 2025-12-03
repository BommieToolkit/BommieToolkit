<div align="center">
    <h1>BommieToolkit</h1>
    <a href="https://github.com/BommieToolkit/BommieToolkit"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <br />
</div>

<p align="center">
    <a href="https://scholar.google.com/citations?user=SDtnGogAAAAJ&hl=en"><strong>Alejandro Fontan</strong></a>
    ·
    <a href="https://scholar.google.com/citations?user=MNrMUPMAAAAJ&hl=en"><strong>Emilio Olivastri</strong></a>
</p>

## Video Recording

## Rig Calibration
Build Kalibr
```bash
pixi run -e kalibr build
```

Extract images from calibration videos
```bash
pixi run extract_images --video videos/cal_left.MP4 --output calibration_output/cam0 --gray --scale
pixi run extract_images --video videos/cal_right.MP4 --output calibration_output/cam1 --gray --scale
or
# Skip is now how many seconds to skip from the start of the video
# You can let the factor be estimated automatically or explicit it 
pixi run extract_images --video videos/cal_left.MP4 --output calibration_output/cam0 --gray --factor 0.5 --skip 2.0
pixi run extract_images --video videos/cal_right.MP4 --output calibration_output/cam1 --gray --factor 0.5 --skip 2.0

```

Run calibration
```bash
pixi run -e kalibr kalibr-calibrate-stereo-rig \
  images_folder_left=calibration_output/cam0 \
  images_folder_right=calibration_output/cam1 \
  output_folder=calibration_output \
  target=files/april_10x6.yaml \
  freq=30
```

Generate .json file with rig configuration
```bash
pixi run get_rig_config_json calibration_output/calibration-camchain.yaml calibration_output/rig_config.json
```

## COLMAP Reconstruction
Extract images from videos

```bash
pixi run extract_images --video videos/monkey_left.MP4 --output monkey_output/monkey_images_left --scale
pixi run extract_images --video videos/monkey_right.MP4 --output monkey_output/monkey_images_right --scale
```

Synch image pairs using timestamps

```bash
pixi run match_images_by_ns \
  --images_folder_left monkey_output/monkey_images_left \
  --images_folder_right monkey_output/monkey_images_right \
  --colmap_folder_left monkey_output/colmap_images/rig1/camera1 \
  --colmap_folder_right monkey_output/colmap_images/rig1/camera2 \
  --threshold-ns 5000000
```

Execute COLMAP
```bash
pixi run -e colmap colmap feature_extractor \
  --image_path monkey_output/colmap_images \
  --database_path monkey_output/database.db \
  --ImageReader.single_camera 1 \
  --ImageReader.single_camera_per_folder 1
```

```bash
pixi run -e colmap colmap rig_configurator \
  --database_path monkey_output/database.db \
  --rig_config_path calibration_output/rig_config.json
```

```bash
pixi run -e colmap colmap sequential_matcher --database_path monkey_output/database.db 
``` 

```bash
mkdir -p monkey_output/sparse
pixi run -e colmap colmap mapper \
  --database_path monkey_output/database.db  \
  #--Mapper.ba_refine_sensor_from_rig 0 \
  --Mapper.ba_refine_focal_length 0 \
  --Mapper.ba_refine_extra_params 0 \
  --image_path monkey_output/colmap_images \
  --output_path monkey_output/sparse \
  --Mapper.ba_use_gpu 1

pixi run -e colmap colmap mapper \
  --database_path monkey_output/database.db  \
  --Mapper.ba_refine_focal_length 0 \
  --Mapper.ba_refine_extra_params 0 \
  --image_path monkey_output/colmap_images \
  --output_path monkey_output/sparse \
  --Mapper.ba_use_gpu 1
``` 

Visualize reconstruction
```bash
pixi run -e colmap colmap gui \
  --database_path monkey_output/database.db  \
  --image_path monkey_output/colmap_images \
  --import_path monkey_output/sparse/0
``` 

Get COLMAP output
```bash
pixi run -e colmap colmap model_converter \
	--input_path monkey_output/sparse/0 \
    --output_path monkey_output/sparse/0 \
    --output_type TXT
``` 

```bash
pixi run -e colmap colmap model_converter \
	--input_path monkey_output/sparse/0 \
    --output_path monkey_output/sparse/0/mesh.ply \
    --output_type PLY
```  

```bash
pixi run colmap2nerf --text sparse/0 --images colmap_images --out transforms.json
```

## GS Reconstruction with nerfstudio

```bash
git clone https://github.com/nerfstudio-project/nerfstudio.git
cd nerfstudio
pixi run post-install
pixi shell
```

```bash
ns-train splatfacto --data /home/alejandro/BommieToolkit/monkey_output
```

```bash
ns-viewer --load-config outputs/monkey_output/splatfacto/2025-11-19_105617/config.yml
```

"ply_file_path" : "/home/alejandro/BommieToolkit/monkey_output/sparse/0/mesh.ply",

## BommieToolkit Roadmap

- [ ] Make Kalibr a Conda package
- [ ] Implement one end-to-end command, from videos to GS.
- [ ] Documentation for the intermediate outputs
- [ ] Documentation on recording calibration/reconstruction data
- [ ] Documentation on gopro settings
- [ ] How to build an underwater calibration pattern
- [ ] Refraction Removal