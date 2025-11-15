Rig Calibration
```bash
pixi run -e kalibr kalibr-calibrate-stereo-rig \
    video_left=videos/cal_left.MP4 \ 
    video_right=videos/cal_right.MP4 \ 
    target=files/april_10x6.yaml \ 
    freq=30 \ 
    output_folder=calibration_output
```

Video extraction
```bash
pixi run extract_images --video videos/monkey_left.MP4 --output monkey_output/monkey_images_left
pixi run extract_images --video videos/monkey_right.MP4 --output monkey_output/monkey_images_right
```

```bash
pixi run match_images_by_ns monkey_output/monkey_images_left monkey_output/monkey_images_right monkey_output/colmap_images/rig1/camera1 monkey_output/colmap_images/rig1/camera2 --threshold-ns 5000000
```

Colmap
```bash
pixi run -e colmap colmap feature_extractor \
    --image_path monkey_output/colmap_images \
    --database_path monkey_output/database.db \
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
#mkdir -p /home/alejandro/colmap_bommie/reconstruction_output/sparse
pixi run -e colmap colmap mapper \
    --database_path monkey_output/database.db  \
    --Mapper.ba_refine_sensor_from_rig 0 \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_extra_params 0 \
    --image_path monkey_output/colmap_images \
    --output_path monkey_output/sparse \
    --Mapper.ba_use_gpu 1
``` 

```bash
pixi run -e colmap colmap gui \
    --database_path monkey_output/database.db  \
    --image_path monkey_output/colmap_images \
    --import_path monkey_output/sparse/0
``` 

```bash
pixi run -e colmap colmap model_converter \
	--input_path monkey_output/sparse/0 \
    --output_path monkey_output/sparse/0 \
    --output_type TXT
``` 

<!-- ```bash
pixi run -e colmap colmap model_converter \
	--input_path monkey_output/sparse/0 \
    --output_path monkey_output/sparse/0/mesh.ply \
    --output_type PLY
```  -->

```bash
pixi run colmap2nerf --text monkey_output/sparse/0 --images monkey_output/colmap_images --colmap_db monkey_output/database.db --out transforms.json

pixi run colmap2nerf --text sparse/0 --images colmap_images --colmap_db database.db --out transforms.json
```

nerfstudio
```bash
git clone https://github.com/nerfstudio-project/nerfstudio.git
cd nerfstudio
pixi run post-install
pixi shell

```
```bash
ns-train splatfacto --data /home/alejandro/BommieToolkit
```

```bash
ns-viewer --load-config outputs/BommieToolkit/splatfacto/2025-11-15_150812/config.yml
```