#!/bin/bash

# Inputs
video_left="videos/cal_left.MP4"
video_right="videos/cal_right.MP4"
freq=30.0
target="files/april_10x6.yaml"
output_folder="calibration_output"

# Check inputs
split_and_assign() {
  local input=$1
  local key=$(echo $input | cut -d'=' -f1)
  local value=$(echo $input | cut -d'=' -f2-)
  eval $key=$value
}

# Split the input string into individual components
for ((i=1; i<=$#; i++)); do
    split_and_assign "${!i}"
done

# Create folder structure
images_folder_left="${output_folder}/cam0"
images_folder_right="${output_folder}/cam1"
output_bag="${output_folder}/calibration.bag"

mkdir -p ${output_folder}
mkdir -p ${images_folder_left}
mkdir -p ${images_folder_right}

# Run calibration steps
source catkin_ws/devel/setup.bash
python scripts/vid2imgs.py --video ${video_left}  --output ${images_folder_left} 
python scripts/vid2imgs.py --video ${video_right} --output ${images_folder_right} 

rosrun kalibr kalibr_bagcreater --folder ${output_folder} --output-bag ${output_bag}
rosrun kalibr kalibr_calibrate_cameras --target ${target} --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --bag ${output_bag} --bag-freq ${freq} --verbose
