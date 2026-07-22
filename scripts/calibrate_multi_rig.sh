#!/bin/bash
set -euo pipefail

# Inputs
freq=""
target=""
output_folder=""
verbose=""
create_bag=""
manual_focal_length_init=""
camera_model=""
declare -A images_folder

# Check inputs
split_and_assign() {
  local input=$1
  local key=$(echo $input | cut -d'=' -f1)
  local value=$(echo $input | cut -d'=' -f2-)
  case "$key" in
     output_folder|freq|target|verbose|create_bag|manual_focal_length_init|camera_model)
      printf -v "$key" '%s' "$value"
      ;;
    images_folder_*)
      local idx="${key#images_folder_}"
      if ! [[ "$idx" =~ ^[0-9]+$ ]]; then
        echo "Invalid camera index in '$key' - expected images_folder_N" >&2
        exit 2
      fi
      images_folder[$idx]="$value"
      ;;
    *)
      echo "Unknown argument: $key" >&2
      exit 2
      ;;
  esac
}

# Split the input string into individual components
for ((i=1; i<=$#; i++)); do
    split_and_assign "${!i}"
done

freq="${freq:-30.0}"
target="${target:-files/april_10x6.yaml}"
output_folder="${output_folder:-calibration_ws}"
verbose="${verbose:-0}"
create_bag="${create_bag:-1}"
camera_model="${camera_model:-pinhole-radtan}"
manual_focal_length_init="${KALIBR_MANUAL_FOCAL_LENGTH_INIT:-0}"

num_cameras=${#images_folder[@]}
if [ "$num_cameras" -lt 1 ]; then
  echo "Need at least one image_folder_N=path argument." >&2
  exit 2
fi

echo "Configuring $num_cameras camera(s)..."

# Create folder structure
mkdir -p "$output_folder"

bag_input_folder="$(dirname "${images_folder[0]}")"
for ((i=0; i<num_cameras; i++)); do
  folder="${images_folder[$i]}"
  if [ "$(dirname "$folder")" != "$bag_input_folder" ]; then
    echo "All images_folder_N must share one parent folder for kalibr_bagcreater. '$folder' does not match '$bag_input_folder'." >&2
    exit 2
  fi
  if [ "$(basename "$folder")" != "cam${i}" ]; then
    echo "Kalibr expects camera folders named cam0, cam1, ...; got '$folder' for camera index $i." >&2
    exit 2
  fi
done

has_calibration_images() {
  local image_folder=$1
  [ -d "$image_folder" ] || return 1
  find "$image_folder" -type f \( -iname '*.png' -o -iname '*.jpg' -o -iname '*.jpeg' \) -print -quit | grep -q .
}

if [ "$create_bag" -eq 1 ]; then
  for ((i=0; i<num_cameras; i++)); do
    if ! has_calibration_images "${images_folder[$i]}"; then
      echo "No calibration images found in '${images_folder[$i]}'." >&2
      exit 1
    fi
  done
fi

output_bag="${output_folder}/calibration.bag"
expected_camchain="${output_folder}/calibration-camchain.yaml"

if [ "$create_bag" -eq 1 ] && [ -f "$output_bag" ]; then
  rm "$output_bag"
fi

verbose_cmd=""
if [ "$verbose" -eq 1 ]; then
  verbose_cmd="--verbose"
fi

# Run calibration steps
set +u
source catkin_ws/devel/setup.bash
set -u
kalibr_bin="${KALIBR_BIN:-catkin_ws/devel/.private/kalibr/lib/kalibr}"

if [ "$create_bag" -eq 1 ]; then
  "${kalibr_bin}/kalibr_bagcreater" --folder "$bag_input_folder" --output-bag "$output_bag"
fi

if [ ! -f "$output_bag" ]; then
  echo "Calibration bag not found: $output_bag" >&2
  echo "Run with create_bag=1, or provide an existing bag at that path when create_bag=0." >&2
  exit 1
fi

if [ "$manual_focal_length_init" -eq 1 ]; then
  export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
else
  unset KALIBR_MANUAL_FOCAL_LENGTH_INIT
fi

# Build --topics and --models lists with num_cameras entries
topics=()
models=()
for ((i=0; i<num_cameras; i++)); do
  topics+=("/cam${i}/image_raw")
  models+=("$camera_model")
done
 
echo "Topics: ${topics[*]}"
echo "Models: ${models[*]}"
 
"${kalibr_bin}/kalibr_calibrate_cameras" \
  --target "$target" \
  --models "${models[@]}" \
  --topics "${topics[@]}" \
  --bag "$output_bag" \
  --bag-freq "$freq" \
  --dont-show-report ${verbose_cmd}
 
if [ ! -f "$expected_camchain" ]; then
  echo "Kalibr completed without producing expected camchain: $expected_camchain" >&2
  exit 1
fi
 
echo "Done. Result: $expected_camchain"
