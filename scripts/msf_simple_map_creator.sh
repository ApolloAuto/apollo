#! /bin/bash
if [ $# -lt 4 ]; then
  echo "Usage: msf_simple_map_creator.sh [records folder] [extrinsic_file] [zone_id] [map folder] [lidar_type]"
  exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

GNSS_LOC_TOPIC="/apollo/localization/msf_gnss"
LIDAR_LOC_TOPIC="/apollo/localization/msf_lidar"
FUSION_LOC_TOPIC="/apollo/localization/pose"
ODOMETRY_LOC_TOPIC="/apollo/sensor/gnss/odometry"

GNSS_LOC_FILE="gnss_loc.txt"
LIDAR_LOC_FILE="lidar_loc.txt"
FUSION_LOC_FILE="fusion_loc.txt"
ODOMETRY_LOC_FILE="odometry_loc.txt"

IN_FOLDER=$1
EXTRINSIC_FILE=$2
ZONE_ID=$3
OUT_MAP_FOLDER=$4
LIDAR_TYPE=${5:-lidar128}

PARSED_DATA_FOLDER="$OUT_MAP_FOLDER/parsed_data"
CLOUD_TOPIC="/apollo/sensor/$LIDAR_TYPE/compensator/PointCloud2"

function data_exporter() {
  local BAG_FILE=$1
  local OUT_FOLDER=$2
  /apollo/bazel-bin/modules/localization/msf/local_tool/data_extraction/cyber_record_parser \
    --bag_file $BAG_FILE \
    --out_folder $OUT_FOLDER \
    --cloud_topic $CLOUD_TOPIC \
    --gnss_loc_topic $GNSS_LOC_TOPIC \
    --lidar_loc_topic $LIDAR_LOC_TOPIC \
    --fusion_loc_topic $FUSION_LOC_TOPIC \
    --odometry_loc_topic $ODOMETRY_LOC_TOPIC
}

function poses_interpolation() {
  local INPUT_POSES_PATH=$1
  local REF_TIMESTAMPS_PATH=$2
  local EXTRINSIC_PATH=$3
  local OUTPUT_POSES_PATH=$4
  /apollo/bazel-bin/modules/localization/msf/local_tool/map_creation/poses_interpolator \
    --input_poses_path $INPUT_POSES_PATH \
    --ref_timestamps_path $REF_TIMESTAMPS_PATH \
    --extrinsic_path $EXTRINSIC_PATH \
    --output_poses_path $OUTPUT_POSES_PATH
}

function create_lossless_map() {
  /apollo/bazel-bin/modules/localization/msf/local_tool/map_creation/lossless_map_creator \
    --use_plane_inliers_only true \
    --pcd_folders $1 \
    --pose_files $2 \
    --map_folder $OUT_MAP_FOLDER \
    --zone_id $ZONE_ID \
    --coordinate_type UTM \
    --map_resolution_type single
}

function create_lossy_map() {
  /apollo/bazel-bin/modules/localization/msf/local_tool/map_creation/lossless_map_to_lossy_map \
    --srcdir $OUT_MAP_FOLDER/lossless_map \
    --dstdir $OUT_MAP_FOLDER

  rm -fr $OUT_MAP_FOLDER/lossless_map
  rm -fr $OUT_MAP_FOLDER/parsed_data
  mv $OUT_MAP_FOLDER/lossy_map $OUT_MAP_FOLDER/local_map
}

cd $IN_FOLDER
mkdir -p $OUT_MAP_FOLDER
mkdir -p $PARSED_DATA_FOLDER
for item in $(ls -l *.record* | awk '{print $9}'); do
  SEGMENTS=$(echo $item | awk -F'.' '{print NF}')
  DIR_NAME=$(echo $item | cut -d . -f ${SEGMENTS})
  DIR_NAME="${PARSED_DATA_FOLDER}/${DIR_NAME}"
  mkdir -p ${DIR_NAME}

  data_exporter "${item}" "${DIR_NAME}"
  poses_interpolation "${DIR_NAME}/pcd/${ODOMETRY_LOC_FILE}" "${DIR_NAME}/pcd/pcd_timestamp.txt" "${EXTRINSIC_FILE}" "${DIR_NAME}/pcd/corrected_poses.txt"
  create_lossless_map "${DIR_NAME}/pcd" "${DIR_NAME}/pcd/corrected_poses.txt"
done

create_lossy_map

echo "Done."
