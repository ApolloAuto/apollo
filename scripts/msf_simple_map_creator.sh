#! /bin/bash
if [ $# -lt 3 ]; then
    echo "Usage: msf_simple_map_creator.sh [bags folder] [extrinsic_file] [zone_id]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

GNSS_LOC_TOPIC="/apollo/localization/msf_gnss"
LIDAR_LOC_TOPIC="/apollo/localization/msf_lidar"
FUSION_LOC_TOPIC="/apollo/localization/pose"
ODOMETRY_LOC_TOPIC="/apollo/sensor/gnss/odometry"
CLOUD_TOPIC="/apollo/sensor/velodyne64/compensator/PointCloud2"

GNSS_LOC_FILE="gnss_loc.txt"
LIDAR_LOC_FILE="lidar_loc.txt"
FUSION_LOC_FILE="fusion_loc.txt"
ODOMETRY_LOC_FILE="odometry_loc.txt"

IN_FOLDER=$1
EXTRINSIC_FILE=$2
ZONE_ID=$3

function data_exporter() {
  local BAG_FILE=$1
  local OUT_FOLDER=$2
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/monitor_data_exporter \
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
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/poses_interpolator \
   --input_poses_path $INPUT_POSES_PATH \
   --ref_timestamps_path $REF_TIMESTAMPS_PATH \
   --extrinsic_path $EXTRINSIC_PATH \
   --output_poses_path $OUTPUT_POSES_PATH
}

function create_lossless_map() {
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/lossless_map_creator \
      --use_plane_inliers_only true \
      --pcd_folders $1 \
      --pose_files $2 \
      --map_folder $IN_FOLDER \
      --zone_id $ZONE_ID \
      --coordinate_type UTM \
      --map_resolution_type single
}

function create_lossy_map() {
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/lossless_map_to_lossy_map \
    --srcdir $IN_FOLDER/lossless_map \
    --dstdir $IN_FOLDER \

  rm -fr $IN_FOLDER/lossless_map
}

cd $IN_FOLDER
for item in $(ls -l *.bag | awk '{print $9}')
do
  DIR_NAME=$(echo $item | cut -d . -f 1)
  mkdir $DIR_NAME
  data_exporter "${item}" "${DIR_NAME}"
  poses_interpolation "${DIR_NAME}/pcd/${ODOMETRY_LOC_FILE}" "${DIR_NAME}/pcd/pcd_timestamp.txt" "${EXTRINSIC_FILE}" "${DIR_NAME}/pcd/corrected_poses.txt"
  create_lossless_map "${DIR_NAME}/pcd" "${DIR_NAME}/pcd/corrected_poses.txt"
done

create_lossy_map

echo "Done."
