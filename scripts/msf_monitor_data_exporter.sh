#! /bin/bash
if [ $# -lt 1 ]; then
    echo "Usage: msf_monitor_data_exporter.sh [bags folder]"
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

function compare_poses() {
  local IN_FOLDER=$1
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/compare_poses \
    --in_folder $IN_FOLDER \
    --loc_file_a $GNSS_LOC_FILE \
    --loc_file_b $ODOMETRY_LOC_FILE \
    --compare_file "compare_gnss_odometry.txt"

  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/compare_poses \
    --in_folder $IN_FOLDER \
    --loc_file_a $LIDAR_LOC_FILE \
    --loc_file_b $ODOMETRY_LOC_FILE \
    --compare_file "compare_lidar_odometry.txt"

  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/compare_poses \
    --in_folder $IN_FOLDER \
    --loc_file_a $FUSION_LOC_FILE \
    --loc_file_b $ODOMETRY_LOC_FILE \
    --compare_file "compare_fusion_odometry.txt"
}

cd $IN_FOLDER
for item in $(ls -l *.bag | awk '{print $9}')
do
  DIR_NAME=$(echo $item | cut -d . -f 1)
  mkdir $DIR_NAME
  data_exporter "${item}" "${DIR_NAME}"
  compare_poses "${DIR_NAME}/pcd"
done

touch compare_fusion_odometry_all.txt
for item in  $(find . -name "compare_fusion_odometry.txt")
do 
  cat $item >> compare_fusion_odometry_all.txt
done

touch compare_lidar_odometry_all.txt
for item in  $(find . -name "compare_lidar_odometry.txt")
do 
  cat $item >> compare_lidar_odometry_all.txt
done

touch compare_gnss_odometry_all.txt
for item in  $(find . -name "compare_gnss_odometry.txt")
do 
  cat $item >> compare_gnss_odometry_all.txt
done

python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py compare_fusion_odometry_all.txt
python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py compare_lidar_odometry_all.txt
python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py compare_gnss_odometry_all.txt
