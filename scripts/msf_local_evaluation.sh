#! /bin/bash
if [ $# -lt 1 ]; then
    echo "Usage:"
    echo "$0 [bags folder]                   evaluate fusion and lidar localization result"
    echo "$0 [bags folder] [ant arm file]    evaluate fusion, lidar and gnss localization result"
    exit 1;
fi

IN_FOLDER=$1
if [ $# -eq 2 ]; then
  ANT_IMU_FILE=$2
else
  ANT_IMU_FILE=""
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

function data_exporter() {
  local BAG_FILE=$1
  local OUT_FOLDER=$2
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/cyber_record_parser \
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
    --imu_to_ant_offset_file "$ANT_IMU_FILE" \
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
for item in $(ls -l *record.* | awk '{print $9}')
do
  SEGMENTS=$(echo $item | awk -F'.' '{print NF}')
  DIR_NAME=$(echo $item | cut -d . -f ${SEGMENTS})
  if [ -d "${DIR_NAME}" ]; then
    rm -r ${DIR_NAME}
  fi
  mkdir ${DIR_NAME}
  data_exporter "${item}" "${DIR_NAME}"
  compare_poses "${DIR_NAME}/pcd"
done

rm -rf compare_fusion_odometry_all.txt
touch compare_fusion_odometry_all.txt
for item in  $(find . -name "compare_fusion_odometry.txt")
do 
  cat $item >> compare_fusion_odometry_all.txt
done

rm -rf compare_lidar_odometry_all.txt
touch compare_lidar_odometry_all.txt
for item in  $(find . -name "compare_lidar_odometry.txt")
do 
  cat $item >> compare_lidar_odometry_all.txt
done

rm -rf compare_gnss_odometry_all.txt
touch compare_gnss_odometry_all.txt
for item in  $(find . -name "compare_gnss_odometry.txt")
do 
  cat $item >> compare_gnss_odometry_all.txt
done

echo ""
echo "Fusion localization result:"
python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py compare_fusion_odometry_all.txt

echo ""
echo "Lidar localization result:"
python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py compare_lidar_odometry_all.txt

if [ $# -eq 2 ]; then
  echo ""
  echo "GNSS localization result:"
  python ${APOLLO_ROOT_DIR}/modules/tools/localization/evaluate_compare.py\
    compare_gnss_odometry_all.txt distance_only
fi
