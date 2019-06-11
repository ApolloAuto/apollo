#! /bin/bash
if [ $# -lt 1 ]; then
    echo "Usage: msf_record_parser.sh [records folder] [output folder]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

GNSS_LOC_TOPIC="/apollo/localization/msf_gnss"
LIDAR_LOC_TOPIC="/apollo/localization/msf_lidar" 
FUSION_LOC_TOPIC="/apollo/localization/pose"
ODOMETRY_LOC_TOPIC="/apollo/sensor/gnss/odometry"
CLOUD_TOPIC="/apollo/sensor/lidar128/compensator/PointCloud2"

GNSS_LOC_FILE="gnss_loc.txt"
LIDAR_LOC_FILE="lidar_loc.txt"
FUSION_LOC_FILE="fusion_loc.txt"
ODOMETRY_LOC_FILE="odometry_loc.txt"

IN_FOLDER=$1
OUT_FOLDER=$2

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

cd $IN_FOLDER
for item in $(ls -l *record.* | awk '{print $9}')
do
  SEGMENTS=$(echo $item | awk -F'.' '{print NF}')
  DIR_NAME=$(echo $item | cut -d . -f ${SEGMENTS})
  mkdir -p $OUT_FOLDER/$DIR_NAME
  data_exporter "${item}" "$OUT_FOLDER/$DIR_NAME"
done
