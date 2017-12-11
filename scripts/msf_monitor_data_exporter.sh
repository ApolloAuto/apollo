#! /bin/bash
if [ $# -lt 2 ]; then
    echo "[bag file] [output folder]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

GNSS_LOC_TOPIC="/apollo/localization/msf_gnss"
LIDAR_LOC_TOPIC="/apollo/localization/msf_lidar" 
FUSION_LOC_TOPIC="/apollo/localization/pose" 
CLOUD_TOPIC="/apollo/sensor/velodyne64/compensator/PointCloud2"  

$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/monitor_data_exporter \
    --bag_file $1 \
    --out_folder $2 \
    --cloud_topic $CLOUD_TOPIC \
    --gnss_loc_topic $GNSS_LOC_TOPIC \
    --lidar_loc_topic $LIDAR_LOC_TOPIC \
    --fusion_loc_topic $FUSION_LOC_TOPIC
