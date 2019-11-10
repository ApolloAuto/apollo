#! /bin/bash
if [ $# -lt 4 ]; then
    echo "Usage: msf_local_map_creator.sh [pcd folder] [pose file] [zone id] [map folder]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/lossless_map_creator\
    --use_plane_inliers_only true \
    --pcd_folders $1 \
    --pose_files $2 \
    --map_folder $4 \
    --zone_id $3 \
    --coordinate_type UTM \
    --map_resolution_type single


$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/lossless_map_to_lossy_map \
    --srcdir $4/lossless_map \
    --dstdir $4 \

rm -fr $4/lossless_map
