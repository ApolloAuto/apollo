#! /bin/bash
if [ $# -lt 4 ]; then
    echo "Usage: msf_poses_interpolator.sh [input_poses_path] [ref_timestamps_path] [extrinsic_path] [output_poses_path]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/poses_interpolator \
    --input_poses_path $1 \
    --ref_timestamps_path $2 \
    --extrinsic_path $3 \
    --output_poses_path $4 \
