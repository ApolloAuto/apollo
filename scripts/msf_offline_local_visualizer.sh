#! /bin/bash
if [ $# -lt 1 ]; then
    echo "Usage: msf_offline_local_visualizer.sh [data path]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/local_visualization/offline_visual/offline_local_visualizer \
    --basedir $1 \
