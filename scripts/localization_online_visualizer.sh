#! /bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

$APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/local_visualization/online_local_visualizer \
    --flagfile=/apollo/modules/localization/conf/localization.conf \
    --log_dir=/apollo/data/log