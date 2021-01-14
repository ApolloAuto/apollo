#!/usr/bin/env bash

# This script is expected to run under APOLLO_ROOT_DIR inside Docker
# cd /apollo
# bash tools/install/run.sh

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
DEST_DIR=/tmp/apollo

python3 ${TOP_DIR}/tools/install/operator.py > ${TOP_DIR}/tools/install/abc.txt
python3 ${TOP_DIR}/tools/install/postprocess.py
python3 ${TOP_DIR}/tools/install/install.py --pre_clean "${DEST_DIR}"

mkdir -p ${DEST_DIR}/data/log

# TODO(storypku): rewrite cyber/setup.bash for release builds
export CYBER_PATH="${DEST_DIR}/cyber"
export GLOG_log_dir="${DEST_DIR}/data/log"
export GLOG_alsologtostderr=1
export GLOG_colorlogtostderr=1

# TODO(storypku): Elegant way to handle different paths of libxxx_component.so for develop & release builds
sed -i 's@/apollo/bazel-bin@'"${DEST_DIR}"'@' ${DEST_DIR}/cyber/examples/common_component_example/common.dag
sed -i 's@/apollo/bazel-bin@'"${DEST_DIR}"'@' ${DEST_DIR}/cyber/examples/timer_component_example/timer.dag

# ${DEST_DIR}/bin/mainboard -d ${DEST_DIR}/cyber/examples/common_component_example/common.dag
${DEST_DIR}/bin/mainboard -d ${DEST_DIR}/cyber/examples/timer_component_example/timer.dag

# TODO(storypku): Notes that cyber_launch also works in this way.
