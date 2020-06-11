#! /usr/bin/env bash
if [ -z "${APOLLO_ROOT_DIR}" ]; then
    export APOLLO_ROOT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
    source ${APOLLO_ROOT_DIR}/scripts/apollo.bashrc
fi

export CYBER_PATH="${APOLLO_ROOT_DIR}/cyber"

bazel_bin_path="${APOLLO_ROOT_DIR}/bazel-bin"
cyber_tool_path="${bazel_bin_path}/cyber/tools"
apollo_tool_path="${bazel_bin_path}/modules/tools"
recorder_path="${cyber_tool_path}/cyber_recorder"
monitor_path="${cyber_tool_path}/cyber_monitor"
visualizer_path="${apollo_tool_path}/visualizer"
launch_path="${CYBER_PATH}/tools/cyber_launch"
channel_path="${CYBER_PATH}/tools/cyber_channel"
node_path="${CYBER_PATH}/tools/cyber_node"
service_path="${CYBER_PATH}/tools/cyber_service"
qt_path=/usr/local/Qt5.12.2/5.12.2/gcc_64
rosbag_to_record_path="${bazel_bin_path}/modules/data/tools/rosbag_to_record"

export LD_LIBRARY_PATH=${qt_path}/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=${qt_path}/plugins

# TODO(all): place all these in one place and add_to_path
for entry in "${recorder_path}" "${monitor_path}" "${launch_path}" \
    "${channel_path}" "${node_path}" "${service_path}" \
    "${qt_path}/bin" \
    "${visualizer_path}" \
    "${rosbag_to_record_path}" ; do
    add_to_path "${entry}"
done

PYTHON_LD_PATH="${bazel_bin_path}/cyber/py_wrapper"
export PYTHONPATH=${PYTHON_LD_PATH}:${CYBER_PATH}/python:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

export GLOG_log_dir="${APOLLO_ROOT_DIR}/data/log"
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

# for DEBUG log
#export GLOG_minloglevel=-1
#export GLOG_v=4

source ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
