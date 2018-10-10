export CYBERTRON_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
library_path="${CYBERTRON_PATH}/third_party:${CYBERTRON_PATH}/python"
binary_path="/apollo/bazel-bin/cybertron"
tool_path="/apollo/bazel-bin/cybertron/tools"
launch_path="${CYBERTRON_PATH}/tools/cyber_launch"
qt_path=${CYBERTRON_PATH}/../framework/third_party/Qt5.5.1/5.5/gcc_64

export LD_LIBRARY_PATH=${library_path}:${qt_path}/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=${qt_path}/plugins
export PATH=${binary_path}:${tool_path}:${launch_path}:${qt_path}/bin:$PATH
export PYTHONPATH=${CYBERTRON_PATH}/python:${CYBERTRON_PATH}/python/cybertron:${CYBERTRON_PATH}/python/cybertron/proto:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

source ${CYBERTRON_PATH}/cyber_tools_auto_complete.bash
