export CYBER_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
binary_path="/apollo/bazel-bin/cyber"
tool_path="/apollo/bazel-bin/cyber/tools"
recorder_path="${tool_path}/cyber_recorder"
PYTHON_LD_PATH="/apollo/bazel-bin/cyber/py_wrapper"
launch_path="${CYBER_PATH}/tools/cyber_launch"
qt_path=${CYBER_PATH}/../third_party/Qt5.5.1/5.5/gcc_64

export LD_LIBRARY_PATH=${qt_path}/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=${qt_path}/plugins
export PATH=${binary_path}:${tool_path}:${recorder_path}:${launch_path}:${qt_path}/bin:$PATH
export PYTHONPATH=${PYTHON_LD_PATH}:${CYBER_PATH}/python:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

source ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
