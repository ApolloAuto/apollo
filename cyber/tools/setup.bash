export CYBER_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
library_path="${CYBER_PATH}/lib:${CYBER_PATH}/third_party:${CYBER_PATH}/lib/python"
binary_path=${CYBER_PATH}/bin/
qt_path=${CYBER_PATH}/../third_party/Qt5.5.1/5.5/gcc_64

export LD_LIBRARY_PATH=${library_path}:${qt_path}/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=${CYBER_PATH}/third_party/plugins
export PATH=${binary_path}:${qt_path}/bin:$PATH
export PYTHONPATH=${CYBER_PATH}/lib/python:${CYBER_PATH}/lib/python/cyber:${CYBER_PATH}/lib/python/cyber/proto:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

source ${CYBER_PATH}/cyber_tools_auto_complete.bash
