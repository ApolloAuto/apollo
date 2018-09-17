export CYBERTRON_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
library_path="${CYBERTRON_PATH}/lib:${CYBERTRON_PATH}/third_party:${CYBERTRON_PATH}/lib/python"
binary_path=${CYBERTRON_PATH}/bin/

export LD_LIBRARY_PATH=${library_path}:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=${CYBERTRON_PATH}/third_party/plugins
export PATH=${binary_path}:$PATH
export PYTHONPATH=${CYBERTRON_PATH}/lib/python/cybertron:${CYBERTRON_PATH}/lib/python/cybertron/proto:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

source ${CYBERTRON_PATH}/cyber_tools_auto_complete.bash
