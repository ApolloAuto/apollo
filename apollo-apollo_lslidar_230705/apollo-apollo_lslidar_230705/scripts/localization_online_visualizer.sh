#! /bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

cyber_launch start /apollo/modules/localization/launch/msf_visualizer.launch
