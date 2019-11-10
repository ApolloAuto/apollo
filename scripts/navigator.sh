#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Get the absolute path.
i=0;
j=0;
for str in $@
do
    # The strings starting with "--" are control arguments and need to be filtered.
    if [[ ${str} =~ ^--.* ]]; then
        CTRL_ARGS[i++]=${str}
        continue
    fi
    DIR=$(cd "$(dirname ${str} )" && pwd)
    FILE_NAME=$(basename ${str})
    PATH_NAME[j++]="${DIR}/${FILE_NAME}"
done

#echo "${CTRL_ARGS[@]}"
#echo "${PATH_NAME[@]}"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."
source "${DIR}/apollo_base.sh"
${APOLLO_BIN_PREFIX}/modules/map/relative_map/tools/navigator --navigator_config_filename=/apollo/modules/map/relative_map/conf/navigator_config.pb.txt  "${CTRL_ARGS[@]}" "${PATH_NAME[@]}"


