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
for file_name in $@
do    
    DIR=$(cd "$(dirname ${file_name} )" && pwd)
    FILE_NAME=$(basename ${file_name})
    PATH_NAME[i++]="${DIR}/${FILE_NAME}"    
done

#echo "${PATH_NAME[@]}"
cd /apollo
./bazel-bin/modules/map/relative_map/tools/navigator "${PATH_NAME[@]}" 

