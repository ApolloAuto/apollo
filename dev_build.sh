#!/usr/bin/env bash

###############################################################################
# Copyright 2018 David Hopper(davidhopper2003@hotmail.com). All Rights Reserved.
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

current_docker=$(docker ps | grep 'apolloauto/apollo')
if [ -z "${current_docker}" ]
then 
    # start the docker firstly. 
    # echo ${current_docker} 
    bash docker/scripts/dev_start.sh
fi

xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u $USER \
    -it apollo_dev \
    /bin/bash apollo.sh $1
xhost -local:root 1>/dev/null 2>&1
