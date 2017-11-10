#!/bin/bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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


SCRIPT_NAME="data-recorder"
PWD=$(cd "$(dirname "$0")"; pwd)

color_message() {
    local message=$1
    local color=$2
    case "${color}" in
        "BLUE" )
            echo -e "\033[34m>> ${message}\033[0m"
            ;;
	"GREEN" )
            echo -e "\033[32m>> ${message}\033[0m"
            ;;
	"RED"  )
	    echo -e "\033[31m>> ${message}\033[0m"
            ;;
	"YELLOW"  )
	    echo -e "\033[33m>> ${message}\033[0m"
            ;;
	"*"  )
	    echo -e "\033[31m>> Only support 'GREEN | RED |YELLOW'" && exit 1
    esac
}

usage_start() {
    color_message "Data Recorder:" "BLUE"
    color_message "Usage: bash ${SCRIPT_NAME} start [debug | ads | collection]" "BLUE"
    color_message "Version: ${SCRIPT_NAME} ${VERSION} Copyright Baidu Inc." "BLUE"
}

usage() {
    color_message "Data Recorder:" "BLUE"
    color_message "Usage: bash ${SCRIPT_NAME} [ start | stop | check-env | config ]" "BLUE"
    color_message "Version: ${SCRIPT_NAME} ${VERSION} Copyright Baidu Inc." "BLUE"
}

start() {
    local task_purpose=$1
    ps -ef|grep 'data_recorder_manager'|grep -v 'data_recorder_control'|grep -v 'grep' &>/dev/null
    [ $? -eq 0 ] && printf "\033[31mThere is another data-recorder process running, if you want to stop that process, you can run following command:\nbash data_recorder_control.sh stop\n\033[0m" && exit 1
    python data_recorder_manager.py -c ../conf/recorder.${task_purpose}.yaml &
    sleep 1
    ps -ef|grep 'data_recorder_manager'|grep -v 'grep' &>/dev/null
    [ $? -eq 0 ] && echo "data-recorder start successfully." || echo "data-recorder start failed."
}

stop() {
    ps -ef|grep 'data_recorder_manager'|grep -v 'grep'|awk '{print $2}'|xargs kill
    [ $? -eq 0 ] && echo "Stop data-recorder successfully" || echo "Stop data-recorder failed."
    return 0
}

restart() {
    stop
    sleep 10
    start
}

main() {
    if [[ $# -lt 1 ]]; then
        usage && exit 0
    fi
    case "$1" in
        start )
               if [ $# -eq 1 ];then
                   start "debug"
               elif [ $# -eq 2 ]; then
                   case "$2" in
                       (debug|ads|collection)
                           start $2 ;;
                       (*)
                           usage_start && exit 0
                   esac
               else
                   usage_start && exit 0
               fi
               ;;
        stop )
               stop
               ;;
        config)
               config
               ;;
        *)
               usage && exit 1
    esac
}

main "$@"
