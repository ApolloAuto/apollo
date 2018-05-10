#!/usr/bin/env bash

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

function print_help() {
   echo "Example: filter_planning.sh -[pc|np|wp] [-d|--dir] bag1 bag2 bag3 ..."
   echo "-d|--dir the target storage directory"
   echo "This script works in three modes:"
   echo "  -pc filter for perfect control, produces *.pc.bag"
   echo "  -po filter only for perception topic, produces *.po.bag"
   echo "  -np filter for planning dependencies, produces *.np.bag"
   echo "  -wp filter for planning and its dependencies, produces *.wp.bag"
   echo "  -pn filter out planning and prediction, produces *.npp.bag"
   echo "  -co filter out perception, prediction and planning, produces *.co.bag"
}

routing_topic="topic == '/apollo/routing_response'"

perception_topic="topic == '/apollo/perception/obstacles' \
   or topic == '/apollo/perception/traffic_light'"

perfect_control_topic="$perception_topic  \
   or $routing_topic \
   or topic == '/apollo/perception/obstacles' \
   or topic == '/apollo/prediction' \
   or topic == '/apollo/perception/traffic_light'"

planning_deps="$perfect_control_topic \
    or topic == '/apollo/canbus/chassis' \
    or topic == '/apollo/localization/pose' \
    or topic == '/apollo/navigation' \
    or topic == '/apollo/relative_map'"

planning_topic="topic == '/apollo/planning'"
prediction_topic="topic == '/apollo/prediction'"

planning_all="topic == '/apollo/planning' \
    or topic == '/apollo/drive_event' \
    or $planning_deps"

camera_only="topic != '/apollo/perception/obstacles' \
    and topic != '/apollo/prediction' \
    and topic != '/apollo/planning'"

#Three different filter mode
#create perfect control mode bag
is_perfect_control=false
#create a rosbag with planning and its dependencies
is_with_planning=false
#create a rosbag only with planning's dependencies
is_no_planning=false

#only perception topic
is_perception=false;

#no prediction and no planning
is_no_prediction_planning=false;

#no perception, no prediction and no planning, with only camera topic
is_camera_only=false;

work_mode_num=0


#argument parsing code from https://stackoverflow.com/a/14203146
POSITIONAL=()
target_dir=""
while [[ $# -gt 0 ]]; do
key="$1"
case $key in
    -po|--perception_only)
    is_perception=true
    work_mode_num=$((work_mode_num+1))
    shift # past argument
    ;;
    -pc|--perfectcontrol)
    is_perfect_control=true
    work_mode_num=$((work_mode_num+1))
    shift # past argument
    ;;
    -np|--noplanning)
    is_no_planning=true
    work_mode_num=$((work_mode_num+1))
    shift # past argument
    ;;
    -pn|--nopredictionplanning)
    is_no_prediction_planning=true
    work_mode_num=$((work_mode_num+1))
    shift # past argument
    ;;
    -wp|--withplanning)
    is_with_planning=true
    work_mode_num=$((work_mode_num+1))
    shift # past value
    ;;
    -co|--cameraonly)
    is_camera_only=true
    work_mode_num=$((work_mode_num+1))
    shift # past value
    ;;
    -d|--dir)
    target_dir="$2"
    shift # past argument
    shift # past value
    ;;
    -h|--help)
    print_help
    exit 0
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

if [[ $work_mode_num -eq 0 ]]; then
   print_help
   exit 0
fi

set -- "${POSITIONAL[@]}" # restore positional parameters


function filter() {
    target=""
    name=$(basename $1)
    if $is_perfect_control; then
        target="$2/${name%.*}.pc.bag"
        rosbag filter $1 "$target" "$perfect_control_topic"

    fi

    if $is_no_prediction_planning; then
        target="$2/${name%.*}.npp.bag"
        rosbag filter $1 "$target" "not ($prediction_topic) and not ($planning_topic)"
    fi


    if $is_perception; then
        target="$2/${name%.*}.po.bag"
        rosbag filter $1 "$target" "$perception_topic"
    fi

    if $is_no_planning; then
        target="$2/${name%.*}.np.bag"
        rosbag filter $1 "$target" "$planning_deps"
    fi

    if $is_with_planning; then
        target="$2/${name%.*}.wp.bag"
        rosbag filter $1 "$target" "$planning_all"
    fi
    
    if $is_camera_only; then
	target="$2/${name%.*}.co.bag"
	rosbag filter $1 "$target" "$camera_only"
    fi
    echo "filtered ${bag} to $target"
}

for bag in $@; do
   folder=""
   if [ -z $target_dir ] ; then
     folder="$(dirname $bag)"
   else
      folder=$target_dir
   fi
   filter $bag $folder
done
