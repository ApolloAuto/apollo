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
  echo "Example: filter_planning.sh -[pc|np|wp] [-d|--dir] record1 record2 record3 ..."
  echo "-d|--dir the target storage directory"
  echo "This script works in three modes:"
  echo "  -pc filter for perfect control, produces *.pc.record"
  echo "  -po filter only for perception topic, produces *.po.record"
  echo "  -np filter for planning dependencies, produces *.np.record"
  echo "  -wp filter for planning and its dependencies, produces *.wp.record"
  echo "  -pn filter out planning and prediction, produces *.npp.record"
  echo "  -co filter out perception, prediction and planning, produces *.co.record"
  echo "  -ni filter out pointcloud and images, produces *.ni.record"
}

routing_topic="/apollo/routing_response"
planning_topic="/apollo/planning"
prediction_topic="/apollo/prediction"
drive_event_topic="/apollo/drive_event"
pointcloud_topic="/apollo/sensor/velodyne64/compensator/PointCloud2"

perception_topic=(
  '/apollo/perception/obstacles'
  '/apollo/perception/traffic_light'
)

perfect_control_topic=()
perfect_control_topic+=${perception_topic[@]}
perfect_control_topic+=($routing_topic)
perfect_control_topic+=("$prediction_topic")
perfect_control_topic+=("/apollo/perception/traffic_light")

planning_deps=()
planning_deps+=${perfect_control_topic[@]}
planning_deps+=('/apollo/canbus/chassis')
planning_deps+=('/apollo/localization/pose')
planning_deps+=('/apollo/navigation')
planning_deps+=('/apollo/guardian')
planning_deps+=('/apollo/monitor/system_status')
planning_deps+=('/apollo/relative_map')

image_topic=(
  '/apollo/sensor/camera/traffic/image_long'
  '/apollo/sensor/camera/traffic/image_short'
)

planning_all=()
planning_all+=${planning_deps[@]}
planning_all+=($planning_topic)
planning_all+=($drive_event_topic)

#Three different filter mode
#create perfect control mode record
is_perfect_control=false
#create a rosrecord with planning and its dependencies
is_with_planning=false
#create a rosrecord only with planning's dependencies
is_no_planning=false

#only perception topic
is_perception=false

#no prediction and no planning
is_no_prediction_planning=false

#no perception, no prediction and no planning, with only camera topic
is_camera_only=false

# no pointcloud, no image
is_no_pointcloud_image=false

work_mode_num=0

#argument parsing code from https://stackoverflow.com/a/14203146
POSITIONAL=()
target_dir=""
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -po | --perception_only)
      is_perception=true
      work_mode_num=$((work_mode_num + 1))
      shift # past argument
      ;;
    -pc | --perfectcontrol)
      is_perfect_control=true
      work_mode_num=$((work_mode_num + 1))
      shift # past argument
      ;;
    -np | --noplanning)
      is_no_planning=true
      work_mode_num=$((work_mode_num + 1))
      shift # past argument
      ;;
    -pn | --nopredictionplanning)
      is_no_prediction_planning=true
      work_mode_num=$((work_mode_num + 1))
      shift # past argument
      ;;
    -wp | --withplanning)
      is_with_planning=true
      work_mode_num=$((work_mode_num + 1))
      shift # past value
      ;;
    -co | --cameraonly)
      is_camera_only=true
      work_mode_num=$((work_mode_num + 1))
      shift # past value
      ;;
    -ni | --noimage)
      is_no_pointcloud_image=true
      work_mode_num=$((work_mode_num + 1))
      shift # past argument
      ;;
    -d | --dir)
      target_dir="$2"
      shift # past argument
      shift # past value
      ;;
    -h | --help)
      print_help
      exit 0
      ;;
    *)                   # unknown option
      POSITIONAL+=("$1") # save it in an array for later
      shift              # past argument
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
  ext=${name##*.}
  name="$name.$ext"
  if $is_perfect_control; then
    target="$2/${name%.*}.pc.record"
    cyber_recorder split -f $1 $(echo ${perfect_control_topic[@]} | sed 's/^\| / -c /g') -o $target
  fi

  if $is_no_prediction_planning; then
    target="$2/${name%.*}.npp.record"
    cyber_recorder split -f $1 -k $prediction_topic -k $planning_topic -o $target
  fi

  if $is_perception; then
    target="$2/${name%.*}.po.record"
    cyber_recorder split -f $1 $(echo ${perception_topic[@]} | sed 's/^\| / -k /g') -o $target
  fi

  if $is_no_planning; then
    target="$2/${name%.*}.np.record"
    cyber_recorder split -f $1 -k $planning_topic -o $target
  fi

  if $is_with_planning; then
    target="$2/${name%.*}.wp.record"
    cyber_recorder split -f $1 $(echo ${planning_all[@]} | sed 's/^\| / -c /g') -o $target
  fi

  if $is_camera_only; then
    target="$2/${name%.*}.co.record"
    cyber_recorder split -f $1 $(echo ${image_topic[@]} | sed 's/^\| / -c /g') -o $target
  fi

  if $is_no_pointcloud_image; then
    target="$2/${name%.*}.ni.record"
    cyber_recorder split -f $1 $(echo ${pointcloud_topic} ${image_topic[@]} | sed 's/^\| / -k /g') -o $target
  fi

  echo "filtered ${record} to $target"
}

for record in $@; do
  folder=""
  if [ -z $target_dir ]; then
    folder="$(dirname $record)"
  else
    folder=$target_dir
  fi
  filter $record $folder
done
