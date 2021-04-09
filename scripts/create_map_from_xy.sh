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

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

print_help() {
   echo "$0 --xy [--map_name MAP_NAME][--left_lane_num 0][--right_lane_num 0]"
   echo "$0 --xy set the input xy file, file that contains x,y"
   echo "$0 --map_name set the name of the map, default is the base name of the csv file"
   #echo "$0 --left_lane_num set the number of lanes at the left side of the trajectory, default is zero"
   #echo "$0 --right_lane_num set the number of lanes at the right side of the trajectory, default is zero"
}

POSITIONAL=()
g_map_name=""
g_xy_file=""
#left_lane_num="0"
#right_lane_num="0"
while [[ $# -gt 0 ]]; do
key="$1"
case $key in
    -d|--dir)
    target_dir="$2"
    shift # past argument
    shift # past value
    ;;
    --xy)
    g_xy_file="$2"
    shift # past argument
    shift # past value
    ;;
    #-l|--left_lane_num)
    #left_lane_num="$2"
    #shift # past argument
    #shift # past value
    #;;
    #-r|--right_lane_num)
    #right_lane_num="$2"
    #shift # past argument
    #shift # past value
    #;;
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

if [ "$g_xy_file" == "" ]; then
  echo "argument --xy is required"
  print_help
  exit 0
fi

if [ "$map_name" == "" ]; then
    map_name=$(basename $g_xy_file)
    map_name="${map_name%.*}"
fi

generate_map() {
    xy_file=$1
    dir_name="${APOLLO_ROOT_DIR}/modules/map/data/$2"
    #left_num=$3
    #right_num=$4
    if [ -d ${dir_name} ]; then
        rm -rf ${dir_name}.last
        mv ${dir_name} ${dir_name}.last
    fi

    mkdir -p ${dir_name}
    python ${APOLLO_ROOT_DIR}/modules/tools/map_gen/map_gen_single_lane.py $xy_file $dir_name/base_map.txt 1.0
    echo "--map_dir=${dir_name}" >> modules/common/data/global_flagfile.txt
    bash ${APOLLO_ROOT_DIR}/scripts/generate_routing_topo_graph.sh 
    ${APOLLO_ROOT_DIR}/bazel-bin/modules/map/tools/sim_map_generator --map_dir=${dir_name} --output_dir=${dir_name}
}

generate_map $g_xy_file $map_name $left_lane_num $right_lane_num
