#!/usr/bin/env bash

setup_env() {
  # Use internally-maintained ROS release.
  WORK_ROOT="../../../../.."
  CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CURR_DIR/$WORK_ROOT/baidu/adu-3rd/protobuf/lib
  export LD_LIBRARY_PATH
  LIBRARY_PATH=$LIBRARY_PATH:$CURR_DIR/$WORK_ROOT/baidu/adu-3rd/protobuf/lib
  export LIBRARY_PATH
}

show_info() {
  echo "-------------------"
  echo "Attention:"
  echo ""
  echo "-------------------"
  read -p "Press Y to start, others to exit! Input: " val

  if [ -z $val ]; then
    exit -1
  fi

  if [ $val != 'Y' ] && [ $val != 'y' ]; then
    exit 0
  fi
}

setup_env

set -x

project_dir=$CURR_DIR/..

# bin file for recorder
bin_file_path="${project_dir}/devel/lib/arbiter/topo_creator"

# directory of base map file
base_map_dir="/home/caros/adu_data/map"

# name of base map file
base_map_name="base_map.bin"

# path of routing topology file to be dumped
dump_topo_path="/home/fuxiaoxin/Desktop/routing_map.bin"

set +x

show_info 

set -x

GLOG_logtostderr=1 GLOG_minloglevel=0 ${bin_file_path} \
        --base_map_dir=${base_map_dir} \
        --base_map_name=${base_map_name} \
        --dump_topo_path=${dump_topo_path}

