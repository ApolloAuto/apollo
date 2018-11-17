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

ROOT_DIR="/apollo/modules/perception/obstacle/lidar/segmentation/cnnseg"
DATA_DIR="/apollo/modules/perception/data/cnnseg_test"
DEFAULT_PCD_NAME="uscar_12_1470770225_1470770492_1349"

# build write_img tool
function build() {
  g++ ${ROOT_DIR}/write_img.cc -o ${ROOT_DIR}/write_img -lgflags -lopencv_core -lopencv_highgui -lpthread
}

# clean write_img exe
function clean() {
  rm -f ${ROOT_DIR}/write_img
  if [ $# == 0 ]
  then
    rm -f ${DATA_DIR}/${DEFAULT_PCD_NAME}-detection.png
  elif [ $# == 1 ]
  then
    rm -f ${DATA_DIR}/$1-detection.png
  else
    echo "[clean] Too many parameters"
  fi
}

# run write_img exe
function run() {
  if [ $# == 0 ]
  then
    ${ROOT_DIR}/write_img
  elif [ $# == 1 ]
  then
    ${ROOT_DIR}/write_img --pcd_name=$1
  else
    echo "[run] Too many parameters"
  fi
}

function main() {
  case $1 in
    clean)
      if [ $# == 1 ]
      then
        clean
      elif [ $# == 2 ]
      then
        clean $2
      else
        echo "Too many pcd files specified!"
      fi
      ;;
    build)
      build
      ;;
    run)
      if [ $# == 1 ]
      then
        clean
        build
        run
      elif [ $# == 2 ]
      then
        clean $2
        build
        run $2
      else
        echo "Too many pcd files specified!"
      fi
      ;;
    *)
      echo "Please input an option: clean|build|run"
      ;;
  esac
}

main $@