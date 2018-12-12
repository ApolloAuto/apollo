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

SRC_DIR=$1
TARGET_DIR=$2

set -e

# TODO(xiaoxq): Remove after rebuilding docker image.
sudo pip install h5py

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash 

FLAGFILE=/apollo/modules/prediction/conf/prediction.conf
echo "--prediction_offline_mode" >> ${FLAGFILE}
echo "--prediction_offline_bags=${SRC_DIR}" >> ${FLAGFILE}
echo "--prediction_data_dir=${TARGET_DIR}" >> ${FLAGFILE}
echo "--junction_distance_threshold=30.0" >> ${FLAGFILE}
echo "--noenable_prioritize_obstacles" >> ${FLAGFILE}

sudo mkdir -p ${TARGET_DIR}
# sudo chown apollo:apollo ${TARGET_DIR}
cyber_launch start /apollo/modules/prediction/launch/prediction.launch

for b in $(find ${TARGET_DIR} -iname "*.bin"); do
  if [ -e "$b.junction.label" ]; then
     echo "skip existing $b.junction.label"
  else
     echo "Generating junction labels for $b"
     python modules/tools/prediction/mlp_train/generate_junction_labels.py \
         $b $b.junction.label
  fi

  if [ -e "$b.junction.label.junction.h5" ]; then
    echo "skip existing $b.junction.label.junction.h5"
  else
    echo "Generating junction h5 for $b"
    python modules/tools/prediction/mlp_train/generate_junction_h5.py \
        $b.junction.label $b.junction.label.junction.h5
  fi
done

for b in $(find ${TARGET_DIR} -iname "*.bin"); do
  if [ -e "$b.cruise.label" ]; then
    echo "skip existing $b.cruise.label"
  else
    echo "Generating cruise labels for $b"
    python modules/tools/prediction/mlp_train/generate_cruise_labels.py \
        $b $b.cruise.label
  fi

  if [ -e "$b.cruise.label.cruise.h5" ]; then
    echo "skip existing $b.cruise.label.cruise.h5"
  else
    echo "Generating cruise h5 for $b"
    python modules/tools/prediction/mlp_train/generate_cruise_h5.py \
        $b.cruise.label $b.cruise.label.cruise.h5
  fi
done
