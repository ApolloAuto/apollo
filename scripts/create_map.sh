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

if [ "$1" == "" ]; then
    echo "Must designate a map name"
else
    echo "Generating map ${MAP}"
	MAP=$1
	rm -rf modules/map/data/${MAP}
	mkdir modules/map/data/${MAP}
	if [ "$2" == "" ]; then
	    echo "Gererating map with a single lane"
		python ./modules/tools/create_map/create_map.py -i /tmp/lane.csv -o modules/map/data/${MAP}/base_map.txt -e modules/map/data/${MAP}/default_end_way_point.txt
	else
		LEFT_LANES=$2
		RIGHT_LANES=$3
	    echo "Gererating map with one center lane, ${LEFT_LANES} left lane(s), ${RIGHT_LANES} right lane(s)"
		python ./modules/tools/create_map/create_map.py -i /tmp/lane.csv -o modules/map/data/${MAP}/base_map.txt -e modules/map/data/${MAP}/default_end_way_point.txt --left_lanes ${LEFT_LANES} --right_lanes ${RIGHT_LANES}
	fi
	echo "--map_dir=modules/map/data/${MAP}" >> modules/common/data/global_flagfile.txt
	./scripts/generate_routing_topo_graph.sh 
	./bazel-bin/modules/map/tools/sim_map_generator --map_dir=modules/map/data/${MAP} --output_dir=modules/map/data/${MAP}
fi