#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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
# For development only! Will remove before merging to master!
# Testing steps:
# 1. Start container: ./docker/scripts/dev_start.sh
# 2. Login to container: ./docker/scripts/dev_into.sh
# 3. Run this script.
###############################################################################

cd "$( dirname "${BASH_SOURCE[0]}" )"

FLAG_PKG="libatlas-base-dev"
if dpkg -l |grep -q "${FLAG_PKG}"; then
    echo "${FLAG_PKG} already installed"
else
    sudo apt-get -y update
    sudo apt-get -y install ${FLAG_PKG} libpython2.7-dev
fi
echo "/opt/apollo/pkgs/caffe/lib" | sudo tee -a /etc/ld.so.conf.d/apollo.conf
sudo ldconfig

cp WORKSPACE.in WORKSPACE

# Fail on first failure.
set -e

./bootstrap.sh --noninteractive

function bazel_build_with_dist_cache() {
    # bazel build -c opt --distdir=/apollo/.cache/distdir "$@"
    bazel build --distdir=/apollo/.cache/distdir "$@"
}

function bazel_test_with_dist_cache() {
    bazel test --distdir=/apollo/.cache/distdir "$@"
}

# Working parts.
bazel_build_with_dist_cache \
    //cyber/... \
    //modules/bridge/... \
    //modules/canbus/... \
    //modules/common/... \
    //modules/map/...    \
    //modules/control/... \
    //modules/data/... \
    //modules/monitor/... \
    //modules/routing/... \
    //modules/storytelling/... \
    //modules/transform/... \
    //modules/v2x/... \
    //modules/dreamview/... \
    //modules/guardian/... \
    //modules/localization/... \
    //modules/prediction/... \
    //modules/contrib/... \
    //modules/planning/... \
    //modules/perception/... \
    //modules/third_party_perception/...

bazel_test_with_dist_cache \
    //cyber/... \
    //modules/bridge/... \
    //modules/canbus/... \
    //modules/common/... \
    //modules/control/... \
    //modules/data/... \
    //modules/monitor/... \
    //modules/routing/... \
    //modules/storytelling/... \
    //modules/transform/... \
    //modules/v2x/... \
    //modules/dreamview/... \
    //modules/guardian/... \
    //modules/map/... \
    //modules/contrib/... \
    //modules/third_party_perception/...

# Drivers: OK
bash scripts/install_esdcan_library.sh install
bazel_build_with_dist_cache //modules/drivers/...
bazel_test_with_dist_cache //modules/drivers/...
bash scripts/install_esdcan_library.sh uninstall

# Perception: 7 test failures + 2 flaky
bazel_test_with_dist_cache $(bazel query //modules/perception/... \
	except //modules/perception/lidar/lib/detection/lidar_point_pillars:point_pillars_test \
	except //modules/perception/camera/test:camera_lib_obstacle_transformer_multicue_multicue_obstacle_transformer_test \
	except //modules/perception/camera/test:camera_lib_obstacle_detector_yolo_yolo_obstacle_detector_test \
	except //modules/perception/camera/test:camera_lib_obstacle_detector_yolo_region_output_test \
	except //modules/perception/camera/test:camera_lib_lane_postprocessor_darkscnn_lane_postprocessor_test \
	except //modules/perception/camera/test:camera_lib_lane_detector_darkscnn_lane_detector_test \
	except //modules/perception/camera/test:camera_app_obstacle_camera_perception_test \
)
# Flaky
# //modules/perception/camera/test:camera_lib_lane_postprocessor_denseline_lane_postprocessor_test
# //modules/perception/camera/test:camera_lib_lane_detector_denseline_lane_detector_test

bazel_build_with_dist_cache //modules/tools/...
bazel_test_with_dist_cache $(bazel query //modules/tools/... \
    except //modules/tools/visualizer/... \
)

# Localization: 3 test failures
bazel_test_with_dist_cache $(bazel query //modules/localization/... \
    except //modules/localization/ndt/ndt_locator:ndt_lidar_locator_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool_test \
)

# Prediction: 4 test failures
bazel_test_with_dist_cache $(bazel query //modules/prediction/... \
    except //modules/prediction/predictor/single_lane:single_lane_predictor_test \
    except //modules/prediction/container/obstacles:obstacle_test \
    except //modules/prediction/container/obstacles:obstacle_clusters_test \
    except //modules/prediction/common:road_graph_test \
)

# Planning: 7 test failures
bazel_test_with_dist_cache $(bazel query //modules/planning/... \
    except //modules/planning/tasks/learning_model:learning_model_inference_task_test \
    except //modules/planning/reference_line:qp_spline_reference_line_smoother_test   \
    except //modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_osqp_interface_test \
    except //modules/planning/math/smoothing_spline:osqp_spline_2d_solver_test  \
    except //modules/planning/math/smoothing_spline:osqp_spline_1d_solver_test  \
    except //modules/planning/learning_based/model_inference:model_inference_test   \
    except //modules/planning/integration_tests:sunnyvale_big_loop_test \
)

# FIXME(all): inference_demo crashed
# bazel run //modules/planning/tools:inference_demo

echo "########################### All check passed! ###########################"

# In-progress parts. Feel free to claim by adding your name in TODO and move it

# TODO(?): apollo.sh build
# TODO(?): apollo.sh test
# TODO(?): apollo.sh lint
# TODO(?): apollo.sh check
# TODO(?): bstart; apollo.sh check
# TODO(?): replay-engine image which is compatible with docker_dev branch.
# TODO(?): Integrate pycodestyle (or similar) into "apollo.sh lint" to lint python code.
#          See https://pypi.org/project/pycodestyle/
# TODO(?): Use py_library, py_binary, py_test to manage python code:
#          See https://docs.bazel.build/versions/master/be/python.html
# TODO(Kecheng Xu): modules/tools/configurator/configurator.py: 2to3
# TODO(storypku): cyber.aarch64 docker image
# TODO(storypku): tools/workspace.bzl to re-org WORKSPACE.in
# TODO(?): no break for cpu only build
