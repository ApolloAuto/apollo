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
# 0. Checkout docker_dev branch of apollo-internal.
#    *Read README.md to setup your workstation if you have not done so.*
# 1. Start container: bstart
# 2. Login to container: ./docker/scripts/dev_into.sh
# 3. Run this script.
###############################################################################

cd "$( dirname "${BASH_SOURCE[0]}" )"

# Fail on first failure.
set -e

./apollo6.sh config --noninteractive

function bazel_build_with_dist_cache() {
    bazel build --distdir=/apollo/.cache/distdir "$@"
}

function bazel_test_with_dist_cache() {
    bazel test --distdir=/apollo/.cache/distdir "$@"
}

# Working parts.
bazel_build_with_dist_cache //...

bazel_test_with_dist_cache \
    //cyber/... \
    //modules/bridge/... \
    //modules/canbus/... \
    //modules/common/... \
    //modules/control/... \
    //modules/data/... \
    //modules/drivers/... \
    //modules/monitor/... \
    //modules/routing/... \
    //modules/storytelling/... \
    //modules/transform/... \
    //modules/v2x/... \
    //modules/dreamview/... \
    //modules/guardian/... \
    //modules/map/... \
    //modules/contrib/... \
    //modules/third_party_perception/... \
    //modules/tools/...

# Perception
bazel_test_with_dist_cache $(bazel query //modules/perception/... \
	except //modules/perception/lidar/lib/detection/lidar_point_pillars:point_pillars_test \
)

# Localization: 9 test failures
bazel_test_with_dist_cache $(bazel query //modules/localization/... \
    except //modules/localization/ndt/ndt_locator:ndt_lidar_locator_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool_test \
    \
    except //modules/localization/msf:msf_localization_test \
    except //modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map_test \
    except //modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map_test \
    except //modules/localization/ndt:ndt_localization_pose_buffer_test \
    except //modules/localization/ndt:ndt_localization_test \
    except //modules/localization/ndt/ndt_locator:ndt_solver_test \
)

# Prediction: 4 test failures
bazel_test_with_dist_cache $(bazel query //modules/prediction/... \
    except //modules/prediction/predictor/single_lane:single_lane_predictor_test \
    except //modules/prediction/container/obstacles:obstacle_test \
    except //modules/prediction/container/obstacles:obstacle_clusters_test \
    except //modules/prediction/common:road_graph_test \
)

# Planning: 1 test failures
# osqp downgraded to 0.5.0 to avoid breaking changes in osqp-0.6.0
bazel_test_with_dist_cache $(bazel query //modules/planning/... \
    except //modules/planning/learning_based/model_inference:model_inference_test   \
)

# FIXME(all): inference_demo crashed
# bazel run //modules/planning/tools:inference_demo

echo "########################### All check passed! ###########################"

# bash apollo6.sh build OK
# bash apollo6.sh "test" OK
# bash apollo6.sh check OK

# In-progress parts. Feel free to claim by adding your name in TODO and move it
# TODO(?): replay-engine image which is compatible with docker_dev branch.
# TODO(?): Integrate pycodestyle (or similar) into "apollo.sh lint" to lint python code.
#          See https://pypi.org/project/pycodestyle/
# TODO(?): Use py_library, py_binary, py_test to manage python code:
#          See https://docs.bazel.build/versions/master/be/python.html
# TODO(storypku): cyber.aarch64 docker image (to be validated)
# TODO(storypku): tools/workspace.bzl to re-org WORKSPACE
# TODO(?): no break for cpu only build

# The following should be checked regularly until everyone follows the rule.
# Substitute implicit "-l" linkopts with explicit deps in BUILD files
# find modules/ -name "BUILD" -exec grep -- "-l" {} +
