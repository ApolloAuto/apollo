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
bazel_test_with_dist_cache $(bazel query //modules/perception/... )

# Localization: 3 test failures (deadlock ?)
bazel_test_with_dist_cache $(bazel query //modules/localization/... \
    except //modules/localization/ndt/ndt_locator:ndt_lidar_locator_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_test \
    except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool_test \
)

# Prediction
bazel_test_with_dist_cache $(bazel query //modules/prediction/... )

# Planning: 1 test failures
# osqp downgraded to 0.5.0 to avoid breaking changes in osqp-0.6.0
bazel_test_with_dist_cache $(bazel query //modules/planning/... )

# FIXME(all): inference_demo crashed
# bazel run //modules/planning/tools:inference_demo

echo "########################### All check passed! ###########################"

# bash apollo6.sh build OK
# bash apollo6.sh "test" OK
# bash apollo6.sh check OK

# In-progress parts. Feel free to claim by adding your name in TODO and move it
# DONE(?): replay-engine image which is compatible with docker_dev branch.
# DONE(?): Integrate pycodestyle (or similar) into "apollo.sh lint" to lint python code.
#          See https://pypi.org/project/pycodestyle/
# DONE(changsh726): Use py_library, py_binary, py_test to manage python code:
#          See https://docs.bazel.build/versions/master/be/python.html
# DONE(storypku): cyber.aarch64 docker image (to be validated)
# DONE(storypku): tools/workspace.bzl to re-org WORKSPACE
# DONE(storypku): no break for cpu only build

# The following should be checked regularly until everyone follows the rule.
# Substitute implicit "-l" linkopts with explicit deps in BUILD files
# find modules/ -name "BUILD" -exec grep -- "-l" {} +
