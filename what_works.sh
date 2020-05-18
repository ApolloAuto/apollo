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
cp WORKSPACE.in WORKSPACE

# Fail on first failure.
set -e

# Working parts.
bazel build \
    //cyber/... \
    //modules/bridge/... \
    //modules/canbus/... \
    //modules/common/... \
    //modules/control/... \
    //modules/monitor/... \
    //modules/routing/... \
    //modules/storytelling/... \
    //modules/v2x/...

bazel test \
    //cyber/... \
    //modules/bridge/... \
    //modules/canbus/... \
    //modules/common/... \
    //modules/control/... \
    //modules/monitor/... \
    //modules/routing/... \
    //modules/storytelling/... \
    //modules/v2x/...

# bazel build //modules/transform/...
# bazel test //modules/transform/...
# bash scripts/install_esdcan_library.sh install
# bazel build //modules/drivers/...
# bazel test //modules/drivers/...
# bash scripts/install_esdcan_library.sh uninstall

bazel build //modules/tools/...
# Note(storypku): bazel test works except some lint errors in cyber_visualizer.
# Will re-check if cyber_visualizer work correctly once it becomes stable
bazel test $(bazel query //modules/tools/... except //modules/tools/visualizer/...)

# TODO(storypku): Add grpc support in proto_build_generator.py

# In-progress parts. Feel free to claim by adding your name in TODO and move it
# above when you finish.

# TODO(?): bazel build //modules/data/...
# TODO(?): bazel build //modules/contrib/...
# TODO(xiaoxq): bazel build //modules/data/...
# TODO(changsh726): bazel build //modules/dreamview/...
# TODO(storypku): bazel build //modules/guardian/...
# TODO(storypku): bazel build //modules/planning/...
# TODO(storypku): bazel build //modules/map/...
# TODO(?): bazel build //modules/localization/...
# TODO(?): bazel build //modules/perception/...
# TODO(?): bazel build //modules/prediction/...
# TODO(?): bazel build //modules/third_party_perception/...

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
