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


#=================================================
#                   Utils
#=================================================
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}"

source "${DIR}/scripts/apollo_base.sh"

# the machine type, currently support x86_64, aarch64
MACHINE_ARCH=$(uname -m)

IS_X86_64=false
IS_AARCH64=false

BUILD_TARGETS=""
TEST_TARGETS=""

rm -rf ./third_party/ros
if [ "$MACHINE_ARCH" == 'x86_64' ]; then
   IS_X86_64=true
elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
   IS_AARCH64=true
fi

if ! $IS_X86_64 && ! $IS_AARCH64 ; then
   fail "Unknown machine architecture $MACHINE_ARCH"
   exit 1
else
  ln -rs "$(pwd)/third_party/ros_$MACHINE_ARCH" "$(pwd)/third_party/ros"
fi

function check_esd_files() {
  if [ -f ./third_party/can_card_library/esd_can/include/ntcan.h \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4 \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4.0.1 ]; then
      USE_ESD_CAN=true
  else
      warning "${YELLOW}ESD CAN library supplied by ESD Electronics does not exit.${NO_COLOR}"
      warning "${YELLOW}If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md${NO_COLOR}"
      USE_ESD_CAN=false
  fi
}

check_esd_files

function generate_build_targets() {
  BUILD_TARGETS=$(bazel query //... | grep -v "_test$" | grep -v "third_party" \
    | grep -v "_cpplint$" | grep -v "release" | grep -v "kernel")
  if ! $USE_ESD_CAN; then
     BUILD_TARGETS=$(echo $BUILD_TARGETS |tr ' ' '\n' | grep -v "hwmonitor" | grep -v "esd")
  fi
}

function generate_test_targets() {
  TEST_TARGETS=$(bazel query //... | grep "_test$" | grep -v "third_party" | grep -v "kernel")
  if ! $USE_ESD_CAN; then
     TEST_TARGETS=$(echo $TEST_TARGETS| tr ' ' '\n' | grep -v "hwmonitor" | grep -v "esd")
  fi
}



source third_party/ros/setup.bash

#=================================================
#              Build functions
#=================================================


function apollo_build() {
  START_TIME=$(($(date +%s%N)/1000000))

  # Avoid query release directory.
  if [ -d release ];then
    rm -rf release
  fi
  generate_build_targets
  echo "Building on $MACHINE_ARCH, with targets:"
  echo "$BUILD_TARGETS"
  echo "$BUILD_TARGETS" | xargs bazel --batch --batch_cpu_scheduling build --define ARCH="$MACHINE_ARCH" --define CAN_CARD=${CAN_CARD} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN} -c dbg
  if [ $? -eq 0 ]; then
    success 'Build passed!'
  else
    fail 'Build failed!'
  fi
  find bazel-genfiles/* -type d -exec touch "{}/__init__.py" \;
}

function check() {
  local check_start_time=$START_TIME
  apollo_build && run_test && run_lint

  START_TIME=$check_start_time
  if [ $? -eq 0 ]; then
    success 'Check passed!'
    return 0
  else
    fail 'Check failed!'
    return 1
  fi
}

# print the given message in red txt with white background.
function echo_red_white() {
  echo -e "$(tput setaf 1)$(tput setab 7)"$1 "\e[0m"
}

function warn_proprietary_sw() {
  echo_red_white "The release built contains proprietary software provided by other parties. "
  echo_red_white "Make sure you have obtained proper licensing agreement for redistribution "
  echo_red_white "if you intend to publish the release package built. "
  echo_red_white "Such licensing agreement is solely between you and the other parties, "
  echo_red_white "and is NOT covered by the license terms of the Apollo project "
  echo_red_white "(see file LICENSE)."
}

function release() {
  ROOT_DIR=$HOME/.cache/release
  rm -rf $ROOT_DIR

  #modules
  MODULES_DIR=$ROOT_DIR/modules
  mkdir -p $MODULES_DIR
  for m in control canbus localization decision perception prediction planning
  do
    TARGET_DIR=$MODULES_DIR/$m
    mkdir -p $TARGET_DIR
    cp bazel-bin/modules/$m/$m $TARGET_DIR
    if [ -d modules/$m/conf ];then
        cp -r modules/$m/conf $TARGET_DIR
    fi
    if [ -d modules/$m/data ];then
        cp -r modules/$m/conf $TARGET_DIR
    fi
  done
  #control tools
  mkdir $MODULES_DIR/control/tools
  cp bazel-bin/modules/control/tools/enter_auto_mode $MODULES_DIR/control/tools
  cp bazel-bin/modules/control/tools/pad_terminal $MODULES_DIR/control/tools
  #remove all pyc file in modules/
  find modules/ -name "*.pyc" | xargs -I {} rm {}
  cp -r modules/tools $MODULES_DIR
  #ros
  cp -Lr third_party/ros $ROOT_DIR/
  #scripts
  cp -r scripts $ROOT_DIR
  #dreamview
  cp -Lr bazel-bin/modules/dreamview/dreamview.runfiles/apollo/modules/dreamview $MODULES_DIR
  cp -r modules/dreamview/conf $MODULES_DIR/dreamview
  #common data
  mkdir $MODULES_DIR/common
  cp -r modules/common/data $MODULES_DIR/common
  #hmi
  mkdir -p $MODULES_DIR/hmi/ros_node $MODULES_DIR/hmi/utils
  cp bazel-bin/modules/hmi/ros_node/ros_node_service $MODULES_DIR/hmi/ros_node/
  cp -r modules/hmi/conf $MODULES_DIR/hmi
  cp -r modules/hmi/web $MODULES_DIR/hmi
  cp -r modules/hmi/utils/*.py $MODULES_DIR/hmi/utils
  # lib
  LIB_DIR=$ROOT_DIR/lib
  mkdir $LIB_DIR
  if $USE_ESD_CAN; then
    for m in esd_can
    do
        cp third_party/can_card_library/$m/lib/* $LIB_DIR
    done
    #hw check
    mkdir -p $MODULES_DIR/monitor/hwmonitor/hw_check/
    cp bazel-bin/modules/monitor/hwmonitor/hw_check/can_check $MODULES_DIR/monitor/hwmonitor/hw_check/
    cp bazel-bin/modules/monitor/hwmonitor/hw_check/gps_check $MODULES_DIR/monitor/hwmonitor/hw_check/
    mkdir -p $MODULES_DIR/monitor/hwmonitor/hw/tools/
    cp bazel-bin/modules/monitor/hwmonitor/hw/tools/esdcan_test_app $MODULES_DIR/monitor/hwmonitor/hw/tools/
  fi
  cp -r bazel-genfiles/* $LIB_DIR
  # doc
  cp -r docs $ROOT_DIR
  cp LICENSE $ROOT_DIR
  cp third_party/ACKNOWLEDGEMENT.txt $ROOT_DIR
  #release info
  META=${ROOT_DIR}/meta.txt
  echo "Git commit: $(git show --oneline  -s | awk '{print $1}')" > $META
  echo "Build time: $TIME" >>  $META
  if $USE_ESD_CAN; then
    warn_proprietary_sw
  fi
}

function gen_coverage() {
  # Avoid query release directory.
  if [ -d release ];then
    rm -rf release
  fi
  bazel clean
  generate_test_targets
  echo "$TEST_TARGETS" | xargs bazel test --define ARCH="$(uname -m)" --define CAN_CARD=${CAN_CARD} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN} -c dbg --config=coverage
  if [ $? -ne 0 ]; then
    fail 'run test failed!'
  fi
  COV_DIR=data/cov
  rm -rf $COV_DIR
  files=$(find bazel-out/local-dbg/bin/modules/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external)
  for f in $files; do
    target="$COV_DIR/objs/modules/${f##*modules}"
    mkdir -p "$(dirname "$target")"
    cp "$f" "$target"
  done
  lcov --capture --directory "$COV_DIR/objs" --output-file "$COV_DIR/conv.info"
  if [ $? -ne 0 ]; then
    fail 'lcov failed!'
  fi
  lcov --remove "$COV_DIR/conv.info" \
      "external/*" \
      "/usr/*" \
      "bazel-out/*" \
      "*third_party/*" \
      "tools/*" \
      -o $COV_DIR/stripped_conv.info
  genhtml $COV_DIR/stripped_conv.info --output-directory $COV_DIR/report
  echo "Generated coverage report in $COV_DIR/report/index.html"
}

function run_test() {
  START_TIME=$(($(date +%s%N)/1000000))

  # Avoid query release directory.
  if [ -d release ];then
    rm -rf release
  fi
  # FIXME(all): when all unit test passed, switch back.
  # bazel test --config=unit_test -c dbg //...
  generate_test_targets
  echo "$TEST_TARGETS" | xargs bazel test --define "ARCH=$MACHINE_ARCH"  --define CAN_CARD=${CAN_CARD} --config=unit_test --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN} -c dbg --test_verbose_timeout_warnings
  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  else
    fail 'Test failed!'
    return 1
  fi
}

function run_cpp_lint() {
  bazel test --config=cpplint //...
}

function run_bash_lint() {
  FILES=$(find "${DIR}" -type f -name "*.sh" | grep -v ros | grep -v kernel)
  echo "${FILES}" | xargs shellcheck
}

function run_lint() {
  START_TIME=$(($(date +%s%N)/1000000))
  run_cpp_lint

  if [ $? -eq 0 ]; then
    success 'Lint passed!'
  else
    fail 'Lint failed!'
  fi
}

function clean() {
   bazel clean --async
}

function buildify() {
   wget \
   https://github.com/bazelbuild/buildtools/releases/download/0.4.5/buildifier \
   -O ~/.buildifier
   chmod +x ~/.buildifier
   find . -name BUILD -type f -exec ~/.buildifier -showlog -mode=fix {} +
   if [ $? -eq 0 ]; then
     success 'Buildify worked!'
   else
     fail 'Buildify failed!'
   fi
   rm ~/.buildifier
}

function print_usage() {
  echo 'Usage:
  ./apollo.sh [OPTION]'
  echo 'Options:
  build : run build only
  buildify: fix style of BUILD files
  check: run build/lint/test, please make sure it passes before checking in new code
  clean: runs Bazel clean
  coverage: generate test coverage report
  doc: generate doxygen document
  lint: run code style check
  print_usage: prints this menu
  release: to build release version
  test: run all the unit tests
  version: current commit and date
  '
}

function gen_doc() {
  rm -rf docs/doxygen
  doxygen apollo.doxygen
}

function version() {
  commit=$(git log -1 --pretty=%H)
  date=$(git log -1 --pretty=%cd)
  echo "Commit: ${commit}"
  echo "Date: ${date}"
}

function buildgnss() {
    CURRENT_PATH=$(pwd)
    MACHINE_ARCH="$(uname -m)"
    INSTALL_PATH="${CURRENT_PATH}/third_party/ros_${MACHINE_ARCH}"

    source "${INSTALL_PATH}/setup.bash"

    protoc modules/common/proto/error_code.proto --cpp_out=./
    protoc modules/common/proto/header.proto --cpp_out=./
    protoc modules/common/proto/geometry.proto --cpp_out=./

    protoc modules/localization/proto/imu.proto --cpp_out=./
    protoc modules/localization/proto/gps.proto --cpp_out=./
    protoc modules/localization/proto/pose.proto --cpp_out=./

    protoc modules/drivers/gnss/proto/gnss.proto --cpp_out=./
    protoc modules/drivers/gnss/proto/imu.proto --cpp_out=./
    protoc modules/drivers/gnss/proto/ins.proto --cpp_out=./
    protoc modules/drivers/gnss/proto/config.proto --cpp_out=./
    protoc modules/drivers/gnss/proto/gnss_status.proto --cpp_out=./
    protoc modules/drivers/gnss/proto/gpgga.proto --cpp_out=./

    cd modules
    catkin_make_isolated --install --source drivers \
        --install-space "${INSTALL_PATH}" -DCMAKE_BUILD_TYPE=Release \
        --cmake-args --no-warn-unused-cli
    find "${INSTALL_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
    cd -

    rm -rf modules/common/proto/*.pb.cc
    rm -rf modules/common/proto/*.pb.h
    rm -rf modules/drivers/gnss/proto/*.pb.cc
    rm -rf modules/drivers/gnss/proto/*.pb.h
    rm -rf modules/localization/proto/*.pb.cc
    rm -rf modules/localization/proto/*.pb.h

    rm -rf modules/.catkin_workspace
    rm -rf modules/build_isolated/
    rm -rf modules/devel_isolated/
}

case $1 in
  check)
    check
    ;;
  build)
    apollo_build
    ;;
  buildify)
    buildify
    ;;
  buildgnss)
    buildgnss
    ;;
  doc)
    gen_doc
    ;;
  lint)
    run_lint
    ;;
  test)
    run_test
    ;;
  release)
    release 1
    ;;
  release_noproprietary)
    release 0
    ;;
  coverage)
    gen_coverage
    ;;
  clean)
    clean
    ;;
  version)
    version
    ;;
  *)
    print_usage
    ;;
esac
