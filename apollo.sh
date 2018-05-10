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

function source_apollo_base() {
  DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  cd "${DIR}"

  source "${DIR}/scripts/apollo_base.sh"
}

function apollo_check_system_config() {
  # check docker environment
  if [ ${MACHINE_ARCH} == "x86_64" ] && [ $(hostname) != "in_dev_docker" ] &&
       [ $(hostname) != "in_release_docker" ]; then
    echo -e "${RED}Must run $0 in dev docker or release docker${NO_COLOR}"
    exit 0
  fi

  # check operating system
  OP_SYSTEM=$(uname -s)
  case $OP_SYSTEM in
    "Linux")
      echo "System check passed. Build continue ..."

      # check system configuration
      DEFAULT_MEM_SIZE="2.0"
      MEM_SIZE=$(free | grep Mem | awk '{printf("%0.2f", $2 / 1024.0 / 1024.0)}')
      if (( $(echo "$MEM_SIZE < $DEFAULT_MEM_SIZE" | bc -l) )); then
         warning "System memory [${MEM_SIZE}G] is lower than minimum required memory size [2.0G]. Apollo build could fail."
      fi
      ;;
    "Darwin")
      warning "Mac OS is not officially supported in the current version. Build could fail. We recommend using Ubuntu 14.04."
      ;;
    *)
      error "Unsupported system: ${OP_SYSTEM}."
      error "Please use Linux, we recommend Ubuntu 14.04."
      exit 1
      ;;
  esac
}

function check_machine_arch() {
  # the machine type, currently support x86_64, aarch64
  MACHINE_ARCH=$(uname -m)

  # Generate WORKSPACE file based on marchine architecture
  if [ "$MACHINE_ARCH" == 'x86_64' ]; then
    sed "s/MACHINE_ARCH/x86_64/g" WORKSPACE.in > WORKSPACE
  elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
    sed "s/MACHINE_ARCH/aarch64/g" WORKSPACE.in > WORKSPACE
  else
    fail "Unknown machine architecture $MACHINE_ARCH"
    exit 1
  fi

  #setup vtk folder name for different systems.
  VTK_VERSION=$(find /usr/include/ -type d  -name "vtk-*" | tail -n1 | cut -d '-' -f 2)
  sed -i "s/VTK_VERSION/${VTK_VERSION}/g" WORKSPACE
}

function check_esd_files() {
  CAN_CARD="fake_can"

  if [ -f ./third_party/can_card_library/esd_can/include/ntcan.h \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4 \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4.0.1 ]; then
      USE_ESD_CAN=true
      CAN_CARD="esd_can"
  else
      warning "ESD CAN library supplied by ESD Electronics does not exist. If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md."
      USE_ESD_CAN=false
  fi
}

function generate_build_targets() {
  if [ -z $NOT_BUILD_PERCEPTION ] ; then
    BUILD_TARGETS=`bazel query //...`
  else
    info 'Skip building perception module!'
    BUILD_TARGETS=`bazel query //... except //modules/perception/... except //modules/calibration/lidar_ex_checker/...`
  fi

  if [ $? -ne 0 ]; then
    fail 'Build failed!'
  fi
  if ! $USE_ESD_CAN; then
     BUILD_TARGETS=$(echo $BUILD_TARGETS |tr ' ' '\n' | grep -v "esd")
  fi
  #skip msf for non x86_64 platforms
  if [ ${MACHINE_ARCH} != "x86_64" ]; then
     BUILD_TARGETS=$(echo $BUILD_TARGETS |tr ' ' '\n' | grep -v "msf")
  fi
}

#=================================================
#              Build functions
#=================================================

function build() {
  START_TIME=$(get_now)


  info "Start building, please wait ..."
  generate_build_targets
  info "Building on $MACHINE_ARCH..."

  MACHINE_ARCH=$(uname -m)
  JOB_ARG=""
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi
  echo "$BUILD_TARGETS" | xargs bazel build $JOB_ARG $DEFINES -c $@
  if [ $? -ne 0 ]; then
    fail 'Build failed!'
  fi

  # Build python proto
  build_py_proto

  # Clear KV DB and update commit_id after compiling.
  rm -fr data/kv_db
  python modules/tools/common/kv_db.py put \
      "apollo:data:commit_id" "$(git rev-parse HEAD)"

  if [ -d /apollo-simulator ] && [ -e /apollo-simulator/build.sh ]; then
    cd /apollo-simulator && bash build.sh build
    if [ $? -ne 0 ]; then
      fail 'Build failed!'
    fi
  fi
  if [ $? -eq 0 ]; then
    success 'Build passed!'
  fi
}

function cibuild() {
  START_TIME=$(get_now)

  echo "Start building, please wait ..."
  generate_build_targets
  echo "Building on $MACHINE_ARCH..."
  BUILD_TARGETS="
  //modules/control
  //modules/dreamview
  //modules/localization
  //modules/perception
  //modules/planning
  //modules/prediction
  //modules/routing
  "
  bazel build $DEFINES $@ $BUILD_TARGETS
  if [ $? -eq 0 ]; then
    success 'Build passed!'
  else
    fail 'Build failed!'
  fi
}

function apollo_build_dbg() {
  build "dbg" $@
}

function apollo_build_opt() {
  build "opt" $@
}

function build_py_proto() {
  if [ -d "./py_proto" ];then
    rm -rf py_proto
  fi
  mkdir py_proto
  PROTOC='./bazel-out/host/bin/external/com_google_protobuf/protoc'
  find modules/ -name "*.proto" \
      | grep -v node_modules \
      | grep -v modules/drivers/gnss \
      | xargs ${PROTOC} --python_out=py_proto
  find py_proto/* -type d -exec touch "{}/__init__.py" \;
}

function check() {
  local check_start_time=$(get_now)

  bash $0 build && bash $0 "test" && bash $0 lint

  START_TIME=$check_start_time
  if [ $? -eq 0 ]; then
    success 'Check passed!'
    return 0
  else
    fail 'Check failed!'
    return 1
  fi
}

function warn_proprietary_sw() {
  echo -e "${RED}The release built contains proprietary software provided by other parties.${NO_COLOR}"
  echo -e "${RED}Make sure you have obtained proper licensing agreement for redistribution${NO_COLOR}"
  echo -e "${RED}if you intend to publish the release package built.${NO_COLOR}"
  echo -e "${RED}Such licensing agreement is solely between you and the other parties,${NO_COLOR}"
  echo -e "${RED}and is not covered by the license terms of the apollo project${NO_COLOR}"
  echo -e "${RED}(see file license).${NO_COLOR}"
}

function release() {
  RELEASE_DIR="${HOME}/.cache/apollo_release"
  if [ -d "${RELEASE_DIR}" ]; then
    rm -rf "${RELEASE_DIR}"
  fi
  APOLLO_RELEASE_DIR="${RELEASE_DIR}/apollo"
  mkdir -p "${APOLLO_RELEASE_DIR}"

  # Find binaries and convert from //path:target to path/target
  BINARIES=$(bazel query "kind(cc_binary, //...)" | sed 's/^\/\///' | sed 's/:/\//')
  # Copy binaries to release dir.
  for BIN in ${BINARIES}; do
    SRC_PATH="bazel-bin/${BIN}"
    DST_PATH="${APOLLO_RELEASE_DIR}/${BIN}"
    if [ -e "${SRC_PATH}" ]; then
      mkdir -p "$(dirname "${DST_PATH}")"
      cp "${SRC_PATH}" "${DST_PATH}"
    fi
  done

  # modules data and conf
  CONFS=$(find modules/ -name "conf")
  DATAS=$(find modules/ -name "data")
  OTHER=("modules/tools"
         "modules/perception/model")
  rm -rf test/*
  for conf in $CONFS; do
    mkdir -p $APOLLO_RELEASE_DIR/$conf
    rsync -a $conf/* $APOLLO_RELEASE_DIR/$conf
  done
  for data in $DATAS; do
    mkdir -p $APOLLO_RELEASE_DIR/$data
    if [ $data != "modules/map/data" ]; then
      rsync -a $data/* $APOLLO_RELEASE_DIR/$data
    fi
  done
  # Other
  for path in "${OTHER[@]}"; do
    mkdir -p $APOLLO_RELEASE_DIR/$path
    rsync -a $path/* $APOLLO_RELEASE_DIR/$path
  done

  # dreamview frontend
  cp -a modules/dreamview/frontend $APOLLO_RELEASE_DIR/modules/dreamview

  # remove all pyc file in modules/
  find modules/ -name "*.pyc" | xargs -I {} rm {}

  # scripts
  cp -r scripts ${APOLLO_RELEASE_DIR}

  # lib
  LIB_DIR="${APOLLO_RELEASE_DIR}/lib"
  mkdir "${LIB_DIR}"
  if $USE_ESD_CAN; then
    warn_proprietary_sw
    for m in esd_can
    do
      cp third_party/can_card_library/$m/lib/* $LIB_DIR
    done
  fi
  cp -r bazel-genfiles/external $LIB_DIR
  cp -r py_proto/modules $LIB_DIR
  cp /apollo/modules/perception/cuda_util/cmake_build/libcuda_util.so $LIB_DIR

  # doc
  cp -r docs "${APOLLO_RELEASE_DIR}"
  cp LICENSE "${APOLLO_RELEASE_DIR}"
  cp third_party/ACKNOWLEDGEMENT.txt "${APOLLO_RELEASE_DIR}"

  # release info
  META="${APOLLO_RELEASE_DIR}/meta.ini"
  echo "git_commit: $(git rev-parse HEAD)" >> $META
  echo "car_type: LINCOLN.MKZ" >> $META
  echo "arch: ${MACHINE_ARCH}" >> $META
}

function gen_coverage() {
  bazel clean
  generate_build_targets
  echo "$BUILD_TARGETS" | grep -v "cnn_segmentation_test" | xargs bazel test $DEFINES -c dbg --config=coverage $@
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

  files=$(find bazel-out/local-opt/bin/modules/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external)
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
  START_TIME=$(get_now)

  generate_build_targets
  if [ "$USE_GPU" == "1" ]; then
    echo -e "${RED}Need GPU to run the tests.${NO_COLOR}"
    echo "$BUILD_TARGETS" | xargs bazel test $DEFINES --config=unit_test -c dbg --test_verbose_timeout_warnings $@
  else
    echo "$BUILD_TARGETS" | grep -v "cnn_segmentation_test" | grep -v "yolo_camera_detector_test" | xargs bazel test $DEFINES --config=unit_test -c dbg --test_verbose_timeout_warnings $@
  fi
  if [ $? -ne 0 ]; then
    fail 'Test failed!'
    return 1
  fi

  if [ -d /apollo-simulator ] && [ -e /apollo-simulator/build.sh ]; then
      cd /apollo-simulator && bash build.sh test
      if [ $? -ne 0 ]; then
        fail 'Test failed!'
        return 1
      fi
  fi

  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  fi
}

function citest() {
  START_TIME=$(get_now)
  BUILD_TARGETS="
  //modules/planning/integration_tests:garage_test
  //modules/planning/integration_tests:sunnyvale_loop_test
  //modules/control/integration_tests:simple_control_test
  //modules/prediction/container/obstacles:obstacle_test
  //modules/dreamview/backend/simulation_world:simulation_world_service_test
  "
  bazel test $DEFINES --config=unit_test --test_verbose_timeout_warnings $@ $BUILD_TARGETS
  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  else
    fail 'Test failed!'
    return 1
  fi
}

function run_cpp_lint() {
  generate_build_targets
  echo "$BUILD_TARGETS" | xargs bazel test --config=cpplint -c dbg
}

function run_bash_lint() {
  FILES=$(find "${APOLLO_ROOT_DIR}" -type f -name "*.sh" | grep -v ros)
  echo "${FILES}" | xargs shellcheck
}

function run_lint() {
  START_TIME=$(get_now)

  # Add cpplint rule to BUILD files that do not contain it.
  for file in $(find modules -name BUILD | \
    xargs grep -l -E 'cc_library|cc_test|cc_binary' | xargs grep -L 'cpplint()')
  do
    sed -i '1i\load("//tools:cpplint.bzl", "cpplint")\n' $file
    sed -i -e '$a\\ncpplint()' $file
  done

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
  START_TIME=$(get_now)

  local buildifier_url=https://github.com/bazelbuild/buildtools/releases/download/0.4.5/buildifier
  wget $buildifier_url -O ~/.buildifier
  chmod +x ~/.buildifier
  find . -name '*BUILD' -type f -exec ~/.buildifier -showlog -mode=fix {} +
  if [ $? -eq 0 ]; then
    success 'Buildify worked!'
  else
    fail 'Buildify failed!'
  fi
  rm ~/.buildifier
}

function build_fe() {
  cd modules/dreamview/frontend
  yarn build
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

function build_gnss() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  protoc modules/common/proto/error_code.proto --cpp_out=./
  protoc modules/common/proto/header.proto --cpp_out=./
  protoc modules/common/proto/geometry.proto --cpp_out=./

  protoc modules/localization/proto/imu.proto --cpp_out=./
  protoc modules/localization/proto/gps.proto --cpp_out=./
  protoc modules/localization/proto/pose.proto --cpp_out=./

  protoc modules/drivers/gnss/proto/gnss.proto --cpp_out=./
  protoc modules/drivers/gnss/proto/imu.proto --cpp_out=./
  protoc modules/drivers/gnss/proto/ins.proto --cpp_out=./ --python_out=./
  protoc modules/drivers/gnss/proto/config.proto --cpp_out=./
  protoc modules/drivers/gnss/proto/gnss_status.proto --cpp_out=./ --python_out=./
  protoc modules/drivers/gnss/proto/gpgga.proto --cpp_out=./
  protoc modules/drivers/gnss/proto/gnss_raw_observation.proto --cpp_out=./ --python_out=./
  protoc modules/drivers/gnss/proto/gnss_best_pose.proto --cpp_out=./ --python_out=./

  cd modules
  catkin_make_isolated --install --source drivers/gnss \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/common/proto/*.pb.cc
  rm -rf modules/common/proto/*.pb.h
  rm -rf modules/drivers/gnss/proto/*.pb.cc
  rm -rf modules/drivers/gnss/proto/*.pb.h
  rm -rf modules/drivers/gnss/proto/*_pb2.py
  rm -rf modules/localization/proto/*.pb.cc
  rm -rf modules/localization/proto/*.pb.h

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_velodyne() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/velodyne \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_usbcam() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/usb_cam \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function config() {
  ${APOLLO_ROOT_DIR}/scripts/configurator.sh
}

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/apollo.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build${NONE}: run build only
  ${BLUE}build_opt${NONE}: build optimized binary for the code
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_gnss${NONE}: build gnss driver
  ${BLUE}build_velodyne${NONE}: build velodyne driver
  ${BLUE}build_usbcam${NONE}: build usb camera driver
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend javascript code, this requires all the node_modules to be installed already
  ${BLUE}build_no_perception [dbg|opt]${NONE}: run build build skip building perception module, useful when some perception dependencies are not satisified, e.g., CUDA, CUDNN, LIDAR, etc.
  ${BLUE}build_prof${NONE}: build for gprof support.
  ${BLUE}buildify${NONE}: fix style of BUILD files
  ${BLUE}check${NONE}: run build/lint/test, please make sure it passes before checking in new code
  ${BLUE}clean${NONE}: run Bazel clean
  ${BLUE}config${NONE}: run configurator tool
  ${BLUE}coverage${NONE}: generate test coverage report
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}lint${NONE}: run code style check
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}release${NONE}: build release version
  ${BLUE}test${NONE}: run all unit tests
  ${BLUE}version${NONE}: display current commit and date
  "
}

function main() {
  source_apollo_base
  check_machine_arch
  apollo_check_system_config
  check_esd_files

  DEFINES="--define ARCH=${MACHINE_ARCH} --define CAN_CARD=${CAN_CARD} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"

  if [ ${MACHINE_ARCH} == "x86_64" ]; then
    DEFINES="${DEFINES} --copt=-mavx2"
  fi

  local cmd=$1
  shift

  case $cmd in
    check)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      check $@
      ;;
    build)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      apollo_build_dbg $@
      ;;
    build_prof)
      DEFINES="${DEFINES} --config=cpu_prof --cxxopt=-DCPU_ONLY"
      apollo_build_dbg $@
      ;;
    build_no_perception)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      NOT_BUILD_PERCEPTION=true
      if [ "$1" == "opt" ]; then
        shift
        apollo_build_opt $@
      elif [ "$1" == "dbg" ]; then
        shift
        apollo_build_dbg $@
      fi
      ;;
    cibuild)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      cibuild $@
      ;;
    build_opt)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      apollo_build_opt $@
      ;;
    build_gpu)
      DEFINES="${DEFINES} --cxxopt=-DUSE_CAFFE_GPU"
      apollo_build_dbg $@
      ;;
    build_opt_gpu)
      DEFINES="${DEFINES} --cxxopt=-DUSE_CAFFE_GPU"
      apollo_build_opt $@
      ;;
    build_fe)
      build_fe
      ;;
    buildify)
      buildify
      ;;
    build_gnss)
      build_gnss
      ;;
    build_py)
      build_py_proto
      ;;
    build_velodyne)
      build_velodyne
      ;;
    build_usbcam)
      build_usbcam
      ;;
    config)
      config
      ;;
    doc)
      gen_doc
      ;;
    lint)
      run_lint
      ;;
    test)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      run_test $@
      ;;
    citest)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      citest $@
      ;;
    test_gpu)
      DEFINES="${DEFINES} --cxxopt=-DUSE_CAFFE_GPU"
      USE_GPU="1"
      run_test $@
      ;;
    release)
      release 1
      ;;
    release_noproprietary)
      release 0
      ;;
    coverage)
      gen_coverage $@
      ;;
    clean)
      clean
      ;;
    version)
      version
      ;;
    usage)
      print_usage
      ;;
    *)
      print_usage
      ;;
  esac
}

main $@
