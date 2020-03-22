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

DISABLED_CYBER_MODULES="except //cyber/record:record_file_integration_test"

function apollo_check_system_config() {
  # check docker environment
  if [ ${MACHINE_ARCH} == "x86_64" ] && [ ${APOLLO_IN_DOCKER} != "true" ]; then
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
  if [ "${MACHINE_ARCH}" != "x86_64" ] && [ "${MACHINE_ARCH}" != "aarch64" ]; then
    fail "Unknown machine architecture $MACHINE_ARCH"
    exit 1
  fi

  #setup vtk folder name for different systems.
  VTK_VERSION=$(find /usr/include/ -type d  -name "vtk-*" | tail -n1 | cut -d '-' -f 2)
  sed "s/VTK_VERSION/${VTK_VERSION}/g" WORKSPACE.in > WORKSPACE
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
  COMMON_TARGETS="//cyber/... union //modules/common/kv_db/... union //modules/dreamview/... $DISABLED_CYBER_MODULES"
  case $BUILD_FILTER in
  cyber)
    BUILD_TARGETS=`bazel query //cyber/... union //modules/tools/visualizer/...`
    ;;
  drivers)
    BUILD_TARGETS=`bazel query //cyber/... union //modules/tools/visualizer/... union //modules/drivers/... except //modules/drivers/tools/... except //modules/drivers/canbus/... except //modules/drivers/video/...`
    ;;
  control)
    BUILD_TARGETS=`bazel query $COMMON_TARGETS union //modules/control/... `
    ;;
  planning)
    BUILD_TARGETS=`bazel query $COMMON_TARGETS union //modules/routing/... union //modules/planning/...`
    ;;
  prediction)
    BUILD_TARGETS=`bazel query $COMMON_TARGETS union //modules/routing/... union //modules/prediction/...`
    ;;
  pnc)
    BUILD_TARGETS=`bazel query $COMMON_TARGETS union //modules/routing/... union //modules/prediction/... union //modules/planning/... union //modules/control/...`
    ;;
  no_perception)
    BUILD_TARGETS=`bazel query //modules/... except //modules/perception/... union //cyber/...`
    ;;
  *)
#    BUILD_TARGETS=`bazel query //modules/... union //cyber/...`
    # FIXME(all): temporarily disable modules doesn't compile in 18.04
    BUILD_TARGETS=`bazel query //modules/... union //cyber/... except //modules/tools/visualizer/... except //modules/v2x/... except //modules/map/tools/map_datachecker/... $DISABLE_CYBER_MODULES`
  esac

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
  if [ "${USE_GPU}" = "1" ] ; then
    echo -e "${YELLOW}Running build under GPU mode. GPU is required to run the build.${NO_COLOR}"
  else
    echo -e "${YELLOW}Running build under CPU mode. No GPU is required to run the build.${NO_COLOR}"
  fi
  info "Start building, please wait ..."

  generate_build_targets
  info "Building on $MACHINE_ARCH..."

  MACHINE_ARCH=$(uname -m)
  JOB_ARG="--jobs=$(nproc) --ram_utilization_factor 80"
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi
  info "Building with $JOB_ARG for $MACHINE_ARCH"

  bazel build $JOB_ARG $DEFINES -c $@ $BUILD_TARGETS
  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'Build failed!'
  fi

  # Build python proto
  build_py_proto

  # Clear KV DB and update commit_id after compiling.
  if [ "$BUILD_FILTER" == 'cyber' ] || [ "$BUILD_FILTER" == 'drivers' ]; then
    info "Skipping revision recording"
  else
    bazel build $JOB_ARG $DEFINES -c $@ $BUILD_TARGETS
    if [ ${PIPESTATUS[0]} -ne 0 ]; then
      fail 'Build failed!'
    fi
    rm -fr data/kv_db*
    REVISION=$(get_revision)
    ./bazel-bin/modules/common/kv_db/kv_db_tool --op=put \
        --key="apollo:data:commit_id" --value="$REVISION"
  fi

  if [ -d /apollo-simulator ] && [ -e /apollo-simulator/build.sh ]; then
    cd /apollo-simulator && bash build.sh build
    if [ $? -ne 0 ]; then
      fail 'Build failed!'
    fi
  fi
  if [ $? -eq 0 ]; then
    success 'Build passed!'
  else
    fail 'Build failed'
  fi
}

function cibuild_extended() {
  info "Building framework ..."

  cd /apollo
  info "Building modules ..."

  JOB_ARG="--jobs=12"
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi

  info "Building with $JOB_ARG for $MACHINE_ARCH"

  # FIXME(all): temporarily disable modules doesn't compile in 18.04
  BUILD_TARGETS=`
    bazel query //cyber/... \
    union //modules/perception/... \
    union //modules/dreamview/... \
    union //modules/drivers/radar/conti_radar/... \
    union //modules/drivers/radar/racobit_radar/... \
    union //modules/drivers/radar/ultrasonic_radar/... \
    union //modules/drivers/gnss/... \
    union //modules/drivers/velodyne/... \
    union //modules/drivers/camera/... \
    union //modules/guardian/... \
    union //modules/localization/... \
    union //modules/map/... \
    union //modules/third_party_perception/... \
    except //modules/map/tools/map_datachecker/...
    `

  bazel build $JOB_ARG $DEFINES $@ $BUILD_TARGETS

  if [ $? -eq 0 ]; then
    success 'Build passed!'
  else
    fail 'Build failed!'
  fi
}
function cibuild() {
  info "Building framework ..."
  cd /apollo

  info "Building modules ..."

  JOB_ARG="--jobs=12"
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi

  info "Building with $JOB_ARG for $MACHINE_ARCH"
  BUILD_TARGETS="
    //modules/canbus/...
    //modules/common/...
    //modules/control/...
    //modules/planning/...
    //modules/prediction/...
    //modules/routing/...
    //modules/transform/..."

  # The data module is lightweight and rarely changed. If it fails, it's
  # most-likely an environment mess. So we try `bazel clean` and then initial
  # the building process.
  bazel build $JOB_ARG $DEFINES $@ "//modules/data/..." || bazel clean

  bazel build $JOB_ARG $DEFINES $@ $BUILD_TARGETS

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
  # TODO(xiaoxq): Retire this as we are using bazel to compile protos into bazel-genfiles.
  if [ -d "./py_proto" ];then
    rm -rf py_proto
  fi
  mkdir py_proto
  find modules/ cyber/ -name "*.proto" \
      | grep -v node_modules \
      | xargs protoc --python_out=py_proto
  find modules/ cyber/ -name "*_service.proto" \
      | grep -v node_modules \
      | xargs python -m grpc_tools.protoc --proto_path=. --python_out=py_proto --grpc_python_out=py_proto
  find py_proto/* -type d -exec touch "{}/__init__.py" \;
}

function check() {
  bash $0 build && bash $0 "test" && bash $0 lint

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

  # cp built dynamic libraries
  LIBS=$(find bazel-out/* -name "*.so" )
  for LIB in ${LIBS}; do
    cp -a --parent ${LIB} ${APOLLO_RELEASE_DIR}
  done
  mkdir ${APOLLO_RELEASE_DIR}/bazel-bin
  mv ${APOLLO_RELEASE_DIR}/bazel-out/local-opt/bin/* ${APOLLO_RELEASE_DIR}/bazel-bin/
  rm -rf ${APOLLO_RELEASE_DIR}/bazel-out

  # reset softlinks
  cd ${APOLLO_RELEASE_DIR}/bazel-bin
  LIST=("_solib_k8")
  for DIR in "${LIST[@]}"; do
    LINKS=$(find ${DIR}/* -name "*.so" -type l | sed '/.*@.*/d')
    for LINK in $LINKS; do
      LIB=$(echo $LINK | sed 's/_S/\//g' | sed 's/_U/_/g')
      if [[ $LIB == *"_solib_k8/lib"* ]]; then
        LIB="/apollo/bazel-bin/$(echo $LIB | awk -F "_solib_k8/lib" '{print $NF}')"
      elif [[ $LIB == *"___"* ]]; then
        LIB="/apollo/bazel-bin/$(echo $LIB | awk -F "___" '{print $NF}')"
      else
        LIB="/apollo/bazel-bin/$(echo $LIB | awk -F "apollo/" '{print $NF}')"
      fi
      ln -sf $LIB $LINK
    done
  done
  cd -

  # setup cyber binaries and convert from //path:target to path/target
  CYBERBIN=$(bazel query "kind(cc_binary, //cyber/... //modules/tools/visualizer/...)" | sed 's/^\/\///' | sed 's/:/\//' | sed '/.*.so$/d')
  for BIN in ${CYBERBIN}; do
    cp -P --parent "bazel-bin/${BIN}" ${APOLLO_RELEASE_DIR}
  done
  cp --parent "cyber/setup.bash" "${APOLLO_RELEASE_DIR}"
  cp --parent -a "cyber/tools/cyber_launch" "${APOLLO_RELEASE_DIR}"
  cp --parent -a "cyber/tools/cyber_tools_auto_complete.bash" "${APOLLO_RELEASE_DIR}"
  cp --parent -a "cyber/python" "${APOLLO_RELEASE_DIR}"


  # setup tools
  TOOLSBIN=$(bazel query "kind(cc_binary, //modules/tools/...)" | sed 's/^\/\///' | sed 's/:/\//' | sed '/.*.so$/d')
  for BIN in ${TOOLSBIN}; do
    cp -P --parent "bazel-bin/${BIN}" ${APOLLO_RELEASE_DIR}
  done
  TOOLSPY=$(find modules/tools/* -name "*.py")
  for PY in ${TOOLSPY}; do
    cp -P --parent "${PY}" "${APOLLO_RELEASE_DIR}/bazel-bin"
  done

  # modules data, conf and dag
  CONFS=$(find modules/ cyber/ -name "conf")
  DATAS=$(find modules/ -name "data" | grep -v "testdata")
  DAGS=$(find modules/ -name "dag")
  LAUNCHS=$(find modules/ -name "launch")
  PARAMS=$(find modules/ -name "params" | grep -v "testdata")

  rm -rf test/*
  for CONF in $CONFS; do
    cp -P --parent -a "${CONF}" "${APOLLO_RELEASE_DIR}"
  done
  for DATA in $DATAS; do
    if [[ $DATA != *"map"* ]]; then
        cp -P --parent -a "${DATA}" "${APOLLO_RELEASE_DIR}"
    fi
  done
  for DAG in $DAGS; do
    cp -P --parent -a "${DAG}" "${APOLLO_RELEASE_DIR}"
  done
  for LAUNCH in $LAUNCHS; do
    cp -P --parent -a "${LAUNCH}" "${APOLLO_RELEASE_DIR}"
  done
  for PARAM in $PARAMS; do
    cp -P --parent -a "${PARAM}" "${APOLLO_RELEASE_DIR}"
  done
  # perception model
  MODEL="modules/perception/model"
  cp -P --parent -a "${MODEL}" "${APOLLO_RELEASE_DIR}"

  # dreamview frontend
  cp -a modules/dreamview/frontend $APOLLO_RELEASE_DIR/modules/dreamview

  # remove all pyc file in modules/
  find modules/ -name "*.pyc" | xargs -I {} rm {}

  # scripts
  cp -r scripts ${APOLLO_RELEASE_DIR}

  # manual traffic light tool
  mkdir -p ${APOLLO_RELEASE_DIR}/modules/tools
  cp -r modules/tools/manual_traffic_light ${APOLLO_RELEASE_DIR}/modules/tools

  # remove mounted models
  rm -rf ${APOLLO_RELEASE_DIR}/modules/perception/model/yolo_camera_detector/

  # lib
  LIB_DIR="${APOLLO_RELEASE_DIR}/lib"
  mkdir "${LIB_DIR}"
  if $USE_ESD_CAN; then
    warn_proprietary_sw
  fi
  THIRDLIBS=$(find third_party/* -name "*.so*")
  for LIB in ${THIRDLIBS}; do
    cp -a "${LIB}" $LIB_DIR
  done
  cp -r py_proto/modules $LIB_DIR

  # doc
  cp LICENSE "${APOLLO_RELEASE_DIR}"
  cp third_party/ACKNOWLEDGEMENT.txt "${APOLLO_RELEASE_DIR}"

  # release info
  META="${APOLLO_RELEASE_DIR}/meta.ini"
  echo "git_commit: $(get_revision)" >> $META
  echo "git_branch: $(get_branch)" >> $META
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
  files=$(find bazel-out/local-dbg/bin/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external | grep -v third_party)
  for f in $files; do
    if [ "$f" != "${f##*cyber}" ]; then
      target="$COV_DIR/objs/cyber${f##*cyber}"
    else
      target="$COV_DIR/objs/modules${f##*modules}"
    fi
    mkdir -p "$(dirname "$target")"
    cp "$f" "$target"
  done

  lcov --rc lcov_branch_coverage=1 --base-directory "/apollo/bazel-apollo" --capture --directory "$COV_DIR/objs" --output-file "$COV_DIR/conv.info"
  if [ $? -ne 0 ]; then
    fail 'lcov failed!'
  fi
  lcov --rc lcov_branch_coverage=1 --remove "$COV_DIR/conv.info" \
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
  JOB_ARG="--jobs=$(nproc) --ram_utilization_factor 80"

  generate_build_targets
  if [ "$USE_GPU" == "1" ]; then
    echo -e "${YELLOW}Running tests under GPU mode. GPU is required to run the tests.${NO_COLOR}"
    bazel test $DEFINES $JOB_ARG --config=unit_test -c dbg --test_verbose_timeout_warnings $@ $BUILD_TARGETS
  else
    echo -e "${YELLOW}Running tests under CPU mode. No GPU is required to run the tests.${NO_COLOR}"
    BUILD_TARGETS="`echo "$BUILD_TARGETS" | grep -v "cnn_segmentation_test\|yolo_camera_detector_test\|unity_recognize_test\|perception_traffic_light_rectify_test\|cuda_util_test"`"
    bazel test $DEFINES $JOB_ARG --config=unit_test -c dbg --test_verbose_timeout_warnings $@ $BUILD_TARGETS
  fi
  if [ $? -ne 0 ]; then
    fail 'Test failed!'
    return 1
  fi

  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  fi
}

function citest_basic() {
  set -e

  info "Building framework ..."
  cd /apollo
  source cyber/setup.bash

  # FIXME(all): temporarily disable modules doesn't compile in 18.04
#   BUILD_TARGETS="
#    `bazel query //modules/... union //cyber/...`
#  "
  BUILD_TARGETS=`bazel query //modules/... union //cyber/... except //modules/tools/visualizer/... except //modules/v2x/... except //modules/drivers/video/tools/decode_video/... except //modules/map/tools/map_datachecker/... `

  JOB_ARG="--jobs=12 --ram_utilization_factor 80"

  BUILD_TARGETS="`echo "$BUILD_TARGETS" | grep "modules\/" | grep "test" \
          | grep -v "modules\/planning" \
          | grep -v "modules\/prediction" \
          | grep -v "modules\/control" \
          | grep -v "modules\/common" \
          | grep -v "can_client" \
          | grep -v "blob_test" \
          | grep -v "pyramid_map" \
          | grep -v "ndt_lidar_locator_test" \
          | grep -v "syncedmem_test" | grep -v "blob_test" \
          | grep -v "perception_inference_operators_test" \
          | grep -v "cuda_util_test" \
          | grep -v "modules\/perception"`"

  bazel test $DEFINES $JOB_ARG --config=unit_test -c dbg --test_verbose_timeout_warnings $@ $BUILD_TARGETS

  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  else
    fail 'Test failed!'
    return 1
  fi
}

function citest_extended() {
  set -e

  info "Building framework ..."
  cd /apollo
  source cyber/setup.bash

  BUILD_TARGETS="
    `bazel query //modules/planning/... union //modules/common/... union //cyber/... $DISABLED_CYBER_MODULES`
    `bazel query //modules/prediction/... union //modules/control/...`
  "

  JOB_ARG="--jobs=12 --ram_utilization_factor 80"

  BUILD_TARGETS="`echo "$BUILD_TARGETS" | grep "test"`"

  bazel test $DEFINES $JOB_ARG --config=unit_test -c dbg --test_verbose_timeout_warnings $@ $BUILD_TARGETS

  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  else
    fail 'Test failed!'
    return 1
  fi
}

function citest() {
  info "Building framework ..."
  cd /apollo
  source cyber/setup.bash

  citest_basic
  citest_extended
  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  else
    fail 'Test failed!'
    return 1
  fi
}

function run_cpp_lint() {
  BUILD_TARGETS="`bazel query //modules/... except //modules/tools/visualizer/... union //cyber/...`"
  bazel test --config=cpplint -c dbg $BUILD_TARGETS
}

function run_bash_lint() {
  FILES=$(find "${APOLLO_ROOT_DIR}" -type f -name "*.sh" | grep -v ros)
  echo "${FILES}" | xargs shellcheck
}

function run_lint() {
  # Add cpplint rule to BUILD files that do not contain it.
  for file in $(find cyber modules -name BUILD | \
    grep -v gnss/third_party | grep -v modules/teleop/encoder/nvenc_sdk6 | \
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
  # Remove bazel cache.
  bazel clean --async

  # Remove bazel cache in associated directories
  if [ -d /apollo-simulator ]; then
    cd /apollo-simulator && bazel clean --async
  fi

  # Remove cmake cache.
  rm -fr framework/build
}

function buildify() {
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
  rev=$(get_revision)
  if [ "$rev" = "unknown" ];then
    echo "Version: $rev"
    return
  fi
  commit=$(git log -1 --pretty=%H)
  date=$(git log -1 --pretty=%cd)
  echo "Commit: ${commit}"
  echo "Date: ${date}"
}

function get_revision() {
  git rev-parse --is-inside-work-tree &> /dev/null
  if [ $? = 0 ];then
    REVISION=$(git rev-parse HEAD)
  else
    warning "Could not get the version number, maybe this is not a git work tree." >&2
    REVISION="unknown"
  fi
  echo "$REVISION"
}

function get_branch() {
  git branch &> /dev/null
  if [ $? = 0 ];then
    BRANCH=$(git rev-parse --abbrev-ref HEAD)
  else
    warning "Could not get the branch name, maybe this is not a git work tree." >&2
    BRANCH="unknown"
  fi
  echo "$BRANCH"
}

function config() {
  ${APOLLO_ROOT_DIR}/scripts/configurator.sh
}

function set_use_gpu() {
  if [ "${USE_GPU}" = "1" ] ; then
    DEFINES="${DEFINES} --define USE_GPU=true"
  else
    DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
  fi
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
  ${BLUE}build_cpu${NONE}: dbg build with CPU
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend javascript code, this requires all the node_modules to be installed already
  ${BLUE}build_cyber [dbg|opt]${NONE}: build Cyber RT only
  ${BLUE}build_drivers [dbg|opt]${NONE}: build drivers only
  ${BLUE}build_planning${NONE}: compile planning and its dependencies.
  ${BLUE}build_control${NONE}: compile control and its dependencies.
  ${BLUE}build_prediction${NONE}: compile prediction and its dependencies.
  ${BLUE}build_pnc${NONE}: compile pnc and its dependencies.
  ${BLUE}build_no_perception [dbg|opt]${NONE}: run build, skip building perception module, useful when some perception dependencies are not satisfied, e.g., CUDA, CUDNN, LIDAR, etc.
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
  cd "$( dirname "${BASH_SOURCE[0]}" )"
  source scripts/apollo_base.sh

  check_machine_arch
  apollo_check_system_config
  check_esd_files

  DEFINES="--define ARCH=${MACHINE_ARCH} --define CAN_CARD=${CAN_CARD} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"
  # Enable bazel's feature to compute md5 checksums in parallel
  DEFINES="${DEFINES} --experimental_multi_threaded_digest"

  if [ ${MACHINE_ARCH} == "x86_64" ]; then
    DEFINES="${DEFINES} --copt=-mavx2"
  fi

  local cmd=$1
  shift

  START_TIME=$(get_now)
  case $cmd in
    check)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      check $@
      ;;
    build)
      set_use_gpu
      apollo_build_dbg $@
      ;;
    build_cpu)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      apollo_build_dbg $@
      ;;
    build_prof)
      DEFINES="${DEFINES} --config=cpu_prof --cxxopt=-DCPU_ONLY"
      apollo_build_dbg $@
      ;;
    build_no_perception)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      BUILD_FILTER="no_perception"
      if [ "$1" == "opt" ]; then
        shift
        apollo_build_opt $@
      else
        shift
        apollo_build_dbg $@
      fi
      ;;
    clean_cyber)
      export LD_PRELOAD=
      clean
      ;;
    build_cyber)
      export LD_PRELOAD=
      BUILD_FILTER="cyber"
      if [ "$1" == "opt" ]; then
        shift
        apollo_build_opt $@
      else
        shift
        apollo_build_dbg $@
      fi
      ;;
    build_drivers)
      export LD_PRELOAD=
      BUILD_FILTER="drivers"
      if [ "$1" == "opt" ]; then
        shift
        apollo_build_opt $@
      else
        shift
        apollo_build_dbg $@
      fi
      ;;
    build_control)
      BUILD_FILTER="control"
      apollo_build_dbg $@
      ;;
    build_planning)
      BUILD_FILTER="planning"
      apollo_build_dbg $@
      ;;
    build_prediction)
      BUILD_FILTER="prediction"
      apollo_build_dbg $@
      ;;
    build_pnc)
      BUILD_FILTER="pnc"
      apollo_build_dbg $@
      ;;
    cibuild)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      cibuild $@
      ;;
    cibuild_extended)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      cibuild_extended $@
      ;;
    build_opt)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY --copt=-fpic"
      apollo_build_opt $@
      ;;
    build_gpu)
      set_use_gpu
      apollo_build_dbg $@
      ;;
    build_opt_gpu)
      set_use_gpu
      DEFINES="${DEFINES} --copt=-fpic"
      apollo_build_opt $@
      ;;
    build_teleop)
      set_use_gpu
      DEFINES="${DEFINES} --copt=-fpic --define WITH_TELEOP=1 --cxxopt=-DTELEOP"
      apollo_build_opt $@
      ;;
    build_fe)
      build_fe
      ;;
    buildify)
      buildify
      ;;
    build_py)
      build_py_proto
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
      set_use_gpu
      run_test $@
      ;;
    test_cpu)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      run_test $@
      ;;
    citest)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      citest $@
      ;;
    citest_basic)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      citest_basic $@
      ;;
    citest_extended)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      citest_extended $@
      ;;
    test_gpu)
      set_use_gpu
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
