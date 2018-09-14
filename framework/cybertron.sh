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

#=================================================
#                   Utils
#=================================================

CYBERTRON_DIR=$(cd $(dirname $0); pwd)
CORE_NUM=$(grep -c ^processor /proc/cpuinfo)
# for ccover
TOOLS_HOME="${CYBERTRON_DIR}/tools"
COV_HOME="${TOOLS_HOME}/ccover/"
CMAKE_OPTIONS=""
export COVFILE="${CYBERTRON_DIR}/test_cybertron.cov"
export COVFILE_INC="${CYBERTRON_DIR}/test-inc.cov"
export PATH=${COV_HOME}/bin:$PATH
TEST_DIR=${CYBERTRON_DIR}/install/test/cybertron/unit_test/
TEST_DIR_PY=${CYBERTRON_DIR}/install/lib/python/test/

RED='\033[0;31m'
YELLOW='\e[33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NONE='\033[0m'
function info() {
    (>&2 echo -e "[\e[34m\e[1mINFO\e[0m] $*")
}

function error() {
    (>&2 echo -e "[${RED}ERROR${NONE}] $*")
}

function warning() {
    (>&2 echo -e "${YELLOW}[WARNING] $*${NONE}")
}

function ok() {
    (>&2 echo -e "[\e[32m\e[1m OK \e[0m] $*")
}

function print_delim() {
    echo '============================'
}

function get_now() {
    echo $(date +%s)
}

function print_time() {
    END_TIME=$(get_now)
    ELAPSED_TIME=$(echo "$END_TIME - $START_TIME" | bc -l)
    MESSAGE="Took ${ELAPSED_TIME} seconds"
    info "${MESSAGE}"
}

function success() {
    print_delim
    ok "$1"
    print_time
    print_delim
}

function fail() {
    print_delim
    error "$1"
    print_time
    print_delim
    exit -1
}

#=================================================
#              Build functions
#=================================================

function build() {
    info "start build"
    export LD_LIBRARY_PATH=${CYBERTRON_DIR}/third_party/protobuf/lib:$LD_LIBRARY_PATH
    mkdir -p build
    cd build
    cmake .. ${CMAKE_OPTIONS}
    make install -j${CORE_NUM} -l${CORE_NUM} || exit $?
    build_py
}

function build_fast() {
    info "start build fast"
    export LD_LIBRARY_PATH=${CYBERTRON_DIR}/third_party/protobuf/lib:$LD_LIBRARY_PATH
    if [[ ! -d "build" ]]; then
        build
    else
        cd build
        make install -j${CORE_NUM} -l${CORE_NUM} || exit $?
    fi
}

function build_py() {
    info "start build py"
    cp -rf ${CYBERTRON_DIR}/python/cybertron/* ${CYBERTRON_DIR}/install/lib/python/cybertron && \
    cp -rf ${CYBERTRON_DIR}/python/examples ${CYBERTRON_DIR}/install/lib/python && \
    cp -rf ${CYBERTRON_DIR}/python/test ${CYBERTRON_DIR}/install/lib/python
    PROTO_PATH=${CYBERTRON_DIR}/install/lib/python/proto/
    if [[ ! -d ${PROTO_PATH} ]]; then
        mkdir ${PROTO_PATH}
    fi
    find ${CYBERTRON_DIR}/cybertron/proto/ -name "*.py" | xargs -i cp -f {} ${PROTO_PATH}
}

function build_cov() {
    info "build coverage file"
    mkdir -p tmp
    export COVTMPDIR=`pwd`/tmp
    ccover_init
    ccover_enable
    ${COV_HOME}/bin/cov01 -1 2>/dev/null
    clean
    build
    ${COV_HOME}/bin/cov01 -0 2>/dev/null
}

function build_cov_fast() {
    info "build coverage file"
    mkdir -p tmp
    export COVTMPDIR=`pwd`/tmp
    ccover_init
    ccover_enable
    ${COV_HOME}/bin/cov01 -1 2>/dev/null
    cd build
    make install -j${CORE_NUM} -l${CORE_NUM}
    ${COV_HOME}/bin/cov01 -0 2>/dev/null
}

function show_cov() {
    ccover_report
    # generate html pages
    covhtml ./cover -d . -f ${COVFILE} 2>/dev/null
}

function package() {
    mkdir -p output
    tar zcvf output/install.tgz install/
}

function run_test() {
    source ${CYBERTRON_DIR}/install/setup.bash
    START_TIME=$(get_now)
    cd ${TEST_DIR}
    total=0
    failed=0
    fails=()
    error=0
    errors=()
    for test_file in `ls *_test`; do
        info "Run ${test_file}"
        cls=""
        tests=`./${test_file} --gtest_list_tests`
        if [[ $? -ne 0 ]]; then
            ((error=$error+1))
            errors[${error}]=${test_file}
        fi
        for line in ${tests}; do
            if [[ ${line} == *. ]]; then
                cls=${line}
            else
                info "Case: ${cls}${line}"
                ./${test_file} --gtest_filter=${cls}${line}
                if [[ $? -ne 0 ]]; then
                    error "${test_file}:${cls}${line} FAILED"
                    fails[${failed}]=${test_file}:${cls}${line}
                    ((failed=${failed}+1))
                fi
                ((total=${total}+1))
            fi
        done
        echo
    done
    echo
    if [[ ${error} -ne 0 ]]; then
        print_delim
        error "Errors occured: ${error}"
        for c in ${errors[@]}; do
            echo $c
        done
        print_delim
    fi
    if [[ ${failed} -ne 0 ]]; then
        print_delim
        error "Failed Cases:"
        for c in ${fails[@]}; do
            echo $c
        done
        print_delim
    fi
    success "Run ${total} cases, FAILED: ${failed}"
    cd -
}

function run_test_py() {
    source ${CYBERTRON_DIR}/install/setup.bash
    START_TIME=$(get_now)
    cd ${TEST_DIR_PY}
    total=0
    failed=0
    fails=()
    error=0
    errors=()
    for test_file in `ls *.py`; do
        info "Run ${test_file}"
        python ${test_file}
        if [[ $? -ne 0 ]]; then
            ((error=$error+1))
            errors[${error}]=${test_file}
            fails[${failed}]=${test_file}:${cls}${line}
            ((failed=${failed}+1))
        fi
        ((total=${total}+1))
        echo
    done
    echo
    if [[ ${error} -ne 0 ]]; then
        print_delim
        error "Errors occured: ${error}"
        for c in ${errors[@]}; do
            echo $c
        done
        print_delim
    fi
    if [[ ${failed} -ne 0 ]]; then 
        print_delim
        error "Failed Cases:"
        for c in ${fails[@]}; do
            echo $c
        done
        print_delim
    fi
    success "Run ${total} cases, FAILED: ${failed}"
    cd -
}

function run_bash_lint() {
    FILES=$(find "${APOLLO_ROOT_DIR}" -type f -name "*.sh" | grep -v ros)
    echo "${FILES}" | xargs shellcheck
}

function run_cpp_lint() {
    START_TIME=$(get_now)

    if [ -z $1 ]; then
      find ./cybertron -name "*.h" | grep -v ".pb.h" | xargs ./cpplint.py --verbose=3 --filter=-build/c++11,-build/include_order
      find ./cybertron -name "*.cpp" -exec ./cpplint.py --verbose=3 --filter=-build/c++11,-build/include_order {} \;
    elif [ -d $1 ]; then
      find $1 -name "*.h" | grep -v ".pb.h" | xargs ./cpplint.py --verbose=3 --filter=-build/c++11,-build/include_order
      find $1 -name "*.cpp" -exec ./cpplint.py --verbose=3 --filter=-build/c++11,-build/include_order {} \;
    else
      ./cpplint.py --verbose=3 --filter=-build/c++11,-build/include_order $1
    fi
}

function run_clang_format() {
    START_TIME=$(get_now)
    FORMAT_DIR=${CYBERTRON_DIR}/cybertron
    if [[ $# -eq 1 ]]; then
        FORMAT_DIR=$1
    fi
    info 'format code with clang-format. Code style: .clang-format'
    type clang-format-3.8 > /dev/null 2>&1 ||
    {
        info 'install clang-format-3.8' &&
        sudo apt-get install clang-format-3.8 &&
        ok 'install clang-format-3.8 success'
    }

    find ${FORMAT_DIR} -name "*.h" | grep -v tf2 | xargs clang-format-3.8 -style=file -i
    find ${FORMAT_DIR} -name "*.c" | grep -v tf2 | xargs clang-format-3.8 -style=file -i
    find ${FORMAT_DIR} -name "*.cpp" | grep -v tf2 | xargs clang-format-3.8 -style=file -i
    find ${FORMAT_DIR} -name "*.cc" | grep -v tf2 | xargs clang-format-3.8 -style=file -i
    find ${FORMAT_DIR} -name "*.cxx" | grep -v tf2 | xargs clang-format-3.8 -style=file -i
    success "Format all c/c++ source files success."
}

function clean() {
    find ./cybertron -name "*.pb.cc" -o -name "*.pb.h" | xargs -i rm {}
    if [[ -d build ]]; then
        rm -rf build
    fi
    if [[ -d install ]]; then
        rm -rf install
    fi
    if [[ -d output ]]; then
        rm -rf output
    fi
}

function gen_doc() {
    rm -rf docs/doxygen
    doxygen cybertron.doxygen
}

function version() {
    commit=$(git log -1 --pretty=%H)
    date=$(git log -1 --pretty=%cd)
    echo "Commit: ${commit}"
    echo "Date: ${date}"
}

#=================================================
#              CCover functions
#=================================================

function ccover_init() {
    if [[ ! -d "${COV_HOME}" ]]; then
        cd ${TOOLS_HOME} && \
        tar -zxvf ccover.tar.gz && \
        cd -
    fi

    ${COV_HOME}/bin/cov01 -h >/dev/null 2>&1
    if [[ $? -ne 1 ]] ; then
        echo -e "ccover not install !";
        exit 1
    fi
    echo -e "ccover already install."
    return 0
}

function ccover_enable() {
    CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCCover="
}

function ccover_select() {
    cd ${CYBERTRON_DIR}
    #清空
    ${COV_HOME}/bin/covselect -d 2>/dev/null
    echo -e "ccover clear file lists"
    #目录下的所有.h .cpp或者.cc文件添加到统计队列，过滤掉test、idl、pb等不需要关心的
    for line in `find cybertron -name "*.h" -o -name "*.cpp" -o -name "*.cc"`
    do
        if [[ "${line}" =~ .*test.* || "${line}" =~ .*bc_out.*
           || "${line}" =~ cybertron/tools/cvt/* || "${line}" =~ cybertron/tf2/*
           || "${line}" =~ .*idl.* || "${line}" =~ .*pb.* ]]; then
            echo -e "ccover ignore file ${DIRNAME}/${line}"
        else
            ${COV_HOME}/bin/covselect -a $line 2>/dev/null
        fi
    done
    cd -
}

#使用产出覆盖率分析的详细报告
function ccover_report() {
    ccover_select
    ${COV_HOME}/bin/covsrc -w1000 -f ${COVFILE} | sed 's/[ ]+/ /g' | awk '{print $1, $2, $4, $7, $9;}' | awk '{
        len=split($1, array, "/");
        if (len > 2) { # in module
            module_dict_content[NR] = $0;
            module_dict[array[2]] = 0;
        } else if (len == 2) {
            file_dict_content[NR] = $0;
        }
    } END {
        g_total_fn = 0;
        g_cov_fn = 0;
        g_total_br = 0;
        g_cov_br = 0;
        for (item in module_dict) {
            total_fn = 0;
            cov_fn = 0;
            total_br = 0;
            cov_br = 0;
            print "#####################################################";
            print "MODULE:"item;
            for (line_no in module_dict_content) {
                split(module_dict_content[line_no], line, " ");
                split(line[1], array, "/");
                if (array[2] == item) {
                    total_fn = total_fn + line[3];
                    cov_fn = cov_fn + line[2];
                    total_br = total_br + line[5];
                    cov_br = cov_br + line[4];
                    g_total_fn = g_total_fn + line[3];
                    g_cov_fn = g_cov_fn + line[2];
                    g_total_br = g_total_br + line[5];
                    g_cov_br = g_cov_br + line[4];
                    print line[1], line[2]"/"line[3], line[4]"/"line[5];
                }
            }
            print "=====================================================";
            print item, cov_fn"/"total_fn"="100*cov_fn/total_fn"%", cov_br"/"total_br"="100*cov_br/total_br"%";
            print "#####################################################";
        }
        print "#####################################################";
        print "FILES:";
        for (file_no in file_dict_content) {
            split(file_dict_content[file_no], line, " ")
            total_fn = line[3];
            cov_fn = line[2];
            total_br = line[5];
            cov_br = line[4];
            g_total_fn = g_total_fn + line[3];
            g_cov_fn = g_cov_fn + line[2];
            g_total_br = g_total_br + line[5];
            g_cov_br = g_cov_br + line[4];
            print line[1], cov_fn"/"total_fn"="100*cov_fn/total_fn"%", cov_br"/"total_br"="100*cov_br/total_br"%";
        }
        print "=====================================================";
        print "CoverageResultTotal", g_cov_fn"/"g_total_fn"="100*g_cov_fn/g_total_fn"%", g_cov_br"/"g_total_br"="100*g_cov_br/g_total_br"%";
        print "#####################################################";
    }' | column -t
    #${COV_HOME}/bin/covsrc -f ${COVFILE} 2>/dev/null
    COV_ALL=(`${COV_HOME}/bin/covsrc -f ${COVFILE} 2>/dev/null | tail -n1 | awk '{print $6,$11}'`) \
        && COV_ALL[0]=`echo ${COV_ALL[0]} | sed 's/%//g'` \
        && COV_ALL[1]=`echo ${COV_ALL[1]} | sed 's/%//g'`
    if [[ $? -ne 0 ]]; then
        echo -e "Get Function&C/D coverage failed!" >&2
        return 1
    else
        DATE=`date +"%F"`
        echo -e "${DATE}\nAll Coverage Result: Fn:${COV_ALL[0]} Br:${COV_ALL[1]}"
    fi
    #${COV_HOME}/bin/covsrc -f $COVFILE --html > ./ccover.html 2>/dev/null
    return 0
}

function print_usage() {
    echo -e "\n${RED}Usage${NONE}:
    .${BOLD}/cybertron.sh${NONE} [OPTION]"

    echo -e "\n${RED}Options${NONE}:
  ${BLUE}build${NONE}: run build only
  ${BLUE}build_fast${NONE}: build fast
  ${BLUE}build_cov${NONE}: build coverage file
  ${BLUE}build_cov_fast${NONE}: build coverage file fast
  ${BLUE}show_cov${NONE}: show coverage
  ${BLUE}clean${NONE}: clean build & install & output
  ${BLUE}config${NONE}: run configurator tool
  ${BLUE}coverage${NONE}: generate test coverage report
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}lint${NONE}: run code style check
  ${BLUE}format${NONE} [FOLDER]: run clang-format
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}release${NONE}: build release version
  ${BLUE}test${NONE}: run all unit tests
  ${BLUE}version${NONE}: display current commit and date
    "
}

function main() {
    case $1 in
        build)
            clean
            build
        ;;
        build_fast)
            build_fast
        ;;
        build_cov)
            build_cov
        ;;
        build_cov_fast)
            build_cov_fast
        ;;
        show_cov)
            show_cov
        ;;
        build_py)
            build_py_proto
        ;;
        doc)
            gen_doc
        ;;
        lint)
            run_cpp_lint $2
        ;;
        format)
            run_clang_format $2
        ;;
        test)
            run_test
        ;;
        test_py)
            run_test_py
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
        package)
            package
        ;;
        *)
            print_usage
        ;;
    esac
}

main $@
