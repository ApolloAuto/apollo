#! /bin/bash
set -e

ROOT_PATH=`cd $(dirname $0); pwd`
cd $ROOT_PATH
MODULE_NAME="routing"
MODULE_DIR="${ROOT_PATH}/${MODULE_NAME}_ut"
RESULT_DIR="${ROOT_PATH}/${MODULE_NAME}_ut/result"

mkdir -p ${RESULT_DIR}
touch ${ROOT_PATH}/../share/build.sh
[ -d /home/caros/ros/share/ ] && touch /home/caros/ros/share/build.sh

source  ${ROOT_PATH}/../setup.bash
export LD_LIBRARY_PATH=${ROOT_PATH}/routing_ut:${ROOT_PATH}/../lib:/home/caros/ros/lib:$LD_LIBRARY_PATH

for test_bin in ${MODULE_DIR}/*.out
do
    if test -x $test_bin; then
        $test_bin
    fi
done > ${RESULT_DIR}/${MODULE_NAME}.txt
