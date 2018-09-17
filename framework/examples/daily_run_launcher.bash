#!/bin/bash

set -e

LOG_DIR=daily_log
RUN_TIME_SEC=10

if [ ! -d $LOG_DIR ]; then
  mkdir $LOG_DIR
fi

LOG_NAME=daily_run_$(date "+%Y%m%d-%H%M%S").txt

function main() {
  if [ -z $1 ]; then
    echo "Please enter a valid loop num!"
    exit 1
  fi

  source setup.bash
  chmod a+x bin/daily_run.bash
  total_loop_num=$1

  echo "******** daily run start ********"
  echo "total loop num: $total_loop_num"

  curr_loop_num=0
  while [ $curr_loop_num -lt $total_loop_num ]; do
    curr_loop_num=$((curr_loop_num + 1))
    echo "current loop num: $curr_loop_num"

    daily_run.bash start >>$LOG_DIR/$LOG_NAME 2>&1
    sleep $RUN_TIME_SEC
    daily_run.bash stop >>$LOG_DIR/$LOG_NAME 2>&1
  done

  error_info=$(grep -i -E "core|terminate|error" $LOG_DIR/$LOG_NAME | wc -l)
  if [ $error_info -ne 0 ]; then
    echo "daily run failed, please check log: $LOG_DIR/$LOG_NAME for more details."
  else
    echo "daily run succ"
  fi

  echo "********  daily run end  ********"
}

main $@
