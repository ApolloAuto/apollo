#!/bin/bash

set -e

LOG_DIR=master_slave_log

if [ ! -d $LOG_DIR ]; then
  mkdir $LOG_DIR
fi

MASTER_LOG_NAME=master.txt

function run() {
  slave_num=8
  if [ -z $1 ]; then
    echo "no slave num, we will use default value: $slave_num"
  else
    echo "slave num: $1"
    slave_num=$1
  fi

  role_num=32
  if [ -z $2 ]; then
    echo "no role num, we will use default value: $role_num"
  else
    echo "role num: $2"
    role_num=$2
  fi

  echo "start master"
  ./topo_master 1>$LOG_DIR/$MASTER_LOG_NAME 2>&1 &
  sleep 1

  echo "start slave"
  curr_slave_num=0
  while [ $curr_slave_num -lt $slave_num ]; do
    curr_slave_num=$((curr_slave_num + 1))
    echo "current slave: $curr_slave_num"
    ./topo_slave $role_num $role_num $role_num $role_num $role_num &
  done
}

function end_and_count() {
  echo "stop master and slave"
  ps -ef | grep topo_master | grep -v grep | awk '{print $2}' | xargs kill -9
  ps -ef | grep topo_slave | grep -v grep | awk '{print $2}' | xargs kill -9

  sleep 3
  echo "****** results ******"

  node_msgs=$(grep -E "CHANGE_NODE" $LOG_DIR/$MASTER_LOG_NAME | wc -l)
  echo "node_change_msgs num: $node_msgs"
  channel_msgs=$(grep -E "CHANGE_CHANNEL" $LOG_DIR/$MASTER_LOG_NAME | wc -l)
  echo "channel_change_msgs num: $channel_msgs"
  service_msgs=$(grep -E "CHANGE_SERVICE" $LOG_DIR/$MASTER_LOG_NAME | wc -l)
  echo "service_change_msgs num: $service_msgs"
  grep -E "CHANGE_" $LOG_DIR/$MASTER_LOG_NAME | \
      awk 'BEGIN{ i=0; latency=0 } \
           { time[i]=$2; ++i; latency+=$6} \
           END{ print "average latency(ms):",latency/i/1000000; \
                print " last msg time:",time[i-1]; \
                print "first msg time:",time[0];}'
}

function main() {
  case $1 in
    start)
      run $2 $3
      ;;
    stop)
      end_and_count
      ;;
    *)
      echo "Usage: ./launcher.bash {start slave_num role_num|stop}"
      exit 1
      ;;
  esac
}

main $@
