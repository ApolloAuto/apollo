#!/bin/bash

set -e

function prepare_env() {
  source setup.bash
}

function example_talker_listener() {
  talker_listener &
}

function run_examples() {
  example_talker_listener
}

function run_all_in_one() {
  cyber_launch start all_in_one.launch &
}

function run() {
  run_examples
  run_all_in_one
}

function main() {
  case $1 in
    start)
      prepare_env
      run
      ;;
    stop)
      prepare_env
      cyber_launch stop
      ps -ef | grep talker_listener | grep -v grep | awk '{print $2}' | xargs kill -9
      ps -ef | grep mainboard | grep -v grep | awk '{print $2}' | xargs kill -9
      ;;
    *)
      echo "Usage: ./daily_run.bash {start|stop}"
      exit 1
      ;;
  esac
}

main $@
