#!/bin/bash

function print_usage() {
    echo 'Options:
    publish: broadcast the extrinsic
    clean: extrinsics clean
    '
}

case $1 in
  publish)
    python /apollo/modules/tools/extrinsics_broadcaster/extrinsics_broadcaster.py $2 & 	
    ;;
  clean)
    ps -ef | grep static_transform_publisher | awk '{print $2}' | xargs kill -2
    ;;
  *)
    print_usage
    ;;
esac
