#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"

function main() {
    if ! command -v perf >/dev/null 2>&1; then
      bash  "${TOP_DIR}/docker/scripts/install_perf.sh"
      
    fi
    if [ -e /apollo-simulator ];then
      cd /apollo-simulator
      bash build.sh run >/dev/null 2>&1 & 
      sleep 1
      ps -ef | grep "replay_engine" | grep -v grep | awk '{print $2}' |  xargs sudo perf record  -g -p
      if [ -e perf.data ];then
        sudo perf report -i perf.data
      fi
      info "perf report was generated here"
    else
      echo "apollo-simulator doesn't exist"
    fi
}

main "$@"
