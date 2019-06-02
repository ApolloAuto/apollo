#!/usr/bin/env bash

#node_pid=$(ps aux|grep "demo_worker"|grep -v grep|awk '{print $2}')
#if [ ! -z "$node_pid" ]; then
#    echo "kill demo_worker."
#    kill -9 $node_pid
#fi
echo 'Starting map-datachecker......'
if [ -z $CYBERTRON_PATH ]; then
  echo "INFO: CYBERTRON_PATH is empty, MUST 'source setup.bash' first."
  exit 1
fi
echo "INFO: CYBERTRON_PATH=$CYBERTRON_PATH"

cd ${CYBERTRON_PATH}
./bin/map-datachecker --flagfile=${CYBERTRON_PATH}/conf/map-datachecker.conf > /home/caros/xlog/log/map_datachecker_$(date '+%Y-%m-%d--%H-%M-%S').log 2>&1 &

PID=$!
trap "break" INT
while [[ true ]]; do
  sleep 2
done

echo 'Stopping map-datachecker......'
kill -INT ${PID}
