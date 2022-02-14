#! /usr/bin/env bash

MOUNT_RES="$(df -h | grep /dev/nvme1n1)"
echo ${MOUNT_RES}

if [ -n MOUNT_RES ];then
echo "mount nvme..."
sudo mount  /dev/nvme1n1 /media/caros/nvme2
sudo chmod 777 -R /media/caros/nvme2
else
echo "nvme has been mount"
fi
if [ -e 'collect_data' ];then
echo 'collect_data exist'
else
echo 'collect_data creating'
mkdir collect_data
fi
echo "finish"
