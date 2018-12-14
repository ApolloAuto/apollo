#!/usr/bin/env python

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

"""
Copy bags between disks.
"""

from __future__ import print_function
import argparse
import glob
import os
import shutil
import sys

from rosbag.bag import Bag
import psutil

from modules.canbus.proto.chassis_pb2 import Chassis

K_CHASSIS_TOPIC = '/apollo/canbus/chassis'
K_DRIVE_EVENT_TOPIC = '/apollo/drive_event'

K_LARGE_SIZE_TOPICS = set([
    '/apollo/sensor/camera/obstacle/front_6mm',
    '/apollo/sensor/camera/traffic/image_long',
    '/apollo/sensor/camera/traffic/image_short',
    '/apollo/sensor/velodyne64/compensator/PointCloud2',
])
K_COPY_LARGE_SIZE_TOPICS_SECONDS_BEFORE_EVENT = 20.0


def GetDisks():
    """Get disks, which should be mounted under /media."""

    disks = [disk.mountpoint for disk in psutil.disk_partitions()
             if disk.mountpoint.startswith('/media/')]
    disks.append('/apollo')
    # The disks are sorted like
    #   /media/apollo/internal_nvme
    #   /media/apollo/apollo8
    #   /apollo
    disks = sorted(disks, reverse=True)
    if len(disks) <= 1:
        print('Cannot find disks.')
        sys.exit(1)

    copy_from = None
    copy_to = None
    for index, disk in enumerate(disks):
        print('\t{}: {}'.format(index, disk))
    try:
        selected = int(input('Which disk do you want to copy from: '))
        copy_from = disks[selected]
    except:
        print('Bad input')
        sys.exit(1)

    for index, disk in enumerate(disks):
        if index != selected:
            print('\t{}: {}'.format(index, disk))
    try:
        selected = int(input('Which disk do you want to copy to: '))
        copy_to = disks[selected]
    except:
        print('Bad input')
        sys.exit(1)

    if copy_from and copy_to and (copy_from != copy_to):
        print('Copy disk: {} -> {}'.format(copy_from, copy_to))
    else:
        sys.exit(1)
    return copy_from, copy_to


def CollectEvents(bags):
    """Collect interested event timestamps."""

    print('Collecting events...', end='')
    events = []
    cur_driving_mode = None
    for bag_file in bags:
        with Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[K_CHASSIS_TOPIC,
                                                           K_DRIVE_EVENT_TOPIC]):
                # For disengagement, take the message time as event time.
                if topic == K_CHASSIS_TOPIC:
                    if (cur_driving_mode == Chassis.COMPLETE_AUTO_DRIVE and
                            msg.driving_mode == Chassis.EMERGENCY_MODE):
                        events.append(t.to_sec())
                    cur_driving_mode = msg.driving_mode
                # For DriveEvent, take the header time as event time.
                elif topic == K_DRIVE_EVENT_TOPIC:
                    events.append(msg.header.timestamp_sec)
    print('Collected {} events.'.format(len(events)))
    return events


def SmartCopyBags(from_dir, to_dir):
    """Copy a task but filter useless sensor data."""

    bags = sorted(glob.glob(os.path.join(from_dir, '*.bag')))
    if len(bags) == 0:
        return
    if not os.path.exists(to_dir):
        os.makedirs(to_dir)

    events = CollectEvents(bags)
    next_event = 0
    for from_bag in bags:
        to_bag = os.path.join(to_dir, os.path.basename(from_bag))
        print('Copy bag: {} -> {}'.format(from_bag, to_bag))

        # Do the copy
        with Bag(from_bag, 'r') as bag_in, Bag(to_bag, 'w') as bag_out:
            for topic, msg, t in bag_in.read_messages():
                # For small size topics, always copy.
                if topic not in K_LARGE_SIZE_TOPICS:
                    bag_out.write(topic, msg, t)
                    continue

                msg_sec = t.to_sec()
                while next_event < len(events) and events[next_event] < msg_sec:
                    next_event += 1
                # For large size topics, only copy when it's near an event.
                if (next_event < len(events) and events[next_event] - msg_sec <
                        K_COPY_LARGE_SIZE_TOPICS_SECONDS_BEFORE_EVENT):
                    bag_out.write(topic, msg, t)


def SmartCopyDir(from_dir, to_dir):
    """Copy directory."""

    print('Copy dir: {} -> {}'.format(from_dir, to_dir))
    is_task_dir = False
    for f in sorted(os.listdir(from_dir), reverse=True):
        sub_path = os.path.join(from_dir, f)
        if os.path.isdir(sub_path):
            SmartCopyDir(sub_path, os.path.join(to_dir, f))
            continue

        if f.endswith('.bag.active'):
            # Found unindexed bag, always copy it.
            # TODO(xiaoxq): Index the bag and go to next step.
            shutil.copy(sub_path, to_dir)
            continue

        if f.endswith('.bag'):
            is_task_dir = True
            break;

    if is_task_dir:
        SmartCopyBags(from_dir, to_dir)


def main():
    """Do the job."""

    copy_from, copy_to = GetDisks()
    from_dir = os.path.join(copy_from, 'data/bag')
    if not os.path.exists(from_dir):
        print('Bag dir doesn\'t exist:', from_dir)
        sys.exit(1)
    to_dir = os.path.join(copy_to, 'data/bag')

    print('\t1. Only keep sensor data for drive events')
    print('\t2. Eveything')
    selected = input('What kind of data do you need: ')
    if selected == '1':
        SmartCopyDir(from_dir, to_dir)
    elif selected == '2':
        shutil.copytree(from_dir, to_dir)
    else:
        print('Bad input')
        sys.exit(1)

    # TODO(xiaoxq): We always try to make data structurized, such as putting
    # them into rosbag, instead of copying raw files around.
    other_data_dirs = {'/apollo/data/gpsbin': 'data/gpsbin'}
    for src, dst in other_data_dirs.iteritems():
        if os.path.exists(src):
            print('Copying ', src)
            shutil.copytree(src, os.path.join(copy_to, dst))


if __name__ == "__main__":
    main()
