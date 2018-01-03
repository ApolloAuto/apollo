#!/usr/bin/env python2

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
Stat a recorded task.
Usage:
    ./arrange_data.py <dir>
"""

import collections
import os
import sys

from rosbag.bag import Bag

from modules.data.proto.static_info_pb2 import StaticInfo


def calc_static_info(bags):
    vehicle_names = collections.defaultdict(int)
    for bag_file in bags:
        with Bag(bag_file, 'r') as bag:
            for _, msg, _ in bag.read_messages(topics=['/apollo/monitor/static_info']):
                if msg.vehicle.name:
                    vehicle_names[msg.vehicle.name] += 1
    largest_k, largest_v, sum_of_v = None, 0, 0
    for k, v in vehicle_names.iteritems():
        sum_of_v += v
        if v > largest_v:
            largest_k, largest_v = k, v
    if sum_of_v > 0:
        return largest_k.lower(), float(largest_v) / sum_of_v
    return largest_k, 0


def arrange_dir(data_dir):
    sys.stderr.write('Processing {}\n'.format(data_dir))
    bags = []
    for f in os.listdir(data_dir):
        f_path = os.path.join(data_dir, f)
        if os.path.isfile(f_path):
            if f_path.endswith('.bag') or f_path.endswith('.bag.active'):
                bags.append(f_path)
            else:
                print '# Unknown file {}'.format(f_path)
        else:
            arrange_dir(f_path)
    if len(bags) == 0:
        return

    print 'echo "Processing {}"'.format(data_dir)
    bags = sorted(bags)
    vehicle_name, belief = calc_static_info(bags)
    print 'echo "Guess as {} with belief {}"'.format(vehicle_name, belief)
    if belief < 0.4:
        print ''
        return

    get_target_dir = lambda vehicle_name, task_id: \
        '/mnt/nfs/public_test/{}/{}/{}'.format(
            task_id[:10], vehicle_name, task_id)

    task_id_len = len('YYYY-MM-DD-HH-mm-SS')
    # Use dirname as task id.
    task_id = os.path.basename(os.path.abspath(data_dir))
    if (len(task_id) != task_id_len or
            not os.path.exists(get_target_dir(vehicle_name, task_id))):
        task_id = os.path.basename(bags[0])[:task_id_len]

    target_dir = get_target_dir(vehicle_name, task_id)
    if not os.path.exists(target_dir):
        print 'mkdir -p {}'.format(target_dir)

    for bag in bags:
        target_bag = os.path.basename(bag)
        if bag.endswith('.bag.active'):
            target_bag = target_bag[:-7]
        # A new task.
        if bag != bags[0] and target_bag.endswith('_0.bag'):
            task_id = os.path.basename(bag)[:task_id_len]
            target_dir = get_target_dir(vehicle_name, task_id)
            if not os.path.exists(target_dir):
                print 'mkdir -p {}'.format(target_dir)
        print 'rsync -aht --size-only --progress "{}" "{}/{}"'.format(bag, target_dir, target_bag)
    print ''


if __name__ == '__main__':
    arrange_dir(sys.argv[1])

