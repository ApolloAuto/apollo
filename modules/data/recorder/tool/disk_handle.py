#!/usr/bin/env python

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
Disk Handle.
"""

import os
import sys
import time
import math
import statvfs
import logging
from collections import namedtuple

disk_ntuple = namedtuple('partition',  'device mountpoint fs')
usage_ntuple = namedtuple('usage',  'total used free percent')

def get_mount_point(path):
    """Get path mount point."""
    if not os.path.islink(path):
        path = os.path.abspath(path)
    elif os.path.islink(path) and os.path.lexists(os.readlink(path)):
        path = os.path.realpath(path)
    while not os.path.ismount(path):
        path = os.path.dirname(path)
        if os.path.islink(path) and os.path.lexists(os.readlink(path)):
            path = os.path.realpath(path)
    return path


def get_disk_free_size(device):
    """Get disk free size. unit: MB"""
    try:
        vfs = os.statvfs(device)
        free = vfs[statvfs.F_BAVAIL] * vfs[statvfs.F_BSIZE]
        return free
    except Exception as e:
        raise Exception("Get disk free size error!") 


def check_disk(output_path, min_space=1024):
    """Check disk free space. default min_space=1024MB"""
    disk_mp = get_mount_point(output_path)
    free_space = get_disk_free_size(disk_mp)
    if free_space < min_space * 1024 * 1024:
        print("Less than %s of space free on disk %s." % (min_space, disk_mp))
        logging.error("Less than %s of space free on disk %s.", min_space, disk_mp)
        return -2
    elif free_space < min_space * 1024 * 1024 * 5:
        print("Less than 5 * %s of space free on disk %s." % (min_space, disk_mp))
        logging.warn("Less than 5 * %s of space free on disk %s.", min_space, disk_mp)
        return -1
    return 0


def get_disk_partitions_info(all=False):
    """Return all mounted partitions as a nameduple."""
    phydevs = []
    f = open("/proc/filesystems", "r")
    for line in f:
        if not line.startswith("nodev"):
            phydevs.append(line.strip())
    retlist = []
    f = open('/etc/mtab', "r")
    for line in f:
        if not all and line.startswith('none'):
            continue
        fields = line.split()
        device = fields[0]
        mountpoint = fields[1]
        fstype = fields[2]
        if not all and fstype not in phydevs:
            continue
        if device == 'none':
            device = ''
        ntuple = disk_ntuple(device, mountpoint, fstype)
        retlist.append(ntuple)
    return retlist          


def get_disk_usage_info(path):
    """Return disk usage info with given path."""
    vfs = os.statvfs(path)
    free = (vfs[statvfs.F_BAVAIL] * vfs[statvfs.F_BSIZE])
    total = (vfs[statvfs.F_BLOCKS] * vfs[statvfs.F_BSIZE])
    used =  (vfs[statvfs.F_BLOCKS] - vfs[statvfs.F_BFREE]) * vfs[statvfs.F_FRSIZE]
    try:
        percent = (float(used) / total) * 100
    except ZeroDivisionError:
        percent = 0
    return usage_ntuple(total, used, free, round(percent, 2))


def get_folder_size(folder):
    """Get size of directory. return total size, unit(bytes)."""
    total_size = os.path.getsize(folder)
    for item in os.listdir(folder):
        itempath = os.path.join(folder, item)
        if os.path.isfile(itempath):
            total_size += os.path.getsize(itempath)
        elif os.path.isdir(itempath):
            total_size += get_folder_size(itempath)
    return total_size
