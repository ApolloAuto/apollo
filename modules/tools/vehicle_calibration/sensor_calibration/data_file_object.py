#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
This is a bunch of classes to manage cyber record channel FileIO.
"""
import os
import re
import sys
import struct

class FileObject(object):
    """Wrapper for file object"""

    # Initalizing file obj
    def __init__(self, file_path, operation='write', filetype='binary'):
        if operation != 'write' and operation != 'read':
            raise ValueError("Unsupported file operation: %s" % operation)

        if filetype != 'binary' and filetype != 'txt':
            raise ValueError("Unsupported file type: %s" % filetype)

        operator = 'w' if operation == 'write' else 'r'
        operator += 'b' if filetype == 'binary' else ''

        try:
            self._file_object = open(file_path, operator)
        except IOError:
            raise ValueError("Cannot open file: {}".format(file_path))

    def file_object(self):
        return self._file_object

    def save_to_file(self, data):
        raise NotImplementedError

    # Safely close file
    def __del__(self):
        self._file_object.close()

class OdometryFileObject(FileObject):
    """class to handle gnss/odometry topic"""

    def save_to_file(self, data):
        """
        odometry data: total 9 elements
        [
        double timestamp
        int32 postion_type
        double qw, qx, qy, qz
        double x, y, z
        ]
        """
        if not isinstance(data, list):
            raise ValueError("Odometry data must be in a list")
        data_size = len(data)
        self._file_object.write(struct.pack('i', data_size))
        # TODO: gchen-Apollo, Check why pack_d still have 72 byte,
        # not 68 = 8 + 4 + 7*8
        s = struct.Struct('di7d')
        for d in data:
            pack_d = s.pack(d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8])
            self._file_object.write(pack_d)
            print("pack_d length: %d" % len(pack_d))
            print(d)
