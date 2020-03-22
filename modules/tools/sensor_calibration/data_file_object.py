#!/usr/bin/env python3

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
import sys
import struct

import numpy as np


class FileObject(object):
    """Wrapper for file object"""

    # Initializing file object
    def __init__(self, file_path, operation='write', file_type='binary'):
        if operation != 'write' and operation != 'read':
            raise ValueError("Unsupported file operation: %s" % operation)

        if file_type != 'binary' and file_type != 'txt':
            raise ValueError("Unsupported file type: %s" % file_type)

        operator = 'w' if operation == 'write' else 'r'
        operator += 'b' if file_type == 'binary' else ''

        try:
            self._file_object = open(file_path, operator)
        except IOError:
            raise ValueError("Cannot open file: {}".format(file_path))

    # Safely close file
    def __del__(self):
        self._file_object.close()

    def file_object(self):
        return self._file_object

    def save_to_file(self, data):
        raise NotImplementedError


class TimestampFileObject(FileObject):
    """class to handle sensor timestamp for each Apollo sensor channel"""

    def __init__(self, file_path, operation='write', file_type='txt'):
        super(TimestampFileObject, self).__init__(file_path,
                                                  operation, file_type)

    def save_to_file(self, data):
        if not isinstance(data, list) and not isinstance(data, np.ndarray):
            raise ValueError("timestamps must be in a list")

        for i, ts in enumerate(data):
            self._file_object.write("%05d %.6f\n" % (i + 1, ts))


class OdometryFileObject(FileObject):
    """class to handle gnss/odometry topic"""

    def load_file(self):
        struct_len = struct.calcsize('i')
        data_size = struct.Struct('i').unpack(self._file_object.read(struct.calcsize('i')))[0]
        s0 = struct.Struct('d')
        s1 = struct.Struct('I')
        s2 = struct.Struct('7d')
        data = np.zeros((data_size, 9), dtype='float64')
        # , int32, float64, float64, float64, float64, float64, float64, float64')
        for d in data:
            #d[0] = s0.unpack_from(self._file_object.read(s0.size))[0]
            d[0] = s0.unpack(self._file_object.read(s0.size))[0]
            d[1] = s1.unpack_from(self._file_object.read(s1.size))[0]
            d[2:] = np.array(s2.unpack_from(self._file_object.read(s2.size)))
        return data.tolist()

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
        self._file_object.write(struct.pack('I', data_size))
        # have to pack separate, to avoid struct padding, now 8+4+7*8 = 68 bytes
        # TODO (yuanfan / gchen-Apollo): follow protobuf across tools.

        s0 = struct.Struct('d')
        s1 = struct.Struct('I')
        s2 = struct.Struct('7d')
        for d in data:
            # print(d[0])
            self._file_object.write(s0.pack(d[0]))
            self._file_object.write(s1.pack(d[1]))
            pack_d = s2.pack(d[2], d[3], d[4], d[5], d[6], d[7], d[8])
            self._file_object.write(pack_d)
