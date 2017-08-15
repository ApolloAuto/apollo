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
"""Protobuf utils."""
import google.protobuf.text_format as text_format


def get_pb_from_text_file(filename, proto_obj):
    """Get a proto from given text file."""
    with open(filename, 'r') as file_in:
        return text_format.Merge(file_in.read(), proto_obj)


def get_pb_from_bin_file(filename, proto_obj):
    """Get a proto from given binary file."""
    with open(filename, 'rb') as file_in:
        proto_obj.ParseFromString(file_in.read())
    return proto_obj

def get_pb_from_file(filename, proto_obj):
    """Get a proto from given file by trying binary mode and text mode."""
    try:
        return get_pb_from_bin_file(filename, proto_obj)
    except:
        print 'Info: Cannot parse %s as binary proto.' % filename

    try:
        return get_pb_from_text_file(filename, proto_obj)
    except:
        print 'Error: Cannot parse %s as text proto' % filename

    return None
