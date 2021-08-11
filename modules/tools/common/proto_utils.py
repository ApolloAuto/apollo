#!/usr/bin/env python3

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


def write_pb_to_text_file(topic_pb, file_path):
    """write pb message to file"""
    with open(file_path, 'w') as f:
        f.write(str(topic_pb))


def get_pb_from_text_file(filename, pb_value):
    """Get a proto from given text file."""
    with open(filename, 'r') as file_in:
        return text_format.Merge(file_in.read(), pb_value)


def get_pb_from_bin_file(filename, pb_value):
    """Get a proto from given binary file."""
    with open(filename, 'rb') as file_in:
        pb_value.ParseFromString(file_in.read())
    return pb_value


def get_pb_from_file(filename, pb_value):
    """Get a proto from given file by trying binary mode and text mode."""
    try:
        return get_pb_from_bin_file(filename, pb_value)
    except:
        try:
            return get_pb_from_text_file(filename, pb_value)
        except:
            print('Error: Cannot parse %s as binary or text proto' % filename)
    return None


def flatten(pb_value, selectors):
    """
    Get a flattened tuple from pb_value. Selectors is a list of sub-fields.

    Usage:
    For a pb_value of:
        total_pb = {
            me: { name: 'myself' }
            children: [{ name: 'child0' }, { name: 'child1' }]
        }
    my_name, child0_name = flatten(total_pb, ['me.name', 'children[0].name'])
    # You get (my_name='myself', child0_name='child0')

    children_names = flatten(total_pb, 'children.name')
    # You get (children_names=['child0', 'child1'])
    """

    def __select_field(val, field):
        if hasattr(val, '__len__'):
            # Flatten repeated field.
            return [__select_field(elem, field) for elem in val]
        if not field.endswith(']'):
            # Simple field.
            return val.__getattribute__(field)
        # field contains index: "field[index]".
        field, index = field.split('[')
        val = val.__getattribute__(field)
        index = int(index[:-1])
        return val[index] if index < len(val) else None

    def __select(val, selector):
        for field in selector.split('.'):
            val = __select_field(val, field)
            if val is None:
                return None
        return val

    # Return the single result for single selector.
    if isinstance(selectors, str):
        return __select(pb_value, selectors)
    # Return tuple result for multiple selectors.
    return tuple((__select(pb_value, selector) for selector in selectors))
