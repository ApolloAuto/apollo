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
"""Utils."""
import os


def find_by_name(name, value_list):
    """Find a value in list by name."""
    return next((value for value in value_list if value.name == name), None)


def subdir_with_title(root_dir):
    """List subdirs of a root dir, return a dict of {titlized_name: subdir}"""
    title_dir = {}
    for candidate in os.listdir(root_dir):
        subdir = os.path.join(root_dir, candidate)
        if os.path.isdir(subdir):
            title = candidate.replace('_', ' ').title()
            title_dir[title] = subdir
    return title_dir
