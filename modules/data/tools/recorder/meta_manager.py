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

"""This script is responsible for meta management."""

import sys
import json
import logging
import ConfigParser

def create_meta(meta_filename, meta_type, meta_content_dict):
    """
    Create meta.
    Attribute:
        filename: meta file name.
        meta_type: json of ini
        meta_content_dict: a dict of meta content.
    Return: None
    """
    if meta_type == 'ini':
        config = ConfigParser.ConfigParser()
        meta_info_list = meta_content_dict.get('meta_info')
        for list_value in iter(meta_info_list):
            for section, items_dict in list_value.items():
                config.add_section(section)
                for items_key, items_value in items_dict.items():
                    config.set(section, items_key, items_value)
        config.write(open(meta_filename, 'w'))
        return 0
    elif meta_type == 'json':
        json.dump(meta_content_dict, open(meta_filename, 'w'), sort_keys = True)
        logging.info("Create meta successfully.")
        return 0
    else:
        logging.error("Create meta failed, meta type should be json or ini")
        return -1
