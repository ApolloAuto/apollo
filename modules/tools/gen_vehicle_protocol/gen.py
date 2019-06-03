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

#-*- coding:utf-8 -*-

import datetime
import os
import shutil
import sys
import yaml
from gen_proto_file import gen_proto_file
from gen_protocols import gen_protocols
from gen_vehicle_controller_and_manager import gen_vehicle_controller_and_manager
from extract_dbc_meta import extract_dbc_meta


def gen(conf):
    """
        doc string:
    """
    dbc_file = conf["dbc_file"]
    protocol_conf_file = conf["protocol_conf"]
    car_type = conf["car_type"]
    black_list = conf["black_list"]
    sender_list = conf["sender_list"]
    sender = conf["sender"]
    output_dir = conf["output_dir"]

    # extract dbc file meta to a internal config file
    if not extract_dbc_meta(dbc_file, protocol_conf_file, car_type, black_list,
                            sender_list, sender):
        return

    # gen proto
    proto_dir = output_dir + "proto/"
    gen_proto_file(protocol_conf_file, proto_dir)

    # gen protocol
    protocol_dir = output_dir + "vehicle/" + car_type.lower() + "/protocol/"
    gen_protocols(protocol_conf_file, protocol_dir)

    # gen vehicle controller and protocol_manager
    vehicle_dir = output_dir + "vehicle/" + car_type.lower() + "/"
    gen_vehicle_controller_and_manager(protocol_conf_file, vehicle_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "usage:\npython %s some_config.yml" % sys.argv[0]
        sys.exit(0)
    with open(sys.argv[1], 'r') as fp:
        conf = yaml.load(fp)
    gen(conf)
