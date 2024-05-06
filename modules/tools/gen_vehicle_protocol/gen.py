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

# -*- coding:utf-8 -*-

import datetime
import os
import shutil
import sys

import yaml

from modules.tools.gen_vehicle_protocol.gen_canbus_conf import gen_canbus_conf
from modules.tools.gen_vehicle_protocol.gen_proto_file import gen_proto_file
from modules.tools.gen_vehicle_protocol.gen_protocols import gen_protocols
from modules.tools.gen_vehicle_protocol.gen_vehicle_controller_and_manager import gen_vehicle_controller_and_manager
from modules.tools.gen_vehicle_protocol.extract_dbc_meta import extract_dbc_meta


def gen(conf):
    """
        doc string:
    """
    current_dir = sys.path[0]
    print("current dir is", current_dir)
    # user_workspace_dir = os.getcwd()
    # print("user workspace dir is", user_workspace_dir)
    dbc_file = conf["dbc_file"]
    print("user dbc file path is ", dbc_file)
    protocol_conf_file = conf["protocol_conf"]
    # print("output protocol conf file is ", protocol_conf_file)
    car_type = conf["car_type"]
    black_list = conf["black_list"]
    sender_list = conf["sender_list"]
    sender = conf["sender"]
    output_dir = conf["output_dir"]
    output_canbus_conf_dir = conf["output_canbus_conf_dir"]
    print("Generate the canbus_vehicle output dir is ", os.path.join(output_dir, car_type.lower()))
    template_dir = "/apollo/modules/tools/gen_vehicle_protocol/template/"
    use_demo_dbc = conf["use_demo_dbc"]
    if use_demo_dbc:
        print("use demo dbc")
    else:
        print("not use demo dbc")

    # extract dbc file meta to an internal config file
    if not extract_dbc_meta(dbc_file, protocol_conf_file, car_type, black_list,
                            sender_list, sender):
        return

    # gen proto
    proto_dir = output_dir + "/" + car_type.lower() + "/" + "proto/"
    gen_proto_file(protocol_conf_file, proto_dir, template_dir)

    # gen protocol
    protocol_dir = output_dir + "/" + car_type.lower() + "/protocol/"
    gen_protocols(protocol_conf_file, protocol_dir, template_dir)

    # gen vehicle controller and protocol_manager
    vehicle_dir = output_dir + "/" + car_type.lower() + "/"
    gen_vehicle_controller_and_manager(protocol_conf_file, vehicle_dir, template_dir, use_demo_dbc)

    # gen canbus conf
    canbus_conf_dir = output_canbus_conf_dir + "/" + car_type.lower() + "/" + "canbus_conf/"
    gen_canbus_conf(protocol_conf_file, canbus_conf_dir, template_dir)
    print("Generate the output canbus conf dir is", canbus_conf_dir)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage:\npython %s some_config.yml" % sys.argv[0])
        sys.exit(0)
    with open(sys.argv[1], 'r', encoding='utf-8') as fp:
        conf = yaml.safe_load(fp)
    gen(conf)
