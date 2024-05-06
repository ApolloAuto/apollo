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


def gen_canbus_gflag_conf(content, output_dir, protocol_template_dir, is_apollo_source):
    if is_apollo_source:
        canbus_gflag_conf_file = protocol_template_dir + "canbus_source.conf.tpl"
    else:
        canbus_gflag_conf_file = protocol_template_dir + "canbus_aem.conf.tpl"
    with open(canbus_gflag_conf_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    canbus_gflag_conf_file = output_dir + "canbus.conf"
    with open(canbus_gflag_conf_file, 'w') as gflag_conf:
        FMT =  "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fmt_val["car_type_cap"] = car_type.capitalize()
        gflag_conf.write(FMT % fmt_val)

def gen_canbus_pb_txt_conf(content, output_dir, protocol_template_dir):
    canbus_pb_txt_conf_file = protocol_template_dir + "canbus_conf.pb.tpl"
    with open(canbus_pb_txt_conf_file, 'r') as tpl:
        fmt = tpl.readlines()
    canbus_pb_txt_conf_file = output_dir + "canbus_conf.pb.txt"
    with open(canbus_pb_txt_conf_file, 'w') as pb_txt_conf:
        FMT = "".join(fmt)
        pb_txt_conf.write(FMT)

def gen_canbus_conf(config_file, output_dir, protocol_template_dir):
    print("Generating canbus conf")
    apollo_env = os.environ.get("APOLLO_DISTRIBUTION_HOME")
    is_apollo_source = False
    if apollo_env == "/opt/apollo/neo":
        # print("current in aem environment.")
        is_apollo_source = False
    elif apollo_env == "/apollo":
        # print("current in apollo source code environment")
        is_apollo_source = True
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    with open(config_file, 'r') as fp:
        content = yaml.safe_load(fp)
        gen_canbus_gflag_conf(content, output_dir, protocol_template_dir, is_apollo_source)
        gen_canbus_pb_txt_conf(content, output_dir, protocol_template_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('Usage: python %s some_config.yml' % sys.argv[0])
        sys.exit(0)

    with open(sys.argv[1], 'r') as fp:
        conf = yaml.safe_load(fp)
    protocol_conf = conf["protocol_conf"]

    output_dir = conf["output_dir"] + "vehicle/" + conf["car_type"].lower() + \
        "/"
    shutil.rmtree(output_dir, True)
    os.makedirs(output_dir)

    protocol_template_dir = sys.path[0] + "/template/"
    gen_canbus_conf(protocol_conf, output_dir, protocol_template_dir)
