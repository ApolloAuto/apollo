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

import datetime
import os
import shutil
import sys
import yaml
import re


def write_single_protocol_vars(pb_fp, p):
    pb_fp.write("\nmessage %s {\n" % p["name"].capitalize())
    if p["protocol_type"] == "control":
        pb_fp.write("// Control Message\n")
    elif p["protocol_type"] == "report":
        pb_fp.write("// Report Message\n")

    for var in p["vars"]:
        fmt = "    %s = %d;\n"
        if var["type"] == "enum":
            pb_fp.write("  enum %s {\n" % (var["name"].capitalize() + "Type"))
            for key in sorted(var["enum"]):
                pb_fp.write(fmt % (var["enum"][key], int(key)))
            pb_fp.write("  }\n")

    var_seq = 1
    for var in p["vars"]:
        fmt = "  optional %s %s = %d;\n"
        t = var["type"]
        if t == "int":
            t = "int32"

        pb_fp.write("  // ")
        if "description" in var:
            pb_fp.write("%s " % var["description"])
        pb_fp.write("[%s] %s\n" %
                    (var["physical_unit"], var["physical_range"]))
        if t == "enum":
            pb_fp.write(fmt % (var["name"].capitalize() + "Type",
                               var["name"].lower(), var_seq))
        else:
            pb_fp.write(fmt % (t, var["name"].lower(), var_seq))
        var_seq = var_seq + 1
    pb_fp.write("}\n")


def update_detail_pb(car_type):
    with open("../../canbus/proto/chassis_detail.proto", 'r+') as pb_fp:
        importline = "import \"modules/canbus/proto/" + car_type.lower(
        ) + ".proto\";\n"
        vehicleline = "    " + car_type.capitalize() + " " + car_type.lower()
        lines = pb_fp.readlines()
        importfound = False
        vehiclefound = False

        oneof = "oneof vehicle"
        index = 0
        startidx = 0
        for l in lines:
            if importline in l:
                importfound = True
            if vehicleline in l:
                vehiclefound = True
            if oneof in l:
                startidx = index
            index = index + 1

        startidx = startidx + 1
        count = 0
        while not "}" in lines[startidx]:
            count = int(lines[startidx].split()[-1][:-1])
            startidx = startidx + 1
        count = count + 1

        if not vehiclefound:
            lines.insert(startidx, vehicleline + " = " + str(count) + ";\n")

        if not importfound:
            lines.insert(4, importline)

        pb_fp.seek(0)
        for l in lines:
            pb_fp.write(l)


def gen_proto_file(config_file, work_dir):
    """
        config_file: the config file is generated with dbc
        work_dir: the protobuf file will be output
    """
    print "Generating proto file"
    if not os.path.exists(work_dir):
        os.makedirs(work_dir)
    with open(config_file, 'r') as fp:
        content = yaml.load(fp)
        protocols = content["protocols"]
        car_type = content["car_type"]
        with open("%s/%s.proto" % (work_dir, car_type.lower()), 'w') as pb_fp:
            pb_fp.write("syntax = \"proto2\";\n\npackage apollo.canbus;\n")
            for pid in protocols:
                p = protocols[pid]
                write_single_protocol_vars(pb_fp, p)
            pb_fp.write("\nmessage %s {\n" % car_type.capitalize())

            pb_var_seq = 1
            for p_name in protocols:
                p = protocols[p_name]
                pb_fp.write("  optional %s %s = %d;" %
                            (p["name"].capitalize(), p["name"], pb_var_seq))
                if protocols[p_name]["protocol_type"] == "control":
                    pb_fp.write(" // control message")
                if protocols[p_name]["protocol_type"] == "report":
                    pb_fp.write(" // report message")
                pb_fp.write("\n")
                pb_var_seq = pb_var_seq + 1
            pb_fp.write("}\n")

            #update_detail_pb(car_type)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "usage:\npython %s some_config.yml" % sys.argv[0]
        sys.exit(0)
    with open(sys.argv[1], 'r') as fp:
        conf = yaml.load(fp)
    protocol_conf = conf["protocol_conf"]

    work_dir = conf["output_dir"] + "proto/"
    gen_proto_file(protocol_conf, work_dir)
