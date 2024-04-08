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

import re
import shlex
import sys

import yaml


MAX_CAN_ID = 4096000000  # include can extended ID
STANDARD_CAN_ID = 2048


def extract_var_info(items):
    """
       Desp: extract var info from line split items.
    """
    car_var = {}
    car_var["name"] = items[1]
    car_var["bit"] = int(items[3].split('|')[0])
    car_var["len"] = int(items[3].split('|')[1].split('@')[0])
    order_sign = items[3].split('|')[1].split('@')[1]
    if order_sign == "0+":
        car_var["order"] = "motorola"
        car_var["is_signed_var"] = False
    elif order_sign == "0-":
        car_var["order"] = "motorola"
        car_var["is_signed_var"] = True
    elif order_sign == "1+":
        car_var["order"] = "intel"
        car_var["is_signed_var"] = False
    elif order_sign == "1-":
        car_var["order"] = "intel"
        car_var["is_signed_var"] = True
    car_var["offset"] = float(items[4].split(',')[1].split(')')[0])
    car_var["precision"] = float(items[4].split(',')[0].split('(')[1])
    car_var["physical_range"] = items[5]
    car_var["physical_unit"] = items[6].replace('_', ' ')
    if car_var["len"] == 1:
        car_var["type"] = "bool"
    elif car_var["physical_range"].find(
            ".") != -1 or car_var["precision"] != 1.0:
        car_var["type"] = "double"
    else:
        car_var["type"] = "int"

    return car_var

def extract_description_info(items):
    car_var = {}
    car_var["description"] = items[4][:-1]
    print("var description is", car_var["description"])



def extract_dbc_meta(dbc_file, out_file, car_type, black_list, sender_list,
                     sender):
    """
        the main gen_config func, use dbc file to gen a yaml file
        parse every line, if the line is:
        eg:BO_ 1104 BMS_0x450: 8 VCU
        5 segments, and segments[0] is "BO_", then begin parse every signal in the following line

    """
    sender_list = map(str, sender_list)
    with open(dbc_file) as fp:
        in_protocol = False
        protocols = {}
        protocol = {}
        p_name = ""
        line_num = 0
        for line in fp:
            items = shlex.split(line)
            line_num = line_num + 1
            if len(items) == 5 and items[0] == "BO_":
                p_name = items[2][:-1].lower()
                # print ("p_name is ", p_name)
                protocol = {}
                if int(items[1]) > MAX_CAN_ID:
                    continue
                protocol["id"] = "%x" % int(items[1])
                if int(items[1]) > STANDARD_CAN_ID:
                    protocol["id"] = gen_can_id_extended(protocol["id"])
                protocol["name"] = "%s_%s" % (p_name, protocol["id"])
                if "throttle" in protocol["name"]:
                    protocol["protocol_category"] = "throttle"
                if "brake" in protocol["name"]:
                    protocol["protocol_category"] = "brake"
                if "steer" in protocol["name"]:
                    protocol["protocol_category"] = "steer"
                if "gear" in protocol["name"]:
                    protocol["protocol_category"] = "gear"
                protocol["sender"] = items[4]
                if protocol["id"] in black_list:
                    continue
                protocol["protocol_type"] = "report"
                if protocol["id"] in sender_list or protocol["sender"] == sender:
                    protocol["protocol_type"] = "control"
                protocol["vars"] = []
                in_protocol = True
            elif in_protocol:
                if len(items) > 3 and items[0] == "SG_":
                    if items[2] == ":":
                        var_info = extract_var_info(items)
                        # current we can't process than 4 byte value
                        if var_info["len"] <= 32:
                            protocol["vars"].append(var_info)
                else:
                    in_protocol = False
                    if len(protocol) != 0 and len(protocol["vars"]) != 0 and len(
                            protocol["vars"]) < 65:
                        protocols[protocol["id"]] = protocol
                        # print protocol
                        protocol = {}

            if len(items) == 5 and items[0] == "CM_" and items[1] == "SG_":
                protocol_id = "%x" % int(items[2])
                if int(items[2]) > MAX_CAN_ID:
                    continue
                if int(items[2]) > STANDARD_CAN_ID:
                    protocol_id = gen_can_id_extended(protocol_id)
                for var in protocols[protocol_id]["vars"]:
                    # print("var is", var)
                    if var["name"] == items[3]:
                        var["description"] = items[4][:-1]
                        # print("var description is ", var["description"])
                        if "enable" in var["description"]:
                            var["signal_type"] = "enable"
                        if "command" in var["description"]:
                            var["signal_type"] = "command"
                        if "speed" in var["description"]:
                            var["signal_type"] = "speed"
                        # extract_description_info(items)

            if len(items) > 2 and items[0] == "VAL_":
                protocol_id = "%x" % int(items[1])
                if int(items[1]) > MAX_CAN_ID:
                    continue
                if int(items[1]) > STANDARD_CAN_ID:
                    protocol_id = gen_can_id_extended(protocol_id)
                for var in protocols[protocol_id]["vars"]:
                    if var["name"] == items[2]:
                        var["type"] = "enum"
                        var["enum"] = {}
                        for idx in range(3, len(items) - 1, 2):
                            enumtype = re.sub('\W+', ' ', items[idx + 1])
                            enumtype = enumtype.strip().replace(" ",
                                                                "_").upper()
                            enumtype = items[2].upper() + "_" + enumtype
                            var["enum"][int(items[idx])] = enumtype

        cpp_reserved_key_words = ['minor', 'major', 'long', 'int']
        for key in protocols:
            for var in protocols[key]["vars"]:
                if var["name"].lower() in cpp_reserved_key_words:
                    var["name"] = "MY_" + var["name"]

        # print protocols
        config = {}
        config["car_type"] = car_type
        config["protocols"] = protocols
        with open(out_file, 'w') as fp_write:
            fp_write.write(yaml.dump(config))

        control_protocol_num =\
            len([key for key in protocols.keys()
                 if protocols[key]["protocol_type"] == "control"])
        report_protocol_num =\
            len([key for key in protocols.keys()
                 if protocols[key]["protocol_type"] == "report"])
        print("Extract car_type:%s's protocol meta info to file: %s" % (
            car_type.upper(), out_file))
        print("Total parsed protocols: %d" % len(protocols))
        print("Control protocols: %d" % control_protocol_num)
        print("Report protocols: %d" % report_protocol_num)
        return True

def gen_can_id_extended(str):
    """
        id string:
    """
    int_id = int(str, 16)
    int_id &= 0x1FFFFFFF
    str = hex(int_id).replace('0x', '')
    return str


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage:\npython %s your_car_parse_config_file.yml" % sys.argv[0])
        sys.exit(0)
    with open(sys.argv[1], 'r') as fp:
        conf = yaml.safe_load(fp)
    dbc_file = conf["dbc_file"]
    protocol_conf_file = conf["protocol_conf"]
    car_type = conf["car_type"]
    black_list = conf["black_list"]
    sender_list = conf["sender_list"]
    sender = conf["sender"]
    extract_dbc_meta(dbc_file, protocol_conf_file, car_type, black_list,
                     sender_list, sender)
