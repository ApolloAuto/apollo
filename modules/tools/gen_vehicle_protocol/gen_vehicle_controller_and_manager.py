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


def gen_vehicle_controller_header(content, output_dir):
    controller_header_tpl_file = "template/controller.h.tpl"
    car_type = content["car_type"]
    with open(controller_header_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    controller_header_file = output_dir + (
        "%s_controller.h" % content["car_type"].lower())
    with open(controller_header_file, 'w') as header:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type
        fmt_val["car_type_upper"] = car_type.upper()
        fmt_val["car_type_cap"] = car_type.capitalize()

        control_protocol_include_list = []
        control_protocol_include_fmt = "#include \"modules/canbus/vehicle/%s/protocol/%s.h\""

        control_protocol_ptr_list = []
        control_protocol_ptr_fmt = "  %s* %s_ = nullptr;"

        protocols = content["protocols"]
        for pid in protocols:
            p = protocols[pid]
            if p["protocol_type"] == "control":
                name = p["name"]
                include = control_protocol_include_fmt % (car_type.lower(),
                                                          name.lower())
                control_protocol_include_list.append(include)

                var_classname = name.replace('_', '').capitalize()
                var_ptr = control_protocol_ptr_fmt % (var_classname, name)
                control_protocol_ptr_list.append(var_ptr)
        control_protocol_include_list.sort()
        control_protocol_ptr_list.sort()
        fmt_val["control_protocol_include_list"] = "\n".join(
            control_protocol_include_list)
        fmt_val["control_protocol_ptr_list"] = "\n".join(
            control_protocol_ptr_list)
        header.write(FMT % fmt_val)


def gen_vehicle_controller_cpp(content, output_dir):
    controller_cpp_tpl_file = "template/controller.cc.tpl"
    with open(controller_cpp_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    controller_cpp_file = output_dir + ("%s_controller.cc" % car_type.lower())
    with open(controller_cpp_file, 'w') as cpp:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fmt_val["car_type_cap"] = car_type.capitalize()

        protocol_ptr_get_list = []
        protocol_ptr_get_fmt = """  %(var_name)s_ = dynamic_cast<%(class_name)s*>
          (message_manager_->GetMutableProtocolDataById(%(class_name)s::ID));
  if (%(var_name)s_ == nullptr) {
     AERROR << "%(class_name)s does not exist in the %(car_type)sMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }
"""
        protocol_add_list = []
        protocol_add_fmt = "  can_sender_->AddMessage(%s::ID, %s_, false);"
        protocols = content["protocols"]
        for pid in protocols:
            p = protocols[pid]
            if p["protocol_type"] == "control":
                var_name = p["name"].lower()
                class_name = p["name"].replace('_', '').capitalize()
                ptr_get_fmt_val = {}
                ptr_get_fmt_val["var_name"] = var_name
                ptr_get_fmt_val["class_name"] = class_name
                ptr_get_fmt_val["car_type"] = car_type.capitalize()
                ptr_get = protocol_ptr_get_fmt % ptr_get_fmt_val
                protocol_ptr_get_list.append(ptr_get)

                protocol_add = protocol_add_fmt % (class_name, var_name)
                protocol_add_list.append(protocol_add)
        protocol_ptr_get_list.sort()
        protocol_add_list.sort()
        fmt_val["protocol_ptr_get_list"] = "\n".join(protocol_ptr_get_list)
        fmt_val["protocol_add_list"] = "\n".join(protocol_add_list)
        cpp.write(FMT % fmt_val)


def gen_message_manager_header(content, output_dir):
    message_manager_header_tpl_file = "template/message_manager.h.tpl"
    with open(message_manager_header_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    message_manager_header_file = output_dir + (
        "%s_message_manager.h" % car_type.lower())
    with open(message_manager_header_file, 'w') as header:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_namespace"] = car_type.lower()
        fmt_val["car_type_cap"] = car_type.capitalize()
        fmt_val["car_type_up"] = car_type.upper()
        header.write(FMT % fmt_val)


def gen_message_manager_cpp(content, output_dir):
    message_manager_cpp_tpl_file = "template/message_manager.cc.tpl"
    with open(message_manager_cpp_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    message_manager_cpp_file = output_dir + (
        "%s_message_manager.cc" % car_type.lower())
    with open(message_manager_cpp_file, 'w') as cpp:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fmt_val["car_type_cap"] = car_type.capitalize()
        protocols = content["protocols"]

        control_header_list = []
        report_header_list = []
        header_fmt = "#include \"modules/canbus/vehicle/%s/protocol/%s.h\""

        control_add_list = []
        report_add_list = []
        add_fmt = "  Add%sProtocolData<%s, true>();"
        for p_name in protocols:
            p = protocols[p_name]
            var_name = "%s" % p["name"].lower()
            class_name = p["name"].replace('_', '').capitalize()
            header = header_fmt % (car_type.lower(), var_name)
            if p["protocol_type"] == "control":
                control_header_list.append(header)
                item = add_fmt % ("Send", class_name)
                control_add_list.append(item)
            elif p["protocol_type"] == "report":
                report_header_list.append(header)
                item = add_fmt % ("Recv", class_name)
                report_add_list.append(item)
        control_header_list.sort()
        report_header_list.sort()
        control_add_list.sort()
        report_add_list.sort()
        fmt_val["control_header_list"] = "\n".join(control_header_list)
        fmt_val["report_header_list"] = "\n".join(report_header_list)
        fmt_val["control_add_list"] = "\n".join(control_add_list)
        fmt_val["report_add_list"] = "\n".join(report_add_list)
        cpp.write(FMT % fmt_val)


def gen_vehicle_factory_header(content, output_dir):
    vehicle_factory_header_tpl_file = "template/vehicle_factory.h.tpl"
    with open(vehicle_factory_header_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    vehicle_factory_header_file = output_dir + (
        "%s_vehicle_factory.h" % car_type.lower())
    with open(vehicle_factory_header_file, 'w') as header:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_cap"] = car_type.capitalize()
        fmt_val["car_type_upper"] = car_type.upper()
        fmt_val["car_type_lower"] = car_type.lower()
        header.write(FMT % fmt_val)


def gen_vehicle_factory_cpp(content, output_dir):
    vehicle_factory_cpp_tpl_file = "template/vehicle_factory.cc.tpl"
    with open(vehicle_factory_cpp_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    vehicle_factory_cpp_file = output_dir + (
        "%s_vehicle_factory.cc" % car_type.lower())
    with open(vehicle_factory_cpp_file, 'w') as cpp:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fmt_val["car_type_cap"] = car_type.capitalize()
        fmt_val["car_type_upper"] = car_type.upper()
        cpp.write(FMT % fmt_val)


def gen_build_file(content, output_dir):
    build_tpl_file = "template/controller_manager_BUILD.tpl"
    with open(build_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    build_file = output_dir + "BUILD"
    with open(build_file, 'w') as fp:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fp.write(FMT % fmt_val)


def gen_vehicle_controller_and_manager(config_file, output_dir):
    print("Generating controller and manager")
    with open(config_file, 'r') as fp:
        content = yaml.load(fp)
        gen_vehicle_controller_header(content, output_dir)
        gen_vehicle_controller_cpp(content, output_dir)
        gen_message_manager_header(content, output_dir)
        gen_message_manager_cpp(content, output_dir)
        gen_vehicle_factory_header(content, output_dir)
        gen_vehicle_factory_cpp(content, output_dir)
        gen_build_file(content, output_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('Usage: python %s some_config.yml' % sys.argv[0])
        sys.exit(0)

    with open(sys.argv[1], 'r') as fp:
        conf = yaml.load(fp)
    protocol_conf = conf["protocol_conf"]

    output_dir = conf["output_dir"] + "vehicle/" + conf["car_type"].lower() + \
        "/"
    shutil.rmtree(output_dir, True)
    os.makedirs(output_dir)
    gen_vehicle_controller_and_manager(protocol_conf, output_dir)
