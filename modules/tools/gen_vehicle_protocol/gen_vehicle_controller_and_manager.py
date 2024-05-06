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


def gen_vehicle_controller_header(content, output_dir, protocol_template_dir):
    controller_header_tpl_file = protocol_template_dir + "controller.h.tpl"
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
        control_protocol_include_fmt = "#include \"modules/canbus_vehicle/%s/protocol/%s.h\""

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


def gen_vehicle_controller_cpp(content, output_dir, protocol_template_dir, use_demo_dbc=False):
    if not use_demo_dbc:
        # print("not use demo")
        controller_cpp_tpl_file = protocol_template_dir + "controller.cc.tpl"
    else:
        # print("use demo")
        controller_cpp_tpl_file = protocol_template_dir + "controllerdemo.cc.tpl"
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
        protocol_ptr_get_fmt = """  %(protocol_name)s_ = dynamic_cast<%(class_name)s*>
          (message_manager_->GetMutableProtocolDataById(%(class_name)s::ID));
  if (%(protocol_name)s_ == nullptr) {
     AERROR << "%(class_name)s does not exist in the %(car_type)sMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }
"""
        protocol_add_list = []
        protocol_add_fmt = "  can_sender_->AddMessage(%s::ID, %s_, false);"

        protocol_chassis_get_list = []
        protocol_chassis_speed_fmt = """  // 4 chassis spd
  if (chassis_detail.has_%(speed_protocol_name)s() &&
      chassis_detail.%(speed_protocol_name)s().has_%(speed_reoport_name)s()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.%(speed_protocol_name)s().%(speed_reoport_name)s()));
  } else {
    chassis_.set_speed_mps(0);
  }
"""
        protocol_chassis_throttle_fmt = """  // 5 throttle
  if (chassis_detail.has_%(throttle_protocol_name)s() &&
      chassis_detail.%(throttle_protocol_name)s().has_%(throttle_report_name)s()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.%(throttle_protocol_name)s().%(throttle_report_name)s()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
"""
        protocol_chassis_brake_fmt = """  // 6 brake
  if (chassis_detail.has_%(brake_protocol_name)s() &&
      chassis_detail.%(brake_protocol_name)s().has_%(brake_report_name)s()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.%(brake_protocol_name)s().%(brake_report_name)s()));
  } else {
    chassis_.set_brake_percentage(0);
  }
"""
        protocol_chassis_gear_fmt = """  // 7 gear
  if (chassis_detail.has_%(gear_protocol_name)s() &&
      chassis_detail.%(gear_protocol_name)s().has_%(gear_report_name)s()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.%(gear_protocol_name)s().%(gear_report_name)s() ==
        %(gear_report_protocol_name_cap)s::%(gear_report_neutral_enum)s) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.%(gear_protocol_name)s().%(gear_report_name)s() ==
        %(gear_report_protocol_name_cap)s::%(gear_report_reverse_enum)s) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.%(gear_protocol_name)s().%(gear_report_name)s() ==
        %(gear_report_protocol_name_cap)s::%(gear_report_drive_enum)s) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.%(gear_protocol_name)s().%(gear_report_name)s() ==
        %(gear_report_protocol_name_cap)s::%(gear_report_park_enum)s) {
      gear_pos = Chassis::GEAR_PARKING;
    }

    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
"""
        protocol_chassis_steer_fmt = """  // 8 steer
  if (chassis_detail.has_%(steer_protocol_name)s() &&
      chassis_detail.%(steer_protocol_name)s().has_%(steer_report_name)s()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.%(steer_protocol_name)s().%(steer_report_name)s() * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }
"""
        protocol_chassis_brake_enable_fmt = """  if (chassis_detail.has_%(brake_protocol_name)s() &&
      chassis_detail.%(brake_protocol_name)s().has_%(brake_report_enable_name)s()) {
    chassis_.mutable_check_response()->set_is_esp_online(
        chassis_detail.%(brake_protocol_name)s().%(brake_report_enable_name)s() == 1);
  }
"""
        protocol_chassis_throttle_enable_fmt = """  if (chassis_detail.has_%(throttle_protocol_name)s() &&
      chassis_detail.%(throttle_protocol_name)s().has_%(throttle_report_enable_name)s()) {
    chassis_.mutable_check_response()->set_is_vcu_online(
        chassis_detail.%(throttle_protocol_name)s().%(throttle_report_enable_name)s() == 1);
  }
"""
        protocol_chassis_steer_enable_fmt = """  if (chassis_detail.has_%(steer_protocol_name)s() &&
      chassis_detail.%(steer_protocol_name)s().has_%(steer_report_enable_name)s()) {
    chassis_.mutable_check_response()->set_is_eps_online(
        chassis_detail.%(steer_protocol_name)s().%(steer_report_enable_name)s() == 1);
  }
"""
        protocol_chassis_enable_fmt = "  // 9 checkresponse signal"

        protocol_auto_enable_add_list = []
        protocol_steer_enable_add_list = []
        protocol_speed_enable_add_list = []
        protocol_brake_command_list = []
        protocol_throttle_command_list = []
        protocol_steer_command_list = []
        protocol_gear_command_list = []
        protocol_brake_command_fmt = "%(brake_command_protocol_name)s_->set_%(brake_command_name)s(pedal);"
        protocol_brake_enable_command_fmt = """  %(brake_command_protocol_name)s_->set_%(brake_command_enable_name)s(
      %(brake_command_protocol_name_cap)s::%(brake_command_enable_enable_enum)s);
"""
        protocol_brake_disenable_command_fmt = """  %(brake_command_protocol_name)s_->set_%(brake_command_enable_name)s(
      %(brake_command_protocol_name_cap)s::%(brake_command_enable_disable_enum)s);
"""
        protocol_throttle_command_fmt = "%(throttle_command_protocol_name)s_->set_%(throttle_command_name)s(pedal);"
        protocol_throttle_enable_command_fmt = """  %(throttle_command_protocol_name)s_->set_%(throttle_command_enable_name)s(
      %(throttle_command_protocol_name_cap)s::%(throttle_command_enable_enable_enum)s);
"""
        protocol_throttle_disenable_command_fmt = """  %(throttle_command_protocol_name)s_->set_%(throttle_command_enable_name)s(
      %(throttle_command_protocol_name_cap)s::%(throttle_command_enable_disable_enum)s);
"""
        protocol_steer_command_fmt = """const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  %(steer_command_protocol_name)s_->set_%(steer_command_name)s(real_angle);
"""
        protocol_steer_enable_command_fmt = """  %(steer_command_protocol_name)s_->set_%(steer_command_enable_name)s(
      %(steer_command_protocol_name_cap)s::%(steer_command_enable_enable_enum)s);
"""
        protocol_steer_disenable_command_fmt = """  %(steer_command_protocol_name)s_->set_%(steer_command_enable_name)s(
      %(steer_command_protocol_name_cap)s::%(steer_command_enable_disable_enum)s);
"""
        protocol_gear_command_fmt = """switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_neutral_enum)s);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_reverse_enum)s);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_drive_enum)s);
      break;
    }
    case Chassis::GEAR_PARKING: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_park_enum)s);
      break;
    }
    case Chassis::GEAR_INVALID: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_neutral_enum)s);
      break;
    }
    default: {
      %(gear_command_protocol_name)s_->set_%(gear_command_name)s(%(gear_command_protocol_name_cap)s::%(gear_command_neutral_enum)s);
      break;
    }
  }
"""
        protocol_gear_enable_command_fmt = """  %(gear_command_protocol_name)s_->set_%(gear_command_enable_name)s(
      %(gear_command_protocol_name_cap)s::%(gear_command_enable_enable_enum)s);
"""
        protocol_gear_disenable_command_fmt = """  %(gear_command_protocol_name)s_->set_%(gear_command_enable_name)s(
      %(gear_command_protocol_name_cap)s::%(gear_command_enable_disable_enum)s);
"""


        protocols = content["protocols"]
        for pid in protocols:
            # print(pid)
            p = protocols[pid]
            # print("p is ", p)
            if p["protocol_type"] == "control":
                protocol_name = p["name"].lower()
                class_name = p["name"].replace('_', '').capitalize()
                ptr_get_fmt_val = {}
                ptr_get_fmt_val["protocol_name"] = protocol_name
                ptr_get_fmt_val["class_name"] = class_name
                ptr_get_fmt_val["car_type"] = car_type.capitalize()
                ptr_get = protocol_ptr_get_fmt % ptr_get_fmt_val
                protocol_ptr_get_list.append(ptr_get)

                protocol_add = protocol_add_fmt % (class_name, protocol_name)
                protocol_add_list.append(protocol_add)

                for var in p["vars"]:
                    # print("var keys is", var.keys())
                    if "signal_type" in var.keys():
                        if "command" in var["signal_type"]:
                            if "protocol_category" in p.keys():
                                if p["protocol_category"] == "throttle":
                                    fmt_val["throttle_command_name"] = var["name"].lower()
                                    fmt_val["throttle_command_protocol_name"] = p["name"].lower()
                                    throttle_command_get = protocol_throttle_command_fmt % fmt_val
                                    protocol_throttle_command_list.append(throttle_command_get)
                                if p["protocol_category"] == "brake":
                                    fmt_val["brake_command_name"] = var["name"].lower()
                                    fmt_val["brake_command_protocol_name"] = p["name"].lower()
                                    brake_command_get = protocol_brake_command_fmt % fmt_val
                                    protocol_brake_command_list.append(brake_command_get)
                                if p["protocol_category"] == "steer":
                                    fmt_val["steer_command_name"] = var["name"].lower()
                                    fmt_val["steer_command_protocol_name"] = p["name"].lower()
                                    steer_command_get = protocol_steer_command_fmt % fmt_val
                                    protocol_steer_command_list.append(steer_command_get)
                                if p["protocol_category"] == "gear":
                                    fmt_val["gear_command_name"] = var["name"].lower()
                                    fmt_val["gear_command_protocol_name"] = p["name"].lower()
                                    fmt_val["gear_command_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["gear_command_park_enum"] = var["enum"][var_enum_key_list[1]]
                                        fmt_val["gear_command_reverse_enum"] = var["enum"][var_enum_key_list[2]]
                                        fmt_val["gear_command_neutral_enum"] = var["enum"][var_enum_key_list[3]]
                                        fmt_val["gear_command_drive_enum"] = var["enum"][var_enum_key_list[4]]
                                    gear_command_get = protocol_gear_command_fmt % fmt_val
                                    protocol_gear_command_list.append(gear_command_get)
                        if "enable" in var["signal_type"]:
                            if "protocol_category" in p.keys():
                                if p["protocol_category"] == "throttle":
                                    fmt_val["throttle_command_enable_name"] = var["name"].lower()
                                    fmt_val["throttle_command_protocol_name"] = p["name"].lower()
                                    fmt_val["throttle_command_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["throttle_command_enable_disable_enum"] = var["enum"][var_enum_key_list[0]]
                                        fmt_val["throttle_command_enable_enable_enum"] = var["enum"][var_enum_key_list[1]]
                                    protocol_auto_enable_add_list.append(protocol_throttle_enable_command_fmt % fmt_val)
                                    protocol_steer_enable_add_list.append(protocol_throttle_disenable_command_fmt % fmt_val)
                                    protocol_speed_enable_add_list.append(protocol_throttle_enable_command_fmt % fmt_val)
                                if p["protocol_category"] == "brake":
                                    fmt_val["brake_command_enable_name"] = var["name"].lower()
                                    fmt_val["brake_command_protocol_name"] = p["name"].lower()
                                    fmt_val["brake_command_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["brake_command_enable_disable_enum"] = var["enum"][var_enum_key_list[0]]
                                        fmt_val["brake_command_enable_enable_enum"] = var["enum"][var_enum_key_list[1]]
                                    protocol_auto_enable_add_list.append(protocol_brake_enable_command_fmt % fmt_val)
                                    protocol_steer_enable_add_list.append(protocol_brake_disenable_command_fmt % fmt_val)
                                    protocol_speed_enable_add_list.append(protocol_brake_enable_command_fmt % fmt_val)
                                if p["protocol_category"] == "steer":
                                    fmt_val["steer_command_enable_name"] = var["name"].lower()
                                    fmt_val["steer_command_protocol_name"] = p["name"].lower()
                                    fmt_val["steer_command_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["steer_command_enable_disable_enum"] = var["enum"][var_enum_key_list[0]]
                                        fmt_val["steer_command_enable_enable_enum"] = var["enum"][var_enum_key_list[1]]
                                    protocol_auto_enable_add_list.append(protocol_steer_enable_command_fmt % fmt_val)
                                    protocol_steer_enable_add_list.append(protocol_steer_enable_command_fmt % fmt_val)
                                    protocol_speed_enable_add_list.append(protocol_steer_disenable_command_fmt % fmt_val)
                                if p["protocol_category"] == "gear":
                                    fmt_val["gear_command_enable_name"] = var["name"].lower()
                                    fmt_val["gear_command_protocol_name"] = p["name"].lower()
                                    fmt_val["gear_command_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["gear_command_enable_disable_enum"] = var["enum"][var_enum_key_list[0]]
                                        fmt_val["gear_command_enable_enable_enum"] = var["enum"][var_enum_key_list[1]]
                                    protocol_auto_enable_add_list.append(protocol_gear_enable_command_fmt % fmt_val)
                                    protocol_steer_enable_add_list.append(protocol_gear_disenable_command_fmt % fmt_val)
                                    protocol_speed_enable_add_list.append(protocol_gear_enable_command_fmt % fmt_val)

            if p["protocol_type"] == "report":
                protocol_name = p["name"].lower()
                for var in p["vars"]:
                    # print("var is", var.keys())
                    if "signal_type" in var.keys():
                        if "speed" in var["signal_type"]:
                            fmt_val["speed_reoport_name"] = var["name"].lower()
                            fmt_val["speed_protocol_name"] = p["name"].lower()
                            speed_get = protocol_chassis_speed_fmt % fmt_val
                            protocol_chassis_get_list.append(speed_get)
                        if "command" in var["signal_type"]:
                            # print("p keys is ", p.keys())
                            if "protocol_category" in p.keys():
                                if p["protocol_category"] == "throttle":
                                    fmt_val["throttle_report_name"] = var["name"].lower()
                                    fmt_val["throttle_protocol_name"] = p["name"].lower()
                                    throttle_get = protocol_chassis_throttle_fmt % fmt_val
                                    protocol_chassis_get_list.append(throttle_get)
                                if p["protocol_category"] == "brake":
                                    fmt_val["brake_report_name"] = var["name"].lower()
                                    fmt_val["brake_protocol_name"] = p["name"].lower()
                                    brake_get = protocol_chassis_brake_fmt % fmt_val
                                    protocol_chassis_get_list.append(brake_get)
                                if p["protocol_category"] == "steer":
                                    fmt_val["steer_report_name"] = var["name"].lower()
                                    fmt_val["steer_protocol_name"] = p["name"].lower()
                                    steer_get = protocol_chassis_steer_fmt % fmt_val
                                    protocol_chassis_get_list.append(steer_get)
                                if p["protocol_category"] == "gear":
                                    fmt_val["gear_report_name"] = var["name"].lower()
                                    fmt_val["gear_protocol_name"] = p["name"].lower()
                                    fmt_val["gear_report_protocol_name_cap"] = p["name"].capitalize()
                                    if var["type"] == "enum":
                                        var_enum_key_list = list(var["enum"].keys())
                                        fmt_val["gear_report_park_enum"] = var["enum"][var_enum_key_list[1]]
                                        fmt_val["gear_report_reverse_enum"] = var["enum"][var_enum_key_list[2]]
                                        fmt_val["gear_report_neutral_enum"] = var["enum"][var_enum_key_list[3]]
                                        fmt_val["gear_report_drive_enum"] = var["enum"][var_enum_key_list[4]]
                                    gear_get = protocol_chassis_gear_fmt % fmt_val
                                    protocol_chassis_get_list.append(gear_get)
                        if "enable" in var["signal_type"]:
                            if "protocol_category" in p.keys():
                                protocol_chassis_get_list.append(protocol_chassis_enable_fmt)
                                if p["protocol_category"] == "throttle":
                                    fmt_val["throttle_protocol_name"] = p["name"].lower()
                                    fmt_val["throttle_report_enable_name"] = var["name"].lower()
                                    throttle_enable_get = protocol_chassis_throttle_enable_fmt % fmt_val
                                    protocol_chassis_get_list.append(throttle_enable_get)
                                if p["protocol_category"] == "brake":
                                    fmt_val["brake_protocol_name"] = p["name"].lower()
                                    fmt_val["brake_report_enable_name"] = var["name"].lower()
                                    brake_enable_get = protocol_chassis_brake_enable_fmt % fmt_val
                                    protocol_chassis_get_list.append(brake_enable_get)
                                if p["protocol_category"] == "steer":
                                    fmt_val["steer_report_enable_name"] = var["name"].lower()
                                    fmt_val["steer_protocol_name"] = p["name"].lower()
                                    steer_enable_get = protocol_chassis_steer_enable_fmt % fmt_val
                                    protocol_chassis_get_list.append(steer_enable_get)

        # print(fmt_val["gear_report_name"])

        protocol_ptr_get_list.sort()
        protocol_add_list.sort()
        fmt_val["protocol_ptr_get_list"] = "\n".join(protocol_ptr_get_list)
        fmt_val["protocol_add_list"] = "\n".join(protocol_add_list)

        if use_demo_dbc:
            protocol_chassis_get_list.sort()
            protocol_auto_enable_add_list.sort()
            protocol_steer_enable_add_list.sort()
            protocol_speed_enable_add_list.sort()
            fmt_val["protocol_chassis_get_list"] = "\n".join(protocol_chassis_get_list)
            fmt_val["protocol_auto_enable_add_list"] = "\n".join(protocol_auto_enable_add_list)
            fmt_val["protocol_steer_enable_add_list"] = "\n".join(protocol_steer_enable_add_list)
            fmt_val["protocol_speed_enable_add_list"] = "\n".join(protocol_speed_enable_add_list)
            fmt_val["protocol_brake_command_list"] = "\n".join(protocol_brake_command_list)
            fmt_val["protocol_throttle_command_list"] = "\n".join(protocol_throttle_command_list)
            fmt_val["protocol_steer_command_list"] = "\n".join(protocol_steer_command_list)
            fmt_val["protocol_gear_command_list"] = "\n".join(protocol_gear_command_list)

        cpp.write(FMT % fmt_val)


def gen_message_manager_header(content, output_dir, protocol_template_dir):
    message_manager_header_tpl_file = protocol_template_dir + "message_manager.h.tpl"
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
        fmt_val["car_type_lower"] = car_type.lower()
        header.write(FMT % fmt_val)


def gen_message_manager_cpp(content, output_dir, protocol_template_dir):
    message_manager_cpp_tpl_file = protocol_template_dir + "message_manager.cc.tpl"
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
        header_fmt = "#include \"modules/canbus_vehicle/%s/protocol/%s.h\""

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


def gen_vehicle_factory_header(content, output_dir, protocol_template_dir):
    vehicle_factory_header_tpl_file = protocol_template_dir + "vehicle_factory.h.tpl"
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


def gen_vehicle_factory_cpp(content, output_dir, protocol_template_dir):
    vehicle_factory_cpp_tpl_file = protocol_template_dir + "vehicle_factory.cc.tpl"
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


def gen_build_file(content, output_dir, protocol_template_dir):
    build_tpl_file = protocol_template_dir + "controller_manager_BUILD.tpl"
    with open(build_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    build_file = output_dir + "BUILD"
    with open(build_file, 'w') as fp:
        FMT = "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        protocols = content["protocols"]

        control_header_list = []
        report_header_list = []
        control_cpp_list = []
        report_cpp_list = []
        header_fmt = "\"protocol/%s.h\","
        cpp_fmt = "\"protocol/%s.cc\","

        for p_name in protocols:
            p = protocols[p_name]
            var_name = "%s" % p["name"].lower()
            header = header_fmt % var_name
            cpp = cpp_fmt % var_name
            if p["protocol_type"] == "control":
                control_header_list.append(header)
                control_cpp_list.append(cpp)
            elif p["protocol_type"] == "report":
                report_header_list.append(header)
                report_cpp_list.append(cpp)
        control_header_list.sort()
        report_header_list.sort()
        control_cpp_list.sort()
        report_cpp_list.sort()
        fmt_val["control_header_list"] = "\n        ".join(control_header_list)
        fmt_val["report_header_list"] = "\n        ".join(report_header_list)
        fmt_val["control_cpp_list"] = "\n        ".join(control_cpp_list)
        fmt_val["report_cpp_list"] = "\n        ".join(report_cpp_list)
        fp.write(FMT % fmt_val)

def gen_cyberfile(content, output_dir, protocol_template_dir):
    cyberfile_tpl_file = protocol_template_dir + "cyberfile.xml.tpl"
    with open(cyberfile_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    cyberfile_file = output_dir + "cyberfile.xml"
    with open(cyberfile_file, 'w') as xml:
        FMT =  "".join(fmt)
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        xml.write(FMT % fmt_val)

def gen_canbus_vehicle_build_file(content, output_dir, protocol_template_dir):
    canbus_vehicle_build_tpl_file = protocol_template_dir + "canbus-vehicle_BUILD.tpl"
    with open(canbus_vehicle_build_tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    car_type = content["car_type"]
    canbus_vehicle_build_file = output_dir + ("canbus-vehicle-%s.BUILD" % car_type.lower())
    with open(canbus_vehicle_build_file, 'w') as fp:
        FMT = "".join(fmt)
        fp.write(FMT)

def gen_vehicle_controller_and_manager(config_file, output_dir, protocol_template_dir, use_demo_dbc):
    print("Generating controller and manager")
    with open(config_file, 'r') as fp:
        content = yaml.safe_load(fp)
        gen_vehicle_controller_header(content, output_dir, protocol_template_dir)
        gen_vehicle_controller_cpp(content, output_dir, protocol_template_dir, use_demo_dbc)
        gen_message_manager_header(content, output_dir, protocol_template_dir)
        gen_message_manager_cpp(content, output_dir, protocol_template_dir)
        gen_vehicle_factory_header(content, output_dir, protocol_template_dir)
        gen_vehicle_factory_cpp(content, output_dir, protocol_template_dir)
        gen_build_file(content, output_dir, protocol_template_dir)
        gen_cyberfile(content, output_dir, protocol_template_dir)

        # gen_canbus_vehicle_build_file(content, output_dir, protocol_template_dir)


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
    gen_vehicle_controller_and_manager(protocol_conf, output_dir, protocol_template_dir)
