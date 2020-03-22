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


def gen_report_header(car_type, protocol, output_dir):
    """
        doc string:
    """
    report_header_tpl_file = "template/report_protocol.h.tpl"
    FMT = get_tpl_fmt(report_header_tpl_file)
    report_header_file = output_dir + "%s.h" % protocol["name"]
    with open(report_header_file, 'w') as h_fp:
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type.lower()
        fmt_val["car_type_upper"] = car_type.upper()
        fmt_val["protocol_name_upper"] = protocol["name"].upper()
        fmt_val["classname"] = protocol["name"].replace('_', '').capitalize()
        func_declare_list = []
        for var in protocol["vars"]:
            fmt = """
  // config detail: %s
  %s %s(const std::uint8_t* bytes, const int32_t length) const;"""
            returntype = var["type"]
            if var["type"] == "enum":
                returntype = protocol["name"].capitalize(
                ) + "::" + var["name"].capitalize() + "Type"
            declare = fmt % (str(var), returntype, var["name"].lower())
            func_declare_list.append(declare)
        fmt_val["func_declare_list"] = "\n".join(func_declare_list)
        h_fp.write(FMT % fmt_val)


def gen_report_cpp(car_type, protocol, output_dir):
    """
        doc string:
    """
    report_cpp_tpl_file = "template/report_protocol.cc.tpl"
    FMT = get_tpl_fmt(report_cpp_tpl_file)
    report_cpp_file = output_dir + "%s.cc" % protocol["name"]
    with open(report_cpp_file, 'w') as fp:
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type
        fmt_val["protocol_name_lower"] = protocol["name"]
        classname = protocol["name"].replace('_', '').capitalize()
        fmt_val["classname"] = classname
        protocol_id = int(protocol["id"].upper(), 16)
        if protocol_id > 2048:
            fmt_val["id_upper"] = gen_esd_can_extended(protocol["id"].upper())
        else:
            fmt_val["id_upper"] = protocol["id"].upper()
        set_var_to_protocol_list = []
        func_impl_list = []
        for var in protocol["vars"]:
            var["name"] = var["name"].lower()

            returntype = var["type"]
            if var["type"] == "enum":
                returntype = protocol["name"].capitalize(
                ) + "::" + var["name"].capitalize() + "Type"
            # gen func top
            fmt = """
// config detail: %s
%s %s::%s(const std::uint8_t* bytes, int32_t length) const {"""
            impl = fmt % (str(var), returntype, classname, var["name"])

            byte_info = get_byte_info(var)
            impl = impl + gen_parse_value_impl(var, byte_info)

            impl = impl + gen_report_value_offset_precision(var, protocol)
            impl = impl + "}"

            func_impl_list.append(impl)
            proto_set_fmt = "  chassis->mutable_%s()->mutable_%s()->set_%s(%s(bytes, length));"
            func_name = var["name"]
            proto_set = proto_set_fmt % (car_type, protocol["name"], var["name"],
                                         func_name)
            set_var_to_protocol_list.append(proto_set)
        fmt_val["set_var_to_protocol_list"] = "\n".join(
            set_var_to_protocol_list)
        fmt_val["func_impl_list"] = "\n".join(func_impl_list)
        fp.write(FMT % fmt_val)


def gen_report_value_offset_precision(var, protocol):
    """
        doc string:
    """
    impl = ""
    if var["is_signed_var"]:
        fmt = "\n  x <<= %d;\n  x >>= %d;\n"
        # x is an int32_t var
        shift_bit = 32 - var["len"]
        impl = impl + fmt % (shift_bit, shift_bit)

    returntype = var["type"]
    if var["type"] == "enum":
        returntype = protocol["name"].capitalize() + "::" + var["name"].capitalize(
        ) + "Type"
    impl = impl + "\n  " + returntype + " ret = "

    if var["type"] == "enum":
        impl = impl + " static_cast<" + returntype + ">(x);\n"
    else:
        impl = impl + "x"
        if var["precision"] != 1.0:
            impl = impl + " * %f" % var["precision"]
        if var["offset"] != 0.0:
            impl = impl + " + %f" % (var["offset"])
        impl = impl + ";\n"
    return impl + "  return ret;\n"


def gen_parse_value_impl(var, byte_info):
    """
        doc string:
    """
    impl = ""
    fmt = "\n  Byte t%d(bytes + %d);\n"
    shift_bit = 0
    for i in range(0, len(byte_info)):
        info = byte_info[i]
        impl = impl + fmt % (i, info["byte"])
        if i == 0:
            impl = impl + "  int32_t x = t%d.get_byte(%d, %d);\n" %\
                (i, info["start_bit"], info["len"])
        elif i == 1:
            impl = impl + "  int32_t t = t%d.get_byte(%d, %d);\n  x <<= %d;\n  x |= t;\n" %\
                (i, info["start_bit"], info["len"], info["len"])
        else:
            impl = impl + "  t = t%d.get_byte(%d, %d);\n  x <<= %d;\n  x |= t;\n" %\
                (i, info["start_bit"], info["len"], info["len"])
        shift_bit = shift_bit + info["len"]
    return impl


def gen_control_header(car_type, protocol, output_dir):
    """
        doc string:
    """
    control_header_tpl_file = "template/control_protocol.h.tpl"
    FMT = get_tpl_fmt(control_header_tpl_file)
    control_header_file = output_dir + "%s.h" % protocol["name"]
    with open(control_header_file, 'w') as h_fp:
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type
        fmt_val["car_type_upper"] = car_type.upper()
        fmt_val["protocol_name_upper"] = protocol["name"].upper()
        classname = protocol["name"].replace('_', '').capitalize()
        fmt_val["classname"] = classname
        declare_public_func_list = []
        declare_private_func_list = []
        declare_private_var_list = []

        fmtpub = "\n  // config detail: %s\n  %s* set_%s(%s %s);"
        fmtpri = "\n  // config detail: %s\n  void set_p_%s(uint8_t* data, %s %s);"
        for var in protocol["vars"]:
            returntype = var["type"]
            if var["type"] == "enum":
                returntype = protocol["name"].capitalize(
                ) + "::" + var["name"].capitalize() + "Type"
            private_var = ""
            public_func_declare = fmtpub % (str(var), classname,
                                            var["name"].lower(), returntype,
                                            var["name"].lower())
            private_func_declare = fmtpri % (str(var), var["name"].lower(),
                                             returntype, var["name"].lower())

            private_var = "  %s %s_;" % (returntype, var["name"].lower())

            declare_private_var_list.append(private_var)
            declare_public_func_list.append(public_func_declare)
            declare_private_func_list.append(private_func_declare)

        fmt_val["declare_public_func_list"] = "\n".join(
            declare_public_func_list)
        fmt_val["declare_private_func_list"] = "\n".join(
            declare_private_func_list)
        fmt_val["declare_private_var_list"] = "\n".join(
            declare_private_var_list)
        h_fp.write(FMT % fmt_val)


def get_byte_info(var):
    """
        doc string: https://wenku.baidu.com/view/3fe9a7a4dd3383c4bb4cd293.html
        u can reference this link to known the difference between motorola and intel encoding
        return : the byte info of a variable in the protocol how many bytes are, and every byte use
                 how many bits, and bit start position
                 for the purpose of easily parsing value from CAN frame, the byte_info is arranged
                 from msb byte to lsb byte order
    """
    bit = var["bit"]
    byte_info = []
    left_len = var["len"]
    byte_idx = bit // 8
    bit_start = bit % 8
    if var["order"] == "motorola":
        while left_len > 0:
            info = {}
            info["byte"] = byte_idx
            info["len"] = min(bit_start + 1, left_len)
            # start_bit is always the lowest bit
            info["start_bit"] = bit_start - info["len"] + 1
            byte_info.append(info)
            left_len = left_len - info["len"]
            byte_idx = byte_idx + 1
            bit_start = 7
    else:
        while left_len > 0:
            info = {}
            info["byte"] = byte_idx
            info["len"] = min(8 - bit_start, left_len)
            info["start_bit"] = bit_start
            byte_info.append(info)
            left_len = left_len - info["len"]
            byte_idx = byte_idx + 1
            bit_start = 0
        # byte_info is always construct with msb(most significant bit) byte to lsb byte
        byte_info.reverse()
    return byte_info


def gen_control_decode_offset_precision(var):
    """
        doc string:
    """
    impl = "\n"
    range_info = get_range_info(var)
    if var["type"] == "double":
        if range_info["low"].find(".") == -1:
            range_info["low"] = "%s.0" % range_info["low"]
        if range_info["high"].find(".") == -1:
            range_info["high"] = "%s.0" % range_info["high"]

    if var["type"] != "enum" and var["type"] != "bool":
        impl = impl + "  %s = ProtocolData::BoundedValue(%s, %s, %s);\n" %\
            (var["name"].lower(), range_info["low"],
             range_info["high"], var["name"].lower())
    impl = impl + "  int x ="
    if var["offset"] != 0.0:
        impl = impl + " (%s - %f)" % (var["name"].lower(), var["offset"])
    else:
        impl = impl + " %s" % var["name"].lower()

    if var["precision"] != 1.0:
        impl = impl + " / %f" % var["precision"]
    return impl + ";\n"


def gen_control_encode_one_byte_value_impl(var, byte_info):
    """
        only has int and double, int can hold all the value whatever it is signed or unsigned
    """
    fmt = """
  Byte to_set(data + %d);
  to_set.set_value(x, %d, %d);
"""
    return fmt % (byte_info["byte"], byte_info["start_bit"], byte_info["len"])


def get_range_info(var):
    """
        doc string:
    """
    info = {}
    if "physical_range" not in var.keys():
        return info
    items = var["physical_range"].split('|')
    info["low"] = items[0].split('[')[1]
    info["high"] = items[1].split(']')[0]
    return info


def gen_control_encode_value_impl(var, byte_info):
    """
        doc string:
    """
    impl = "  uint8_t t = 0;\n"
    fmt = """
  t = x & %s;
  Byte to_set%d(data + %d);
  to_set%d.set_value(t, %d, %d);
"""
    shift_bit = 0
    for i in range(0, len(byte_info)):
        info = byte_info[i]
        if i != 0:
            impl = impl + "  x >>= %d;\n" % shift_bit
        mask_bit = "0x%X" % ((1 << info["len"]) - 1)
        impl = impl + fmt % (mask_bit, i, info["byte"], i, info["start_bit"],
                             info["len"])
        shift_bit = info["len"]
    return impl


def gen_control_value_func_impl(classname, var, protocol):
    """
        doc string:
    """
    impl = ""
    if var["len"] > 32:
        print("This generator not support big than four bytes var." +
              "protocol classname: %s, var_name:%s " % (
                  class_name, var["name"]))
        return impl

    fmt = """
%(classname)s* %(classname)s::set_%(var_name)s(
    %(var_type)s %(var_name)s) {
  %(var_name)s_ = %(var_name)s;
  return this;
 }

// config detail: %(config)s
void %(classname)s::set_p_%(var_name)s(uint8_t* data,
    %(var_type)s %(var_name)s) {"""
    fmt_val = {}
    fmt_val["classname"] = classname
    fmt_val["var_name"] = var["name"].lower()
    returntype = var["type"]
    if var["type"] == "enum":
        returntype = protocol["name"].capitalize() + "::" + var["name"].capitalize(
        ) + "Type"
    fmt_val["var_type"] = returntype
    fmt_val["config"] = str(var)
    impl = impl + fmt % fmt_val
    impl = impl + gen_control_decode_offset_precision(var)

    # get lsb to msb order
    byte_info = get_byte_info(var)
    byte_info.reverse()
    if len(byte_info) == 1:
        impl = impl + gen_control_encode_one_byte_value_impl(var, byte_info[0])
    else:
        impl = impl + gen_control_encode_value_impl(var, byte_info)

    return impl + "}\n"


def gen_control_cpp(car_type, protocol, output_dir):
    """
        doc string:
    """
    control_cpp_tpl_file = "template/control_protocol.cc.tpl"
    FMT = get_tpl_fmt(control_cpp_tpl_file)
    control_cpp_file = output_dir + "%s.cc" % protocol["name"]
    with open(control_cpp_file, 'w') as fp:
        fmt_val = {}
        fmt_val["car_type_lower"] = car_type
        fmt_val["protocol_name_lower"] = protocol["name"]
        protocol_id = int(protocol["id"].upper(), 16)
        if protocol_id > 2048:
            fmt_val["id_upper"] = gen_esd_can_extended(protocol["id"].upper())
        else:
            fmt_val["id_upper"] = protocol["id"].upper()
        classname = protocol["name"].replace('_', '').capitalize()
        fmt_val["classname"] = classname

        set_private_var_list = []
        set_private_var_init_list = []
        set_func_impl_list = []
        for var in protocol["vars"]:
            func_impl = gen_control_value_func_impl(classname, var, protocol)
            set_func_impl_list.append(func_impl)
            set_private_var = "  set_p_%s(data, %s_);" % (var["name"].lower(),
                                                          var["name"].lower())
            set_private_var_list.append(set_private_var)
            init_val = "0"
            if var["type"] == "double":
                init_val = "0.0"
            elif var["type"] == "bool":
                init_val = "false"
            elif var["type"] == "enum":
                if 0 in var["enum"]:
                    init_val = protocol["name"].capitalize(
                    ) + "::" + var["enum"][0].upper()
                else:
                    init_val = protocol["name"].capitalize(
                    ) + "::" + list(var["enum"].values())[0].upper()

            set_private_var_init_list.append("  %s_ = %s;" %
                                             (var["name"].lower(), init_val))
        fmt_val["set_private_var_list"] = "\n".join(set_private_var_list)
        fmt_val["set_private_var_init_list"] = "\n".join(
            set_private_var_init_list)
        fmt_val["set_func_impl_list"] = "\n".join(set_func_impl_list)
        fp.write(FMT % fmt_val)


def get_tpl_fmt(tpl_file):
    """
        get fmt from tpl_file
    """
    with open(tpl_file, 'r') as tpl:
        fmt = tpl.readlines()
    fmt = "".join(fmt)
    return fmt


def gen_build_file(car_type, work_dir):
    """
        doc string:
    """
    build_tpl_file = "template/protocol_BUILD.tpl"
    fmt = get_tpl_fmt(build_tpl_file)
    with open(work_dir + "BUILD", "w") as build_fp:
        fmt_var = {}
        fmt_var["car_type"] = car_type.lower()
        build_fp.write(fmt % fmt_var)


def gen_protocols(protocol_conf_file, protocol_dir):
    """
        doc string:
    """
    print("Generating protocols")
    if not os.path.exists(protocol_dir):
        os.makedirs(protocol_dir)
    with open(protocol_conf_file, 'r') as fp:
        content = yaml.load(fp)
        protocols = content["protocols"]
        car_type = content["car_type"]
        for p_name in protocols:
            protocol = protocols[p_name]

            if protocol["protocol_type"] == "report":
                gen_report_header(car_type, protocol, protocol_dir)
                gen_report_cpp(car_type, protocol, protocol_dir)
            elif protocol["protocol_type"] == "control":
                gen_control_header(car_type, protocol, protocol_dir)
                gen_control_cpp(car_type, protocol, protocol_dir)

            else:
                print("Unknown protocol_type:%s" % protocol["protocol_type"])
        gen_build_file(car_type, protocol_dir)


def gen_esd_can_extended(str):
    """
        id string:
    """
    int_id = int(str, 16)
    int_id &= 0x1FFFFFFF
    int_id |= 0x20000000
    str = hex(int_id).replace('0x', '')
    return str


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage:\npython %s some_config.yml" % sys.argv[0])
        sys.exit(0)
    with open(sys.argv[1], 'r') as fp:
        conf = yaml.load(fp)
    protocol_conf = conf["protocol_conf"]

    protocol_dir = conf["output_dir"] + "vehicle/" + conf["car_type"].lower(
    ) + "/protocol/"
    shutil.rmtree(output_dir, True)
    os.makedirs(output_dir)
    gen_protocols(protocol_conf, protocol_dir)
