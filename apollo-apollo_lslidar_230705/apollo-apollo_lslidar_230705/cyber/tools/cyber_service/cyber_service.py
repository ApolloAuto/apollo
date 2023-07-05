#!/usr/bin/env python3
# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************

import os
import sys

from cyber.python.cyber_py3 import cyber

def print_service_info(service_name, sleep_s=2):
    roleattr_rawdata = cyber.ServiceUtils.get_service_attr(service_name, sleep_s)
    from cyber.proto.role_attributes_pb2 import RoleAttributes
    try:
        msg = RoleAttributes()
        msg.ParseFromString(roleattr_rawdata)
        assert(service_name == msg.service_name)
    except:
        print("RoleAttributes ParseFromString failed. size is ", len(roleattr_rawdata),
              ", service name: ", service_name)
        return
    print(msg.service_name)
    print("\tprocessid\t", msg.process_id)
    print("\tnodename\t", msg.node_name)
    print("\thostname\t", msg.host_name)
    print("")


def _service_cmd_info(argv):
    """
    Command-line parsing for 'cyber_service info' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(
        usage="usage: cyber_service info servicename ")
    parser.add_option("-a", "--all",
                      dest="all_services", default=False,
                      action="store_true",
                      help="display all services info")

    (options, args) = parser.parse_args(args)
    if len(args) == 0 and not options.all_services:
        parser.error("servicename must be specified")
    elif len(args) > 1:
        parser.error("you may only specify one service name")
    elif len(args) == 1:
        print_service_info(args[0])
    elif len(args) == 0 and options.all_services:
        services = cyber.ServiceUtils.get_services()
        for servicename in services:
            print_service_info(servicename, 0)


def print_service_list():
    services = sorted(cyber.ServiceUtils.get_services())
    print("The number of services is: ", len(services))
    for service_name in services:
        print(service_name)


def _service_cmd_list(argv):
    """
    Command-line parsing for 'cyber_service list' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(
        usage="usage: cyber_service list")
    (options, args) = parser.parse_args(args)
    if len(args) > 0:
        parser.error("param is too much")
    print_service_list()


def _printallusage():
    print("""cyber_service is a command-line tool for printing information about CyberRT Services.

Commands:
\tcyber_service list\tlist active services
\tcyber_service info\tprint information about active service

Type cyber_service <command> -h for more detailed usage, e.g. 'cyber_service info -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))

if __name__ == '__main__':
    if len(sys.argv) == 1:
        _printallusage()

    cyber.init()

    argv = sys.argv[0:]
    command = argv[1]
    if command == 'list':
        _service_cmd_list(argv)
    elif command == 'info':
        _service_cmd_info(argv)
    else:
        _printallusage()

    cyber.shutdown()
