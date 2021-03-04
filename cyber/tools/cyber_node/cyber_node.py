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

from optparse import OptionParser

from cyber.python.cyber_py3 import cyber
from cyber.proto.role_attributes_pb2 import RoleAttributes


def print_node_info(node_name, sleep_s=2):
    raw_data = cyber.NodeUtils.get_node_attr(node_name, sleep_s)
    try:
        msg = RoleAttributes()
        msg.ParseFromString(raw_data)
        assert(node_name == msg.node_name)
    except:
        print("RoleAttributes ParseFromString failed. size is ",
              len(raw_data))
        return
    print("Node:    \t%s" % msg.node_name)
    print("ProcessId: \t%d" % msg.process_id)
    print("Hostname:\t%s" % msg.host_name)

    print("[Reading Channels]:")
    reading_channels = sorted(cyber.NodeUtils.get_readersofnode(node_name, 0))
    for channel in reading_channels:
        print(channel)
    print("")

    print("[Writing Channels]:")
    writing_channels = sorted(cyber.NodeUtils.get_writersofnode(node_name, 0))
    for channel in writing_channels:
        print(channel)
    print("")


def _node_cmd_info(argv):
    """
    Command-line parsing for 'cyber_node info' command.
    """
    args = argv[2:]
    parser = OptionParser(
        usage="usage: cyber_node info [OPTION...] [NODE...]")
    parser.add_option("-a", "--all",
                      dest="all_nodes", default=False,
                      action="store_true",
                      help="display all nodes' info")
    (options, args) = parser.parse_args(args)
    if options.all_nodes:
        if len(args) != 0:
            parser.error(
                """"-a/--all" option is expected to run w/o node name(s)""")
        else:
            nodes = cyber.NodeUtils.get_nodes()
            for nodename in nodes:
                print_node_info(nodename, 0)
    elif len(args) == 0:
        parser.error("No node name provided.")
    else:
        for arg in args:
            print_node_info(arg)


def print_node_list():
    nodes = cyber.NodeUtils.get_nodes()
    print("Number of active nodes: {}".format(len(nodes)))
    for node_name in sorted(nodes):
        print(node_name)


def _node_cmd_list(argv):
    """
    Command-line parsing for 'cyber_node list'
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: cyber_node list")
    (options, args) = parser.parse_args(args)
    if len(args) > 0:
        parser.error("too many arguments")
    print_node_list()


def _usage():
    print("""cyber_node is a command-line tool to show information about CyberRT Nodes.

Commands:
\tcyber_node list \tList active nodes.
\tcyber_node info \tPrint node info.

Type cyber_node <command> -h for more detailed usage, e.g. 'cyber_node info -h'
""")
    sys.exit(getattr(os, "EX_USAGE", 1))


if __name__ == '__main__':
    if len(sys.argv) == 1:
        _usage()
    cyber.init()

    argv = sys.argv[0:]
    command = argv[1]
    if command == "list":
        _node_cmd_list(argv)
    elif command == "info":
        _node_cmd_info(argv)
    else:
        _usage()

    cyber.shutdown()
