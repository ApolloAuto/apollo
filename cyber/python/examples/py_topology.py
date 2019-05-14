# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.

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
# -*- coding: utf-8 -*-
"""Module for example of topology manager."""

import sys
from cyber_py import topology_manager

sys.path.append("../")


class TestTopologyManager(object):

    """
    Class for cyber Node wrapper.
    """

    def __init__(self):
        """
        Init Topology Manager.
        """
        print('Init test topology successful')
        self.test_topology_manager = topology_manager.Topology_Manager()

    def test_has_node(self, node_name):
        """
        Test Has Specific Node.
        """
        print('Node name: %s' % node_name)
        flag = self.test_topology_manager.has_node(node_name)
        if flag:
            print("Node {} exist.".format(node_name))
        else:
            print("Node {} does not exist.".format(node_name))

    def test_get_node_list(self):
        """
        Test Get Node List.
        """
        node_list = self.test_topology_manager.get_node_list()
        print(len(node_list))
        if node_list:
            for node in node_list:
                print(node)
        else:
            print('Failed to get all nodes.')

    def test_show_node_info(self, node_name):
        """
        Test Show Node Info.
        """
        ret = self.test_topology_manager.show_node_info(node_name)
        for item in ret:
            print(item)

    def test_get_channel_list(self):
        """
        Test Get Channel List Info.
        """
        channel_list = self.test_topology_manager.get_channel_list()
        if channel_list:
            for channel in channel_list:
                print(channel)
        else:
            print('Failed to get all channels.')

    def test_get_reader_list(self):
        """
        Test Get Reader List.
        """
        reader_list = self.test_topology_manager.get_reader_list()
        if reader_list:
            for reader in reader_list:
                print(reader)
        else:
            print('Failed to get all reader.')

    def test_get_writer_list(self):
        """
        Test Get Writer List.
        """
        writer_list = self.test_topology_manager.get_writer_list()
        if writer_list:
            for writer in writer_list:
                print(writer)
        else:
            print('Failed to get all writer.')

    def test_get_node_writes(self, node_name):
        """
        Test Get Node Writes.
        """
        node_writers = self.test_topology_manager.get_node_writes(node_name)
        if node_writers:
            for node_writer in node_writers:
                print(node_writer)
        else:
            print('Get all writers of node :{} failed'.format(node_name))

    def test_get_node_readers(self, node_name):
        """
        Test Get Node Readers.
        """
        node_readers = self.test_topology_manager.get_node_readers(node_name)
        if node_readers:
            for node_reader in node_readers:
                print(node_reader)
        else:
            print('Get all readers of node :{} failed'.format(node_name))

    def test_show_channel_info(self, channel_name):
        """
        Test Show Channel Info.
        """
        ret = self.test_topology_manager.show_channel_info(channel_name)
        for item in ret:
            print(item)

if __name__ == "__main__":
    topology_manager_instance = TestTopologyManager()
    # topology_manager_instance.test_has_node("monitor")
    # topology_manager_instance.test_get_node_list()
    # topology_manager_instance.test_get_channel_list()
    # topology_manager_instance.test_get_node_writes("monitor")
    # topology_manager_instance.test_get_node_readers("monitor")
    # topology_manager_instance.test_get_reader_list()
    # topology_manager_instance.test_get_writer_list()
    # topology_manager_instance.test_show_node_info("control")
    # topology_manager_instance.test_show_channel_info("/apollo/control")
