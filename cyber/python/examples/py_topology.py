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

sys.path.append("../")
from cyber_py import topology_manager


class Test_Topology_Manager(object):

    """
    Class for cyber Node wrapper.
    """

    def __init__(self):
        """
        Init Topology Manager.
        """
        print("init test topology successful")
        self.test_topology_manager = topology_manager.Topology_Manager()

    def test_has_node(self, node_name):
        """
        Test Has Specific Node.
        """
        print("node name:", node_name)
        flag = self.test_topology_manager.has_node(node_name)
        if(flag is True):
            print("node {} exist.".format(node_name))
        else:
            print("node {} not exist.".format(node_name))

    def test_get_node_list(self):
        """
        Test Get Node List.
        """
        node_list = self.test_topology_manager.get_node_list()
        print(len(node_list))
        if(len(node_list) != 0):
            for node in node_list:
                print(node)
        else:
            print("get all nodes failed")

    def test_show_node_info(self, node_name):
        """
        Test Show Node Info.
        """
        ret = self.test_topology_manager.show_node_info(node_name)
        if(len(ret) != 0):
            for item in ret:
                print item

    def test_get_channel_list(self):
        """
        Test Get Channel List Info.
        """
        channel_list = self.test_topology_manager.get_channel_list()
        if(len(channel_list) != 0):
            for channel in channel_list:
                print(channel)
        else:
            print("get all channels failed")

    def test_get_reader_list(self):
        """
        Test Get Reader List.
        """
        reader_list = self.test_topology_manager.get_reader_list()
        if(len(reader_list) != 0):
            for reader in reader_list:
                print(reader)
        else:
            print("get all reader failed")

    def test_get_writer_list(self):
        """
        Test Get Writer List.
        """
        writer_list = self.test_topology_manager.get_writer_list()
        if(len(writer_list) != 0):
            for writer in writer_list:
                print(writer)
        else:
            print("get all writer failed")

    def test_get_node_writes(self, node_name):
        """
        Test Get Node Writes.
        """
        node_writers = self.test_topology_manager.get_node_writes(node_name)
        if(len(node_writers) != 0):
            for node_writer in node_writers:
                print(node_writer)
        else:
            print("get all writers of node :{} failed".format(node_name))

    def test_get_node_readers(self, node_name):
        """
        Test Get Node Readers.
        """
        node_readers = self.test_topology_manager.get_node_readers(node_name)
        if(len(node_readers) != 0):
            for node_reader in node_readers:
                print(node_reader)
        else:
            print("get all readers of node :{} failed".format(node_name))

    def test_show_channel_info(self, channel_name):
        """
        Test Show Channel Info.
        """
        ret = self.test_topology_manager.show_channel_info(channel_name)
        if(len(ret) != 0):
            for item in ret:
                print item

if __name__ == "__main__":
    topology_manager_instance = Test_Topology_Manager()
    # topology_manager_instance.test_has_node("monitor")
    # topology_manager_instance.test_get_node_list()
    # topology_manager_instance.test_get_channel_list()
    # topology_manager_instance.test_get_node_writes("monitor")
    # topology_manager_instance.test_get_node_readers("monitor")
    # topology_manager_instance.test_get_reader_list()
    # topology_manager_instance.test_get_writer_list()
    # topology_manager_instance.test_show_node_info("control")
    # topology_manager_instance.test_show_channel_info("/apollo/control")
