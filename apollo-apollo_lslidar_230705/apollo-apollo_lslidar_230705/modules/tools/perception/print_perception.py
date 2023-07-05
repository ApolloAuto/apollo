#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

"""
print received perception message
"""
import argparse
from cyber.python.cyber_py3 import cyber

from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles


def receiver(data):
    """receiver"""
    print(data)


def perception_receiver(perception_channel):
    """publisher"""
    cyber.init()
    node = cyber.Node("perception")
    node.create_reader(perception_channel, PerceptionObstacles, receiver)
    node.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
                                     prog="print_perception.py")
    parser.add_argument("-c", "--channel", action="store", type=str,
                        default="/apollo/perception/obstacles",
                        help="set the perception channel")

    args = parser.parse_args()
    perception_receiver(args.channel)
