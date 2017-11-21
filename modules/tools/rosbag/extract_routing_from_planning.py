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

import rospy
from std_msgs.msg import String
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.routing.proto.routing_pb2 import RoutingResponse


class Extractor(object):

    def __init__(self):
        self.routing = rospy.Publisher(
            '/apollo/routing_response', RoutingResponse, queue_size=1)

    def callback_planning(self, data):
        self.routing.publish(data.debug.planning_data.routing)
        print "New Planning"


def main():
    """
    Main function
    """
    extract = Extractor()
    rospy.init_node('extract_routing', anonymous=True)
    planning_sub = rospy.Subscriber(
        '/apollo/planning',
        ADCTrajectory,
        extract.callback_planning,
        queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
