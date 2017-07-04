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
"""
Data Collector
"""
import sys
import rospy
from std_msgs.msg import String
from modules.localization.proto import localization_pb2
from modules.canbus.proto import chassis_pb2
from modules.control.proto import control_cmd_pb2
import time
import os


class DataCollector(object):
    """
    DataCollector Class
    """

    def __init__(self, file):
        self.proc = [line.rstrip('\n') for line in open(file)]
        self.index = 0
        outfile = file + '_recorded.csv'
        i = 0
        outfile = file + str(i) + '_recorded.csv'
        while os.path.exists(outfile):
            i += 1
            outfile = file + str(i) + '_recorded.csv'

        self.file = open(outfile, 'w')
        self.file.write(
            "time,io,ctlmode,ctlbrake,ctlthrottle,ctlgear_location,vehicle_speed,"
            +
            "engine_rpm,driving_mode,throttle_percentage,brake_percentage,gear_location, imu\n"
        )

        self.sequence_num = 0
        self.control_pub = rospy.Publisher(
            '/apollo/control', control_cmd_pb2.ControlCommand, queue_size=1)
        rospy.sleep(0.3)
        self.controlcmd = control_cmd_pb2.ControlCommand()

        # Send First Reset Message
        print "Send Reset Command"
        self.controlcmd.header.module_name = "control"
        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        self.controlcmd.header.timestamp_sec = rospy.get_time()
        self.controlcmd.pad_msg.action = 2
        self.control_pub.publish(self.controlcmd)

        rospy.sleep(0.3)
        # Set Default Message
        print "Send Default Command"
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0
        self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL

        self.printedcondition = False
        self.runtimer = False
        self.canmsg_received = False
        self.localization_received = False

    def callback_localization(self, data):
        """
        New Localization
        """
        self.acceleration = data.pose.linear_acceleration_vrf.y
        self.localization_received = True

    def callback_canbus(self, data):
        """
        New CANBUS
        """
        if not self.localization_received:
            print "No Localization Message Yet"
            return
        timenow = data.header.timestamp_sec
        self.vehicle_speed = data.speed_mps
        self.engine_rpm = data.engine_rpm
        self.throttle_percentage = data.throttle_percentage
        self.brake_percentage = data.brake_percentage
        self.gear_location = data.gear_location
        self.driving_mode = data.driving_mode

        self.write_file(timenow, 0)
        self.canmsg_received = True

    def publish_control(self):
        """
        New Control Command
        """
        if not self.canmsg_received:
            print "No CAN Message Yet"
            return

        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1

        while self.index < len(self.proc):
            cmdtype = self.proc[self.index][0]
            proc = self.proc[self.index][2:].lstrip()
            if cmdtype == 'a':
                command = 'self.controlcmd.' + proc
                exec (command)
                self.index = self.index + 1
                self.printedcondition = False
                print proc
            elif cmdtype == 'c':
                condition = 'self.' + proc
                if eval(condition):
                    self.index = self.index + 1
                    self.printedcondition = False
                    print proc
                else:
                    if not self.printedcondition:
                        print "Waiting for condition: ", proc
                        self.printedcondition = True
                    break
            elif cmdtype == 't':
                delaytime = float(proc)
                if not self.runtimer:
                    self.starttime = rospy.get_time()
                    self.runtimer = True
                    print "Waiting for time: ", delaytime
                    break
                elif rospy.get_time() > (self.starttime + delaytime):
                    self.index = self.index + 1
                    self.runtimer = False
                    print "Delayed for: ", delaytime
                else:
                    break
            else:
                print "Invalid Command, What are you doing?"
                print "Exiting"
                self.file.close()
                rospy.signal_shutdown("Shutting down")

        self.controlcmd.header.timestamp_sec = rospy.get_time()
        #self.control_pub.publish(self.controlcmd.SerializeToString())
        self.control_pub.publish(self.controlcmd)
        self.write_file(self.controlcmd.header.timestamp_sec, 1)
        if self.index >= len(self.proc):
            print "Reached end of commands, shutting down"
            self.file.close()
            rospy.signal_shutdown("Shutting down")

    def write_file(self, time, io):
        """
        Write Message to File
        """
        self.file.write(
            "%.4f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" %
            (time, io, 1, self.controlcmd.brake, self.controlcmd.throttle,
             self.controlcmd.gear_location, self.vehicle_speed, self.engine_rpm,
             self.driving_mode, self.throttle_percentage, self.brake_percentage,
             self.gear_location, self.acceleration))


def main():
    """
    Main function
    """
    if len(sys.argv) <= 1:
        print "Require Command Script"
        return
    elif len(sys.argv) > 2:
        print "Too many inputs"
        return
    file = sys.argv[1]
    rospy.init_node('data_collector', anonymous=True)

    data_collector = DataCollector(file)
    localizationsub = rospy.Subscriber('/apollo/localization/pose',
                                       localization_pb2.LocalizationEstimate,
                                       data_collector.callback_localization)
    canbussub = rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                                 data_collector.callback_canbus)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        data_collector.publish_control()
        rate.sleep()


if __name__ == '__main__':
    main()
