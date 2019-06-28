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
import os
import sys
import time
import signal

import rospy
from std_msgs.msg import String

from plot_data import Plotter

from modules.canbus.proto import chassis_pb2
from modules.control.proto import control_cmd_pb2
from modules.localization.proto import localization_pb2


class DataCollector(object):
    """
    DataCollector Class
    """

    def __init__(self):
        self.sequence_num = 0
        self.control_pub = rospy.Publisher(
            '/apollo/control', control_cmd_pb2.ControlCommand, queue_size=1)
        rospy.sleep(0.3)
        self.controlcmd = control_cmd_pb2.ControlCommand()

        self.canmsg_received = False
        self.localization_received = False

        self.case = 'a'
        self.in_session = False

        self.outfile = ""

    def run(self, cmd):
        signal.signal(signal.SIGINT, self.signal_handler)

        self.in_session = True
        self.cmd = map(float, cmd)
        out = ''
        if self.cmd[0] > 0:
            out = out + 't'
        else:
            out = out + 'b'
        out = out + str(int(self.cmd[0]))
        if self.cmd[2] > 0:
            out = out + 't'
        else:
            out = out + 'b'
        out = out + str(int(self.cmd[2])) + 'r'
        i = 0
        self.outfile = out + str(i) + '_recorded.csv'
        while os.path.exists(self.outfile):
            i += 1
            self.outfile = out + str(i) + '_recorded.csv'
        self.file = open(self.outfile, 'w')
        self.file.write(
            "time,io,ctlmode,ctlbrake,ctlthrottle,ctlgear_location,vehicle_speed,"
            +
            "engine_rpm,driving_mode,throttle_percentage,brake_percentage,gear_location, imu\n"
        )

        print "Send Reset Command"
        self.controlcmd.header.module_name = "control"
        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        self.controlcmd.header.timestamp_sec = rospy.get_time()
        self.controlcmd.pad_msg.action = 2
        self.control_pub.publish(self.controlcmd)

        rospy.sleep(0.2)
        # Set Default Message
        print "Send Default Command"
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0
        self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

        self.canmsg_received = False

        rate = rospy.Rate(100)
        while self.in_session:
            self.publish_control()
            rate.sleep()

    def signal_handler(self, signal, frame):
        self.in_session = False

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

        self.canmsg_received = True
        if self.in_session:
            self.write_file(timenow, 0)

    def publish_control(self):
        """
        New Control Command
        """
        if not self.canmsg_received:
            print "No CAN Message Yet"
            return

        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1

        if self.case == 'a':
            if self.cmd[0] > 0:
                self.controlcmd.throttle = self.cmd[0]
                self.controlcmd.brake = 0
            else:
                self.controlcmd.throttle = 0
                self.controlcmd.brake = -self.cmd[0]
            if self.vehicle_speed >= self.cmd[1]:
                self.case = 'd'
        elif self.case == 'd':
            if self.cmd[2] > 0:
                self.controlcmd.throttle = self.cmd[0]
                self.controlcmd.brake = 0
            else:
                self.controlcmd.throttle = 0
                self.controlcmd.brake = -self.cmd[2]
            if self.vehicle_speed == 0:
                self.in_session = False

        self.controlcmd.header.timestamp_sec = rospy.get_time()
        self.control_pub.publish(self.controlcmd)
        self.write_file(self.controlcmd.header.timestamp_sec, 1)
        if self.in_session == False:
            self.file.close()

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
    rospy.init_node('data_collector', anonymous=True)

    data_collector = DataCollector()
    plotter = Plotter()
    localizationsub = rospy.Subscriber('/apollo/localization/pose',
                                       localization_pb2.LocalizationEstimate,
                                       data_collector.callback_localization)
    canbussub = rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                                 data_collector.callback_canbus)

    print "Enter q to quit"
    print "Enter p to plot result from last run"
    print "Enter x to remove result from last run"
    print "Enter x y z, where x is acceleration command, y is speed limit, z is decceleration command"
    print "Positive number for throttle and negative number for brake"

    while True:
        cmd = raw_input("Enter commands: ").split()
        if len(cmd) == 0:
            print "Quiting"
            break
        elif len(cmd) == 1:
            if cmd[0] == "q":
                break
            elif cmd[0] == "p":
                print "Plotting result"
                if os.path.exists(data_collector.outfile):
                    plotter.process_data(data_collector.outfile)
                    plotter.plot_result()
                else:
                    print "File does not exist"
            elif cmd[0] == "x":
                print "Removing last result"
                if os.path.exists(data_collector.outfile):
                    os.remove(data_collector.outfile)
                else:
                    print "File does not exist"
        elif len(cmd) == 3:
            data_collector.run(cmd)


if __name__ == '__main__':
    main()
