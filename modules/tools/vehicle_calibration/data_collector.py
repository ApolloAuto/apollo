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
"""
Data Collector
"""

import os
import signal
import sys
import time

import math

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.tools.vehicle_calibration.plot_data import Plotter

class PidControlr(object):
    def __init__(self, kp, ki, kd, integral_limit, ouput_level_high):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0
        self.output_limit_high = ouput_level_high
        self.output_limit_low = 27
        self.integral_limit = integral_limit
        
        self.error_limit = 0.05
        self.is_first_hit = True
        
    def calculate(self, error, dt):
        diff = 0.0
        output = 0.0
        if self.is_first_hit:
            self.is_first_hit = False
        else:
            diff = (error - self.last_error) / dt
        self.integral += error * dt * self.ki 
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        output = self.kp * error + self.integral + self.kd * diff
        self.last_error = error
        if 0< output and output < self.output_limit_low:
            output = self.output_limit_low
        elif output < 0 and output > -self.output_limit_low:
            output = -self.output_limit_high

        if output > self.output_limit_high:
            output = self.output_limit_high
        elif output < -self.output_limit_high:
            output = -self.output_limit_high

        if error < self.error_limit and error > -self.error_limit:
            self.last_error = 0.0
            error = 0.0
            output = 0.0
        return output


class DataCollector(object):
    """
    DataCollector Class
    """

    def __init__(self, node):
        self.sequence_num = 0
        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)
        time.sleep(0.3)
        self.controlcmd = control_cmd_pb2.ControlCommand()

        self.canmsg_received = False
        self.localization_received = False

        self.case = 'a'
        self.in_session = False

        self.outfile = ""

        self.thro_pid = PidControlr(75.0, 2.0, 0.5, 10, 65)
        self.brake_pid = PidControlr(95.0, 2.5, 0.5, 10, 85)

    def run(self, cmd):
        signal.signal(signal.SIGINT, self.signal_handler)

        self.in_session = True
        self.cmd = list(map(float, cmd))
        out = './modules/tools/vehicle_calibration/my_data/'
        if self.cmd[0] > 0:
            out += 't'
        else:
            out += 'b'
        out = out + str(int(self.cmd[0]))
        if self.cmd[2] > 0:
            out += 't'
        else:
            out += 'b'
        out += str(int(self.cmd[2])) + 'r'
        i = 0
        self.outfile = out + str(i) + '_recorded.csv'
        while os.path.exists(self.outfile):
            i += 1
            self.outfile = out + str(i) + '_recorded.csv'
        self.file = open(self.outfile, 'w')
        self.file.write(
            "time,io,ctlmode,ctlbrake,ctlthrottle,ctlgear_location," +
            "vehicle_speed,engine_rpm,driving_mode,throttle_percentage," +
            "brake_percentage,gear_location,imu\n"
        )

        print('Send Reset Command.')
        self.controlcmd.header.module_name = "control"
        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.controlcmd.pad_msg.action = 2
        self.control_pub.write(self.controlcmd)

        time.sleep(0.2)
        # Set Default Message
        print('Send Default Command.')
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0
        self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

        self.canmsg_received = False
        self.case = 'a'

        while self.in_session:
            now = cyber_time.Time.now().to_sec()
            self.publish_control()
            sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def signal_handler(self, signal, frame):
        self.in_session = False

    def callback_localization(self, data):
        """
        New Localization
        """
        # self.acceleration = data.pose.linear_acceleration_vrf.y
        ax = data.pose.linear_acceleration.x
        ay = data.pose.linear_acceleration.y
        az = data.pose.linear_acceleration.z
        self.acceleration = math.sqrt(ax**2 + ay**2 + az**2)-9.8
        # print(self.acceleration)
        self.localization_received = True

    def callback_canbus(self, data):
        """
        New CANBUS
        """
        if not self.localization_received:
            print('No Localization Message Yet')
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
            print('No CAN Message Yet')
            return

        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num += 1
        error = 0.0
        if self.case == 'a':
            if self.cmd[0] > 0:
                error = self.cmd[0] - self.throttle_percentage
                self.controlcmd.throttle = self.thro_pid.calculate(error, 0.01)
                error = 2.5 - self.brake_percentage
                self.controlcmd.brake = self.brake_pid.calculate(error, 0.01)
                #self.controlcmd.throttle = self.cmd[0]
                #self.controlcmd.brake = 0
            else:
                self.controlcmd.throttle = 0
                self.controlcmd.brake = -self.cmd[0]
            #if self.vehicle_speed >= self.cmd[1]:
            if self.acceleration >= self.cmd[1]:
                self.case = 'd'
        elif self.case == 'd':
            if self.cmd[2] > 0:
                self.controlcmd.throttle = self.cmd[0]
                self.controlcmd.brake = 0
            else:
                # self.controlcmd.throttle = 0
                # self.controlcmd.brake = -self.cmd[2]
                error = 1.5 - self.throttle_percentage
                out_thro = self.thro_pid.calculate(error, 0.01)
                self.controlcmd.throttle = out_thro
                error = -self.cmd[2] - self.brake_percentage
                out_bra = self.brake_pid.calculate(error, 0.01)
                self.controlcmd.brake = out_bra
            #if self.vehicle_speed == 0:
                if out_thro == 0.0 and out_bra == 0.0 and self.acceleration < 0.03 and self.acceleration >-0.03: 
                    self.in_session = False

        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.control_pub.write(self.controlcmd)
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
    node = cyber.Node("data_collector")

    data_collector = DataCollector(node)
    plotter = Plotter()
    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       data_collector.callback_localization)
    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       data_collector.callback_canbus)

    print('Enter q to quit.')
    print('Enter p to plot result from last run.')
    print('Enter x to remove result from last run.')
    print('Enter x y z, where x is acceleration command, ' +
          'y is speed limit, z is decceleration command.')
    print('Positive number for throttle and negative number for brake.')

    while True:
        cmd = input("Enter commands: ").split()
        if len(cmd) == 0:
            print('Quiting.')
            break
        elif len(cmd) == 1:
            if cmd[0] == "q":
                break
            elif cmd[0] == "p":
                print('Plotting result.')
                if os.path.exists(data_collector.outfile):
                    plotter.process_data(data_collector.outfile)
                    plotter.plot_result()
                else:
                    print('File does not exist: %s' % data_collector.outfile)
            elif cmd[0] == "x":
                print('Removing last result.')
                if os.path.exists(data_collector.outfile):
                    os.remove(data_collector.outfile)
                else:
                    print('File does not exist: %s' % date_collector.outfile)
        elif len(cmd) == 3:
            data_collector.run(cmd)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
