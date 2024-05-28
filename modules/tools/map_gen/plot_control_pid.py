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

import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
import gflags
from cyber.python.cyber_py3 import cyber
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
import pygame
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
from scipy.interpolate import interp1d

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.dt = 0.01

    def update(self, setpoint, measured_value, value):
        print(f"Kp: {self.Kp:.2f}, Ki: {self.Ki:.2f}, Kd: {self.Kd:.3f}")
        print(f"Setpoint: {setpoint:.2f}, Measured Value: {measured_value:.2f}")
        error = setpoint - measured_value
        # if error < value and error > -value:
        #     return 0.0
        self.integral += error*self.Ki
        if self.integral > 10:
            self.integral = 10
        elif self.integral < -10:
            self.integral = -10
        derivative = (error - self.prev_error)
        self.prev_error = error
        output = self.Kp * error + self.integral + self.Kd * derivative
        if error < value and error > -value:
            self.integral = 0.0
            self.prev_error = 0.0
            output = 0.0
        return output

class SteeringControl:
    def __init__(self, node):
        self.current_angle = 0
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.target_angle = 0.5
        # self.pid = PID(0.45, 2.16, 0.002)
        self.pid = PID(10.0, 0.00, 0.00)  #油门pid
        self.speed = 0
        self.history = {'time': [], 'target': [], 'current': [], 'speed': [], 'error': []}
        self.start_time = time.time()
        self.is_first = True
        self.stop = False
        self.start_record = False

        # Initialize Pygame for joystick control
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(3, 1, figsize=(10, 8))
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=10)

        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)

    def get_current_angle(self):
        # This function should be replaced with the actual method to get the current steering angle
        # return self.current_angle
        return self.current_throttle

    def update_control(self):
        # self.current_angle = self.get_current_angle()
        # self.speed = self.pid.update(self.target_angle, self.current_angle, 2.0)
        # self.speed = max(min(self.speed, 40), -40)
        # self.speed = 1 if self.speed > 0 and self.speed < 1 else self.speed
        # self.speed = -1 if self.speed < 0 and self.speed > -1 else self.speed

        self.current_angle = self.get_current_angle()
        self.speed = self.pid.update(self.target_angle, self.current_angle, 0.03)
        self.speed = max(min(self.speed, 80), -80)
        self.speed = 25 if self.speed > 0 and self.speed < 25 else self.speed
        self.speed = -25 if self.speed < 0 and self.speed > -25 else self.speed

    def handle_joystick_input(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYHATMOTION:
                value1 = event.value[0]
                if value1 == -1:  # Assume the left stick horizontal axis controls the target angle
                    # self.target_angle = -18 + self.current_angle # Scale joystick input to angle
                    self.target_angle += 0.1
                    self.history = {'time': [], 'target': [], 'current': [], 'speed': [], 'error': []}
                    
                if value1 == 1:
                    # self.target_angle = 18 + self.current_angle
                    self.target_angle -= 0.1
                    self.history = {'time': [], 'target': [], 'current': [], 'speed': [], 'error': []}
                    

            elif event.type == pygame.JOYBUTTONDOWN:
                print(event.button)
                if event.button == 0:  # Increase Kp
                    # self.pid.Kp += 0.01
                    self.pid.Kp += 0.1
                elif event.button == 1:  # Decrease Kp
                    # self.pid.Kp -= 0.01
                    self.pid.Kp -= 0.1
                elif event.button == 2:  # Increase Ki
                    self.pid.Ki += 0.01
                elif event.button == 3:  # Decrease Ki
                    self.pid.Ki -= 0.01
                elif event.button == 4:  # Increase Kd
                    self.pid.Kd -= 0.001
                elif event.button == 5:  # Decrease Kd
                    self.pid.Kd += 0.001
                elif event.button == 8:  # Reset PID gains to default
                    self.stop = True
                

    def run(self):
        while not self.stop:
            self.handle_joystick_input()
            self.update_control()

            #if self.start_record:
            current_time = time.time() - self.start_time
            self.history['time'].append(current_time)
            self.history['target'].append(self.target_angle)
            # self.history['current'].append(self.current_angle)
            self.history['current'].append(self.current_throttle)
            self.history['speed'].append(self.speed)
            # self.history['error'].append(abs(self.target_angle - self.current_angle))
            self.history['error'].append(abs(self.target_angle - self.current_throttle))

            self.animate(None)
            control_cmd = control_cmd_pb2.ControlCommand()
            # control_cmd.steering_rate = self.speed
            control_cmd.steering_rate = 0.0
            control_cmd.throttle = self.speed
            control_cmd.brake = 0.0
            self.control_pub.write(control_cmd)
            time.sleep(0.01)
            
    def smooth_curve(self, x, y):
        if len(x) < 2:
            return x, y
        f = interp1d(x, y, kind='linear', fill_value="extrapolate")
        x_smooth = np.linspace(min(x), max(x), 2000)
        y_smooth = f(x_smooth)
        return x_smooth, y_smooth

    def animate(self, i):
        self.ax[0].clear()
        self.ax[1].clear()
        self.ax[2].clear()
        # Smooth curves for target and current angles
        x_smooth, target_smooth = self.smooth_curve(self.history['time'], self.history['target'])
        _, current_smooth = self.smooth_curve(self.history['time'], self.history['current'])
        
        self.ax[0].plot(x_smooth, target_smooth, label='Target Angle')
        self.ax[0].plot(x_smooth, current_smooth, label='Current Angle')
        self.ax[0].legend(title=f'Kp: {self.pid.Kp:.2f}, Ki: {self.pid.Ki:.2f}, Kd: {self.pid.Kd:.3f}')
        self.ax[0].set_ylabel('Angle (degrees)')

        # Smooth curve for speed
        _, speed_smooth = self.smooth_curve(self.history['time'], self.history['speed'])
        self.ax[1].plot(x_smooth, speed_smooth, label='Speed')
        self.ax[1].legend()
        self.ax[1].set_ylabel('Speed')

        # Smooth curve for error
        _, error_smooth = self.smooth_curve(self.history['time'], self.history['error'])
        self.ax[2].plot(x_smooth, error_smooth, label='Error')
        self.ax[2].legend()
        self.ax[2].set_ylabel('Error')
        self.ax[2].set_xlabel('Time (s)')
        plt.savefig('plot.png')

    def callback_canbus(self, data):
        self.current_angle = data.steering_percentage
        self.current_throttle = data.throttle_percentage_cmd
        self.current_brake = data.brake_percentage_cmd
        if self.is_first:
            # self.target_angle = self.current_angle
            self.target_angle = self.current_throttle
            self.is_first = False
def main():
    node = cyber.Node("control_pid")
    steer_control = SteeringControl(node)
    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       steer_control.callback_canbus)
    plt.show(block=False)
    steer_control.run()

if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()