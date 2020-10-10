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
import math


class ViewSubplot:
    def __init__(self, ax):
        # self.ax = ax
        self.right_lane, = ax.plot(
            [-10, 10, -10, 10], [-10, 150, 150, -10],
            'bo', lw=3, alpha=0.4)
        self.left_lane, = ax.plot(
            [0], [0], 'go', lw=3, alpha=0.5)
        self.obstacles, = ax.plot(
            [0], [0], 'r.', ms=20, alpha=0.5)
        self.ref_lane, = ax.plot(
            [0], [0], 'k--', lw=3, alpha=0.8)
        self.vehicle = ax.plot(
            [-1.055, 1.055, 1.055, -1.055, -1.055], [0, 0, -4.933, -4.933, 0],
            'r-', lw=1)
        self.routing, = ax.plot(
            [0], [0], 'r--', lw=3, alpha=0.8)

        self.speed_line, = ax.plot([0], [0], 'r-', lw=3, alpha=0.4)
        self.acc_line, = ax.plot([0], [0], 'y-', lw=3, alpha=1)

        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 100])
        ax.relim()
        ax.set_xlabel("lat(m)")
        self.next_lanes = []
        for i in range(8):
            lane, = ax.plot([0], [0], 'b-', lw=3, alpha=0.4)
            self.next_lanes.append(lane)

        self.left_lane.set_visible(False)
        self.right_lane.set_visible(False)
        self.ref_lane.set_visible(False)

    def show(self, mobileye_data, localization_data, planning_data,
             chassis_data, routing_data):
        self.left_lane.set_visible(True)
        self.right_lane.set_visible(True)
        self.ref_lane.set_visible(True)

        mobileye_data.lane_data_lock.acquire()
        self.right_lane.set_xdata(mobileye_data.right_lane_x)
        self.right_lane.set_ydata(mobileye_data.right_lane_y)
        self.left_lane.set_xdata(mobileye_data.left_lane_x)
        self.left_lane.set_ydata(mobileye_data.left_lane_y)
        mobileye_data.lane_data_lock.release()

        planning_data.path_lock.acquire()
        self.ref_lane.set_xdata(planning_data.path_x)
        self.ref_lane.set_ydata(planning_data.path_y)
        planning_data.path_lock.release()

        if chassis_data.is_auto():
            self.ref_lane.set_color('r')
        else:
            self.ref_lane.set_color('k')

        mobileye_data.obstacle_data_lock.acquire()
        self.obstacles.set_ydata(mobileye_data.obstacle_x)
        self.obstacles.set_xdata(mobileye_data.obstacle_y)
        mobileye_data.obstacle_data_lock.release()

        mobileye_data.next_lane_data_lock.acquire()
        for i in range(len(mobileye_data.next_lanes_x)):
            if i >= len(self.next_lanes):
                mobileye_data.next_lane_data_lock.release()
                break
            self.next_lanes[i].set_xdata(mobileye_data.next_lanes_x[i])
            self.next_lanes[i].set_ydata(mobileye_data.next_lanes_y[i])
        mobileye_data.next_lane_data_lock.release()

        if localization_data.localization_pb is None:
            return

        vx = localization_data.localization_pb.pose.position.x
        vy = localization_data.localization_pb.pose.position.y

        routing_data.routing_data_lock.acquire()
        path_x = [x - vx for x in routing_data.routing_x]
        path_y = [y - vy for y in routing_data.routing_y]
        routing_data.routing_data_lock.release()

        heading = localization_data.localization_pb.pose.heading
        npath_x = []
        npath_y = []

        for i in range(len(path_x)):
            x = path_x[i]
            y = path_y[i]
            # newx = x * math.cos(heading) - y * math.sin(heading)
            # newy = y * math.cos(heading) + x * math.sin(heading)
            newx = x * math.cos(- heading + 1.570796) - y * math.sin(
                -heading + 1.570796)
            newy = y * math.cos(- heading + 1.570796) + x * math.sin(
                -heading + 1.570796)
            npath_x.append(newx)
            npath_y.append(newy)

        self.routing.set_xdata(npath_x)
        self.routing.set_ydata(npath_y)

        speed_x = localization_data.localization_pb.pose.linear_velocity.x
        speed_y = localization_data.localization_pb.pose.linear_velocity.y
        acc_x = localization_data.localization_pb.pose.linear_acceleration.x
        acc_y = localization_data.localization_pb.pose.linear_acceleration.y
        heading = localization_data.localization_pb.pose.heading

        new_speed_x = math.cos(-heading + math.pi / 2) * speed_x - math.sin(
            -heading + math.pi / 2) * speed_y
        new_speed_y = math.sin(-heading + math.pi / 2) * speed_x + math.cos(
            -heading + math.pi / 2) * speed_y

        new_acc_x = math.cos(-heading + math.pi / 2) * acc_x - math.sin(
            -heading + math.pi / 2) * acc_y
        new_acc_y = math.sin(-heading + math.pi / 2) * acc_x + math.cos(
            -heading + math.pi / 2) * acc_y

        # self.speed_line.set_xdata([0, new_speed_x])
        # self.speed_line.set_ydata([0, new_speed_y])
        # self.acc_line.set_xdata([0, new_acc_x])
        # self.acc_line.set_ydata([0, new_acc_y])
