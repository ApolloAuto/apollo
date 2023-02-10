#!/usr/bin/env python3

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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

from datetime import datetime


def to_timestamp(sensor_time):
  date_sec, nano_sec = sensor_time.split('.')
  time_sec = datetime.strptime(date_sec, '%Y-%m-%d %H:%M:%S')
  return datetime.timestamp(time_sec) + float(nano_sec)*1e-9


class Sensor(object):
  def __init__(self, timestamp, file_path) -> None:
    self.timestamp = to_timestamp(timestamp)
    self.file_path = file_path
    self.parse()

  def parse(self):
    raise NotImplementedError("Must override!")


class Lidar(Sensor):
  def __init__(self, timestamp, file_path) -> None:
    super().__init__(timestamp, file_path)

  def parse(self):
    pass


class Camera(Sensor):
  def __init__(self, timestamp, file_path) -> None:
    super().__init__(timestamp, file_path)

  def parse(self):
    pass


class IMU(Sensor):
  def __init__(self, timestamp, file_path) -> None:
    super().__init__(timestamp, file_path)
    self.lat = None
    self.lon = None
    self.alt = None
    self.roll = None
    self.pitch = None
    self.yaw = None
    self.vn = None
    self.ve = None
    self.vf = None
    self.vl = None
    self.vu = None
    self.ax = None
    self.ay = None
    self.az = None
    self.af = None
    self.al = None
    self.au = None
    self.wx = None
    self.wy = None
    self.wz = None
    self.wf = None
    self.wl = None
    self.wu = None
    self.pos_accuracy = None
    self.vel_accuracy = None
    self.navstat = None
    self.numsats = None
    self.posmode = None
    self.velmode = None
    self.orimode = None
    self.parse()

  def parse(self):
    with open(self.file_path, 'r') as f:
      for line in f:
        data = line.strip().split()
        if len(data) != 30:
          # If length < 30 then we just read the required data
          self.lat = float(data[0])
          self.lon = float(data[1])
          self.alt = float(data[2])
          self.roll = float(data[3])
          self.pitch = float(data[4])
          self.yaw = float(data[5])
          print("IMU data length error! require 30 but {}".format(len(data)))
          continue

        self.lat = float(data[0])
        self.lon = float(data[1])
        self.alt = float(data[2])
        self.roll = float(data[3])
        self.pitch = float(data[4])
        self.yaw = float(data[5])
        self.vn = float(data[6])
        self.ve = float(data[7])
        self.vf = float(data[8])
        self.vl = float(data[9])
        self.vu = float(data[10])
        self.ax = float(data[11])
        self.ay = float(data[12])
        self.az = float(data[13])
        self.af = float(data[14])
        self.al = float(data[15])
        self.au = float(data[16])
        self.wx = float(data[17])
        self.wy = float(data[18])
        self.wz = float(data[19])
        self.wf = float(data[20])
        self.wl = float(data[21])
        self.wu = float(data[22])
        self.pos_accuracy = float(data[23])
        self.vel_accuracy = float(data[24])
        self.navstat = data[25]
        self.numsats = data[26]
        self.posmode = data[27]
        self.velmode = data[28]
        self.orimode = data[29]
