#!/usr/bin/python
# -*- coding: latin-1 -*-
import matplotlib.pyplot as plt
import numpy as np
import re

clock = 0
time = []
stationerror = []
lateralerror = []
headingerror = []

velocity_station_error = []
velocity_lateral_error = []
truevelocity_station = []
truevelocity_lateral = []

acceleration_station_error = []
acceleration_lateral_error = []
trueacceleration_station = []
trueacceleration_lateral = []

input = open('/apollo/data/log/localization.INFO', 'r')
data = input.readlines()

for line in data:
    if re.findall('position error', line):
        line = line.split(',')
        station = line[5].split('[')[1].split(']')[0]
        lateral = line[6].split('[')[1].split(']')[0]
        stationerror.append((float)(station))
        lateralerror.append((float)(lateral))
        time.append(clock)
        clock += 0.02

for line in data:
    if re.findall('heading error', line):
        line = line.split()
        heading = line[-1].split('[')[1].split(']')[0]
        headingerror.append(57.3*(float)(heading))

for line in data:
    if re.findall('True velocity', line):
        line = line.split(',')
        true_x = line[1].split('[')[1].split(']')[0]
        true_y = line[2].split('[')[1].split(']')[0]
        error_x = line[4].split('[')[1].split(']')[0]
        error_y = line[5].split('[')[1].split(']')[0]
        velocity_station_error.append(error_x)
        velocity_lateral_error.append(error_y)
        truevelocity_station.append(true_x)
        truevelocity_lateral.append(true_y)

for line in data:
    if re.findall('True acceleration', line):
        a = line.split('[')[2].split(']')[0]
        line = line.split(',')
        true_x = line[1].split('[')[1].split(']')[0]
        true_y = line[2].split('[')[1].split(']')[0]
        error_x = line[4].split('[')[1].split(']')[0]
        error_y = line[5].split('[')[1].split(']')[0]
        acceleration_station_error.append(error_x)
        acceleration_lateral_error.append(error_y)
        trueacceleration_station.append(true_x)
        trueacceleration_lateral.append(true_y)

plt.figure('Position Error')
plt.title('Position Error Analysis')
plt.plot(time, stationerror, color='blue', label='$station$')
plt.plot(time, lateralerror, color='red', label='$lateral$')
plt.plot(time, headingerror, color='black', label='$heading$')
plt.legend()

# 设置坐标轴名称
plt.xlabel('time(s)')
plt.ylabel('error(m)')
# plt.show()

plt.figure('Velocity Error')
plt.title('Velocity Error Analysis')
plt.plot(time, velocity_station_error, color='blue', label='$StationError$')
plt.plot(time, velocity_lateral_error, color='red', label='$LateralError$')
# plt.plot(time,truevelocity_station,color='black',label='$TrueStation$')
# plt.plot(time,truevelocity_lateral,color='green',label='$TrueLateral$')
plt.legend()
# 设置坐标轴名称
plt.xlabel('time(s)')
plt.ylabel('error(m/s)')
# plt.show()

plt.figure('Acceleration Error')
plt.title('Acceleration Error Analysis')
plt.plot(time, acceleration_station_error,
         color='blue', label='$StationError$')
#plt.plot(time, acceleration_lateral_error, color='red', label='$LateralError$')
# plt.plot(time, trueacceleration_station,
#         color='black', label='$TrueStation$')
# plt.plot(time, trueacceleration_lateral,
#         color='green', label='$TrueLateral$')
plt.legend()
# 设置坐标轴名称
plt.xlabel('time(s)')
plt.ylabel('error(m/s^2)')
plt.show()

# 画gps图
trueX = []
trueY = []
for line2 in data:
    if re.findall('True position', line2):
        line2 = line2.split(',')
        x = line2[1].split('[')[1].split(']')[0]
        y = line2[2].split('[')[1].split(']')[0]
        trueX.append((float)(x))
        trueY.append((float)(y))

# print(trueX)
plt.figure('GPSX')
plt.plot(time, trueX, '.r')
plt.plot(time, trueX, 'b')

plt.figure('GPSY')
plt.plot(time, trueY, '.b',)
plt.plot(time, trueY, 'r')

# plt.show()
