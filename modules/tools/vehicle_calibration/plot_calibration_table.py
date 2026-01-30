import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys


from modules.tools.common.logger import Logger
import modules.tools.common.proto_utils as proto_utils

import modules.control.control_component.proto.calibration_table_pb2 as calibration_table_pb2


current_dir = sys.path[0]
print("current dir is", current_dir)

calibration_table_filename = '/apollo/modules/control/control_component/conf/calibration_table.pb.txt'
calibration_table_brake = ''
calibration_table_throttle = ''

print("train_conf_file path is %s", str(calibration_table_filename))

calibration_table_pb = calibration_table_pb2.calibration_table()
origin_config = proto_utils.get_pb_from_text_file(calibration_table_filename, calibration_table_pb)

speed = []
acc = []
cmd = []

for calibration in origin_config.calibration:
    speed.append(calibration.speed)
    acc.append(calibration.acceleration)
    cmd.append(calibration.command)

print("len(speed): ", len(speed))
print("len(acc): ", len(acc))
print("len(cmd): ", len(cmd))


# 绘制图表
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(speed, cmd, acc)

# 设置坐标轴标签
ax.set_xlabel('speed')
ax.set_ylabel('cmd')
ax.set_zlabel('acc')

# 显示图表
plt.show()

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
p = ax.plot_trisurf(speed, cmd, acc, cmap='rainbow')
ax.set_xlabel('speed')
ax.set_ylabel('cmd')
ax.set_zlabel('acc')
plt.show()