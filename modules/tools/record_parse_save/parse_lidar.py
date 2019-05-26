"""
function to parse lidar data from *.record files, created using Apollo-Auto

parsed data is saved to *.txt file, for each scan

current implementation for:
* Velodyne VLS-128 lidar

"""

import sys, os

from cyber_py import cyber
from cyber_py import record

from modules.drivers.proto.pointcloud_pb2 import PointCloud
###########################################################
def parse_data(channelname, msg, out_folder):
    """
    """
    msg_lidar = PointCloud()
    msg_lidar.ParseFromString(msg)
    nPts = len(msg_lidar.point)

    pcd = []
    for j in range(nPts):
        p = msg_lidar.point[j]
        pcd.append([p.x, p.y, p.z, p.intensity])

    tstamp = msg_lidar.measurement_time
    temp_time = str(tstamp).split('.')

    if len(temp_time[1])==1:
        temp_time1_adj = temp_time[1] + '0'
    else:
        temp_time1_adj = temp_time[1]

    pcd_time = temp_time[0] + '_' + temp_time1_adj
    pcd_filename = "pcd_" + pcd_time + ".txt"

    with open(out_folder + pcd_filename, 'w') as outfile:
        for item in pcd:
            data = str(item)[1:-1]
            outfile.write("%s\n" % data)

    return tstamp
