#!/usr/bin/env python3

"""
velodyne check
"""

import time

from cyber_py3 import cyber_time
from sensor_msgs.msg import PointCloud2


prev_stamp = 0
count = 0
LOG_FILE = None
EXCEPT_LOG_FILE = None


def create_log_file():
    data_time = time.strftime(
        '%Y-%m-%d-%H-%M-%S', time.localtime(cyber_time.Time.now().to_sec()))
    file_name = '/apollo/data/log/velodyne_hz.' + data_time + '.log'
    except_file_name = '/apollo/data/log/velodyne_hz.' + data_time + '.log.err'
    global LOG_FILE
    global EXCEPT_LOG_FILE
    LOG_FILE = open(file_name, 'a+')
    EXCEPT_LOG_FILE = open(except_file_name, 'a+')


def log_latency(log_file, frequence):
    pass


def callback(pointcloud):
    global count
    global prev_stamp
    count += 1
    stamp = pointcloud.header.stamp.to_time()
    if prev_stamp == 0:
        prev_stamp = stamp
        return
    interval = stamp - prev_stamp
    frequence = 1.0 / interval
    log_info = "%f: %.2fms\t%.2fHz\n" % (stamp, interval * 1000, frequence)
    LOG_FILE.write(log_info)
    if frequence < 9:
        EXCEPT_LOG_FILE.write(log_info)
    prev_stamp = stamp


def listener():
    node_name = 'velodyne_check'
    topic = '/apollo/sensor/velodyne64/compensator/PointCloud2'
    rospy.init_node(node_name)
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()


if __name__ == "__main__":
    create_log_file()
    listener()
