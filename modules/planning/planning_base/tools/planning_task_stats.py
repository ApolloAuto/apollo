import argparse
import os
import fnmatch
from matplotlib import patches
from matplotlib import lines
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
import matplotlib.pyplot as plt


def is_record_file(path):
    """Naive check if a path is a record."""
    return path.endswith('.record') or \
        fnmatch.fnmatch(path, '*.record.?????') or \
        fnmatch.fnmatch(path, '*.record.????')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Process and analyze control and planning data')
    parser.add_argument('--bag', type=str, help='use Rosbag')
    parser.add_argument('--path', type=str, help='path for bag files')
    args = parser.parse_args()
    bags = []
    data = {}
    mode_time = []
    cur_driving_mode = 0
    if args.path:
        dir_path = args.path
        if os.path.isdir(dir_path):
            dirs = os.listdir(dir_path)
            dirs.sort()
            for file in dirs:
                # print "os.path.splitext(file)[1]", os.path.splitext(file)[1]
                if is_record_file(file):
                    bags.append(dir_path + file)
    elif args.bag:
        bags.append(args.bag)
    else:
        exit(0)
    last_time = 0
    for bag in bags:
        reader = RecordReader(bag)
        print("Begin reading the file: ", bag)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/canbus/chassis":
                chassis_pb = chassis_pb2.Chassis()
                chassis_pb.ParseFromString(msg.message)
                if chassis_pb.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                    if cur_driving_mode == 0:
                        mode_time.append(chassis_pb.header.timestamp_sec)
                        cur_driving_mode = 1
                elif cur_driving_mode == 1:
                    mode_time.append(chassis_pb.header.timestamp_sec)
                    cur_driving_mode = 0
                last_time = chassis_pb.header.timestamp_sec
            if msg.topic == "/apollo/planning":
                planning_pb = planning_pb2.ADCTrajectory()
                planning_pb.ParseFromString(msg.message)
                if not planning_pb.HasField('latency_stats'):
                    continue
                for task in planning_pb.latency_stats.task_stats:
                    time_stamp = planning_pb.header.timestamp_sec
                    if not task.name in data.keys():
                        data[task.name] = [[time_stamp], [task.time_ms]]
                    else:
                        data[task.name][0].append(time_stamp)
                        data[task.name][1].append(task.time_ms)
    # plot data
    fig, ax = plt.subplots()
    for key in data.keys():
        ax.plot(data[key][0], data[key][1], label=key)
    ax.legend()
    ax.grid(True)
    ax.set_xlabel('timestamp(s)')
    ax.set_ylabel('time(ms)')
    print("mode_time:", mode_time)
    if len(mode_time) % 2 == 1:
        mode_time.append(last_time)
    for i in range(0, len(mode_time), 2):
        ax.axvspan(mode_time[i],
                    mode_time[i + 1],
                    fc='0.1',
                    alpha=0.1)
        ax.axvspan(mode_time[i],
                    mode_time[i + 1],
                    fc='0.1',
                    alpha=0.1)
    plt.show()
