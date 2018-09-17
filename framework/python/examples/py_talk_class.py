import time
import sys
import os

# first excute for init vars
cybertron_path = os.environ['CYBERTRON_PATH'] # CYBERTRON_PATH=/home/work/baidu/adu-lab/python-wrapper/install/
if (cybertron_path == ""):
    print("CYBERTRON_PATH is null")
else:
    print("CYBERTRON_PATH=%s" % cybertron_path)
    print("env inited succ!")

cybertron_dir=os.path.split(cybertron_path)[0]
sys.path.append(cybertron_path + "/third_party/")
sys.path.append(cybertron_path + "/lib/")
sys.path.append(cybertron_path + "/python/cybertron")

sys.path.append(cybertron_dir + "/python/")
sys.path.append(cybertron_dir + "/cybertron/")

from proto import chatter_pb2
from cybertron import pynode

def test_talker_class():
    msg = chatter_pb2.Chatter()
    msg.content = "talker:send Alex!"
    msg.seq = 0
    msg.timestamp = 0
    msg.lidar_timestamp = 0

    node = pynode.Node("node_name1")
    g_count = 1

    w = node.create_writer("channel/chatter", chatter_pb2.DESCRIPTOR.message_types_by_name['Chatter'].full_name)

    while not node.is_shutdown():
        time.sleep(1)
        g_count = g_count + 1
        msg.seq = g_count
        w.write(msg)

def main():
    test_talker_class()

if __name__ == '__main__':
  main()