import time
import sys

sys.path.append("../")
from cybertron import pynode
from cybertron import pyinit
from proto import chatter_pb2

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
        print("="*80)
        print("write msg -> %s" % msg)
        w.write(msg)

def main():
    pyinit.init()
    test_talker_class()
    pyinit.shutdown()

if __name__ == '__main__':
  main()