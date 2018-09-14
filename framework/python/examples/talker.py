import time
import sys

sys.path.append("../")
from cybertron import node
from cybertron import init
from proto import chatter_pb2

def test_talker_class():
    msg = chatter_pb2.Chatter()
    msg.content = "talker:send Alex!"
    msg.seq = 0
    msg.timestamp = 0
    msg.lidar_timestamp = 0

    test_node = node.Node("node_name1")
    g_count = 1

    w = test_node.create_writer("channel/chatter", chatter_pb2.Chatter.DESCRIPTOR.full_name)
    while not test_node.is_shutdown():  
        time.sleep(1)
        g_count = g_count + 1
        msg.seq = g_count
        print("="*80)
        print("write msg -> %s" % msg)
        w.write(msg)

def main():
    init.init()
    test_talker_class()
    init.shutdown()

if __name__ == '__main__':
  main()