import time
import sys

sys.path.append("../")
from cybertron import node
from cybertron import init
from proto import chatter_pb2

def callback(data):
    print("="*80)
    print("py:reader callback msg->:")
    print data
    print("="*80)

def test_listener_class():
    print("=" * 120)
    test_node = node.Node("listener")    
    r = test_node.create_reader("channel/chatter", chatter_pb2.Chatter, callback)
    test_node.spin()
    # while not node.is_shutdown():  
    #     time.sleep(0.002)

def main():
    init.init()
    test_listener_class()
    init.shutdown()

if __name__ == '__main__':
  main()