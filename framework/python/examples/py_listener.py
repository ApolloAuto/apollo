import time
import sys

sys.path.append("../")
from cybertron import pynode
from cybertron import pyinit
from proto import chatter_pb2

def callback(data):
    print("="*80)
    print("py:reader callback msg->:")
    print data
    print("="*80)

def test_listener_class():
    print("=" * 120)
    node = pynode.Node("listener")    
    r = node.create_reader("channel/chatter", chatter_pb2.Chatter, callback)
    node.spin()
    # while not node.is_shutdown():  
    #     time.sleep(0.002)

def main():
    pyinit.init()
    test_listener_class()
    pyinit.shutdown()

if __name__ == '__main__':
  main()