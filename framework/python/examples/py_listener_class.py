import threading
import time
import sys
import os
import importlib
import ctypes
from google.protobuf.descriptor_pb2 import FileDescriptorProto

py_callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
py_callback_type_t = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init global var: ywf todo:will delete
# first excute for init vars
cybertron_path = os.environ['CYBERTRON_PATH'] # CYBERTRON_PATH=/home/work/baidu/adu-lab/python-wrapper/install
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

def callback(data):
    print("="*80)
    print("py:reader callback msg->:")
    print data
    print("="*80)

def test_listener_class():
    print("=" * 120)
    node = pynode.Node("listener")    
    r = node.create_reader("channel/chatter", chatter_pb2.Chatter, callback)
    while not node.is_shutdown():  
        time.sleep(0.002)

def main():
    test_listener_class()

if __name__ == '__main__':
  main()