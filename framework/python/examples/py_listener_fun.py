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

# begin wrapper cxx interface to py
_cyber_node = importlib.import_module('_cyber_node') 
    
subs = {}

def reader_callback(name):
    v = subs[name]        
    msg_str = _cyber_node.PyReader_read( v[0], False)
    print("="*80)
    print("py:PyReader_read-> [ %s ]" % msg_str)
    if (len(msg_str) > 0):
        proto = v[1]()
        msg2 = v[1]() # msgtype=chatter_pb2.Chatter()
        msg2.ParseFromString(msg_str)
        print(msg2)

    return 0

def cyber_py_lisener():
    node = _cyber_node.new_PyNode("listener")
    channel_name = "channel/chatter"
    reader = _cyber_node.PyNode_create_reader(node, channel_name, str(chatter_pb2.Chatter))
    v = (reader, chatter_pb2.Chatter)
    subs[channel_name] = v
    f = py_callback_type(reader_callback)
    f_ptr = ctypes.cast(f, ctypes.c_void_p).value
    _cyber_node.PyReader_register_func(reader, f_ptr)
    
    while not _cyber_node.py_is_shutdown():
        time.sleep(0.002)
    
def main():
    cyber_py_lisener()

if __name__ == '__main__':
  main()