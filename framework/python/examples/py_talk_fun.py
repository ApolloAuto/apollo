import time
import sys
import os
import importlib


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

#////////////////////////////////fun///////////////////////////////
def test_talker_fun():
    msg = chatter_pb2.Chatter()
    msg.content = "talker:send!"
    msg.seq = 0
    msg.timestamp = 0
    msg.lidar_timestamp = 0

    _cyber_node.cyber_test0(33, 77)
    node = _cyber_node.new_PyNode("node_name1")
    
    g_count = 1
    w = _cyber_node.PyNode_create_writer(node, "channel/chatter", chatter_pb2.DESCRIPTOR.message_types_by_name['Chatter'].full_name)

    # str1 = _cyber_node.cyber_test0(1,2)
    # print(type(str1))
    # print("++++++" + str1)
    # return
          
    while not _cyber_node.py_is_shutdown():  
        time.sleep(1)
        g_count = g_count + 1
        msg.seq = g_count
        print("="*80)
        print("py:PyWriter_write-> [ %s ]" % msg.SerializeToString())
        _cyber_node.PyWriter_write(w, msg.SerializeToString())

    print("----------end----------")
    _cyber_node.delete_PyWriter(w)
    _cyber_node.delete_PyNode(node)
    
def main():
    test_talker_fun()
    #cyber_py_talker_class()

if __name__ == '__main__':
  main()