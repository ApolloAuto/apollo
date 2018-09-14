import time
import sys
import unittest

sys.path.append("../")
from cybertron import node
from cybertron import init
from proto import chatter_pb2

def callback(data):
    print("="*80)
    print("py:reader callback msg->:")
    print data
    print("="*80)

class Test_node(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        init.init()

    @classmethod
    def tearDownClass(cls):
        init.shutdown()

    def test_writer(self):     
        msg = chatter_pb2.Chatter()
        msg.content = "talker:send Alex!"
        msg.seq = 0
        msg.timestamp = 0
        msg.lidar_timestamp = 0
        
        self.assertTrue(init.ok())
        test_node = node.Node("node_name1")
        w = test_node.create_writer("channel/chatter", chatter_pb2.Chatter.DESCRIPTOR.full_name)
        self.assertEqual(w.name, "channel/chatter")
        self.assertEqual(w.data_type, "apollo.cybertron.proto.Chatter")
        self.assertTrue(w.write(msg))

    def test_reader(self):
        self.assertTrue(init.ok())
        test_node = node.Node("listener")
        r = test_node.create_reader("channel/chatter", chatter_pb2.Chatter, callback)
        self.assertEqual(r.name, "channel/chatter")
        self.assertEqual(r.data_type, chatter_pb2.Chatter)
        self.assertEqual(chatter_pb2.Chatter.DESCRIPTOR.full_name, "apollo.cybertron.proto.Chatter")

if __name__ == '__main__':
  unittest.main()