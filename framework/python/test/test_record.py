import time
import sys
import os
import unittest

sys.path.append("../")
from cybertron import init
from cybertron import record
from proto import record_pb2

TEST_RECORD_FILE = "test02.record"
CHAN_1 = "channel/chatter"
CHAN_2 = "/test2"
MSG_TYPE = "apollo.cybertron.proto.Test"
STR_10B = "1234567890"
TEST_FILE = "test.record"
TIME = 999

class TestRecord(unittest.TestCase):
    def test_record_writer_read(self):       
        self.assertTrue(init.init())

        # writer
        fw = record.RecordWriter()
        self.assertTrue(fw.Open(TEST_RECORD_FILE))
        fw.WriteChannel(CHAN_1, MSG_TYPE, STR_10B)
        fw.WriteMessage(CHAN_1, STR_10B, TIME)
        fw.Close()
        
        # reader
        fr = record.RecordReader()
        self.assertTrue(fr.Open(TEST_RECORD_FILE))
        time.sleep(1)

        self.assertFalse(fr.EndOfFile())
        self.assertTrue(fr.ReadMessage())
        channelname = fr.CurrentMessageChannelName()
        self.assertEqual(CHAN_1, fr.CurrentMessageChannelName())
        self.assertEqual(STR_10B, fr.CurrentRawMessage())
        self.assertEqual(TIME, fr.CurrentMessageTime())
        self.assertEqual(1, fr.GetMessageNumber(channelname))
        self.assertEqual(MSG_TYPE, fr.GetMessageType(channelname))
        msg = record_pb2.Header()
        header_msg = fr.GetHeaderString()
        msg.ParseFromString(header_msg)
        self.assertEqual(1, msg.major_version)
        self.assertEqual(0, msg.minor_version)
        self.assertEqual(1, msg.chunk_number)
        self.assertEqual(1, msg.channel_number)
        self.assertTrue(msg.is_complete)

        self.assertTrue(fr.EndOfFile())    
        fr.Close()
        
        init.shutdown()

if __name__ == '__main__':
  unittest.main()
  
