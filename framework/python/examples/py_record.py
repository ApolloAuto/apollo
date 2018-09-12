import time
import sys
import os

sys.path.append("../")
from cybertron import pyinit
from cybertron import pyrecord
from proto import record_pb2

#TEST_RECORD_FILE = "p20180903154123.record"
TEST_RECORD_FILE = "test02.record"
CHAN_1 = "channel/chatter"
CHAN_2 = "/test2"
MSG_TYPE = "apollo.cybertron.proto.Test"
STR_10B = "1234567890"
TEST_FILE = "test.record"

def test_record_writer(writer_path):
    fw = pyrecord.RecordWriter()
    if (not fw.Open(writer_path)):
        print("writer open failed!")
        return 
    print("+++ begin to writer...")
    fw.WriteChannel(CHAN_1, MSG_TYPE, STR_10B)
    msg1 = record_pb2.SingleMessage()
    msg1.channel_name = CHAN_1
    msg1.content = STR_10B
    msg1.time = 888
    msg_str = msg1.SerializeToString()
    fw.WriteMessage(msg_str)
    fw.WriteMessage(msg_str)
    fw.Close()

def test_record_reader(reader_path):
    fr = pyrecord.RecordReader()
    if (not fr.Open(reader_path)):
        print("reader open failed!")
        return 
    time.sleep(1)
    print("+"*80)
    print("+++begin to read...")
    count = 0
    read_msg_succ = True
    while not fr.EndOfFile():
        print("="*80)
        print("read [%d] msg" % count)
        read_msg_succ = fr.ReadMessage()
        if(read_msg_succ):
            channelname = fr.CurrentMessageChannelName()
            print("chnanel_name -> %s" % fr.CurrentMessageChannelName())
            #print("msg -> %s" % fr.CurrentRawMessage())
            print("msgtime -> %d" % fr.CurrentMessageTime())
            print("msgnum -> %d" % fr.GetMessageNumber(channelname))
            print("msgtype -> %s" % fr.GetMessageType(channelname))
            #print("pbdesc -> %s" % fr.GetProtoDesc(channelname))
            count = count + 1
    fr.Close()

    pyinit.shutdown()

def main():
    pyinit.init()
    test_record_writer(TEST_RECORD_FILE)
    test_record_reader(TEST_RECORD_FILE)
    pyinit.shutdown()

if __name__ == '__main__':
  main()
  
