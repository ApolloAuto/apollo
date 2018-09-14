import sys
import os
import importlib


# init vars
cybertron_path = os.environ['CYBERTRON_PATH']
cybertron_dir=os.path.split(cybertron_path)[0]
sys.path.append(cybertron_path + "/third_party/")
sys.path.append(cybertron_path + "/lib/")
sys.path.append(cybertron_path + "/python/cybertron")

sys.path.append(cybertron_dir + "/python/")
sys.path.append(cybertron_dir + "/cybertron/")

_cyber_record = importlib.import_module('_cyber_record') 

#//////////////////////////////record file class//////////////////////////////
class RecordReader:
    def __init__(self):
        self.record_reader = _cyber_record.new_PyRecordReader()

    def __del__(self):
        _cyber_record.delete_PyRecordReader(self.record_reader)

    def Open(self, path):
        return _cyber_record.PyRecordReader_Open(self.record_reader, path)

    def Close(self):
        _cyber_record.PyRecordReader_Close(self.record_reader)

    def ReadMessage(self):
        return _cyber_record.PyRecordReader_ReadMessage(self.record_reader)

    def EndOfFile(self):
        return _cyber_record.PyRecordReader_EndOfFile(self.record_reader)

    def CurrentMessageChannelName(self):
        return _cyber_record.PyRecordReader_CurrentMessageChannelName(self.record_reader)

    def CurrentRawMessage(self):
        return _cyber_record.PyRecordReader_CurrentRawMessage(self.record_reader)

    def CurrentMessageTime(self):
        return _cyber_record.PyRecordReader_CurrentMessageTime(self.record_reader)

    def GetMessageNumber(self, channel_name):
        return _cyber_record.PyRecordReader_GetMessageNumber(self.record_reader, channel_name)

    def GetMessageType(self, channel_name):
        return _cyber_record.PyRecordReader_GetMessageType(self.record_reader, channel_name)

    def GetProtoDesc(self, channel_name):
        return _cyber_record.PyRecordReader_GetProtoDesc(self.record_reader, channel_name)
        
    def GetHeaderString(self):
        return _cyber_record.PyRecordReader_GetHeaderString(self.record_reader)
        
class RecordWriter:
    def __init__(self):
        self.record_writer = _cyber_record.new_PyRecordWriter()

    def __del__(self):
        _cyber_record.delete_PyRecordWriter(self.record_writer)

    def Open(self, path):
        return _cyber_record.PyRecordWriter_Open(self.record_writer, path)

    def Close(self):
        _cyber_record.PyRecordWriter_Close(self.record_writer)

    def WriteChannel(self, channel_name, type_name, proto_desc):
        return _cyber_record.PyRecordWriter_WriteChannel(self.record_writer, channel_name, type_name, proto_desc)
    
    def WriteMessage(self, channel_name, rawmessage, time):
        return _cyber_record.PyRecordWriter_WriteMessage(self.record_writer, channel_name, rawmessage, time)
