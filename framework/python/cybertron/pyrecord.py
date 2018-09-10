import time
import sys
import os
import importlib
import threading
import ctypes

from google.protobuf.descriptor_pb2 import FileDescriptorProto

py_callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
py_callback_type_t = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
cybertron_path = os.environ['CYBERTRON_PATH']
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
_cyber_record = importlib.import_module('_cyber_record') 

#//////////////////////////////record file class//////////////////////////////
class PyRecordFileReader:
    def __init__(self):
        self.record_reader = _cyber_record.new_PyRecordFileReader()

    def __del__(self):
        print("record_reader __del___")
        _cyber_record.delete_PyRecordFileReader(self.record_reader)

    def Open(self, path):
        return _cyber_record.PyRecordFileReader_Open(self.record_reader, path)
    
    def Close(self):
        _cyber_record.PyRecordFileReader_Close(self.record_reader)

class PyRecordFileWriter:
    def __init__(self):
        self.record_writer = _cyber_record.new_PyRecordFileWriter()

    def __del__(self):
        print("+++ record_writer __del___")
        _cyber_record.delete_PyRecordFileWriter(self.record_writer)

    def Open(self, path):
        return _cyber_record.PyRecordFileWriter_Open(self.record_writer, path)
    
    def Close(self):
        _cyber_record.PyRecordFileWriter_Close(self.record_writer)

    def WriteHeader(self, header_str):
        return _cyber_record.PyRecordFileWriter_WriteHeader(self.WriteHeader, header_str)


#///////////////////////////////////////////////////////////////////////////
def main():
    print("no call!")
    
if __name__ == '__main__':
  main()