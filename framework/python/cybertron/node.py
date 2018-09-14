import time
import sys
import os
import importlib
import threading
import ctypes

py_callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
py_callback_type_t = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
cybertron_path = os.environ['CYBERTRON_PATH'] # CYBERTRON_PATH=/home/work/baidu/adu-lab/python-wrapper/install/
cybertron_dir=os.path.split(cybertron_path)[0]
sys.path.append(cybertron_path + "/third_party/")
sys.path.append(cybertron_path + "/lib/")
sys.path.append(cybertron_path + "/python/cybertron")

sys.path.append(cybertron_dir + "/python/")
sys.path.append(cybertron_dir + "/cybertron/")

# begin wrapper cxx interface to py
_cyber_node = importlib.import_module('_cyber_node')

#//////////////////////////////class//////////////////////////////
class Writer:
    def __init__(self, name, writer, data_type):
        self.name = name
        self.writer = writer
        self.data_type = data_type

    # def __del__(self):
    #     print("+++ Writer __del___")
        #_cyber_node.delete_PyWriter(self.writer)

    def write(self, data):
        return _cyber_node.PyWriter_write(self.writer, data.SerializeToString())

class Reader:
    def __init__(self, name, reader, data_type):
        self.name = name
        self.reader = reader
        self.data_type = data_type

    # def __del__(self):
    #     print("+++ Reader __del___")

class Client:
    def __init__(self, client, data_type):
        self.client = client
        self.data_type = data_type

    def __del__(self):
        print("+++ Client __del___")

    def send_request(self, data):
        # c++ fun can use msgbin
        response_str =  _cyber_node.PyClient_send_request(self.client, data.SerializeToString())
        if len(response_str) == 0:
            return None

        response = self.data_type()
        response.ParseFromString(response_str)
        return response

class Node:
    def __init__(self, name):
        self.node = _cyber_node.new_PyNode(name)
        self.list_writer = []
        self.list_reader = []
        self.list_client = []
        self.list_service = []
        self.subs = {}
        self.pubs = {}
        self.services = {}
        self.mutex = threading.Lock()

    def __del__(self):
        #print("+++ node __del___")
        for w in self.list_writer:
            _cyber_node.delete_PyWriter(w)
        for r in self.list_reader:
            _cyber_node.delete_PyReader(r)
        for c in self.list_client:
            _cyber_node.delete_PyClient(c)
        for s in self.list_service:
            _cyber_node.delete_PyService(s)
        _cyber_node.delete_PyNode(self.node)

    def create_writer(self, name, data_type):
        w = _cyber_node.PyNode_create_writer(self.node, name, data_type)
        self.list_writer.append(w)
        return Writer(name, w, data_type)

    def reader_callback(self, name):
        v = self.subs[name]
        msg_str = _cyber_node.PyReader_read( v[0], False)
        if (len(msg_str) > 0):
            proto = v[3]()
            proto.ParseFromString(msg_str)
            response = None
            if v[2] is None:
                response = v[1](proto)
            else:
                response = v[1](proto, v[2])
        return 0

    def create_reader(self, name, data_type, callback, args = None):
        self.mutex.acquire()
        if name in self.subs.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        datatype = data_type.DESCRIPTOR.full_name
        #reader = self.node.create_reader(str(name), str(datatype))
        reader = _cyber_node.PyNode_create_reader(self.node, name, str(data_type))
        if reader is None:
            return None
        self.list_reader.append(reader)
        v = (reader, callback, args, data_type, False)

        self.mutex.acquire()
        self.subs[name] = v
        self.mutex.release()
        f = py_callback_type(self.reader_callback)
        f_ptr = ctypes.cast(f, ctypes.c_void_p).value
        #reader.register_func(f_ptr)
        _cyber_node.PyReader_register_func(reader, f_ptr)

        return Reader(name, reader, data_type)

    def create_client(self, name, request_data_type, response_data_type):
        datatype = request_data_type.DESCRIPTOR.full_name
        c = _cyber_node.PyNode_create_client(self.node, name, str(datatype))
        self.list_client.append(c)
        return Client(c, response_data_type)

    def service_callback(self, name):
        v = self.services[name]
        msg_str = _cyber_node.PyService_read(v[0])

        if (len(msg_str) > 0):
            proto = v[3]()
            proto.ParseFromString(msg_str)
            response = None
            if v[2] is None:
                response = v[1](proto)
            else:
                response = v[1](proto, v[2])

            _cyber_node.PyService_write(v[0], response.SerializeToString())
        return 0

    def create_service(self, name, req_data_type, res_data_type, callback, args = None):
        self.mutex.acquire()
        if name in self.services.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        datatype = req_data_type.DESCRIPTOR.full_name
        s = _cyber_node.PyNode_create_service(self.node, name, str(datatype))
        self.list_service.append(s)
        v = (s, callback, args, req_data_type, False)

        self.mutex.acquire()
        self.services[name] = v
        self.mutex.release()

        f = py_callback_type(self.service_callback)
        f_ptr = ctypes.cast(f, ctypes.c_void_p).value

        #s.register_func(f_ptr)
        _cyber_node.PyService_register_func(s, f_ptr)
        return s

    def spin(self):
        while not _cyber_node.py_is_shutdown():
            time.sleep(0.002)
            self.do_executable()

    def do_executable(self):
        self.mutex.acquire()
        for _, v in self.subs.items():
            msg_str = _cyber_node.PyReader_read( v[0], False)
            if (len(msg_str) > 0):
                if v[4]:
                    if v[2] is None:
                        v[1](msg_str)
                    else:
                        v[1](msg_str, v[2])
                else:
                    proto = v[3]()
                    proto.ParseFromString(msg_str)
                    if v[2] is None:
                        v[1](proto)
                    else:
                        v[1](proto, v[2])
        self.mutex.release()

    def is_shutdown(self):
        return _cyber_node.py_is_shutdown()
