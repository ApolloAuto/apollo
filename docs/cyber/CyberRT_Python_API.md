## 1. Background

The core functions of Cyber RT are developed in C++. We also provide more python interfaces to help developers build their own utilities for specific projects.

## 2. Cyber RT Python Interfaces

The python interfaces of Cyber RT are wrapper the corresponding C++ interfaces. The implementation doesn't rely on other third-party tools, e.g. swig, which makes it easier to maintain.

## 3. Overview of Python Interfaces in Cyber RT

So far, the python interfaces covers:

* access the information of channels
* server/client communication
* query informatoin in record files
* read and write from/to record files
* Time/Duration/Rate related operations
* Timer

### 3.1 Read/Write of Channels

Steps shown as below:
  1. First create a Node；
  2. Create corresponding reader or writer;
  3. If write to a channel, use write interface in writer.
  4. If read from a channel, use the spin interface in the node, and process the messages in your callback function

The interfaces are shown below:

```python
class Node:
    """
    Class for cyber Node wrapper.
    """

    def create_writer(self, name, data_type, qos_depth=1):
        """
        create a topic writer for send message to topic.
        @param self
        @param name str: topic name
        @param data_type proto: message class for serialization
        """

   def create_reader(self, name, data_type, callback, args=None):
        """
        create a topic reader for receive message from topic.
        @param self
        @param name str: topic name
        @param data_type proto: message class for serialization
        @callback fn: function to call (fn(data)) when data is
                   received. If args is set, the function must
                   accept the args as a second argument,
                   i.e. fn(data, args)
        @args any: additional arguments to pass to the callback
        """
	def create_client(self, name, request_data_type, response_data_type):
	"""
	"""
	def create_service(self, name, req_data_type, res_data_type, callback, args=None):

    def spin(self):
        """
        spin in wait and process message.
        @param self
        """

class Writer(object):
    """
    Class for cyber writer wrapper.
    """
    def write(self, data):
        """
        writer msg string
        """
```

### 3.2 Record Interfaces

Read from record：

1. Create a RecordReader;
2. Read messages from Record;

Write to record：

1. Create a RecordWriter
2. Write messages to record；

The interfaces are shown below:

```python
class RecordReader(object):
    """
    Class for cyber RecordReader wrapper.
    """
    def read_messages(self, start_time=0, end_time=18446744073709551615):
        """
        read message from bag file.
        @param self
        @param start_time:
        @param end_time:
        @return: generator of (message, data_type, timestamp)
        """
	def get_messagenumber(self, channel_name):
        """
        return message count.
        """
	def get_messagetype(self, channel_name):
        """
        return message type.
        """
	def get_protodesc(self, channel_name):
        """
        return message protodesc.
        """
	def get_headerstring(self):
        """
        return message header string.
        """
	def reset(self):
        """
        return reset.
        """
        return _CYBER_RECORD.PyRecordReader_Reset(self.record_reader)

    def get_channellist(self):
        """
        return channel list.
        """
        return _CYBER_RECORD.PyRecordReader_GetChannelList(self.record_reader)


class RecordWriter(object):
    """
    Class for cyber RecordWriter wrapper.
    """
	def open(self, path):
        """
        open record file for write.
        """
	def write_channel(self, channel_name, type_name, proto_desc):
        """
        writer channel by channelname,typename,protodesc
        """
	def write_message(self, channel_name, data, time, raw = True):
        """
        writer msg:channelname,data,time,is data raw
        """

	def set_size_fileseg(self, size_kilobytes):
        """
        return filesegment size.
        """

    def set_intervaltime_fileseg(self, time_sec):
        """
        return file interval time.
        """

    def get_messagenumber(self, channel_name):
        """
        return message count.
        """

    def get_messagetype(self, channel_name):
        """
        return message type.
        """

    def get_protodesc(self, channel_name):
        """
        return message protodesc.
        """

```

### 3.3 Time Interfaces

```python
class Time(object):
	@staticmethod
    def now():
        time_now = Time(_CYBER_TIME.PyTime_now())
        return time_now

    @staticmethod
    def mono_time():
        mono_time = Time(_CYBER_TIME.PyTime_mono_time())
        return mono_time

    def to_sec(self):
        return _CYBER_TIME.PyTime_to_sec(self.time)

    def to_nsec(self):
        return _CYBER_TIME.PyTime_to_nsec(self.time)

    def sleep_until(self, nanoseconds):
        return _CYBER_TIME.PyTime_sleep_until(self.time, nanoseconds)
```

### 3.4 Timer Interfaces

```python
class Timer(object):

    def set_option(self, period, callback, oneshot=0):
        """
        set the option of timer.
        @param period The period of the timer, unit is ms.
        @param callback The tasks that the timer needs to perform.
        @param oneshot 1:perform the callback only after the first timing cycle
        0:perform the callback every timed period
        """

    def start(self):

    def stop(self):

```

## 4. Examples

### 4.1 Read from Channel (in cyber/python/cyber_py3/examples/listener.py)

```python
from cyber_py3 import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark


def callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:reader callback msg->:")
    print(data)
    print("=" * 80)


def test_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener")
    test_node.create_reader("channel/chatter", ChatterBenchmark, callback)
    test_node.spin()


if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    cyber.shutdown()
 ```

### 4.2 Write to Channel(in cyber/python/cyber_py3/examples/talker.py)

 ```python
import time

from cyber_py3 import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark


def test_talker_class():
    """
    Test talker.
    """
    msg = ChatterBenchmark()
    msg.content = "py:talker:send Alex!"
    msg.stamp = 9999
    msg.seq = 0
    print(msg)
    test_node = cyber.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("channel/chatter", ChatterBenchmark, 6)
    while not cyber.is_shutdown():
        time.sleep(1)
        g_count = g_count + 1
        msg.seq = g_count
        msg.content = "I am python talker."
        print("=" * 80)
        print("write msg -> %s" % msg)
        writer.write(msg)


if __name__ == '__main__':
    cyber.init("talker_sample")
    test_talker_class()
    cyber.shutdown()

```

### 4.3 Read and Write Messages from/to Record File(in cyber/python/cyber_py3/examples/record.py)

```python
"""
Module for example of record.
Run with:
    bazel run //cyber/python/cyber_py3/examples:record
"""

import time

from google.protobuf.descriptor_pb2 import FileDescriptorProto

from cyber.proto.unit_test_pb2 import Chatter
from cyber.python.cyber_py3 import record
from modules.common.util.testdata.simple_pb2 import SimpleMessage


MSG_TYPE = "apollo.common.util.test.SimpleMessage"
MSG_TYPE_CHATTER = "apollo.cyber.proto.Chatter"


def test_record_writer(writer_path):
    """
    Record writer.
    """
    fwriter = record.RecordWriter()
    fwriter.set_size_fileseg(0)
    fwriter.set_intervaltime_fileseg(0)

    if not fwriter.open(writer_path):
        print('Failed to open record writer!')
        return
    print('+++ Begin to writer +++')

    # Writer 2 SimpleMessage
    msg = SimpleMessage()
    msg.text = "AAAAAA"

    file_desc = msg.DESCRIPTOR.file
    proto = FileDescriptorProto()
    file_desc.CopyToProto(proto)
    proto.name = file_desc.name
    desc_str = proto.SerializeToString()
    print(msg.DESCRIPTOR.full_name)
    fwriter.write_channel(
        'simplemsg_channel', msg.DESCRIPTOR.full_name, desc_str)
    fwriter.write_message('simplemsg_channel', msg, 990, False)
    fwriter.write_message('simplemsg_channel', msg.SerializeToString(), 991)

    # Writer 2 Chatter
    msg = Chatter()
    msg.timestamp = 99999
    msg.lidar_timestamp = 100
    msg.seq = 1

    file_desc = msg.DESCRIPTOR.file
    proto = FileDescriptorProto()
    file_desc.CopyToProto(proto)
    proto.name = file_desc.name
    desc_str = proto.SerializeToString()
    print(msg.DESCRIPTOR.full_name)
    fwriter.write_channel('chatter_a', msg.DESCRIPTOR.full_name, desc_str)
    fwriter.write_message('chatter_a', msg, 992, False)
    msg.seq = 2
    fwriter.write_message("chatter_a", msg.SerializeToString(), 993)

    fwriter.close()


def test_record_reader(reader_path):
    """
    Record reader.
    """
    freader = record.RecordReader(reader_path)
    time.sleep(1)
    print('+' * 80)
    print('+++ Begin to read +++')
    count = 0
    for channel_name, msg, datatype, timestamp in freader.read_messages():
        count += 1
        print('=' * 80)
        print('read [%d] messages' % count)
        print('channel_name -> %s' % channel_name)
        print('msgtime -> %d' % timestamp)
        print('msgnum -> %d' % freader.get_messagenumber(channel_name))
        print('msgtype -> %s' % datatype)
        print('message is -> %s' % msg)
        print('***After parse(if needed),the message is ->')
        if datatype == MSG_TYPE:
            msg_new = SimpleMessage()
            msg_new.ParseFromString(msg)
            print(msg_new)
        elif datatype == MSG_TYPE_CHATTER:
            msg_new = Chatter()
            msg_new.ParseFromString(msg)
            print(msg_new)


if __name__ == '__main__':
    test_record_file = "/tmp/test_writer.record"

    print('Begin to write record file: {}'.format(test_record_file))
    test_record_writer(test_record_file)

    print('Begin to read record file: {}'.format(test_record_file))
    test_record_reader(test_record_file)
```
