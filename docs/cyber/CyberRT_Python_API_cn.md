## 1. 背景

   Cyber 核心代码是由 C++ 开发，同时为了方便开发者，提供了 Python 接口。

## 2. CyberRT Python 接口实现思路

   Cyber Python 接口的实现思路是在 Cyber C++ 实现的基础上，做了一层 Python 的封装，由 Python 来调用 C++ 的实现函数。Cyber Python Wrapper 的实现没有使用 swig 等第三方工具，完全自主实现，以此保证代码的高可维护性和可读性。

## 3. 主要接口

   目前提供的主要接口包括：

* channel 读、写
* server/client 通信
* record 信息查询
* record 文件读、写
* Time/Duration/Rate 时间操作
* Timer

### 3.1 Channel 读写接口

使用步骤是：

1. 首先创建 Node;
2. 创建对应的 reader 或 writer；
3. 如果是向 channel 写数据，调用 writer 的 write 接口；
4. 如果是从 channel 读数据，调用 node 的 spin，对收到的消息进行消费；

接口定义如下：

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

### 3.2 Record 接口

Record 读的操作是：

1. 创建一个 RecordReader;
2. 对 Record 进行迭代读；

Record 写的操作是：

1. 创建一个 RecordWriter
2. 对 Record 进行写消息；

接口定义如下：

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

### 3.3 Time 接口

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

### 3.4 Timer 接口

```

class Timer(object):

	def set_option(self, period, callback, oneshot=0):
        '''
        period The period of the timer, unit is ms
        callback The tasks that the timer needs to perform
        oneshot 1: perform the callback only after the first timing cycle
                0:perform the callback every timed period
        '''


    def start(self):


    def stop(self):


```

## 4. 例子

### 4.1 读 channel （参见 cyber/python/cyber_py3/examples/listener.py)

```python
"""Module for example of listener."""
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

### 4.2 写 channel（参见 cyber/python/cyber_py3/examples/talker.py)

```python
"""Module for example of talker."""
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

### 4.3 读写消息到 Record 文件（参见 cyber/python/cyber_py3/examples/record.py)

```python
cyber/python/cyber_py3/examples/record.py)

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
