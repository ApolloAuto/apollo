Python API tutorial
========================

In this tutorial we introduce the basic concepts of the CyberRT Python API, as
well as an overview of its most important functionalities.

``Note:``
    **This document applies only to the latest development version**. The source code is [CyberRT Python API](https://github.com/ApolloAuto/apollo/tree/master/cyber/python)<br>
    
#### Talker and listener

To send messages we need to create a "writer" object, to do so we need
to provide the channel name and message type.

```py
def test_talker_class():
    """
    Test talker.
    """
    msg = SimpleMessage()
    msg.text = "talker:send Alex!"
    msg.integer = 0

    test_node = cyber.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("channel/chatter",
                                     SimpleMessage, 6)
    while not cyber.is_shutdown():
        time.sleep(1)
        g_count = g_count + 1
        msg.integer = g_count
        print "=" * 80
        print "write msg -> %s" % msg
        writer.write(msg)
```

The listener should create a "reader" object, we need to provide the channel name and the message type, also need the callback function to process received data.

```py
def callback(data):
    """
    Reader message callback.
    """
    print "=" * 80
    print "py:reader callback msg->:"
    print data
    print "=" * 80


def test_listener_class():
    """
    Reader message.
    """
    print "=" * 120
    test_node = cyber.Node("listener")
    test_node.create_reader("channel/chatter",
                            SimpleMessage, callback)
    test_node.spin()
```

#### Record

A record is a file for storing CyberRT message data. We provide Python API for you to store, process, analyze them.

Example 1: [get record channel info](https://github.com/ApolloAuto/apollo/blob/master/cyber/python/examples/record_channel_info.py)

```py
def print_channel_info(file_path):
    freader = record.RecordReader(file_path)
    channels = freader.get_channellist()

    header_msg = freader.get_headerstring()
    header = record_pb2.Header()
    header.ParseFromString(header_msg)

    print('\n++++++++++++Begin Channel Info Statistics++++++++++++++')
    print('-' * 40)
    print('record version: [%d:%d]' % (header.major_version, header.minor_version))
    print('record message_number: %s' % str(header.message_number))
    print('record file size(Byte): %s' % str(header.size))
    print('chunk_number: %d' % header.chunk_number)
    print('channel count: %d' % len(channels))
    print('-' * 40)
    count = 0
    for channel in channels:
        desc = freader.get_protodesc(channel)
        count += 1
        print('Channel: %s, count: %d, desc size: %d' % (channel, count, len(desc)))
        # print desc
    print "++++++++++++Finish Channel Info Statistics++++++++++++++\n"
```

Example 2: [trans record](https://github.com/ApolloAuto/apollo/blob/master/cyber/python/examples/record_trans.py)

```
def test_record_trans(reader_path):
    """
    Record trans.
    """
    fwriter = record.RecordWriter()
    if not fwriter.open(TEST_RECORD_FILE):
        print('Failed to open record writer!')
        return
    print('+++ Begin to trans +++')

    fread = record.RecordReader(reader_path)
    count = 0
    for channelname, msg, datatype, timestamp in fread.read_messages():
        # print channelname, timestamp, fread.get_messagenumber(channelname)
        desc = fread.get_protodesc(channelname)
        fwriter.write_channel(channelname, datatype, desc)
        fwriter.write_message(channelname, msg, timestamp)
        count += 1
    print('-' * 80)
    print('Message count: %d' % count)
    print('Channel info: ')
    channel_list = fread.get_channellist()
    print('Channel count: %d' % len(channel_list))
    print(channel_list)
```

#### Client and service
Client and service provide another mode for communication. The publish / subscribe model is a very flexible communication mode, but its many-to-many one-way transport is not appropriate for RPC request / reply interactions, which are often required in a distributed system. Request / reply is done via a Service, which is defined by a pair of messages: one for the request and one for the reply. 

A client can make a persistent connection to a service, which enables higher performance at the cost of less robustness to service provider changes.

Example 1:[Client](https://github.com/ApolloAuto/apollo/blob/master/cyber/python/examples/client.py)

```
def test_client_class():
    """
    Client send request
    """
    node = cyber.Node("client_node")
    client = node.create_client(
        "server_01", ChatterBenchmark, ChatterBenchmark)
    req = ChatterBenchmark()
    req.content = "clt:Hello service!"
    req.seq = 0
    count = 0
    while not cyber.is_shutdown():
        time.sleep(1)
        count = count + 1
        req.seq = count
        print "-" * 80
        response = client.send_request(req)
        print "get Response [ ", response, " ]"

```

Example 2:[Service](https://github.com/ApolloAuto/apollo/blob/master/cyber/python/examples/service.py)

```
def callback(data):
    print "-" * 80
    print "get Request [ ", data, " ]"
    # print "req type is ", type(data) # <class
    # 'cyber.proto.chatter_pb2.ChatterBenchmark'>
    response = ChatterBenchmark()
    response.content = "svr: Hello client!"
    response.seq = data.seq + 2
    return response


def test_service_class():
    """
    Reader message.
    """
    print "=" * 120
    node = cyber.Node("service_node")
    r = node.create_service(
        "server_01", ChatterBenchmark, ChatterBenchmark, callback)
    node.spin()
```

