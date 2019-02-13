# Vehicle Modes

This folder contains modes configuration for Apollo vehicle. Each pb.txt should
be an instance of HMIMode. Check the proto for detailed information.

## Name Convention

We'll convert the file name to a readable title-case name automatically to
display on Dreamview. So please make it simple, clean and meaningful.

Some examples:

* mkz_standard_debug.pb.txt -> "Mkz Standard Debug"
* mkz_close_loop.pb.txt     -> "Mkz Close Loop"
* mkz_map_collection.pb.txt -> "Mkz Map Collection"

## Monitor New Channels

The Cyber Reader API requires client to provide data type when subscribing a
channel. So you need to extend the ChannelMonitor to
[handle new channels](https://github.com/ApolloAuto/apollo/blob/master/modules/monitor/software/channel_monitor.cc#L51).
