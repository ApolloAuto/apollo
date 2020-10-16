# Bridge Header Protocol

This document describes the details about bridge header protocol.

## Introduction

Bridge is responsible for forwarding the proto message specified in Apollo to
the external module in UDP mode, and receiving the proto message from the
external module. Due to the limitation of the UDP packet (the maximum packet
size is 65535 bytes), the proto message exceeds this limit after serialization.
It is possible, so the serialized packets need to be fragmented, and each slice
is called a frame. The receiver receives all the frames and then combines and
deserializes them. In order to conveniently distinguish each frame of data, to
facilitate subsequent combination and deserialization, it is necessary to add
frame header information at the head of each frame of data, as shown in the
following figure:

```
    +-----+------------+------------+-------+-----------+
    |Flag |Header Size |Header Item |... ...|Header Item|
    +-----+------------+------------+-------+-----------+
```

As shown in the figure above, the Header consists of three segments: Flag, Size,
and Items, where the segment is separated by '\n'. Flag: The sign of this
agreement, currently a string "ApolloBridgeHeader"; Size: the length of the
entire Items section (unit: byte) Items: The specific content of the header,
where each item has three components, each of which is divided by ':'. The
specific schematic is shown in the figure below.

```
    +----------+----------+-------------+
    |Item Type |Item Size | Item Content|
    +----------+----------+-------------+
```

The Item Type is defined as follows:

```c++
Enum HType {
Header_Ver, // header version number, type: uint32_t
Msg_Name,   // proto name, such as Chassis, type: std::string
Msg_ID,     // sequence num of proto, guaranteed unique value in the same proto, type: uint32_t
Msg_Size,   // the size of the proto serialized, type: size_t
Msg_Frames, // how many frames are proto serialized, type: uint32_t
Frame_Size, // the size of the current frame, type: size_t
Frame_Pos   // The offset of the current frame in the meta-serialized buf, type: size_t
Frame_Index,// The sequence number of the current frame in the total number of frames, starting from 0, type: uint32_t
Time_Stamp, // timestamp, type: double
Header_Tail,// indicates the maximum value of the currently supported Type, always at the end of the enum
}
```
