# Data Tools

## Recorder

A customized rosbag recorder written in Python, which will be upgraded to newer
version soon.

## Event Collector

Collect events which we are interested in, including *drive events* and
*disengagements*.

### Usage

It will generate `events.txt` in current directory, with row format:

```text
<timestamp> <type> <description if any>
```

On termination with SIGINT, it will also generate `events_related_bags.txt` in
current directory, with row format:

```text
/path/to/bag <event timestamp offset>
```

Note that the offset may be larger than the bag end time, which means the event
is actually in the next bag.
