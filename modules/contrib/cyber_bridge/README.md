# cyber-bridge

## Introduction

This is bridge that exposes custom TCP socket for accepting and transmitting Cyber messages.

## Directory Structure
```shell
modules/contrib/cyber_bridge/
├── bridge.cc
├── BUILD
├── client.cc
├── client.h
├── clients.cc
├── clients.h
├── cyber-bridge.BUILD
├── cyberfile.xml
├── LICENSE
├── node.cc
├── node.h
├── README.md
├── server.cc
└── server.h
```

## Building

Run the build inside docker.

```shell
cd /apollo && bash apollo.sh build contrib                      # in source env
cd /apollo_workspace && buildtool build -p modules/contrib      # in package management env
```

## Running

```shell
cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
```

For extra logging:

```shell
GLOG_v=4 GLOG_logtostderr=1 ./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge # in source env
GLOG_v=4 GLOG_logtostderr=1 cyber_bridge                                          # in package management env 
```

Add extra `-port 9090` argument for custom port (9090 is default).

## Example

In one terminal launch `cyber_bridge`:

```shell
cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
```

In another terminal launch example talker:

```shell
cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/talker    # in source env
talker                                                              # in package management env     
```

In one more terminal launch example listener:

```shell
cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/listener  # in source env
listener                                                            # in package management env 
```

Now you should observe talker and listener sending & receiving message with incrementing integer.

