# Apollo Cyber bridge for the `master` branch

This is bridge that exposes custom TCP socket for accepting and transmitting Cyber messages.

# Building

Run the build from the `/apollo` directory inside docker.

```
bazel build //modules/contrib/cyber_bridge
```

# Running

From the `/apollo` directory, execute:

```
./bazel-bin/module/contrib/cyber_bridge/cyber_bridge
```

For extra logging:

```
GLOG_v=4 GLOG_logtostderr=1 ./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge
```

Add extra `-port 9090` argument for custom port (9090 is default).

# Example

In one terminal launch `cyber_bridge`:

```
./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge
```

In another terminal launch example talker:

```
./bazel-bin/cyber/python/cyber_py3/examples/talker
```

In one more terminal launch example listener:

```
./bazel-bin/cyber/python/cyber_py3/examples/listener
```

Now you should observe talker and listener sending & receiving message with incrementing integer.

