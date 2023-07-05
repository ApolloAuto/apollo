# Common Component Example of the CyberRT framework


## How to Build

```
./apollo.sh build cyber
```

Or if you use Bazel directly,

```
bazel build //cyber/examples/common_component_example/...
```

## How to Run

### Enable logging to stderr

- Change `GLOG_alsologtostderr` from `0` to `1` in `cyber/setup.bash`
- Run `source cyber/setup.bash` in current console.

```
export GLOG_alsologtostderr=1
```

### Start the sample component

```
cyber_launch start cyber/examples/common_component_example/common.launch
```

Or

```
mainboard -d cyber/examples/common_component_example/common.dag
```

### Start the writer nodes

Open two more terminals, run the following commands respectively.

```
bazel run  //cyber/examples/common_component_example:channel_test_writer
```

and ...

```
bazel run //cyber/examples/common_component_example:channel_prediction_writer
```

Now you should see console output of the `CommonComponentSample` example from the first terminal.


