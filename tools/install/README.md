# Release Build for Apollo

## How to Run

Please login to Dev Docker container and change directory to `/apollo`.

Approach 1:

```bash
bash tools/install/run.sh
```

This is the hard-coded implementation of Release build, and you can see that the
TimerComponentExample in CyberRT works.

Approach 2:

```bash
./apollo.sh release
cd /tmp/apollo
source cyber/setup.bash
```

The current issue with Approach 2 is that shared libraries which `mainboard` links
to didn't get installed automatically.

Need to figure out how [Drake](https://github.com/RobotLocomotion/drake) project
handles this.

## Known Issues

```text
warning: working around a Linux kernel bug by creating a hole of 2183168 bytes in ‘/tmp/apollo/cyber/examples/common_component_example/channel_prediction_writer’
warning: working around a Linux kernel bug by creating a hole of 2183168 bytes in ‘/tmp/apollo/cyber/examples/common_component_example/channel_test_writer’
warning: working around a Linux kernel bug by creating a hole of 2183168 bytes in ‘/tmp/apollo/cyber/examples/talker’
warning: working around a Linux kernel bug by creating a hole of 2183168 bytes in ‘/tmp/apollo/cyber/examples/listener’
```
