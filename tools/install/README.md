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
bazel run //:install -- /tmp
# To just view what will be installed
bazel run //:install -- --list /tmp
```

The current issue with Approach 2 is that only `mainboard` was installed. Shared
libraries it links didn't get installed automatically.

Need to figure out how [Drake](https://github.com/RobotLocomotion/drake) project
handles this.
