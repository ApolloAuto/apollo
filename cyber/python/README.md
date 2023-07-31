# Cyber RT Python API : An Example

This document is an example demonstrating how to use Cyber RT Python API
to write your own Python3 programs. Please make sure you have built Apollo
successfully.

## Step 1: Write your own code.

Save it as, say, `path/to/my_demo.py`.

```python3
#!/usr/bin/env python3

import sys

from cyber.python.cyber_py3 import cyber


cyber.init()

if not cyber.ok():
    print('Well, something went wrong.')
    sys.exit(1)

# Do your job here.
cyber.shutdown()
```

## Step 2: Write Python rule for Bazel to build

Edit `path/to/BUILD` file, add the followng section:

```
load("@rules_python//python:defs.bzl", "py_binary")

# blablahblah...

# Add your own section here
py_binary(
    name = "my_demo",
    srcs = ["my_demo.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
    ],
)
```

**Note**: Like C++, Python code is also managed by Bazel starting from Apollo 6.0.
Please refer to [How to Build and Run Python Apps in Apollo](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/howto/how_to_build_and_run_python_app.md) for more on that.

## Step 3: Build and run the demo program

Now you can run the following commands to build and run the demo program.

```
bazel build //path/to:my_demo
./bazel-bin/path/to/my_demo
```

Or simply run

```
bazel run //path/to:my_demo
```

## More Examples ...

Learn more Cyber RT Python examples under the [examples](cyber_py3/examples/) and
[tests](cyber_py3/test/) directory.

