# How to Build, Test and Run your Python Application

Starting from Apollo 6.0, building and testing Python applications in Apollo is
done using [Bazel](https://docs.bazel.build/versions/master/be/python.html)
exclusively. We use Bazel Python rules to build, run, and test Python programs.
This not only frees us from hand-crafting Protobuf dependencies and managing
Python related Env variables manually, but also helps with managing third party
Python module dependency.

## Create the BUILD file

Generally you need a BUILD target for each python file, which could be one of

- `py_library(name="lib_target", ...)`
- `py_binary(name="bin_target", ...)`
- `py_test(name="test_target", ...)`

### Example

```python
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "foo",
    srcs = ["foo.py"],
    data = ["//path/to/a/data/dependency"],  # which we invoke at run time, maybe a cc_binary etc.
    deps = [
        ":foolib",
        "//path/to/a/py/library",
        ...
    ],
)

py_library(
    name = "foolib"
    srcs = ["foolib.py"],
    deps = [
        "//path/to/a/py/library",
        ...
    ],
)

py_test(
    name = "foo_test",
    srcs = ["foo_test.py"],
    deps = [
        ":foolib",
        "//path/to/a/py/library",
        ...
    ],
)
```

Above is a BUILD file template, you can also use the
[BUILD](../../cyber/python/BUILD) and
[BUILD](../../cyber/python/cyber_py3/examples/BUILD)
file as examples.

## Build, Test and Run commands

1. To build any target:

   ```bash
   bazel build //path/to:target
   ```

1. To run a binary target:

   ```bash
   bazel run //path/to:target
   ```

1. To run a unit test target:

   ```bash
   bazel test //path/to:target_test
   ```
