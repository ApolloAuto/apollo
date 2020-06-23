# How to Build and Run(Test) your Python Application

Everything managed by pure [Bazel](https://docs.bazel.build/versions/master/be/python.html) in Apollo 6.0.

## Create the BUILD file

Generally you need a BUILD target for each python file, which could be one of

* `py_library(name="lib_target", ...)`
* `py_binary(name="bin_target", ...)`
* `py_test(name="test_target", ...)`

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

## Build & Run commands

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
   bazel test //path/to:test_target
   ```
