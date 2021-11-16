# How to Add a New External Dependency

The bazel files about third-party dependencies are all in the folder
[third_party](../../third_party)
which has a structure as following.

```shell
  third_party
      ├── absl
      │   ├── BUILD
      │   └── workspace.bzl
      ├── ACKNOWLEDGEMENT.txt
      ├── adolc
      │   ├── adolc.BUILD
      │   ├── BUILD
      │   └── workspace.bzl
      ├── ad_rss_lib
      │   ├── ad_rss_lib.BUILD
      │   ├── BUILD
      │   └── workspace.bzl
      ├── benchmark
      │   ├── benchmark.BUILD
      │   ├── BUILD
      │   └── workspace.bzl
      ├── boost
      │   ├── boost.BUILD
      │   ├── BUILD
      │   └── workspace.bzl
      ├── BUILD
  ......
```

For each external denpendency library, there is a seperate folder that contains
a `BUILD` file with the contents of:

```python
package(
    default_visibility = ["//visibility:public"],
)
```

Further, libraries can be divided into several categories.

## 1. Library that supports bazel

For example,

- [Abseil](https://github.com/abseil/abseil-cpp)
- [Google Test](https://github.com/google/googletest)

Import it into workspace.bzl:

```python
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_google_absl",
        sha256 = "f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111",
        strip_prefix = "abseil-cpp-20200225.2",
        urls = [
            "https://github.com/abseil/abseil-cpp/archive/20200225.2.tar.gz",
        ],
    )
```

## 2. Library with handcrafted BUILD file

It's pretty common to do so. But it needs very solid knowledge with bazel.

[workspace.bzl](../../third_party/yaml_cpp/workspace.bzl):

```python
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "com_github_jbeder_yaml_cpp",
        build_file = clean_dep("//third_party/yaml_cpp:yaml_cpp.BUILD"),
        sha256 = "77ea1b90b3718aa0c324207cb29418f5bced2354c2e483a9523d98c3460af1ed",
        strip_prefix = "yaml-cpp-yaml-cpp-0.6.3",
        urls = [
            "https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz",
        ],
    )
```

[yaml.BUILD](../../third_party/yaml_cpp/yaml.BUILD):

```python
load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "yaml-cpp",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/yaml-cpp/*.h",
        "src/*.h",
    ]),
    includes = [
        "include",
        "src",
    ],
    visibility = ["//visibility:public"],
)
```

## 3. Library which is pre-installed into the operating system

It's NOT recommended, as it breaks the rule of a self-contained bazel WORKSPACE.
However, some libraries are very complicated to build with bazel, while the
operating system, such as Ubuntu, provides easy installation.

Please do raise a discussion before doing so. Then we can add it to the docker
image.

For example,

- [Poco](https://github.com/pocoproject/poco)

[workspace.bzl](../../third_party/poco/workspace.bzl):

```python
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "poco",
        build_file = clean_dep("//third_party/poco:poco.BUILD"),
        path = "/opt/apollo/sysroot/include",
    )
```

[poco.BUILD](../../third_party/poco/poco.BUILD):

```python
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "PocoFoundation",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lPocoFoundation",
    ],
)
```

Note that in such case, you should include the headers like
`#include <Poco/SharedLibrary.h>` instead of `#include "Poco/SharedLibrary.h"`
as they are in the system path.

## Note

For all of the above types of external dependencies, we also need to add them
into
[tools/workspace.bzl](../../tools/workspace.bzl)

## References

For a detailed description on adding a dependency with Bazel, refer to the
following:

- [Workspace Rules](https://bazel.build/versions/master/docs/be/workspace.html)
- [Working with external dependencies](https://docs.bazel.build/versions/master/external.html).
