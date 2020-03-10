# How to Add a New External Dependency

## 1. Library that supports bazel

For example,

* [Abseil](https://github.com/abseil/abseil-cpp)
* [Google Test](https://github.com/google/googletest)

Simply import it into WORKSPACE:

```python
git_repository(
  name = "com_google_absl",
  remote = "https://github.com/abseil/abseil-cpp",
  tag = "20190808",
)
```

## 2. Library that has third-party bazel rule

Some popular libraries don't support bazel natively, but may be resolved by
other developers.

Try boost as an example:

```python
git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "9f9fb8b2f0213989247c9d5c0e814a8451d18d7f",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1570056263 -0700",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
```

Now you can refer to it in BUILD files with `@boost//:system`, etc.

## 3. Library with handcrafted BUILD file

It's pretty common to do so. But it needs very solid knowledge with bazel.

```python
new_http_archive(
    name = "civetweb",
    url = "https://github.com/civetweb/civetweb/archive/v1.11.tar.gz",
    sha256 = "de7d5e7a2d9551d325898c71e41d437d5f7b51e754b242af897f7be96e713a42",
    build_file = "third_party/civetweb.BUILD",
    strip_prefix = "civetweb-1.11",
)
```

## 4. Library which is pre-installed into the operating system

It's NOT recommended, as it breaks the rule of a self-contained bazel WORKSPACE.
However, some libraries are very complicated to build with bazel, while the
operating system, such as Ubuntu, provides easy installation.

For example,

* [OpenCV](https://github.com/opencv/opencv)
* [Poco](https://github.com/pocoproject/poco)

Please do raise a discussion before doing so. Then we can add it to the docker
image:

```bash
sudo apt install libopencv-dev libpoco-dev
```

Then add it as a third_party library in
[third_party/BUILD](https://github.com/ApolloAuto/apollo/blob/master/third_party/BUILD).

```python
cc_library(
    name = "PocoFoundation",
    linkopts = ["-lPocoFoundation"],
)
```

Note that in such case, you should include the headers like
`#include <Poco/SharedLibrary.h>` instead of `#include "Poco/SharedLibrary.h"`
as they are in the system path.

## References

For a detailed description on adding a dependency with Bazel, refer to the following:
* [Workspace Rules](https://bazel.build/versions/master/docs/be/workspace.html)
* [Working with external dependencies](https://docs.bazel.build/versions/master/external.html).
