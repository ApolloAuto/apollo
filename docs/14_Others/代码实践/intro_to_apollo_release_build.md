# Introduction to Apollo Release Build

## Background

For quite some time, binary distribution was unavailable for CyberRT-based
Apollo. Users have to build the whole project themselves inside Apollo Dev
container before they can run various modules and tools. This incapability of
deployment causes trouble in certain situations. For example, people from
SVL Simulator find that the total size of all Docker images, volumes and Bazel
caches and build artifacts sums up to 40+ GB, which was rather inconvenient for
their use case.

## Principle of Release Build Implementation

The root cause of this incapability was Bazel's lack of out-of-box installation
support similar to those provided in other build systems (e.g., `make install`).

To resolve this issue, we borrowed ideas from the
[Drake](https://github.com/RobotLocomotion/drake) project, and implemented the
`install` extension in the Starlark language, which can be used to install
binaries, shared libraries, resource files (config, data, launch files, dags,
etc) and documents for the Apollo project.

Installation for standalone binaries was rather straightforward. As you may
already know, the core concept of the CyberRT framework was to load each module
(e.g, Perception, Prediction, Planning, etc) dynamically as a component in the
form of `libX_component.so`. Both the `mainboard` binary program and
`libX_component.so` links thousands of other shared libraries themselves. For
example, running `ldd` on the Planning module with the following command

```bash
ldd bazel-bin/modules/planning/libplanning_component.so
```

will show the following output:

```text
	linux-vdso.so.1 (0x00007ffc8a77c000)
	libmodules_Splanning_Slibplanning_Ucomponent_Ulib.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Slibplanning_Ucomponent_Ulib.so (0x00007fe8a7f9f000)
	libmodules_Splanning_Slibnavi_Uplanning.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Slibnavi_Uplanning.so (0x00007fe8a7d81000)
	libmodules_Splanning_Slibon_Ulane_Uplanning.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Slibon_Ulane_Uplanning.so (0x00007fe8a7b53000)
	libmodules_Splanning_Slibplanning_Ubase.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Slibplanning_Ubase.so (0x00007fe8a7945000)
	libmodules_Splanning_Scommon_Ssmoothers_Slibsmoother.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Scommon_Ssmoothers_Slibsmoother.so (0x00007fe8a7739000)
	libmodules_Splanning_Splanner_Slibplanner_Udispatcher.so => /apollo/bazel-bin/modules/planning/../../_solib_local/libmodules_Splanning_Splanner_Slibplanner_Udispatcher.so (0x00007fe8a752e000)
    ...
```

How to _install_ `libplanning_component.so` with all the shared objects (i.e.,
"\*.so" files) it links becomes the hardest issue to solve to implement the
`install` rule.

`patchelf` comes to rescue. All the shared objects can be retrieved from
`runfiles_data`, while `patchelf --force-rpath --set-rpath` can be used to
change RPATH settings.

Refer to [tools/install/install.bzl](../../../tools/install/install.bzl) for a
thorough understanding.

## How to Run Release Build for Apollo

To generate binary outputs for Apollo, simply run:

```bash
./apollo.sh release -c
```

where `-c` is an optional argument used for pre-cleaning. The output was located
at `/apollo/output` inside Apollo Dev container.

The command above is roughly equivalent to the following:

```bash
bazel run --config=opt --config=gpu //:install \
        -- --pre_clean /apollo/output
```

Please type `./apollo.sh release -h` for more usage of `apollo.sh release`.

## How to Run Apollo with Release Build Output

In order to run Apollo Runtime Docker with release build output, type the
following command from the root of release build output directory outside Apollo
Dev Docker:

```bash
bash docker/scripts/runtime_start.sh
```

For users in China, use `-g cn` to speed up pulling of Docker images/volumes.

```bash
bash docker/scripts/runtime_start.sh -g cn
```

Log into Apollo Runtime Docker by running:

```bash
bash docker/scripts/runtime_into.sh
```

Start Dreamview by running:

```bash
./scripts/bootstrap.sh
```

from inside Apollo Runtime container.

## How to Use `install` Rule to Install a Custom Module

To _install_ a custom module, you can follow examples for installing other
modules from the Apollo repository.

Take the Planning module as an example.

This is part of the topmost [BUILD](../../BUILD) file:

```python
install(
    name = "install",
    deps = [
        "//cyber:install",
        # ...
        "//modules/planning:install",
        # ...
    ],
)
```

This is a snippet from [modules/planning/BUILD](../../../modules/planning/BUILD):

```python
filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libplanning_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)
```

## Arguments of the `install` Bazel Rule

From [tools/install/install.bzl](../../../tools/install/install.bzl):

```python
install = rule(
    attrs = {
        "deps": attr.label_list(providers = [InstallInfo]),
        "data": attr.label_list(allow_files = True),
        "data_dest": attr.string(default = "@PACKAGE@"),
        "data_strip_prefix": attr.string_list(),
        "targets": attr.label_list(),
        "library_dest": attr.string(default = "@PACKAGE@"),
        "library_strip_prefix": attr.string_list(),
        "mangled_library_dest": attr.string(default = "lib"),
        "mangled_library_strip_prefix": attr.string_list(),
        "runtime_dest": attr.string(default = "bin"),
        "runtime_strip_prefix": attr.string_list(),
        "rename": attr.string_dict(),
        "install_script_template": attr.label(
            allow_files = True,
            executable = True,
            cfg = "target",
            default = Label("//tools/install:install.py.in"),
        ),
    },
    executable = True,
    implementation = _install_impl,
)
```

The detailed attributes of the `install` rule was listed below:

| Argument             | Notes                                                                                    |
| -------------------- | ---------------------------------------------------------------------------------------- |
| deps                 | List of other install rules that this rule should include.                               |
| data                 | List of (platform-independent) resource files to install                                 |
| data_dest            | Destination for resource files (default = "@PACKAGE@")                                   |
| data_strip_prefix    | List of prefixes to remove from resource paths                                           |
| targets              | List of targets to install                                                               |
| runtime_dest         | Destination for executable targets (default = "bin")                                     |
| runtime_strip_prefix | List of prefixes to remove from executable paths                                         |
| rename               | Mapping of install paths to alternate file names, used to rename files upon installation |

## Limitations of Current Release Build Implementation

- C++ support only. Installation for Python was not ready.
- x86_64 support only. ARM64 support was not ready at the moment.
