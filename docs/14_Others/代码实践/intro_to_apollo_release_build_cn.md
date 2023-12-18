# Apollo Release Build 简介

## 背景

基于 CyberRT 的 Apollo 在相当长的时间内一直没有二进制发布版本。用户需要先在
Apollo 开发容器内自行完成对整个项目的编译构建才能运行 Apollo 中的模块和工具。这
种部署上的不足，在若干情形下对用户相当不便。SVL 模拟器的开发者发现，将 Apollo
中 Docker 镜像、Docker 卷及 Bazel 缓存和构建查出加起来，足有 40 多 GB！

## Release Build 的实现原理

这种部署上的不足的根本原因，在于 Bazel 缺少其他构建系统通常具备的开箱即用的「安
装」支持，如`make install`.

为解决这一问题，我们借鉴了[Drake](https://github.com/RobotLocomotion/drake) 项目
中的「安装」实现，，利用 Starlark 语言，实现了 适用于 Apollo 的 Bazel「安装」扩
展，支持 Apollo 中二进制程序、共享库、资源文件（配置、数据、DAG 文件等）以及文档
的安装。

单独完备的二进制程序的安装是简单的。然而，CyberRT 框架的核心概念即为将每个模块（
如感知、预测、规划）作为组件，以共享库的形式（`libX_component.so`）动态加载。在
目前的 Bazel 构建下，`mainboard`二进制程序和`libX_component.so`链接了成千上百各
其他共享库对象。如，对规划模块运行如下`ldd`命令：

```bash
ldd bazel-bin/modules/planning/libplanning_component.so
```

会输出如下消息：

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

如何实现对`libplanning_component.so` 及其链接的所有共享库（后缀为".so"）文件的「
安装」，成为实现`install`规则中最难的部分。

幸好有`patchelf`。利用 Bazel 中`runfiles_data`的概念来确定出链接的所有共享库文件
，再利用`patchelf --force-rpath --set-rpath` 来修改其 RPATH 设置。

欲要更深入了解，请参考：
[tools/install/install.bzl](../../../tools/install/install.bzl)。

## 如何执行 Release Build 构建

可运行如下命令以生成二进制发布构建产物：

```bash
./apollo.sh release -c
```

其中，`-c`为可选参数，用于清理先前构建的残留。产物位于`/apollo/output`目录。

上述命令略等价于如下 Bazel 命令：

```bash
bazel run --config=opt --config=gpu //:install \
        -- --pre_clean /apollo/output
```

可输入`./apollo.sh release -h` 查看`apollo.sh release`子命令的更多用法。

## 通过二进制发布构建产物运行 Apollo

在二进制发布产物根目录下，运行如下命令以启动 Apollo Runtime Docker 镜像：

```bash
bash docker/scripts/runtime_start.sh
```

国内用户可使用`-g cn`选项来加速 Docker 镜像的拉取。

```bash
bash docker/scripts/runtime_start.sh -g cn
```

运行如下命令以进入 Apollo Runtime Docker 环境：

```bash
bash docker/scripts/runtime_into.sh
```

启动 Dreaview：

```bash
./scripts/bootstrap.sh
```

## 如何将`install`规则应用到任一自定义模块

欲实现自定义模块的*安装*，可参考 Apollo 代码中其他模块的示例，还是以规划模块为例
：

这是最上层的[BUILD](../../BUILD) 文件的一部分：

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

这是规划模块自身的 BUILD 文件
[modules/planning/BUILD](../../../modules/planning/BUILD):

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

## `install`规则的参数列表

`install`规则定义在
[tools/install/install.bzl](../../../tools/install/install.bzl):

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

其具体参数列举如下

| 参数                 | 含义                                      |
| -------------------- | ----------------------------------------- |
| deps                 | 本规则依赖的其它安装规则                  |
| data                 | 待安装的资源文件（平台无关）列表          |
| data_dest            | 资源文件目标安装地址                      |
| data_strip_prefix    | 需去掉的资源文件路径前缀列表              |
| targets              | 待安装目标                                |
| runtime_dest         | 可执行目标的目标安装地址，默认为 bin 目录 |
| runtime_strip_prefix | 需去掉可执行目标路径的前缀                |
| rename               | 安装时的文件重命名                        |

## 局限性

当前的 Release Build 实现

- 只支持 C++，不支持 Python。
- 只支持 x86_64 架构，Aarch64 支持尚待完善。
