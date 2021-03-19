# 如何准备 Bazel 的依赖缓存目录

本文档描述了在 Apollo 项目中准备 Bazel 的依赖缓存目录
([Distribution Directory](https://docs.bazel.build/versions/master/guide.html#distribution-files-directories))
的方法。

## Introduction

根
据[Bazel 官方指南：在封闭环境中运行 Bazel](https://docs.bazel.build/versions/master/guide.html#running-bazel-in-an-airgapped-environment)
的说明，Bazel 的隐式依赖项是在 Bazel 初次运行的时候从网络上拉取的。然而，即使所
有在 WORKSPACE 中指定的依赖项均已到位，这在封闭环境或者网络连接不稳定的状况下依
然会造成问题。

为便利国内开发者，对 Apollo 中使用到的各 Bazel 版本，Apollo 均提供**与之对应**的
Bazel 隐式依赖压缩包文件。请注意，对不同版本的 Bazel 来说，其隐式依赖项不尽相同
。当 Bazel 版本更新后，请务必按照本文档描述的方法，**重新**执行一遍。

接下来，我们先熟悉把 Apollo 提供的 Bazel 隐式依赖压缩包解压到依赖缓存目录的流程
。

## 如何利用 Apollo 提供的 Bazel 隐式依赖压缩包

1. 在 Apollo 容器中执行`bazel version`确定 Bazel 版本：

```bash
$ bazel version
Build label: 3.5.0
Build target: bazel-out/aarch64-opt/bin/src/main/java/com/google/devtools/build/lib/bazel/BazelServer_deploy.jar
Build time: Wed Sep 2 21:11:43 2020 (1599081103)
Build timestamp: 1599081103
Build timestamp as int: 1599081103
```

在本示例中的 Bazel 版本为`3.5.0`。在以下行文中，我们将以`BAZEL_VERSION`指代之。

2. 运行如下命令以下载对应的 Bazel 隐式依赖压缩包

```bash
wget https://apollo-system.cdn.bcebos.com/archive/bazel_deps/bazel-dependencies-${BAZEL_VERSION}.tar.gz
```

3. 将压缩包解压到由环境变量`${APOLLO_BAZEL_DIST_DIR}`指代的 Bazel 依赖缓存目录

```bash
tar xzf bazel-dependencies-${BAZEL_VERSION}.tar.gz
source ${APOLLO_ROOT_DIR}/cyber/setup.bash
mv bazel-dependencies-${BAZEL_VERSION}/* "${APOLLO_BAZEL_DIST_DIR}"
```

## 自己动手构建 Bazel 的隐式依赖项的方法

如果您需要的 Bazel 版本 Apollo 未能提供，除了
在[GitHub Issues 页](https://github.com/ApolloAuto/apollo/issues) 提交 Issue 外
，您还可以通过如下方法在一台网络连接良好的机器上自己动手构建：

```bash
# 克隆对应分支的Bazel源码
git clone --depth=1 -b "${BAZEL_VERSION}" https://github.com/bazelbuild/bazel bazel.git

cd bazel.git

# 构建对应的Bazel隐式依赖压缩文件
bazel build @additional_distfiles//:archives.tar

# 确保${APOLLO_BAZEL_DIST_DIR} 变量已定义，且对应的目录存在
source ${APOLLO_ROOT_DIR}/cyber/setup.bash
[[ -d "${APOLLO_BAZEL_DIST_DIR}" ]] || mkdir -p "${APOLLO_BAZEL_DIST_DIR}"

# 将与该Bazel版本对应的各隐式依赖项全部解压到Bazel依赖缓存目录中
tar xvf bazel-bin/external/additional_distfiles/archives.tar \
  -C "${APOLLO_BAZEL_DIST_DIR}" --strip-components=3
```

然后，你就可以在 Apollo 容器中顺利执行`./apollo.sh build`，`./apollo.sh test` 等
命令而无需担心依赖项拉不下来的问题了。
