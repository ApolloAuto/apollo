# How to Prepare Bazel Distribution Directory

This document describes the steps to prepare Bazel distribution directory (See
[Ref](https://docs.bazel.build/versions/master/guide.html#distribution-files-directories).)
for Apollo.

## Introduction

According to
[Bazel Docs: Running Bazel in an airgapped environment](https://docs.bazel.build/versions/master/guide.html#running-bazel-in-an-airgapped-environment),
Bazel’s implicit dependencies are fetched over the network while running for the
first time. However, this may cause problems when running Bazel in an airgapped
environment or with unstable Internet access, even if you have vendored all of
your WORKSPACE dependencies.

To solve that, the Apollo team decided to provide these dependencies for every
Bazel binary version used in Apollo. Please do remember to do this once for
every new Bazel binary version, since the implicit dependencies can be different
for every Bazel release.

The section below describes the steps to prepare distribution directory using
Apollo-provided tarballs.

## Prepare Distribution Directory using Apollo-provided tarballs

1. Check Bazel version used by Apollo by running `bazel version` from inside
   Apollo container.

```bash
$ bazel version
Build label: 3.5.0
Build target: bazel-out/aarch64-opt/bin/src/main/java/com/google/devtools/build/lib/bazel/BazelServer_deploy.jar
Build time: Wed Sep 2 21:11:43 2020 (1599081103)
Build timestamp: 1599081103
Build timestamp as int: 1599081103
```

Here the Bazel version used is `3.5.0`. In the sections below, we will refer to
it as `BAZEL_VERSION`.

2. Download corresponding bazel-dependencies tarball by running:

```bash
wget https://apollo-system.cdn.bcebos.com/archive/bazel_deps/bazel-dependencies-${BAZEL_VERSION}.tar.gz
```

3. Uncompress the tarball and move the dependencies into the distribution
   directory defined by the `${APOLLO_BAZEL_DIST_DIR}` environment variable:

```bash
tar xzf bazel-dependencies-${BAZEL_VERSION}.tar.gz
source ${APOLLO_ROOT_DIR}/cyber/setup.bash
mv bazel-dependencies-${BAZEL_VERSION}/* "${APOLLO_BAZEL_DIST_DIR}"
```

In case that bazel-dependencies for the Bazel version you used were not provided
by the Apollo team, you can build it on your own following the recipe below.

## Recipe to build Bazel's implicit dependencies

The following is the recipe to produce Bazel's implicit dependencies:

```bash
# Checkout Bazel-${BAZEL_VERSION} branch
git clone --depth=1 -b "${BAZEL_VERSION}" https://github.com/bazelbuild/bazel bazel.git

cd bazel.git

# Build Bazel's implicit dependencies for that Bazel version
bazel build @additional_distfiles//:archives.tar

# Make sure ${APOLLO_BAZEL_DIST_DIR} is defined
source ${APOLLO_ROOT_DIR}/cyber/setup.bash

# Uncompress the tarball to the distribution directory for Apollo
tar xvf bazel-bin/external/additional_distfiles/archives.tar \
  -C "${APOLLO_BAZEL_DIST_DIR}" --strip-components=3
```
