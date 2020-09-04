# Apollo Docker Image Build Process

## Introduction
As you may already know, Apollo was run inside Docker container, and there are two flavors of Apollo docker images,  `CyberRT` (`Cyber` for short) and `Dev` . `Cyber` images were for developers who want to play with the `CyberRT` framework only, while `Dev` images were used to build the whole Apollo project.

Currently, Apollo comes with support for two CPU architectures, namely, `x86_64` and `aarch64`.  (Please note that till the time this document was updated, the `dev-aarch64` image  was not complete. Hope we can make it ready in the next few months.)

In the next section, I will describe briefly the steps to build these Docker images.

## Build Apollo Cyber Image

There is a dedicated bash script for this purpose, `build_cyber.sh`.

Type `./build_cyber.sh -h` and it will print its usage:

```
$ ./build_cyber.sh -h
./build_cyber.sh -h
Usage:
    build_cyber.sh [-l] [-c] -f <cyber_dockerfile> [-m <build|download>] [-g <us|cn>]
    build_cyber.sh -h/--help    # Show this message
E.g.,
    build_cyber.sh -f cyber.x86_64.dockerfile -m build
    build_cyber.sh -l -f cyber.aarch64.dockerfile -m download

```

Here, `-g` stands for `geo`, an option for geolocation settings (E.g., APT/PYPI mirrors). At this moment, only two `geo` codes ( `us` & `cn`) are supported, and the default is `us`. Issues and PRs are welcome if you found your `geo` code missing.

To build the latest `Cyber` image, simply run

```
./build_cyber.sh -f cyber.<TARGET_ARCH>.dockerfile
```

For users in mainland China who want to speed up the build process:

```
./build_cyber.sh -f cyber.<TARGET_ARCH>.dockerfile -g cn
```

## Build Apollo Dev Image

The script used to build Develop image is `build_dev.sh`, and its usage is similar to that of `build_cyber.sh`.

```
$ ./build_dev.sh --help
Usage:
    build_dev.sh [-l] [-c] -f <dev_dockerfile> [-m <build|download>] [-g <us|cn>]
    build_dev.sh -h/--help    # Show this message
E.g.,
    build_dev.sh -f dev.x86_64.dockerfile -m build
    build_dev.sh -l -f dev.aarch64.dockerfile -m download
```

On success, output messages like the following will be shown at the end of your build.

```
Successfully built baca71e567e6
Successfully tagged apolloauto/apollo:dev-x86_64-18.04-20200824_0339
Built new image apolloauto/apollo:dev-x86_64-18.04-20200824_0339
```

> Note: Please don't forget to change the `FROM ...` line if you want to use the `Cyber` image freshly built.

## A Few More Words ...

### Build Log

Once completed, there exists a log file located at `/opt/apollo/build.log` inside docker container recording package download links for your build.

### Enable Local HTTP Cache to Speed Up Build

You can enable local HTTP cache to speed up package downloading phase by performing the following steps on your docker **host** (outside docker):

1. Download all prerequisite packages to a directory (say, `$HOME/archive`) with URLs listed in the build log. Pay attention to their checksum.
2. Change to that archive directory and start your local HTTP cache server at port **8388**.

```
cd $HOME/archive
nohup python3 -m http.server 8388 &
```

> Note: Another advantage with the local HTTP cache mechanism is, it has
> little influence on the final image size. Even if the cached package was
> missing or broken, it can still be downloaded from the original URL.

3. Rerun `build_cyber.sh` or `build_dev.sh`.

## Understand the Dependencies

To help users understand the dependencies, we are trying to compile a high
level [dependency graph](dependencies.dot). You can build your own Dockerfile or
ISO package according to it.

To view the graph, please run:

```bash
sudo apt -y install xdot
xdot dependencies.dot
```

## Add New Dependency

When you need to add something to the Docker image or Bazel `WORKSPACE` file, it's
defined as a new dependency. Before doing that, please add it as well as all its
dependencies to the `dependencies.dot` first, so we understand all the
dependencies and dependents, which helps us manage its lifecycle like upgrading
and retiring in the future.

## Add New Installer

The best practice of a new installer would be:

1. Well tested.

   Of course. Make it work, and don't break other installers, such as
   incompatible versions of libraries.

1. Standalone.

   Have minimum assumption about the basement, which means, you can depend on
   the base image and `installers/installer_base.sh`. Other than that, you should
   install all the dependencies in your own installer.

1. Thin.

   It will generate a new layer in the final image, so please keep it as thin as
   possible. For example, clean up all intermediate files:

   ```bash
   wget xxx.zip
   # Unzip, make, make install
   rm -fr xxx.zip xxx
   ```

1. Cross-architecture.

   It would be awesome to work perfectly for different architectures such as `x86_64` and `aarch64`.

