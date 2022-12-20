# Apollo 1.0 Quick Start Guide for Developers

## Contents


* [About This Guide](#about-this-guide)
* [Introduction](#introduction)
* [Build the Apollo Kernel](#build-the-apollo-kernel)
* [Access the Apollo Dev Container](#access-the-apollo-dev-container)
* [Build the Apollo ROS](#build-the-apollo-ros)
* [Build Apollo](#build-apollo)
* [Release](#release)



# About This Guide

*The Apollo 1.0 Quick Start for Developers* provides the basic instructions to all Developers who want to build the Apollo Kernel, the Robot Operating System (ROS), and Apollo.


## Document Conventions

The following table lists the conventions that are used in this document:

| **Icon**                            | **Description**                          |
| ----------------------------------- | ---------------------------------------- |
| **Bold**                            | Emphasis                                 |
| `Mono-space font`                   | Code, typed data                         |
| _Italic_                            | Titles of documents, sections, and headingsTerms used |
| ![info](images/info_icon.png)       | **Info**  Contains information that might be useful.  Ignoring the Info icon has no negative consequences. |
| ![tip](images/tip_icon.png)         | **Tip**. Includes helpful hints or a shortcut that might assist you in completing a task. |
| ![online](images/online_icon.png)   | **Online**. Provides a link to a particular web site where you can get more information. |
| ![warning](images/warning_icon.png) | **Warning**. Contains information that must **not** be ignored or you risk failure when you perform a certain task or step. |

# Introduction

It is assumed that you have read and performed the instructions in the companion guide, the *Apollo 1.0 Quick Start Guide*, to set up the basic environment. Use this guide to build your own version of the Apollo Kernel, the ROS, and Apollo. There are also instructions on how to release your own Apollo container to others who might want to build on what you have developed.  It is strongly recommended that you build all of the components (Apollo Kernel, ROS, and Apollo) in the Apollo pre-specified dev Docker container.

# Build the Apollo Kernel

The Apollo runtime in the vehicle requires the [Apollo Kernel](https://github.com/ApolloAuto/apollo-kernel). You are strongly recommended to install the pre-built kernel.

## Use pre-built Apollo Kernel.

You get access and install the pre-built kernel with the following commands.

1. Download the release packages from the release section on github
```
https://github.com/ApolloAuto/apollo-kernel/releases
```
2. Install the kernel
After having the release package downloaded:
```
tar zxvf linux-4.4.32-apollo-1.0.0.tar.gz
cd install
sudo ./install_kernel.sh
```
3. Reboot your system by the `reboot` command
4. Build the ESD CAN driver source code
Now you need to build the ESD CAN driver source code according to [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)

## Build your own kernel.
If have modified the kernel, or the pre-built kernel is not the best for your platform, you can build your own kernel with the following steps.

1. Clone the code from repository
```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```
2. Build the ESD CAN driver source code according to [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)

3. Build the kernel with the following command.
```
bash build.sh
```
4. Install the kernel the same way as using a pre-built Apollo Kernel.

# Access the Apollo Dev Container

1. Please follow the [Quick Start Guide](../02_Quick%20Start/apollo_1_0_quick_start.md) to clone the Apollo source code.

![tip](images/tip_icon.png) In the following sections, it is assumed that the Apollo directory is located in  `$APOLLO_HOME`.

2. Apollo provides a build environment Docker image: `dev-latest`.  Run the following command to start a container with the build image:
```
cd $APOLLO_HOME
bash docker/scripts/dev_start.sh
```

3. Run the following command to log into the container:
```
bash docker/scripts/dev_into.sh
```
In the container, the default directory lives in `/apollo` , which contains the mounted source code repo.


# Build the Apollo ROS

Check out Apollo ROS from [github source](https://github.com/ApolloAuto/apollo-platform):

```bash
git clone https://github.com/ApolloAuto/apollo-platform.git apollo-platform
cd apollo-platform/ros
bash build.sh build
```

# Build Apollo

Before your proceed to build, please obtain ESD CAN library according to instructions in [ESD CAN README](https://github.com/ApolloAuto/apollo/blob/master/third_party/can_card_library/esd_can/README.md).

Run the following command:

```bash
cd $APOLLO_HOME
bash apollo.sh build
```

# Release

Apollo uses Docker images to release packages. **For advanced developers**: you can generate a new Docker image to test in an actual vehicle. Apollo has set up a base Docker image environment to test the Apollo build.

The image is called: `run-env-latest`.

1. Run the following command:

```
bash apollo.sh release
```

The release command generates a release directory, which contains:

- ROS environment
- Running scripts
- Binaries
- Dependent shared libraries (`.so` files)

2. Open a new terminal and run the following command in an Apollo source directory outside of Docker:


```
cd $APOLLO_HOME
bash apollo_docker.sh gen
```

The command creates a new Docker image using the release directory.

The release image tag will be named:`release-yyyymmdd_hhmm`. The existing release image tag, `release-latest`, is always pointing to the most current release.

3. Push your release image online using your own Docker registry set up from outside of the container:


```
cd $APOLLO_HOME
bash apollo_docker.sh push
```

The command pushes the newly built Docker image to the release Docker registry. To set up a new Docker registry, see [this page](https://docs.docker.com/registry).
