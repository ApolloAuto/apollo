# Apollo Environment Manager

## Summary

Apollo environment manager, aem is a command line tool, providing the ability to manage Apollo containers. With aem, there is no need to run Apollo scripts to start and enter the container, avoiding the problem of Apollo scripts polluting the code in the workspace.

## Installation

To install aem, you first need to add apt source of Apollo:

```shell
$ sudo -i
$ echo "deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main" >> /etc/apt/sources.list && wget -O - https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | apt-key add -
$ exit
```

Then, use apt to install aem:

```shell
$ sudo apt install apollo-neo-env-manager-dev
```

When the installation is completed, you can enter the following command to check whether buildtool has been installed correctly:

```shell
aem -h
```

If all goes well, you will see a prompt similar to the image below:

```shell
Usage:
    aem [OPTION]

Options:
    start : start a docker container with apollo development image.
    start_gpu : start a docker container with apollo gpu support development image.
    enter : enter into the apollo development container.
    stop : stop all apollo development container.
    install_core : install the core module of apollo.
    bootstrap : run dreamview and monitor module.
    build : build package in workspace.
    install : install package in workspace.
    init: init single workspace.
    update: update core modules of apollo.
```

## subcommand

Similar to common command-line tools such as **git** or **apt**, the functionality of **aem** is organized into subcommands, such as **start** responsible for starting a container, **enter** responsible for entering a started container, etc.

Some parameters may be required after the action, you can enter -h, --help after the action to view the detailed parameters.

## start, start_gpu

This subcommand start a Apollo env container. This command will check whether the apollo container has been started. If the container has been started or stopped, it will use the started container; and if the container has never been started, it will start a new container.

### usage

Start a normal container:

```shell
aem start
```

Start a gpu-enabled container where you can build or run modules that require gpu support:

```shell
aem start_gpu
```

> Note: NVIDIA graphics driver and nvidia toolkit must be installed correctly

Start a container using local image:

```shell
aem start_gpu -l
```

Force to start a new container:

```shell
aem start_gpu -f
```

> Note: this command will completely delete current container.

Give the container a name:

```shell
aem start_gpu -n apollo_container
```

Specify the mount point:

```shell
aem start_gpu -m /home/apollo/workspace:/apollo_workspace
```

### detailed parameters

```shell
OPTIONS:
    -h, --help             Display this help and exit.
    -f, --force            force to restart the container.
    -n, --name             specify container name to start a container.
    -m, --mount            specify the mount point in container, such as /home/apollo/workspace:/apollo_workspace
    -t, --tag <TAG>        Specify docker image with tag <TAG> to start.
    -y                     Agree to Apollo License Agreement non-interactively.
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
    --gpu                  Use gpu image instead of cpu image.
```

## enter

This subcommand enters an Apollo env container.

### usage

Enter a container:

```shell
aem enter
```

Specify the container name:

```shell
aem enter -n apollo_container
```

## bootstrap

This subcommand is used to boot Dreamview and monitor.

### usage

boot Dreamview and monitor in container.

```shell
aem bootstrap
```

stop Dreamview and monitor in container

```shell
aem bootstrap stop
```

restart Dreamview and monitor in container

```shell
aem bootstrap restart
```

## build

This subcommand is a simple wrapper of buildtool build action. For detailed usage, please refer to the buildtool documentation.

## install
This subcommand is a simple wrapper of buildtool install action. For detailed usage, please refer to the buildtool documentation.

## init
This subcommand is a simple wrapper of buildtool init action. For detailed usage, please refer to the buildtool documentation.

## update

This subcommand is used to update preinstalled buildtool and apollo-core modules in the image

### usage

```shell
aem update
```

