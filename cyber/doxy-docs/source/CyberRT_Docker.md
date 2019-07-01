Develop inside Docker Environment
==================================

To make life easier, Apollo Cyber RT has released a docker image and a number of scripts to help developers to build and play with Cyber RT.

The official Cyber RT docker image is built based on Ubuntu 18.04. It comes with the full support for building Cyber RT and the drivers on top of it. So if you are interested in Cyber RT only, that would be the ideal point to start with.

**Note: ARM platform support has added recently fully integrated with Apollo development environment. You will be able to develop Cyber RT on both x86 and ARM platform with the same set of scripts. However, since ARM platfrom has only been verified on Nvdia Drive PX, so if you are trying on some dev board other than Drive PX. Please let us know if you run into any issues**

The following sections will show to how to start and play with Cyber RT docker and also how to build your own docker image from scratch.

## Build and Test with Cyber RT in Docker

To start the official Cyber RT docker, you need to run the command below first:

**Note**: Running this command for the first time could take a while because you will be downloading the full docker image, depending on your network bandwidth.

**Note**:  You will lose all your previous changes in the docker if you have ran this command before. Unless you would like to start a fresh docker environment.

```bash
./docker/scripts/cyber_start.sh
```

To enter the docker you just started:

**Note**: you can enter and exit multiple times whenever you like, the docker environment will stay there until the next time you start the docker again.

```bash
./docker/scripts/cyber_into.sh
```

To build Cyber RT only and test it:


```bash
./apollo.sh build_cyber
bazel test cyber/...
```
You should be able to see all the tests passed before developing your project.

To build drivers on Cyber RT only:

```bash
./apollo.sh build_drivers
```


**Note: start of instructions for ARM platform only**

Due to some limitation of docker on Drive PX platform, you need to follow the steps below on top of the procedure above.

For the first time after running cyber_into.sh to get into the Cyber RT container, please run the following two commands:

```bash
/apollo/scripts/docker_adduser.sh
su nvidia
```

**To exit, please use 'ctrl+p ctrl+q' instead of 'exit'**. Otherwise, you will lose your current running container.

**Note: end instructions for ARM platform only**

## Build Cyber RT Docker Image

To build your owner docker image for Cyber RT, please run the following commands on x86 platform:

```bash
cd docker/build/
./build_cyber.sh cyber.x86_64.dockerfile
```

For ARM platform,

```bash
cd docker/build/
./build_cyber.sh cyber.aarch64.dockerfile
```

To save you some time due to the performance on ARM platform, you can add the following option to download some prebuilt packages.

```bash
cd docker/build/
./build_cyber.sh cyber.aarch64.dockerfile download
```

If you would like to save your docker image for long term, use the following commands
as example to save it in your own docker registry, please use "dockder images" to find the name of your own image.

```bash
docker push [YOUR_OWN_DOCKER_REGISTRY]:cyber-x86_64-18.04-20190531_1115
```
