Build and Release
==========================

* [1. Install Docker](#docker)
* [2. Build and Release](#build_release)
* [3. Build in VSCode](#build_in_vscode)
* [4. Test](#test)

## <span id="docker">Install Docker</span>
The system requirement for building Apollo is Ubuntu 14.04. Docker container is the simplest way to set up the build environment for Apollo project. Detailed docker tutorial is [here](https://docs.docker.com/).

To install, you may refer to
[official guide to install the docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu).
Don't forget the
[post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).

## <span id="build_release">Build and Release</span>
### Start container
We provide a build image named *dev-latest*. Container will mount your local apollo repo to */apollo*.
```bash
bash docker/scripts/dev_start.sh
```
### Get into container
```bash
bash docker/scripts/dev_into.sh
```
### Build modules
```bash
bash apollo.sh build
```
### Release binaries
```bash
bash apollo.sh release
```
This will generate a release directory, which contains ROS environment, running scripts, binaries and dependent shared libraries (.so files).

To create a release excluding proprietary software (such as ESD CAN library), do:
```bash
bash apollo.sh release_noproprietary
```
### Generate release image
```bash
bash apollo_docker.sh gen
```
This will create a new docker image with the release directory. The release image will be named as *release-yyyymmdd_hhmm*. Meanwhile, your most recent built image will be tagged as *release-latest*. **The docker_release needed to be executed outside of container.**
### Push docker images
```bash
bash apollo_docker.sh push
```
The command will push your most recent release docker image to the docker hub.

## <span id="build_in_vscode">Build in Visual Studio Code</span>
### Install VSCode
The easiest way to install for Debian/Ubuntu based distributions is to download from  https://code.visualstudio.com and install the .deb package (64-bit) either through the graphical software center if it's available or through the command line with:
```bash
sudo dpkg -i <file>.deb
sudo apt-get install -f # Install dependencies
```
### Start VSCode
Start VSCode with the following command: 
```bash
code
```
### Open the Apollo project in VSCode
Use the keyboard shortcut (Ctrl+K Ctrl+O) to open the Apollo project. 
### Build the Apollo project in VSCode
Use the keyboard shortcut (Ctrl+Shift+B) to build the Apollo project. 
### Run all unit tests for the Apollo project in VSCode
Select the "Tasks->Run Tasks..." menu command and click "run all unit tests for the apollo project" from a popup menu to check the code style for the Apollo project. 
### Run a code style check task for the Apollo project in VSCode
Select the "Tasks->Run Tasks..." menu command and click "code style check for the apollo project" from a popup menu to check the code style for the Apollo project. 
### Clean the Apollo project in VSCode
Select the "Tasks->Run Tasks..." menu command and click "clean the apollo project" from a popup menu to clean the Apollo project. 
### Change the building option
 You can change the "build" option to another one such as "build_gpu" (refer to the "apollo.sh" file for details) in ".vscode/tasks.json"

## <span id="test">Test</span>
```bash
bash docker/scripts/release_start.sh [release tag]
```
The *HMI* will automatically start and you can control each apollo module through any web browser by inputting IP address and port number, such as *localhost:8887*. You can get into the release container if quick fix needed.
```bash
bash docker/scripts/release_into.sh
```

## Legal Disclaimer
The docker image that you build may contain ESD CAN library files provided by ESD Electronics (hereby referred as ESD), which you should have obtained via a licensing agreement with ESD. The licensing agreement shall have granted you (as an individual or a business entity) the right to use the said software provided by ESD; however, you may (and likely you do) need explicit re-distribution permission from ESD to publish the docker image for any other third party to consume. Such licensing agreement is solely between you and ESD, and is not covered by the license terms of the Apollo project (see file LICENSE under Apollo top directory).
