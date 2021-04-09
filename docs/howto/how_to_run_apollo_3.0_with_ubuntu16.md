# Introduction

The Ubuntu version recommended by Apollo to use is 14.04. This document decribes steps to run Apollo on Ubuntu 16.04, and the problems that might occur and how to resolve these problems.

The Apollo version used in this document is the lastest release version which is 3.0. And this document focuses on How to Install Software, and conforms to the steps and rules provided by Apollo.

All the tests decribed in this document are based on X86 architecture.

# Download Apollo

please refer to [Apollo Software Installation](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md)，download Apollo version 3.0 source code onto the computer.

# Install Docker

Please follow the
[official guide to install the docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu).

Don't forget the
[post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).

We recommend to install using Docker repository, but some errors which are shown below may occur if we perform the installation from a package.

### Install from a package

Download `Docker` installation package from [Docker installation package website](https://download.docker.com/linux/ubuntu/dists/), the `Docker` version we recommend is 18.03.1。 There are various versions within directory `pool/stable/`.

Install `Docker`：

```sudo dpkg –i docker_packagename.deb```

An error may occur:
```
dpkg: dependency problems prevent configuration of docker-ce:
docker-ce depends on libsystemd-journal0 (>= 201); however:
  Package libsystemd-journal0 is not installed.
  ```
  
This error indicates that `Docker-ce` depends on `libsystemd-journal0`，but `libsystemd-journal0` has not been installed and could not be found in current repositories.
  
Please refer to [Website](https://ubuntu.pkgs.org/14.04/ubuntu-main-amd64/libsystemd-journal0_204-5ubuntu20_amd64.deb.html) to download `libsystemd-journal0`.

Install `libsystemd-journal0` after downloading:

```sudo dpkg –i libsystemd-journal0_packagename.deb ```

An error may occur:
```
dpkg: dependency problems prevent configuration of libsystemd-journal0:amd64:
 libsystemd-journal0:amd64 depends on libgcrypt11 (>= 1.5.1); however:
  Package libgcrypt11 is not installed.
  ```
This error indicates that `libsystemd-journal0` depends on `libgcrypt11`，but `libgcrypt11` has not been installed and could not be found in current repositories.
Please refer to [Website](https://ubuntu.pkgs.org/14.04/ubuntu-main-amd64/libgcrypt11_1.5.3-2ubuntu4_amd64.deb.html) to download `libgcrypt11`.

Install `libgcrypt11` after downloading:

```sudo dpkg –i libgcrypt11_packagename.deb ```

And then install `libsystemd-journal0` and `Docker-ce`.

After installing successfully, perform command ```docker --version```, and make sure the output is:
```
Docker version 18.03.1-ce, build 9ee9f40
```

### Verify the installation
Follow the steps provided by Apollo, perform command:

```sudo docker run hello-world	```

An error may occur:
```
docker: Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?.
See 'docker run --help'.
```
This error indicates that service `Docker` is not running.

Start service `Docker`:

```sudo service docker restart```

And then perform command ```sudo docker run hello-world``` to verify again.

# Build Apollo

Please refer to [How to Build and Release your Docker Container](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)。

# Launch and Run Apollo

Please refer to [How to Launch and Run Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_launch_Apollo.md)。
