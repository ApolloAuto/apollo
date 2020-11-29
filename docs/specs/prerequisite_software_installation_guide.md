# Pre-requisite Software Installation Guide

This article describes all the pre-requisite steps needed before installing
Apollo.

- Installing Ubuntu Linux
- Installing NVIDIA GPU Driver
- Installing Docker Engine
- Installing NVIDIA Container Toolkit

![tip_icon](images/tip_icon.png) Working knowledge of Linux is assumed for
successful software installation in this guide.

## Installing Ubuntu Linux

Although other Linux distributions may be OK, we have only tested Apollo on
Ubuntu systems. To be specific,
[Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.5/). So
we would recommend using Ubuntu 18.04.5+ (including Ubuntu 20.04) as the host
system.

The steps required to install Ubuntu 18.04+ are available at
[Tutorial from ubuntu.com on How to Install Ubuntu](https://ubuntu.com/tutorials/install-ubuntu-desktop).
Please follow the guide there for a successful Ubuntu installation.

Don't forget to perform software updates after the installation is done:

```shell
sudo apt-get update
sudo apt-get upgrade
```

![tip_icon](images/tip_icon.png) Internet access is needed for successful
software updates. Make sure either WiFi or Ethernet cable is connected to a
network with Internet access. You might need to configure the network for your
host if the connected network is not using the Dynamic Host Configuration
Protocol (DHCP).

## Installing NVIDIA GPU Driver

The Apollo runtime in the vehicle requires NVIDIA GPU Driver.

According to
[How to Install NVIDIA Driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver),
the recommended way for Ubuntu is to use the `apt-get` commands, or use an
official "runfile" from
[www.nvidia.com/en-us/drivers/unix/](https://www.nvidia.com/en-us/drivers/unix/)

For Ubuntu 18.04+, you can simply run:

```
sudo apt-get update
sudo apt-add-repository multiverse
sudo apt-get update
sudo apt-get install nvidia-driver-440
```

You can type `nvidia-smi` to check if NVIDIA GPU works fine on your host. (You
may restart your host for the changes to take effect.) On success, the following
message will be shown.

```
Prompt> nvidia-smi
Thu Sep  3 16:01:19 2020
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 440.33.01    Driver Version: 440.33.01    CUDA Version: 10.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce GTX 1070    On   | 00000000:01:00.0  On |                  N/A |
| 26%   38C    P8    11W / 151W |    890MiB /  8114MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|    0      1290      G   /usr/lib/xorg/Xorg                            40MiB |
|    0      1426      G   /usr/bin/gnome-shell                          51MiB |
|    0      2411      G   /usr/lib/xorg/Xorg                           293MiB |
|    0      2571      G   /usr/bin/gnome-shell                         227MiB |
|    0      7071      G   ...AAAAAAAAAAAACAAAAAAAAAA= --shared-files   272MiB |
+-----------------------------------------------------------------------------+
```

## Installing Docker Engine

Apollo 6.0+ requires Docker 19.03+ to work properly. Just follow the
[Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
doc to install docker engine.

Docker-CE on Ubuntu can also be setup using Docker’s official convenience
script:

```
curl https://get.docker.com | sh
sudo systemctl start docker && sudo systemctl enable docker
```

You can choose whichever method you would prefer. Just don't forget the
[Post-installation Actions for Linux](https://docs.docker.com/engine/install/linux-postinstall/),
esp. the section on
[Manage Docker as Non-root User](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
and
[Configure Docker to Start on Boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot).

There is also a
[dedicated bash script](../../docker/setup_host/install_docker.sh)
Apollo provides to ease Docker installation, which works both for X86_64 and
AArch64 platforms.

## Installing NVIDIA Container Toolkit

The NVIDIA Container Toolkit for Docker is required to run Apollo's CUDA based
Docker images.

You can run the following to install NVIDIA Container Toolkit:

```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get -y update
sudo apt-get install -y nvidia-docker2
```

![tip_icon](images/tip_icon.png) Don't forget to restart the Docker daemon for
the changes above to take effect.

```
sudo systemctl restart docker
```

Refer to
[NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
for more.

## What's Next

With successful installation of the pre-requisites above, you can now move to
"Git Clone Apollo Repo" section now.
