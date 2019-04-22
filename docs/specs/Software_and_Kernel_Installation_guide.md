# Software and Kernel Installation Guide

This section describes the steps to install the following:

- Ubuntu Linux
- Apollo Kernel
- Nvidia GPU Driver

![tip_icon](images/tip_icon.png)It is assumed that you have experience working with Linux to successfully perform the software installation.

## Installing Ubuntu Linux

Follow these steps:

1. Create a bootable Ubuntu Linux USB flash drive:

   Download Ubuntu 14.04 (or a variant such as Xubuntu) [create a bootable USB flash drive](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#0). It is recommended that you use Ubuntu 14.04. You can type F2 during the system boot process to enter the BIOS settings. It is recommended that you disable Quick Boot and Quiet Boot in the BIOS to make it easier to catch any issues in the boot process.

2. Install Ubuntu Linux:

   a.   Insert the Ubuntu installation drive into a USB port and turn on the system.
   b.   Install Linux by following the on-screen instructions.

3. Perform a software update:
   a.   Reboot into Linux after the installation is done.
   b.   Launch the Software Updater to update to the latest software packages (for the installed distribution) or type the following commands in a terminal program such as GNOME Terminal.

   ```shell
   sudo apt-get update; sudo apt-get upgrade
   ```

   c. Launch a terminal program such as GNOME Terminal and type the following command to install the Linux 4.4 kernel:

   ```shell
   sudo apt-get install linux-generic-lts-xenial
   ```

![tip_icon](images/tip_icon.png)The IPC must have Internet access to update and install software. Make sure that the Ethernet cable is connected to a network with Internet access. You might need to configure the network for the IPC if the network that it is connected to is not using the Dynamic Host Configuration Protocol (DHCP).

## Installing the Apollo Kernel

The Apollo runtime in the vehicle requires the [Apollo Kernel](https://github.com/ApolloAuto/apollo-kernel). It is strongly recommended to install the pre-built kernel.

### Use the pre-built Apollo Kernel.

You get access to and install the pre-built kernel using the following commands.

1. Download the release packages from the release section on GitHub:

```
https://github.com/ApolloAuto/apollo-kernel/releases
```

2. Install the kernel after having downloaded the release package:

```
tar zxvf linux-4.4.32-apollo-1.5.0.tar.gz
cd install
sudo bash install_kernel.sh
```

3. Reboot your system using the `reboot` command.
4. [Optional - if you are using CAN card for interface] Build the CAN driver source code, according to the vendor's instructions

### Build your own kernel.

If you have modified the kernel, or the pre-built kernel is not the best for your platform, you can build your own kernel using the following steps:

1. Clone the code from the repository

```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```

2. Add the CAN driver source code according to vendor's instruction
3. Build the kernel using the following command:

```
bash build.sh
```

4. Install the kernel using the steps for a pre-built Apollo Kernel as described in the previous section.

## Installing NVIDIA GPU Driver

The Apollo runtime in the vehicle requires the [NVIDIA GPU Driver](http://www.nvidia.com/download/driverResults.aspx/114708/en-us). You must install the NVIDIA GPU driver with specific options.

1. Download the installation files

```
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
```

2. Start the driver installation

```
sudo bash ./NVIDIA-Linux-x86_64-375.39.run --no-x-check -a -s --no-kernel-module
```

## References

1. Ubuntu official [website](https://www.ubuntu.com/desktop)