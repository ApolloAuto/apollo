# Installing the Apollo Kernel
## Helpful hints before you begin
* The Apollo runtime environment in the vehicle requires the Apollo Kernel. You are strongly recommended to install the pre-built kernel.

## Installing Apollo Kernel

1.  Download the release packages from the release section on GitHub:
https://github.com/ApolloAuto/apollo-kernel/releases
2.  Install the kernel after having the release package downloaded:
    * ```tar zxvf linux-4.4.32-apollo-1.5.0.tar.gz```
    * ```cd install```
    * ```sudo bash install_kernel.sh```
3.  Reboot your system using the reboot command
4.  After rebooting, build the ESD CAN driver source code. Resources to build the ESD CAN driver source code can be found here: [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)
