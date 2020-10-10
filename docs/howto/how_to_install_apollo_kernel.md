# How to Install Apollo Kernel

## Before you begin

The Apollo Kernel is **ONLY** required by Apollo runtime in the vehicle. If your
sole purpose was developing and testing on the Apollo platform, or running
simulator software (e.g., the LGSVL simulator), then you don't need to install
Apollo kernel at all.

## Installing the Apollo Kernel

- Download (latest) Apollo-kernel release from
  [Apollo-kernel Releases on GitHub](https://github.com/ApolloAuto/apollo-kernel/releases)

- Run the following commands to install the kernel after having the release
  package downloaded:

```bash
tar zxvf linux-4.4.32-apollo-1.5.5.tar.gz
cd install
sudo bash install_kernel.sh
```

- Reboot your system

- After rebooting, build the ESD CAN driver source code following the
  [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)
  guide.
