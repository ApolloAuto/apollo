# Installing Your Own Kernel

## Helpful hints before you begin
If you have modified the kernel, or the pre-built kernel is not the best for your platform, you can build your own kernel with the following steps:

## Installing your own kernel
1.  Clone the code from repository.
```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```

2.  Add the ESD CAN driver source code according to [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md).
3.  Build the kernel with the following command:
```bash build.sh```
4.  Install the kernel the same way as using a pre-built [Apollo Kernel](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_install_apollo_kernel.md).
