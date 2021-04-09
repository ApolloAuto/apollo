# 安装Apollo内核
## 开始工作前的帮助性信息
* Apollo在车辆的运行时环境依赖于Apollo内核。我们强烈建议使用者安装预先编译的内核版本。

## 安装内核

1.  从[Github](https://github.com/ApolloAuto/apollo-kernel/releases)上下载发布版本包

2.  下载成功后安装内核：
    * ```tar zxvf linux-4.4.32-apollo-1.5.0.tar.gz```
    * ```cd install```
    * ```sudo bash install_kernel.sh```
3.  使用 ```reboot``` 指令重启系统
4.  重启后，编译ESD CAN驱动。编译ESD CAN驱动的源代码可以在[ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)文件中获得。
