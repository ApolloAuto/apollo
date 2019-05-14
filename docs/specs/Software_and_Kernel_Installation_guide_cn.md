# 工业级PC（IPC）软件安装指南

本文档介绍下述软件的安装步骤：

- Ubuntu Linux
- Apollo Kernel
- Nvidia GPU Driver

![tip_icon](images/tip_icon.png)成功完成本文档中介绍的软件安装需要使用者有使用Linux系统的经验。

## 安装Unbuntu Linux

按照如下步骤执行：

1. 创建一个可引导的Ubuntu Linux USB启动盘：

   下载Ubuntu 14.04（或其他的变种系统如Xubuntu）并[创建一个可引导的USB启动盘](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#0)。我们推荐使用Ubuntu 14.04。在系统启动时按下F2（或其他按键，请参考系统文档）进入BIOS设置，我们建议禁用Quick Boot和Quiet Boot设置以更容易的在启动时捕获错误信息。
   
2. 安装Ubuntu Linux：

   a.   将安装Unbuntu的USB启动盘插入USB接口中并启动系统
   
   b.   按照屏幕提示执行安装
   
3. 执行软件更新：

   a.   安装结束后重启并进入系统
   
   b.   启动Software Update并更新到最新软件包，或在终端程序如GNOME Terminal中执行下述指令完成更新：

   ```shell
   sudo apt-get update; sudo apt-get upgrade
   ```
   
   c. 启动终端程序如GNOME Terminal，执行下述指令安装Linux 4.4内核
   
   ```shell
   sudo apt-get install linux-generic-lts-xenial
   ```
   
   ![tip_icon](images/tip_icon.png)IPC必须有网络连接以更新和安装软件。确保IPC的以太网线接入了有互联网访问权限的网络。如果接入的网络没有使用动态主机配置协议（DHCP），使用者可能需要对IPC的网络进行配置。

## 安装Apollo内核

Apollo在车辆上的运行需要[Apollo内核](https://github.com/ApolloAuto/apollo-kernel)。我们强烈推荐安装预先构建的内核版本。

### 使用预先构建的内核版本

使用者使用下述指令获取和安装预先构建的内核版本。

1. 从GitHub下载发布版本包：

```
https://github.com/ApolloAuto/apollo-kernel/releases
```

2. 成功下载发布版本包后安装内核：

```
tar zxvf linux-4.4.32-apollo-1.5.0.tar.gz
cd install
sudo bash install_kernel.sh
```

3. 使用 `reboot` 指令重启系统
4. 【可选步骤-如果使用者使用了CAN卡】参考CAN卡供应商提供的指令构建CAN卡驱动程序

### 构建个人的内核版本

如果使用者修改了内核，或者预先构建的版本对使用者的工作平台不是最好的选择，使用者可以使用下述指令构建个人的内核版本：

1. 从资源库中clone源代码

```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```

2. 参考CAN卡供应商提供的指令加入CAN卡驱动的源代码
3. 使用下述指令构建内核：

```
bash build.sh
```

4. 参考上面章节中介绍的如何安装预先构建内核版本的步骤进行内核的安装

## 安装NVIDIA GPU驱动

Apollo在车辆上的运行需要[NVIDIA GPU驱动](http://www.nvidia.com/download/driverResults.aspx/114708/en-us)。使用者必须使用指定的参数选项安装NVIDIA GPU驱动。

1. 下载安装文件

```
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
```

2. 执行驱动安装

```
sudo bash ./NVIDIA-Linux-x86_64-375.39.run --no-x-check -a -s --no-kernel-module
```

## 参考资料

1. [Ubuntu官方网站](https://www.ubuntu.com/desktop)
