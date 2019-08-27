Apollo软件安装
===================

## 目录
      
 - [概览](#概览)
 - [工控机系统安装](#工控机系统安装)
     - [工控机软件系统安装](#工控机软件系统安装)

## 概览

该用户手册旨在帮助用户在教学小车上安装、配置apollo软件。

## 工控机系统安装

在集成车辆之前，首先需要完成工控机的硬件安装，如CAN卡安装；之后需要完成工控机的软件安装，包括Ubuntu Linux安装、Apollo软件系统安装等。

### 工控机软件系统安装

工控机软件系统安装包括计算机操作系统的安装，硬件驱动的安装，应用软件的安装和Apollo软件系统的安装。

#### 安装Ubuntu Linux
Apollo软件系统依赖于Linux操作系统而运行，而Linux操作系统种类繁多，且又分为服务器版本和桌面版本，这里我们选择当下比较流行的Ubuntu桌面操作系统的64位版本。安装Ubuntu Linux的操作系统的步骤如下：
##### 创建引导盘
创建一个可以引导启动的Ubuntu Linux USB闪存驱动器，下载Ubuntu，并按照在线说明创建可引导启动的USB闪存驱动器。

![tip_icon](../images/tip_icon.png) 推荐使用 **Ubuntu 14.04.3**.

![tip_icon](../images/tip_icon.png)开机按F2进入BIOS设置菜单，建议禁用BIOS中的快速启动和静默启动，以便捕捉引导启动过程中的问题。建议您在BIOS中禁用“快速启动”和“安静启动”，以便了解启动过程中遇到的问题。

获取更多Ubuntu信息，可访问: 
![online_icon](../images/link.png)Ubuntu桌面站点:

[https://www.ubuntu.com/desktop](https://www.ubuntu.com/desktop)

##### 安装 Ubuntu Linux

a.将Ubuntu安装驱动器插入USB端口并启动IPC。

b.按照屏幕上的说明安装Linux。

##### 执行软件更新与安装

a.安装完成，重启进入Linux。

b.在终端执行以下命令完成最新软件包的更新：

```
sudo apt-get update
```
c.打开终端，输入以下命令，安装Linux4.4内核：

```
sudo apt-get install linux-generic-lts-xenial
```
![tip_icon](../images/tip_icon.png)IPC必须接入网络以便更新与安装软件，所以请确认网线插入并连接，如果连接网络没有使用动态分配（DHCP），需要更改网络配置。

#### 安装Apollo内核

车上运行Apollo需要[Apollo Kernel](https://github.com/ApolloAuto/apollo-kernel)。你可以依照如下步骤获取、安装预编译的内核：

a.从release文件夹下载发布的包

```
https://github.com/ApolloAuto/apollo-kernel/releases
```
b.安装包下载完成后，解压后安装:

```
tar zxvf linux-4.4.32-apollo-1.5.5.tar.gz
cd install
sudo bash install_kernel.sh
```
c.使用`reboot`命令重新启动计算机。


#### 安装GPU驱动

a.下载Nvidia GPU驱动

可以在终端中输入以下命令进行下载：

```
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
```

下载成功后安装包会放在终端当前的目录中。

b.安装Nvidia GPU驱动

在终端运行以下命令进行安装：

```
sudo bash ./NVIDIA-Linux-x86_64-375.39.run --no-x-check -a -s --no-kernel-module
```

#### 安装Can驱动

a.从CAN卡供应商那里拿到CAN卡的驱动安装包esdcan-pcie4.2-linux-2.6.x-x86_64-3.10.3.tgz。

ｂ.解压此安装包，cd到解压后的文件夹里。

c.编译安装CAN卡驱动，在终端执行以下命令：
```
cd src/
make -C /lib/modules/`uname -r`/build M=`pwd`
sudo make -C /lib/modules/`uname -r`/build M=`pwd` modules_install
```
d.CAN卡驱动esdcan-pcie402.ko可以在/lib/modules/4.4.32-apollo-2-RT/extra/文件夹下找到。

#### 安装Docker

在docker的官网下载deb安装包，双击deb安装包，在Software Center进行安装后，重新启动计算机即可。

#### 编译Apollo源代码

![warning_icon](../images/warning_icon.png)**WARNING**：在本模块及以下的操作中，如非本文档或操作系统要求，禁用一切`sudo`操作，切记！

a.获取Apollo源代码
可以在github上下载，在终端中输入以下命令：
```
cd ~
git clone https://github.com/ApolloAuto/apollo.git
```
代码下载的时间视网速的快慢而有所区别，请耐心等待；
下载完成后的代码在～/apollo目录下，然后执行`git checkout -b r3.0.0 origin/r3.0.0`将代码切换到我们所需要的工作分支r3.0.0上。

b.设置环境变量，在终端输入以下命令：

```
echo "export APOLLO_HOME=$(pwd)" >> ~/.bashrc && source ~/.bashrc
source ~/.bashrc
```

c.启动并进入docker容器，在终端输入以下命令：

```
cd ~/apollo
bash docker/scripts/dev_start.sh -C
```

其中，参数`-C`是可选的，这是拉取中国的镜像，请根据自己的网络情况选择。第一次进入docker时或者image镜像有更新时会自动下载apollo所需的image镜像文件，下载镜像文件的过程会很长，请耐心等待；这个过程完成后，请输入以下命令以进入docker环境中：
```
bash docker/scripts/dev_into.sh
```

d.编译apollo，在终端输入以下命令，等待编译完成，整个编译过程大约耗时20分钟：

```
bash apollo.sh build
```

#### 运行DreamView

a.启动apollo
在终端输入以下命令：

```
bash scripts/bootstrap.sh
```

如果启动成功，在终端会输出以下信息：

```
Start roscore...
Launched module monitor.
Launched module dreamview.
Dreamview is running at http://localhost:8888
```

在浏览器中输入以下地址： 

```
http://localhost:8888
```

可以访问DreamView。

b.回放数据包
在终端输入以下命令下载数据包：

```
python docs/demo_guide/rosbag_helper.py demo_2.0.bag 
```

输入以下命令可以回放数据包，在浏览器DreamView中可以看到回放画面。

```
rosbag play -l docs/demo_guide/demo_2.0.bag
```