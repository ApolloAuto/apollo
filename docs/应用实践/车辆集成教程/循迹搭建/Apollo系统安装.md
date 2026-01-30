<!---操作指南使用对象：高级开发者。以装车测试为目的开发者。本文档不涉及源代码的修改和解读，只偏向于使用。-->

<!---本文档是模板-->

<!---模板的使用：段落的临近处都有写作指导，写之前先看指导。-->

<!---标题与段落，段落之间留出一行的空行。-->

<!--md文件的文件名为默认的一级标题-->

<!--整个文档开始之前，推荐给出一个图片描述之后要讲的内容的整体框架，例如该模块在整体系统中的哪个位置-->

<!---一级标题为文档名称，文档内以二级标题为最高级标题。-->

<!---还可以写应用场景、前置条件、注意事项、多个子任务的逻辑关系。-->

<!---所有标题应采用动宾结构，例如：创建仿真场景，下载分支库。-->

<!---当二级任务有子任务时，可以再继续写到三级标题任务。四级标题尽量少用。如内容过多可以考虑，另启一个文档-->

<!---所有的图片符合截图规范；可以多个步骤使用一张截图。-->

<!---不会使用链接时，可参见Markdown速查表，链接包括：页内链接，页间链接，其他产品链接，图片链接-->

<!---最后一步必须要给出能够查看的结果,直观的进行验收最终步骤。-->

<!--标题名称：准备符合 Apollo 协议的底盘 -->

<!--内容超过10页的情况下，推荐分成几个文件进行说明-->

<!--# Apollo软件集成教程-->

本文档以 Lincoln MKZ 作为目标车辆，介绍 Apollo 软件系统的集成安装，您可以了解到如何在车辆计算单元 Hardware3.0（HW3.0）上完成 Apollo 自动驾驶软件系统的集成安装和配置。本文档针对于 Apollo 6.0 及以上版本。

## 前提条件

在 MKZ 上进行软件集成安装之前，您需要先按照 [车辆集成](../应用实践/车辆集成教程/循迹搭建/车辆集成.md) 完成车辆硬件集成。

## 软件集成安装

请按照以下步骤进行软件集成安装。

### 1. 安装 Ubuntu 系统

请参考安装指南步骤安装 Ubuntu 系统。

### 2. 安装实时内核

您可以按照 [ROS2: 构建实时 Linux](https://docs.ros.org/en/foxy/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2.html) 构建一个实时内核，或者直接安装使用我们预构建的版本：

```bash
wget https://apollo-system.cdn.bcebos.com/release_installer/linux-kernel-5.4.115-apollo-rt.zip
unzip linux-kernel-5.4.115-apollo-rt.zip
cd linux-kernel-5.4.115-apollo-rt
sudo dpkg -i *.deb
```

### 3. 安装 CANBUS 驱动

1. 下载 [apollo-contrib](https://github.com/ApolloAuto/apollo-contrib/tree/master/baidu) 下的源码在本地编译，编译完生成 basa.ko 模块文件。

2. 设置系统上电自动加载驱动：

```bash
sudo su
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/baidu/basa/
cp basa.ko /lib/modules/$(uname -r)/kernel/drivers/baidu/basa/
echo "basa" >>/etc/modules
depmod -a
modprobe basa
```

### 4. 安装实时内核下的 Nvidia 驱动

请先参考安装指南步骤中的方法完成 Nvidia 驱动的初步安装。

注意：由于 Nvidia 驱动不支持实时内核，在上面执行 `sudo apt-get install nvidia-driver-455` 时会报如下错误消息：

```
The kernel you are installing for is a PREEMPT_RT kernel!

The NVIDIA driver does not support real-time kernels. If you
are using a stock distribution kernel, please install
a variant of this kernel that does not have the PREEMPT_RT
patch set applied; if this is a custom kernel, please
install a standard Linux kernel.  Then try installing the
NVIDIA kernel module again.

*** Failed PREEMPT_RT sanity check. Bailing out! ***
```

您可以通过在编译 Nvidia 驱动的时候设置 `IGNORE_PREEMPT_RT_PRESENCE=1` 来绕过这 一点。步骤如下：

1. 运行如下命令来编译 Nvidia 驱动：

```bash
# 切换到Nvidia 驱动的源码目录
cd "$(dpkg -L nvidia-kernel-source-455 | grep -m 1 "nvidia-drm" | xargs dirname)"

# 设置 IGNORE_PREEMPT_RT_PRESENCE=1 来编译Nvidia 驱动
sudo env NV_VERBOSE=1 \
    make -j8 NV_EXCLUDE_BUILD_MODULES='' \
    KERNEL_UNAME=$(uname -r) \
    IGNORE_XEN_PRESENCE=1 \
    IGNORE_CC_MISMATCH=1 \
    IGNORE_PREEMPT_RT_PRESENCE=1 \
    SYSSRC=/lib/modules/$(uname -r)/build \
    LD=/usr/bin/ld.bfd \
    modules

sudo mv *.ko /lib/modules/$(uname -r)/updates/dkms/
sudo depmod -a
```

2. 重启系统。

3. 运行 `nvidia-smi` 来检查 `Nvidia` 驱动是否正常工作。

### 5. 安装其他必备软件

请先参考安装指南中的方法安装 Docker Engine 和 Nvidia Container Toolkit。

### 6. 安装 Apollo 系统

参考安装指南进行安装。

### 7. 系统配置

在运行 Apollo 软件之前，建议先对系统的一些配置进行修改。

- 禁止软件自动更新
  
  Ubuntu 默认没有关闭软件自动更新选项，软件自动更新有时会带来一些难以预知的后果。比如，内核的自动升级会让已安装的一些驱动失效。您可以通过修改配置文件来禁止软件自动更新。

对于修改 `/etc/apt/apt.conf.d/10periodic` 为以下内容：

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
```

修改 `/etc/apt/apt.conf.d/20auto-upgrades` 为以下内容：

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
```

- 设置固定 IP

IP 的频繁变动不利于调试。

对于 Hardware3.0，可以修改 `/etc/netplan/01-network-manager-all.yaml` 为如下内容：

```
# Let NetworkManager manage all devices on this system
network:
 version: 2
 renderer: NetworkManager
 ethernets:
   eth0:
     addresses: [192.168.10.6/24]
     gateway4: 192.168.10.1
     nameservers:
       addresses: [192.168.10.1]
   eth1:
     addresses: [192.168.20.8/24, 192.168.1.6/24, 192.168.19.250/24]
     routes:
     - to: 224.100.100.0/24
       via: 192.168.20.1
```
