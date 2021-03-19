# 如何为 Apollo 安装低时延/实时内核

本文档描述了在 Ubuntu 18.04 上安装低时延 (Low-Latency) 或实时 (Realtime)内核及
Nvidia 驱动的步骤。

## 在开始之前

仅在实车上运行 Apollo 软件栈才需要低时延或实时内核。 如果您的目的只是基于 Apollo
平台开发/测试您的算法，或者运行仿真软件（如 Lgsvl 模拟器），则可能您根本不需要安
装这里描述的低时延或实时内核。

## Ubuntu 自带的低时延内核

Ubuntu 软件仓库中的低时延内核足以为实车上运行 Apollo 提供低（或者零）时延。在其
内核配置中，任务抢占式 (PREEMPT)优化是开启了的，时延可低至 0.1 毫秒。

下面是安装 Ubuntu 低时延内核的步骤：

1. 安装最新的低时延内核及其头文件

```bash
sudo apt-get update
sudo apt-get install linux-image-$(uname -r)-lowlatency linux-headers-$(uname -r)-lowlatency
```

**注意**：

> 如果在执行了`sudo apt-get update`后通过`apt list --upgradable`查看有新版本内核
> ，请将上述命令中的`$(uname -r)`改为 Ubuntu 软件仓库中最新的内核版本号。截至本
> 文写作时（2020 年 12 月 2 日），Ubuntu 软件仓库中的最新内核是`5.4.0-56`。

2. 重启系统以启动低时延内核。

```bash
sudo reboot
```

## 安装实时内核

请按
照[ROS2：构建实时 Linux](https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2)
中描述的步骤来构建和安装最新的稳定版实时内核。虽然该文档是按照 Ubuntu 20.04 来讲
的，但其中的步骤完全适用于 Ubuntu 18.04。

## 安装 Nvidia 驱动

### 在低时延内核上安装 Nvidia 驱动

对 Ubuntu 低时延内核而言，安装 Nvidia 驱动的步骤比较简单：

1. 从[CUDA Toolkit 下载](https://developer.nvidia.com/cuda-downloads?target_os=Linux)
   页下载并安装 Nvidia 最新驱动。

在选择安装类型(Installer Type)时，建议选择 **本地安装(deb[local])** 或者 **网络
安装(deb(network])** 模式。

![Download CUDA Toolkit for x86_64](images/download_cuda_x86_64.png)

![Download CUDA Toolkit for AArch64](images/download_cuda_aarch64.png)

**注意**：

> 可能需要注册并签署 CUDA 最终用户使用协议(EULA) 才可以下载 Nvidia 驱动及 CUDA
> 安装包。

比如，如下是在 x86_64 架构的 Ubuntu 18.04.5 上通过 **本地安装 Deb 软件包**的方式
安装 Nvidia 驱动：

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.1.1/local_installers/cuda-repo-ubuntu1804-11-1-local_11.1.1-455.32.00-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1804-11-1-local_11.1.1-455.32.00-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu1804-11-1-local/7fa2af80.pub
sudo apt-get update
sudo apt-get install nvidia-driver-455
```

**注意**:

> `nvidia-driver-XXX` 的数字应该与 CUDA 本地安装包中的数字一致。本例中是`455`。

2. 重启系统。

3. 运行`nvidia-smi` 检查是否生效。

### 在实时 (PREEMPT_RT) 内核上安装 Nvidia 驱动

请先按照低时延内核 Nvidia 驱动的安装方法完成 Nvidia 驱动的初步安装。请注意，由于
Nvidia 驱动不支持实时内核，所以在上面执
行`sudo apt-get install nvidia-driver-455` 的步骤时会报如下错误消息：

```text
The kernel you are installing for is a PREEMPT_RT kernel!

The NVIDIA driver does not support real-time kernels. If you
are using a stock distribution kernel, please install
a variant of this kernel that does not have the PREEMPT_RT
patch set applied; if this is a custom kernel, please
install a standard Linux kernel.  Then try installing the
NVIDIA kernel module again.

*** Failed PREEMPT_RT sanity check. Bailing out! ***
```

我们可以通过在编译 Nvidia 驱动的时候设置 `IGNORE_PREEMPT_RT_PRESENCE=1` 来绕过这
一点。

步骤如下：

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

2. 重启系统
3. 运行 `nvidia-smi` 来检查 Nvidia 驱动是否正常工作。

## （可选）安装 ESD-CAN 内核驱动

如有需要，可按照
[ESD-CAN 安装说明](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)
来编译安装 ESD-CAN 驱动。
