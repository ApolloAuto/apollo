对TX2刷机可以用JetPack3.2（为了支持docker必须使用3.2版本），这种方式下JetPack会使用官方提供的内核代码，编译后刷到TX2主板上，如果我们希望对内核进行更改则不能使用这个方法。

如果要更改TX2的内核基本上有两个办法：

* 在TX2上修改内核代码并重新安装内核
* 修改内核代码，使用Nvidia提供的工具手动刷机

因为TX2的内核是定制的内核，我们修改时也必须修改定制的内核，所以为了方便起见我们采用方法2。

在刷机之前，我们先检查一下TX2的内核版本：
`$ uname -a`

输出为：

`Linux tegra-ubuntu 4.4.38-tegra #1 SMP PREEMPT Thu May 17 00:15:19 PDT 2018 aarch64 aarch64 aarch64 GNU/Linux`

`$ cat /proc/version`

输出为：

`Linux version 4.4.38-tegra (buildbrain@mobile-u64-773) (gcc version 4.8.5 (GCC) ) #1 SMP PREEMPT Thu May 17 00:15:19 PDT 2018`

可以看到，系统的版本为tegra-ubuntu 4.4.38-tegra，编译的gcc版本为4.8.5

可以参考网址：
[Compiling_Tegra_X1/X2_source_code](https://developer.ridgerun.com/wiki/index.php?title=Compiling_Tegra_X1/X2_source_code)

[how-can-i-use-can-bus-in-tx2](https://devtalk.nvidia.com/default/topic/1006762/jetson-tx2/how-can-i-use-can-bus-in-tx2-/post/5165934/#5165934)


根据上述网站上的介绍尝试了一下，对刷机过程总结如下：

#### ToolChain
官方建议的ToolChain为gcc4.8.5，安装gcc4.8.5的步骤为：

`$ sudo mkdir /opt/l4t-gcc-toolchain-64-bit-28-2.1`

`$ sudo chown $USER:$USER /opt/l4t-gcc-toolchain-64-bit-28-2.1`

`$ cd /opt/l4t-gcc-toolchain-64-bit-28-2.1`

`$ wget https://developer.nvidia.com/embedded/dlc/l4t-gcc-toolchain-64-bit-28-2-ga`

`$ mv l4t-gcc-toolchain-64-bit-28-2-ga gcc-4.8.5-aarch64.tgz`

`$ tar -xvf gcc-4.8.5-aarch64.tgz`


在解决ESD CAN的问题过程中，德国方面给我们提供的测试驱动必须支持
`CONFIG_CC_STACKPROTECTOR_STRONG`特性，而该特定必须在gcc5.4.0更高版本上才能支持。所以我们也尝试一下使用gcc5.4.0+版本作为ToolChain。

gcc5.4.1的下载网址为：
[http://releases.linaro.org/components/toolchain/binaries](http://releases.linaro.org/components/toolchain/binaries)
我们下载的版本为gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz

按照对gcc4.8.5的操作一样，将下载的包解压缩到/opt文件夹内。


####　JetPack3.2

我们假设已经有了工具JetPack3.2，并且已经执行JetPack3.2工具到下载并解压缩安装结束的步骤，此时刷机所有的脚本都已经下载到了本地。假设JetPack3.2下载文件的路径为：
`$HOME/jetpack/`

#### Getting kernel

`$ export DEVDIR=$HOME/jetpack/64_TX2/Linux_for_Tegra`

`$ cd $DEVDIR/`

下载源码：

`$ ./source_sync.sh`

#### Build kernel

依次执行如下步骤：

`$ mkdir -p $DEVDIR/images/modules`

`$ mkdir -p $DEVDIR/images/packages`

`$ mkdir -p $DEVDIR/images/dtb`

如果使用的是gcc4.8.5则执行如下步骤：

`$ export CROSS_COMPILE=/opt/l4t-gcc-toolchain-64-bit-28-2.1/install/bin/aarch64-unknown-linux-gnu-`

如果是gcc5.4.1则执行下述步骤：

`$ export CROSS_COMPILE=/opt/l4t-gcc-toolchain-64-bit-28-2.1/install/bin/aarch64-linux-gnu-`

然后再依次执行下述步骤：

`$ export KERNEL_MODULES_OUT=$DEVDIR/images/modules`

`$ export TEGRA_KERNEL_OUT=$DEVDIR/images`

`$ export ARCH=arm64`

下面开始执行编译步骤：

`$ cd $DEVDIR/sources/kernel/kernel-4.4`

`$ make mrproper`

`$ sudo apt-get install libncurses5 libncurses5-dev`

`$ make O=$TEGRA_KERNEL_OUT tegra18_defconfig`

`$ make O=$TEGRA_KERNEL_OUT menuconfig`

请注意`tegra18_defconfig`是版本特性的配置文件，如`CONFIG_CC_STACKPROTECTOR=y`，`CONFIG_CC_STACKPROTECTOR_STRONG=y`等都需要配置在该文件中。
`menuconfig`的执行步骤会弹出一个菜单框，如果没有需要额外配置的，选择`Exit`退出即可。

`$ make O=$TEGRA_KERNEL_OUT zImage`

`$ make O=$TEGRA_KERNEL_OUT dtbs`

`$ make O=$TEGRA_KERNEL_OUT modules`

`$make O=$TEGRA_KERNEL_OUT modules_install INSTALL_MOD_PATH=$KERNEL_MODULES_OUT`

上述编译步骤执行完毕后，内核镜像为：

`$DEVDIR/images/arch/arm64/boot/Image`

设备树文件为：

`$DEVDIR/images/arch/arm64/boot/dts/*.dtb`

然后依次执行下述指令：

`$ export KERNEL_MODULES_NAME=4.4.38+`

删除软链接文件

`$ cd $DEVDIR/images/modules/lib/modules/$KERNEL_MODULES_NAME`

`$ rm build source`

创建JetPack必须的内核版本包

`$ cd $DEVDIR/images/modules/`

`$ tar -cjf kernel_supplements.tbz2 *`

`$ mv kernel_supplements.tbz2 $DEVDIR/images/packages`

打包内核文件

`$ cd $DEVDIR/kernel`

`$ tar -xf kernel_headers.tbz2`

备份文件

`$ mkdir -p $DEVDIR/images/packages-backup`

`$ cp -rf $DEVDIR/kernel/* $DEVDIR/images/packages-backup`

拷贝dtb文件

`$ cp $DEVDIR/images/arch/arm64/boot/dts/tegra186-quill-p3310-1000-c03-00-base.dtb $DEVDIR/kernel/dtb`

`$ cd $DEVDIR/images`

`$ cp -rf arch/arm64/boot/Image arch/arm64/boot/zImage packages/kernel_supplements.tbz2 $DEVDIR/kernel/`

----$ cp -rf packages/kernel_headers_custom.tbz2 $DEVDIR/kernel/kernel_headers.tbz2-----

生成镜像

`$ cd $DEVDIR/`
`$ sudo ./apply_binaries.sh`

下一步就是开始刷机，使用mircoUSB将主机和TX2相连，并使TX2进入recovery模式：
`Power on the Jetson board.
Hold the recovery button and then press the reset button.
The Jetson board should be connected to the host computer via USB, so you can check if it is on reset/recovery mode by typing:`

`lsusb`

`You should see "NVidia Corp." as part of the returned message.`

然后执行如下指令刷机

`$ cd $DEVDIR/`

`$ sudo ./flash.sh jetson-tx2 mmcblk0p1`

上述步骤执行后，可以查看TX2的内核版本为：

`tegra-ubuntu 4.4.38+`

表示确实是刷机成功了。




