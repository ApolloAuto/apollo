# Apollo在TX2平台上问题集锦

## 1. 在Ubuntu上部署Apollo3.0的准备工作。
 （1）需要下载ARM版本的Docker以及Ubuntu14（ARM版本）的Image。首先Apollo3.0本身依赖Ubuntu14上的底层库。Ubuntu16虽然也有对应的库，但是版本不对应；其次perception模块所依赖的Caffe需要Ubuntu14显卡驱动的支持。

（2）安装所需要的库文件，比如Ipopt、vtk、opencv2、pcl需重新编译glog、gtest、gflags的aarch64版本。


## 2. 百度caffe编译需要注意哪些？
（1）protobuf的版本需要和Apollo3.0的一致

（2）重新编译gflags和glogs，版本需与Apollo3.0一致

（3）修改编译脚本中的引用路径，例如很多引用路径是../../adu/...

（4）编译完成后的.so存放在output-GPU目录下

（5）编译完成后需要调用相关的测试程序，查看是否成功调用GPU并测试通过

## 3.  perception启动失败
查看日志，可以通过deviceQuery查看是否是因为GPU无法被识别导致的问题，如果是，可以重新安装CUDA，或者采用root方式执行deviceQuery。否则，尝试重新编译caffe。

## 4. 在Docker Image中编译Apollo，如果出现cuda头文件没有依赖规则的错误怎么办？
（1）检查CUDA头文件的安装路径是否和编译脚本的一致

（2）如果一致，则重新安装CUDA即可

## 5. 在编译Ipopt时遇到头文件没有依赖规则的错误怎么办？
这个错误是由于Ipopt的默认安装位置与编译脚本不一致导致的，可修改编译脚本使其能找到正确的头文件位置

## 6. Apollo编译过程中出现（Exit 4）的错误怎么办？
这个问题是由于gcc占用大量内存导致的，解决办法有两个

（1）修改—jobs参数为1（即 –jobs=1）

（2）添加local_resoueces选项，例如：--local_resources 2048,.5,1.0

## 7. 如何下载TX2下对应的第三方库文件？
可以到`http://ports.ubuntu.com/pool`的子目录下下载对应版本的库文件，注意一定是*\_arm64.deb文件。下载完成后可通过如下命令执行安装:

```shell
dpkg -i <package name>
```

如果安装包与依赖，应该先下载对应的版本的依赖包

## 8. 启动rostopic或者roscore失败
执行如下命令：

```shell
export ROS_HOSTNAME=localhost 
export ROS_MASTER_URI=http://localhost:8888
```

以上命令已经写成脚本ros_set，可直接执行
如果是报引用缺少的缺少库文件netifaces，可执行如下命令进行安装：

```shell
pip install netifaces
```

## 9. 如何查看TX2资源占用情况？
执行如下命令：

```shell
sudo ~/tegrastats
```

或者执行：

```shell
top
```

## 10. 如何开启TX2的最大功率？
TX2的CPU一共有六个核，默认情况下cpu1核cpu2处于关闭状态，有两种方式开启关闭的两个核：

1.执行如下命令：

```shell
sudo su
echo 1 > /sys/devices/system/cpu/cpu1/online
echo 1 > /sys/devices/system/cpu/cpu2/online
```

2.执行如下命令：

```shell
sudo nvpmodel -p -verbose #查看当前工作模式，TX2一共有5个工作模式，其中0为火力全开 
sudo nvpmodel -m 0 #修改工作模式为0
```

## 11. 编译百度caffe时，链接时找不到glog或者gflags对应的函数。
需要将glog和gflags编译成动态库（.so）。在执行编译配置时如下：

```shell
./configure CPPFLAGS="-fPIC"
```

## 12. CUDA报错，错误代码为35
先检查cuda驱动和cudnn是否安装，且版本一致（cuda为8.0，cudnn为6.0.21）；然后执行source envset，以设置环境变量
