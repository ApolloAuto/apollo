---
title: TX2源码安装Apollo+ROS环境指导手册
tags: Apollo,TX2,ROS
grammar_cjkRuby: true
---

>本文档用于源码安装Apollo和Apollo ros，并且列出了可能遇到的问题的解决方案。


- 整个环境需要较大的硬盘空间，因此我们建议使用一块128G以上空间的固态硬盘。另外考虑到ROS具有一些绝对路径的依赖关系，TX2的Apollo安装将不使用交叉编译。

- Apollo使用一些基于Ubuntu14的一些环境，而TX2默认支持Ubuntu16，并且NVIDIA并没有提供对Ubuntu14的支持，而Ubuntu16 for arm官方版又不对NVIDIA显卡提供支持，因此我们需要安装docker，以支持Apollo的编译和运行。

- 如果是全新的tx2开发板，如果无法安装docker，需要使用JetPack3.3为其升级，升级后将自带docker。

# 环境搭建的准备工作
## 1.docker安装：
```
	$ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
	$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | $ sudo apt-key add -
	$ sudo apt-add-repository "deb [arch=arm64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
	$ sudo apt-get update
	$ apt-cache madison docker-ce
	$ sudo apt-get install docker-ce=18.03.1~ce-0~ubuntu
	$ sudo groupadd docker
	$ sudo usermod -aG docker $USER
	$ sudo service docker restart
```
## 2.docker验证：
```
nvidia@tegra-ubuntu:~$ docker run hello-world
Unable to find image 'hello-world:latest' locally
latest: Pulling from library/hello-world
e610d22537c5: Pull complete 
Digest: sha256:0add3ace90ecb4adbf7777e9aacf18357296e799f81cabc9fde470971e499788
Status: Downloaded newer image for hello-world:latest

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (arm64v8)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/

```
## 3.接下来需要制作docker镜像，在空文件夹下以如下内容创建dockerfile
```
	FROM arm64v8/ubuntu:14.04
	ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra
```
使用如下命令建立docker镜像
```
	docker build -t haotian/apollo .
```
运行docker镜像
```
	docker run -i -t haotian/apollo
```
假设我们的固态硬盘挂载在`/home/nvidia/work`位置，并且从git上下载的`apollo-master.tar.gz`解压在`/home/nvidia/work/apollo-master`位置，可以对宿主机和容器中的镜像机进行目录映射，另外，还有关于设备文件和网络环境等设置需要添加，因此我们需要运行如下命令启动docker镜像
```
	docker run -v /home/nvidia/apollo-master:/root/apollo-master -v /dev:/dev --net host --shm-size 2G -i -t haotian/apollo
```
这个docker镜像是一个非常精简的镜像，需要专门部署很多必备环境，可以使用类似如下命令的方式先进行安装准备和测试。
```
	$ sudo apt-get update
	$ sudo apt-get install vim
	$ sudo apt-get install wget
	...
```
接下来我们将配置Apollo运行需要的环境。
# 一，安装bazel：
## 1.安装jdk8
```
	来此地址下载java_8u144
	https://www.oracle.com/technetwork/cn/java/javase/downloads/java-archive-javase8-2177648-zhs.html
	选择jdk-8u144-linux-arm64-vfp-hflt.tar.gz
	解压，设置环境变量
	$ export JAVA_HOME="java解压目录"
```
## 2.下载编译安装bazel
```
	$ wget https://github.com/bazelbuild/bazel/releases/download/0.10.0/bazel-0.5.3-dist.zip
	$ export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-arm64/
	$ bash ./comple.sh
```
相关问题解决：
```
	if(ERROR: Must specify PROTOC if not bootstrapping from the distribution artifact)
	{	
		sudo apt install protobuf-compiler
		export PROTOC=/usr/bin/protoc
	}
	
	if(ERROR: /root/apollo-master/docker/build/installers/bazel_053/src/main/java/com/google/devtools/build/lib/BUILD:117:1: Building src/main/java/com/google/devtools/build/lib/libconcurrent.jar (18 source files) failed: Worker process sent response with exit code: 1.)
	{
		export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-arm64/
		export PROTOC=/usr/local/bin/protoc
		export JRE_HOME=${JAVA_HOME}/jre
		export CLASSPATH=.:${JAVA_HOME}/lib:${JRE_HOME}/lib
		export PATH=${JAVA_HOME}/bin:/usr/local/bin/:$PATH
		export JAVA_OPTS="-server -Xms2048m -Xmx2048m"
	}
```
# 二.安装cuda8.0
```
	$ wget http://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/013/linux-x64/cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb
	$ dpkg -i cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb
	$ sudo apt-get update
	$ sudo apt-get install cuda-toolkit-8-0
		#Install cudnn7
	$ wget http://123.57.58.164/apollo-docker/cuda8_cudnn6/libcudnn6_6.0.21-1+cuda8.0_arm64.deb
	$ dpkg -i libcudnn6_6.0.21-1+cuda8.0_arm64.deb

	$ Install cudnn7-dev
	$ wget http://123.57.58.164/apollo-docker/cuda8_cudnn6/libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb
	$ dpkg -i libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb
```
# 三.安装gflag和glog
```
	$ sudo apt-get install cmake
	$ wget https://github.com/google/glog/archive/master.zip
	$ unzip master.zip
	$ cd glog-master/
	$ ./autogen.sh
	$ ./configure CPPFLAGS="-fPIC"
	$ make
	$ make install
```
# 四.安装glew
```
	$ ln -s /usr/lib64/libGLEW.so /usr/lib/libGLEW.so
	$ ln -s /usr/lib64/libGLEW.so.2.0 /usr/lib/libGLEW.so.2.0
	$ sudo apt-get install libxi-dev
	$ ./install_glew.sh
```
# 五.安装glfw3
```
	http://www.glfw.org/
	cmake -DBUILD_SHARED_LIBS=ON ../
	make;make install
```
相关故障排除：
```
	if(The RandR library and headers were not found)
		sudo apt-get install libxrandr-dev
	if(The Xinerama library and headers were not found)
	{
		sudo apt-get install libsdl2-2.0-0
		sudo apt-get install libsdl2-dev
	}
```
# 六.安装gpu_caffe
先安装caffa相关环境
```
	$ apt-get update -y && apt-get install -y \
		libatlas-base-dev \
		libflann-dev \
		libhdf5-serial-dev \
		libicu-dev \
		liblmdb-dev \
		libopenblas-dev \
		libopencv-dev \
		libopenni-dev \
		libqhull-dev \
		libsnappy-dev \
		libvtk5-dev \
		libvtk5-qt4-dev \
		mpi-default-dev
	$ apt-get install libvtk5-qt4-dev
    $ wget http://123.57.58.164/apollo-docker_no/openblas-0.2.18.tar.gz
    $ tar -xzvf openblas-0.2.18.tar.gz 
    $ sudo mkdir /usr/include/openblas
    $ sudo cp openblas-0.2.18/include/* /usr/include/openblas/
    $ sudo cp -r openblas-0.2.18/lib/* /usr/lib/
```

安装Caffe使用预先构建好的二进制包caffe_aarch64.tar.gz，可以从github上[下载](https://github.com/apolloauto/apollo/releases/download/caffe_aarch64.tar.gz)

```
tar -xzvf caffe_aarch64.tar.gz
sudo mv caffe_aarch64/lib* /usr/lib/aarch64-linux-gnu/
sudo mv caffe_aarch64/caffe/ /usr/include/
```
# 七.安装ipopt
```
	$ vim docker/build/installer/install_ipopt.sh
	#把x86_64改成arm
	$ ./install_ipopt.sh
```
# 八.安装node
```
	$ ./install_node.sh
	(https://github.com/tj/n/pull/448/files)
	$ vim /usr/local/bin/n
	case "$uname" in
		*x86_64*) arch=x64 ;;
		*armv6l*) arch=armv6l ;;
		*armv7l*) arch=armv7l ;;
	#new	*arm64*) arch=arm64 ;;
	#new	*aarch64*) arch=arm64 ;;
	esac

	./install_ipopt.sh
```
相关故障排除：
```
	if(node不可用，无文件)
		下载https://nodejs.org/dist/v8.12.0/node-v8.12.0-linux-arm64.tar.xz
	把里面的node解压出来，放在/usr/local/bin/下
```
# 九.安装pcl

ros依赖PCL，需要重新编译PCL的aarch64版本。下载[下载PCL源码](https://github.com/PointCloudLibrary/pcl)
编译之前请确认安装依赖的库文件，比如libgtest-dev、libboost-all-dev、eigen、flann、HDF5、libvtk5-qt4-dev、
    liblog4cxx10-dev、libogg-dev、libtheora-dev、liblua5.1-0-dev、liblua50-dev、liblualib50-dev、liblivemedia-dev、
    libmad0-dev、libfaad-dev、liba52-dev、libflac-dev、libmpeg2-4-dev等
```
    $ mkdir build
    $ cd build
    $ cmake ../ 
#如果编译release版本：
    $ cmake -DCMAKE_BUILD_TYPE=Release ../
    $ make && make install
```
编译完成后在如下目录的CMakefiles.txt文件中添加include path (vision_opencv/cv_bridge/include)：
- image_transport_plugin/compressed_depth_image_transport
- image_transport_pligin/image_transport_plugin

# 十.源码安装ros
```
	$ wget https://github.com/ApolloAuto/apollo-platform/archive/2.1.2.tar.gz
	$ tar xvf apollo-platform-2.1.2.tar.gz
	$ echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
		> /etc/apt/sources.list.d/ros-latest.list
	$ apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
		--recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

	$ apt-get update -y && apt-get install -y \
		libbz2-dev \
		#libceres-dev \#可能没有
		libconsole-bridge-dev \
		libeigen3-dev \
		libgstreamer-plugins-base0.10-dev \
		libgstreamer0.10-dev \
		liblog4cxx10-dev \
		liblz4-dev \
		libpoco-dev \
		libproj-dev \
		libtinyxml-dev \
		libyaml-cpp-dev \
		#ros-indigo-catkin \#可能没有
		sip-dev \
		uuid-dev \
		zlib1g-dev
	开始编译
	$ ./build.sh build
```
相关故障排除：
```
	if(Make sure that you have installed "catkin_pkg")
		sudo apt-get install python-catkin-pkg
	if(CMake Error at cmake/empy.cmake:29 (message):)
		sudo apt-get install python-empy
	if(CMake Error at CMakeLists.txt:11 (find_package):
  By not providing "Findconsole_bridge.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "console_bridge", but CMake did not find one.

  Could not find a package configuration file provided by "console_bridge")
	{
		mkdir cb_ws
		cd cb_ws
		wget https://github.com/ros/console_bridge/archive/0.4.0.tar.gz
		tar xvf 0.4.0.tar.gz
		cd console_bridge-0.4.0/
		cmake ./
		make
		make install
	}
	if(CMake Error at /usr/share/cmake-2.8/Modules/FindBoost.cmake:1131 (message):
  Unable to find the requested Boost libraries.)
	{
		wget http://sourceforge.net/projects/boost/files/boost/1.53.0/boost_1_53_0.tar.bz2
		tar xvf boost_1_53_0.tar.bz2
		cd boost_1_53_0/
		sudo ./bootstrap.sh
		sudo ./b2
		sudo ./b2 install
	}
	if(Project 'class_loader' tried to find library 'PocoFoundation'.)
	{
		wget https://github.com/pocoproject/poco/archive/poco-1.9.0-release.tar.gz
		tar xvf poco-1.9.0-release.tar.gz
		./configure
		make
		make install
		if(fatal error: openssl/opensslv.h: No such file or directory)
		{
			wget https://www.openssl.org/source/openssl-1.0.2h.tar.gz
			tar xvf openssl-1.0.2h.tar.gz
			./Configure
			./config shared zlib
			make depend
			make install
			mv /usr/bin/openssl /usr/bin/openssl.bak
			mv /usr/include/openssl /usr/include/openssl.bak
			ln -s /usr/local/ssl/bin/openssl /usr/bin/openssl
			ln -s /usr/local/ssl/include/openssl /usr/include/openssl
			ln -s /usr/local/ssl/lib/libcrypto.so /usr/local/lib/libcrypto.so
			ln -s /usr/local/ssl/lib/libssl.so /usr/local/lib/libssl.so
			echo "/usr/local/ssl/lib" >> /etc/ld.so.conf
			ldconfig -v
			检测安装是否成功：openssl version -a
		}
	}
	if(-fpermissive)
	{
		apt-get install libvtk5-qt4-dev
		apt-get install -y liblog4cxx10-dev
		在所有logdebug,logwarn等等报错函数前面加 CONSOLE_BRIDGE_
		ps:(https://talk.apolloauto.io/t/topic/77)
	}
	if(Cannot specify link libraries for target "image_geometry-utest" which is)
	{
		sudo apt install libgtest-dev
	}
	if(CMake Error at /usr/local/share/pcl-1.7/PCLConfig.cmake:41 (message):
  common is required but boost was not found)
	{
		apt-get install libboost-all-dev
		ln -s /usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.21 /usr/lib/aarch64-linux-gnu/libstdc++.so.6
	}
	if(package 'ogg' not found)
	{
		sudo apt-get install libogg-dev
	}
	if(package 'theora' not found)
	{
		sudo apt-get install libtheora-dev liblua5.1-0-dev liblua50-dev liblualib50-dev liblivemedia-dev libogg-dev libmad0-dev libfaad-dev liba52-dev libflac-dev
		sudo apt-get install libmpeg2-4-dev
	}
```
使用如下命令进行install
```
	$ ln -s install/ros_aarch64/ /home/tmp/ros
```
至此，安装完毕，使用roscore命令检查安装是否成功

![roscore](./images/roscore.JPG)

相关故障排除：
	 
```
	if(root@efd07ecdb525:~/apollo-master# roscore相关if(root@efd07ecdb525:~/apollo-master# roscorennagguanif(root@efd07ecdb525:~/apollo-master# roscore相关if(root@efd07ecdb525:~/apollo-master# roscore 
Traceback (most recent call last):
  File "/opt/ros/bin/roscore", line 36, in <module>
    from rosmaster.master_api import NUM_WORKERS)
	{
		
		apt-get install git
		cd third_party
		./build.sh build
		cd ../
		./build.sh build
		cd third_party/swig_wrappe
		bash build.sh
		PS：
		
		build and install fast-rtps follow the build.sh in https://github.com/ApolloAuto/apollo-platform/blob/master/ros/third_party/build.sh
		build and install the apollo-platform
		build and install _participant.so follow the build.sh in https://github.com/ApolloAuto/apollo-platform/blob/master/ros/third_party/swig_wrapper/build.sh
	}
	if(while loading shared libraries: libxxxxx.so.xxx)
	{
		find this.so
		export LD_LIBRARY_PATH=xxx
		or try do like this:
		ln -s /usr/lib/aarch64-linux-gnu/libboost_system.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_system.so.1.58.0
		if(symbol lookup error: /opt/ros/lib/librosconsole_log4cxx.so)
		{
			# mv /opt/ros/lib/librosconsole_log4cxx.so /opt/ros/lib/xxx_librosconsole_log4cxx.so
			# cp ~/apollo-master/PCL/librosconsole_log4cxx.so /opt/ros/lib/
		}
	}
```
# 十一.others_1
```
	./install_adv_plat.sh
	./install_bazel_packages.sh
	./install_conda.sh
	./install_libjsonrpc-cpp.sh
	./install_nlopt.sh
	./install_ota.sh
	./install_protobuf.sh
	./install_python_modules.sh
	if(hdf5.h: No such file or directory)
		sudo apt-get install libhdf5-dev
	./install_qp_oases.sh
	./install_undistort.sh
	./install_user.sh
```
# 十二.安装supervisor
```
	wget https://pypi.python.org/packages/source/s/supervisor/supervisor-3.1.3.tar.gz
	tar zxvf supervisor-3.1.3.tar.gz
	cd supervisor
	python setup.py install
```
# 十三.安装yarn
```
	./install_yarn.sh
	if(./install_yarn.sh: line 22: curl: command not found)
	{
		sudo add-apt-repository ppa:costamagnagianfranco/ettercap-stable-backports
		sudo apt-get update
		sudo apt-get install curl
		ln -s /usr/bin/curl /usr/local/bin/curl
	}
	if(E: The method driver /usr/lib/apt/methods/https could not be found)
		sudo apt-get install apt-transport-https
```
# 十四.编译apollo
回到apollo-master下编译Apollo与激光雷达相关的驱动源码。
(若不使用激光雷达，此步可以不做)
```
	./apollo.sh build_velodyne
```
编译成功后是这样的

![build_velodyne](./images/build_velodyne.JPG)

再执行如下命令进行真正的源码编译
```
	$ ./apollo.sh build_gpu
```
	if(/root/apollo-master/scripts/apollo_base.sh: line 51: bc: command not found)
	{
		apt-get install bc
	}
	if(Couldn't find java at '/usr/lib/java/bin/java'.)
	{
		ln -s /usr/lib/jvm/java-8-openjdk-arm64/ /usr/lib/java
	}
	if(ERROR: Failed to get information about path, for docker/build/installers/ros_source/apollo-platform-2.1.2
	/ros/third_party/fast-rtps_aarch64/fast-rtps_aarch64, skipping: Infinite symlink expansion)
	{
		rm docker/build/installers/ros_source/apollo-platform-2.1.2/ros/third_party/fast-rtps/fast-rtps_aarch64
	}
	./apollo.sh build_velodyne
	if(ERROR: /root/apollo-master/modules/canbus/BUILD:32:1: Linking of rule '//modules/canbus:canbus' failed (Exit 1).
	/usr/bin/ld: skipping incompatible bazel-out/local-dbg/bin/_solib_arm/_U_S_Sthird_Uparty_Scan_Ucard_Ulibrary_Shermes_Ucan_Chermes_Ucan___Uthird_Uparty_Scan_Ucard_Ulibrary_Shermes_Ucan_Slib/libbcan.so when searching for -lbcan
	/usr/bin/ld: cannot find -lbcan
	collect2: error: ld returned 1 exit status
	INFO: Elapsed time: 1639.952s, Critical Path: 141.56s
	============================
	[ERROR] Build failed!
	[INFO] Took 1642 seconds
	============================)
	{
		根据./modules/drivers/canbus/can_client/hermes_can/bcan.h文件声明的函数，编写bcan.cc，对这些函数进行空定义
		gcc bbcan.cc -shared -o libbcan.so
		将得到的libbcan.so覆盖到./third_party/can_card_library/hermes_can/lib/libbcan.so
	}
	if(external/curlpp/include/curlpp/OptionBase.hpp:29:23: fatal error: curl/curl.h: No such file or directory
	#include <curl/curl.h>)
	{
		cp docker/build/installers/curl_src/curl-7.55.1/include/curl/ /usr/include/ -r
	}
	if(/usr/bin/ld: cannot find -lcurl)
	{
		ln -s /usr/lib/aarch64-linux-gnu/libcurl.so.4.3.0 /usr/lib/aarch64-linux-gnu/libcurl.so
	}
	if(gcc: error: unrecognized command line option '-fsanitize=fuzzer,address,undefined')
	{
		mv modules/tools/fuzz/ ~
	}
该过程将持续比较长的时间，成功的话约数小时后将会有如下效果。
```
____Loading package: @glew//
____Loading package: @glfw//
____Loading package: @vtk//
____Loading package: @caffe//
____Loading package: @opengl//
[INFO] Building on aarch64...
INFO: Reading 'startup' options from /apollo/tools/bazel.rc: --batch_cpu_scheduling
INFO: Found 1637 targets...
INFO: Elapsed time: 98.435s, Critical Path: 1.91s
fatal: Not a git repository (or any parent up to mount point /apollo)
Stopping at filesystem boundary (GIT_DISCOVERY_ACROSS_FILESYSTEM not set).
============================
[ OK ] Build passed!
[INFO] Took 19408 seconds
============================
```
运行Apollo工程进行测试
```
	$ source envset
	$ source ros_set
	$ source /home/tmp/ros/setup.sh
	$ scripts/bootstrap.sh start
	#下载测试bag文件，在期目录执行：
	$ rosbag play demo_2.5.bag -l --clock
```
网页登录tx2 ip:8888
页面如下

![apollo_web](./images/apollo_run_web.JPG)

测试完成后，提交并且打包镜像