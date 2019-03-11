#  Apollo3.0 编译

> *为了提高Apollo源码在TX2平台上的编译速度，需要启用swap分区。具体操作参见以下命令：（路径`/var/swap`根据文件系统的可用空间调整位置，最好放在读写速度较快的SSD上！）*
```shell
$ sudo dd if=/dev/zero of=/var/swap bs=1024 count=8192000
$ sudo mkswap -f /var/swap
$ echo "/var/swap swap swap defaults 0 0" | sudo tee -a /etc/fstab
$ sudo swapon /var/swap
$ echo 'vm.swappiness=0' | sudo tee -a /etc/sysctl.conf
$ sudo sysctl -w vm.drop_caches=3
$ sudo reboot
```
> *同时需要开启TX2的最大功率运行模式提高编译速度：*
```shell
$ sudo nvpmodel -p —verbose #查看当前工作模式，TX2一共有5个工作模式，其中0为最大功率模式
$ sudo nvpmodel -m 0
$ sudo /home/nvidia/jetson_clocks.sh
```

Docker环境配置完成后，开始从源码构建Apollo3.0。  
建议在启动Docker容器之前从github上将代码下载到对应的目录中。源码的目录树结果如下

```txt
nvidia@tegra-ubuntu:~/work/ApolloAuto$ tree -L 1
.
├── apollo
├── apollo-kernel
├── apollo-platform
├── cuda
├── images
```

从github上下载最新的[apollo-platform](https://github.com/ApolloAuto/apollo-platform)、[apollo](https://github.com/ApolloAuto/apollo)、[apollo-kernel](https://github.com/ApolloAuto/apollo-kernel)到ApolloAuto/目录中

```shell
cd ~/work/ApolloAuto
git clone https://github.com/ApolloAuto/apollo-platform.git
git clone https://github.com/ApolloAuto/apollo.git
git clone https://github.com/ApolloAuto/apollo-kernel.git
```

编译Apollo3.0源码包含两个步骤；依次在Docker容器中执行的；启动Docker镜像的命令如下

```shell
$ docker run -it --privileged=true --pid=host --net host -w /apollo --add-host in_dev_docker:127.0.0.1 --hostname in_dev_docker --shm-size 2G -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/gstreamer-1.0 -v /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl -v /usr/lib/aarch64-linux-gnu/mesa-egl:/usr/lib/aarch64-linux-gnu/mesa-egl -v /lib/firmware/tegra18x:/lib/firmware/tegra18x -v /home/nvidia/work/ApolloAuto/apollo/:/apollo -v /home/nvidia/work/ApolloAuto/apollo-platform:/apollo-platform apolloauto/dev:dev-aarch64-2018 /bin/bash
```  

也可以执行docker/scripts目录下的`dev_start.sh`脚本启动Docker容器，然后执行`dev_into.sh`脚本进入容器并得到一个交互式的Shell

```shell
docker/scripts/dev_start.sh -l
docker/scripts/dev_into.sh
```

> *首次执行dev_start.sh文件之前需要修改`VERSION_AARCH64`为之前构建的Dev镜像的名称*

****

## 激光雷达驱动编译

Apollo代码编译过程中依赖激光雷达驱动的头文件和库文件，所以在apollo-platform（ROS）编译安装完成后（Apollo3.0编译之前），需要执行如下命令编译激光雷达的编译

```shell
$ cd /apollo
$ mkdir packages
$ cd packages/
$ ln -s /apollo-platform/ apollo-platform-master
$ cd ../
$ ./apollo.sh build_velodyne
```

编译完成后的提示如下

```txt
<== Finished processing package [4 of 4]: 'velodyne_pointcloud'
/apollo
```

> *执行ln -s /apollo-platform/ apollo-platform-master命令的主要原因是编译过程中出现找不到/apollo/packages/apollo-platform-master/ros/install/ros_aarch64/include目录*

```txt
......前面的打印省略
-- Found Threads: TRUE  
-- Found PROTOBUF: /usr/lib/libprotobuf.so  
-- Using these message generators: gencpp;genlisp;genpy
CMake Error at /home/tmp/ros/share/eigen_conversions/cmake/eigen_conversionsConfig.cmake:106 (message):
  Project 'eigen_conversions' specifies
  '/apollo/packages/apollo-platform-master/ros/install/ros_aarch64/include'
  as an include dir, which is not found.  It does neither exist as an
  absolute directory nor in
  '/apollo/packages/apollo-platform-master/ros/install/ros_aarch64//apollo/packages/apollo-platform-master/ros/install/ros_aarch64/include'.
  Ask the maintainer 'Tully Foote <tfoote@osrfoundation.org>' to fix it.
Call Stack (most recent call first):
  /home/tmp/ros/share/catkin/cmake/catkinConfig.cmake:76 (find_package)
  CMakeLists.txt:15 (find_package)


-- Configuring incomplete, errors occurred!
See also "/apollo/modules/build_isolated/velodyne_pointcloud/CMakeFiles/CMakeOutput.log".
See also "/apollo/modules/build_isolated/velodyne_pointcloud/CMakeFiles/CMakeError.log".
<== Failed to process package 'velodyne_pointcloud': 
  Command '['/home/tmp/ros/env.sh', 'cmake', '/apollo/modules/drivers/velodyne/velodyne_pointcloud', '-DCATKIN_DEVEL_PREFIX=/apollo/modules/devel_isolated/velodyne_pointcloud', '-DCMAKE_INSTALL_PREFIX=/home/tmp/ros', '-DCMAKE_BUILD_TYPE=Release', '--no-warn-unused-cli', '-G', 'Unix Makefiles']' returned non-zero exit status 1

Reproduce this error by running:
==> cd /apollo/modules/build_isolated/velodyne_pointcloud && /home/tmp/ros/env.sh cmake /apollo/modules/drivers/velodyne/velodyne_pointcloud -DCATKIN_DEVEL_PREFIX=/apollo/modules/devel_isolated/velodyne_pointcloud -DCMAKE_INSTALL_PREFIX=/home/tmp/ros -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli -G 'Unix Makefiles'

Command failed, exiting.
```

## Apollo3.0 编译

Apollo源码中使用了一些OpenCV函数进行图像处理，Docker镜像中默认安装的OpenCV版本是2.4.8，且没有利用GPU加速图像处理。根据Apollo源码中WORKSPACE文件里的配置信息，Apollo使用的OpenCV版本是2.4.13；在编译Apollo源码之前，可以重新编译一版使用CUDA加速的OpenCV库。

编译OpenCV需要安装的依赖如下（以下命令在Docker容器Ubuntu14.04中执行）：
```shell
$ sudo apt-get install -y \
    cmake \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libeigen3-dev \
    libglew-dev \
    libgtk2.0-dev \
    libgtk-3-dev \
    libjasper-dev \
    libjpeg-dev \
    libpng12-dev \
    libpostproc-dev \
    libswscale-dev \
    libtbb-dev \
    libtiff5-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    qt5-default \
    zlib1g-dev \
    pkg-config
$ sudo apt-get install -y python-dev python-numpy python-py python-pytest
$ sudo apt-get install -y python3-dev python3-numpy python3-py python3-pytest
$ sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```
编译OpenCV的命令行参数如下（以下命令在Docker容器Ubuntu14.04中执行）：
```shell
$ cd /home/tmp/
$ unzip opencv-2.4.13.2.zip
$ cd opencv-2.4.13.2
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
 -D CMAKE_INSTALL_PREFIX=/usr \
 -D WITH_CUDA=ON \
 -D CUDA_ARCH_BIN=6.2 \
 -D CUDA_ARCH_PTX="" \
 -D ENABLE_FAST_MATH=ON \
 -D CUDA_FAST_MATH=ON \
 -D WITH_CUBLAS=ON \
 -D WITH_LIBV4L=ON \
 -D WITH_GSTREAMER=ON \
 -D WITH_GSTREAMER_0_10=OFF \
 -D WITH_OPENGL=ON \
 -D WITH_NVCUVID=ON \
 -D WITH_QT=OFF ..
$ sudo rm -rf /usr/lib/aarch64-linux-gnu/libopencv_*
$ sudo make install
```
编译之后可以禁用bazel再重新编译OpenCV2.4.13，提高Apollo源码整体的编译速度。修改WORKSPACE.in文件将
```txt
# OpenCV 2.4.13.2
new_http_archive(
    name = "opencv2",
    build_file = "third_party/opencv2.BUILD",
    strip_prefix = "opencv-2.4.13.2",
    url = "file:///home/tmp/opencv-2.4.13.2.zip",
)
```
提换为
```txt
# OpenCV 2.4.13.2
new_local_repository(
    name = "opencv2",
    build_file = "third_party/opencv2.BUILD",
    path = "/usr/include",
)
```
同时将third_party/opencv2.BUILD文件的内容整体提换为
```txt
licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    includes = ["."],
    linkopts = [
#        "-lopencv_calib3d",
#        "-lopencv_contrib",
        "-lopencv_core",
#        "-lopencv_features2d",
#        "-lopencv_flann",
        "-lopencv_gpu",
        "-lopencv_highgui",
        "-lopencv_imgproc",
#        "-lopencv_legacy",
        "-lopencv_ml",
#        "-lopencv_objdetect",
#        "-lopencv_ocl",
#        "-lopencv_photo",
#        "-lopencv_stitching",
#        "-lopencv_superres",
        "-lopencv_video",
        "-lopencv_videostab",
    ],
)
cc_library(
    name = "imgproc",
    includes = ["."],
    linkopts = [
#        "-lopencv_calib3d",
#        "-lopencv_contrib",
        "-lopencv_core",
#        "-lopencv_features2d",
#        "-lopencv_flann",
        "-lopencv_gpu",
        "-lopencv_highgui",
        "-lopencv_imgproc",
#        "-lopencv_legacy",
        "-lopencv_ml",
#        "-lopencv_objdetect",
#        "-lopencv_ocl",
#        "-lopencv_photo",
#        "-lopencv_stitching",
#        "-lopencv_superres",
        "-lopencv_video",
        "-lopencv_videostab",
    ],
)
cc_library(
    name = "ml",
    includes = ["."],
    linkopts = [
#        "-lopencv_calib3d",
#        "-lopencv_contrib",
        "-lopencv_core",
#        "-lopencv_features2d",
#        "-lopencv_flann",
        "-lopencv_gpu",
        "-lopencv_highgui",
        "-lopencv_imgproc",
#        "-lopencv_legacy",
        "-lopencv_ml",
#        "-lopencv_objdetect",
#        "-lopencv_ocl",
#        "-lopencv_photo",
#        "-lopencv_stitching",
#        "-lopencv_superres",
        "-lopencv_video",
        "-lopencv_videostab",
    ],
)
cc_library(
    name = "video",
    includes = ["."],
    linkopts = [
#        "-lopencv_calib3d",
#        "-lopencv_contrib",
        "-lopencv_core",
#        "-lopencv_features2d",
#        "-lopencv_flann",
        "-lopencv_gpu",
        "-lopencv_highgui",
        "-lopencv_imgproc",
#        "-lopencv_legacy",
        "-lopencv_ml",
#        "-lopencv_objdetect",
#        "-lopencv_ocl",
#        "-lopencv_photo",
#        "-lopencv_stitching",
#        "-lopencv_superres",
        "-lopencv_video",
        "-lopencv_videostab",
    ],
)
cc_library(
    name = "highgui",
    includes = ["."],
    linkopts = [
#        "-lopencv_calib3d",
#        "-lopencv_contrib",
        "-lopencv_core",
#        "-lopencv_features2d",
#        "-lopencv_flann",
        "-lopencv_gpu",
        "-lopencv_highgui",
        "-lopencv_imgproc",
#        "-lopencv_legacy",
        "-lopencv_ml",
#        "-lopencv_objdetect",
#        "-lopencv_ocl",
#        "-lopencv_photo",
#        "-lopencv_stitching",
#        "-lopencv_superres",
        "-lopencv_video",
        "-lopencv_videostab",
    ],
)
```

Apollo3.0在TX2平台上可以编译Debug和Release版本；由编译脚本./apollo.sh后的参数指定

- build_gpu
- build_opt_gpu

```shell
$ ./apollo.sh build_gpu
```
或
```shell
$ ./apollo.sh build_opt_gpu
```

一开始编译如果遇到以下问题：

```txt
System check passed. Build continue ...
[WARNING] ESD CAN library supplied by ESD Electronics does not exist. If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md.
[INFO] Start building, please wait ...
WARNING: Processed legacy workspace file /apollo/tools/bazel.rc. This file will not be processed in the next release of Bazel. Please read https://github.com/bazelbuild/bazel/issues/6319 for further information, including how to upgrade.
ERROR: /root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/BUILD:441:1: Traceback (most recent call last):
        File "/root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/BUILD", line 441
                cc_proto_library(name = "cc_test_protos", srcs = (L...), <4 more arguments>)
        File "/root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/protobuf.bzl", line 247, in cc_proto_library
                cc_libs += [default_runtime]
trying to mutate a frozen object
ERROR: /root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/BUILD:742:1: Traceback (most recent call last):
        File "/root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/BUILD", line 742
                py_proto_library(name = "python_specific_test_pro...", <6 more arguments>)
        File "/root/.cache/bazel/_bazel_root/540135163923dd7d5820f3ee4b306b32/external/com_google_protobuf/protobuf.bzl", line 373, in py_proto_library
                py_libs += [default_runtime]
trying to mutate a frozen object
ERROR: Evaluation of query "//..." failed due to BUILD file errors
============================
[ERROR] Build failed!
[INFO] Took 4 seconds
============================
```
或
```txt
System check passed. Build continue ...
[WARNING] ESD CAN library supplied by ESD Electronics does not exist. If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md.
[INFO] Start building, please wait ...
WARNING: Processed legacy workspace file /apollo/tools/bazel.rc. This file will not be processed in the next release of Bazel. Please read https://github.com/bazelbuild/bazel/issues/6319 for further information, including how to upgrade.
Starting local Bazel server and connecting to it...
ERROR: /apollo/modules/monitor/hardware/BUILD:5:1: name 'cc_inc_library' is not defined (did you mean '$cc_library'?)
ERROR: package contains errors: modules/monitor/hardware
ERROR: error loading package 'modules/monitor/hardware': Package 'modules/monitor/hardware' contains errors
============================
[ERROR] Build failed!
[INFO] Took 16 seconds
============================
```
> *这是bazel版本造成的错误（系统中原来安装的bazel-0.18.0，降到bazel-0.5.3后正常通过以上步骤）。*  

如果出现`undefined reference to testing::internal::MakeAndRegisterTestInfo`报错信息，如下所示：
```txt
ERROR: /apollo/modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/BUILD:41:1: Linking of rule '//modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor:cc_lane_post_processor_test' failed (Exit 1).
bazel-out/local-opt/bin/modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/_objs/cc_lane_post_processor_test/modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/cc_lane_post_processor_test.o: In function `_GLOBAL__sub_I__ZN6apollo10perception3fLS14FLAGS_test_dirE':
cc_lane_post_processor_test.cc:(.text.startup._GLOBAL__sub_I__ZN6apollo10perception3fLS14FLAGS_test_dirE+0x2d8): undefined reference to `testing::internal::MakeAndRegisterTestInfo(char const*, char const*, char const*, char const*, void const*, void (*)(), void (*)(), testing::internal::TestFactoryBase*)'
collect2: error: ld returned 1 exit status
```
> *这是由于gtest版本问题导致的；Apollo源码需要使用的gtest版本为1.8.0，重新编译gtest即可。操作命令：*

```shell
sudo rm -rf /usr/include/gtest

cd /home/tmp/
sudo tar -xzvf googletest-release-1.8.0.tar.gz
cd googletest-release-1.8.0
sudo cmake .
sudo make install
```


开始编译后输出如下

```txt
[INFO] Start building, please wait ...
INFO: Reading 'startup' options from /apollo/tools/bazel.rc: --batch_cpu_scheduling
.
____Loading package: modules/drivers/canbus/can_client/fake
____Loading package: modules/monitor/hardware/can/socketcan
____Loading package: modules/localization/msf/local_map/test
____Loading package: modules/localization/msf/local_tool/data_extraction
____Loading package: modules/prediction/predictor/sequence
____Loading package: third_party/can_card_library/hermes_can
____Loading package: modules/calibration/republish_msg
____Loading package: modules/drivers/canbus
____Loading package: modules/perception/conf
____Loading package: modules/routing/graph
____Loading package: modules/third_party_perception/integration_tests
____Loading package: modules/localization/rtk
____Loading package: modules/canbus/vehicle/gem
____Loading package: tools/platforms
____Loading package: modules/drivers/gnss/test
____Loading package: modules/routing/common
____Loading package: modules/planning/tools
____Loading package: modules/planning/common
____Loading package: modules/planning/proto
____Loading package: modules/control
____Loading package: modules/guardian/proto
____Loading package: modules/guardian
____Loading package: modules/prediction
____Loading package: modules/drivers/gnss
____Loading package: modules/common/proto
____Loading package: modules/common
____Loading package: modules/planning/reference_line
____Loading package: modules/common/monitor_log
____Loading package: modules/monitor/conf
____Loading package: modules/third_party_perception
____Loading package: 
____Loading package: modules/map
____Loading package: modules/canbus
____Loading package: @proj4//
____Loading package: @tinyxml2//
____Loading package: @google_styleguide//
____Loading package: @glog//
____Loading package: @gtest//
____Loading package: @com_google_protobuf//
____Loading package: @ros//
____Loading package: @cuda//
____Loading package: @eigen//
____Loading package: @qpOASES//
____Loading package: @curlpp//
____Loading package: @yaml_cpp//
____Loading package: @pcl//
____Loading package: @civetweb//
____Loading package: @opencv2//
____Loading package: @ipopt//
____Loading package: @glew//
____Loading package: @glfw//
____Loading package: @vtk//
____Loading package: @caffe//
____Loading package: @opengl//
[INFO] Building on aarch64...
INFO: Reading 'startup' options from /apollo/tools/bazel.rc: --batch_cpu_scheduling
INFO: Found 1637 targets...
......
```

编译成功的输出如下

```txt
============================
[ OK ] Build passed!
[INFO] Took 14540 seconds
============================
```

> *在没有启用swap分区，且apollo.sh脚本中JOB_ARG="--jobs=3 --local_resources 2048,.5,1.0"时，Debug版编译大约需要4个小时；Release版编译大约需要5个小时多。*


> **启用swap分区，且apollo.sh脚本中调整JOB_ARG="--jobs=6 --local_resources 16384,6,1.0"后，Debug版编译大约需要1小时，Release版约1小时50分钟。**