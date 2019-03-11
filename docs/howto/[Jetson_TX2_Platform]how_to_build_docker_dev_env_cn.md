# 构建Docker Dev镜像

通过docker build命令创建Ubuntu 14.04镜像之后，下一步是安装编译Apollo3.0源码所必需的库。通过下面的命令启动Docker镜像：

```shell
docker run -it -v /home/nvidia/work/ApolloAuto/apollo:/apollo apolloauto/dev /bin/bash
```

进入容器之后开始安装和编译Apollo所需的工具和库  

- 更新软件源

```shell
apt-get update -y
export APOLLO_LIB_PATH=/usr/local/apollo
mkdir -p ${APOLLO_LIB_PATH}
cd ${APOLLO_LIB_PATH}
wget https://apollocache.blob.core.windows.net/apollo-cache/jsoncpp.zip
unzip jsoncpp.zip
wget https://apollocache.blob.core.windows.net/apollo-cache/adv_plat.zip
unzip adv_plat.zip
rm -fr jsoncpp.zip adv_plat.zip
```

> *如果执行apt-get update出现提示信息：*  
> *W: GPG error: http://ppa.launchpad.net trusty Release: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 3EB9326A7BF6DFCD*  
> *请执行apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 3EB9326A7BF6DFCD*

- 编译bazel

> *编译bazel需要jdk8,可以从Oracle官方网站下载[ARM64版本的JDK](http://download.oracle.com/otn/java/jdk/8u144-b01/090f390dda5b47b9b721c7dfaa008135/jdk-8u144-linux-arm64-vfp-hflt.tar.gz)*

```shell
tar -xzvf jdk-8u144-linux-arm64-vfp-hflt.tar.gz
mv jdk1.8.0_144/ java
mv java/ /usr/lib/
vi /etc/skel/.bashrc
```

在.bashrc文件末尾追加

```txt
export JAVA_HOME=/usr/lib/java
export JRE_HOME=${JAVA_HOME}/jre
export PATH=${JAVA_HOME}/bin:$PATH
export CLASSPATH=.:${JAVA_HOME}/lib:${JRE_HOME}/lib
```

在github下载[bazel 0.5.3](https://github.com/bazelbuild/bazel/releases/download/0.5.3/bazel-0.5.3-dist.zip)

```shell
unzip bazel-0.5.3-dist.zip -d bazel-0.5.3-dist
cd bazel-0.5.3-dist
export JAVA_HOME=/usr/lib/java
export JRE_HOME=${JAVA_HOME}/jre
export CLASSPATH=.:${JAVA_HOME}/lib:${JRE_HOME}/lib
export PATH=${JAVA_HOME}/bin:$PATH
./compile.sh
cp ./output/bazel /usr/bin/
```

> *编译bazel0.5.3如果出现以下问题*

```txt
ERROR: /root/bazel-0.5.3-dist/third_party/java/jarjar/BUILD:19:1: Building third_party/java/jarjar/libjarjar_core.jar (43 source files) failed: Worker process sent response with exit code: 1.
java.lang.InternalError: Cannot find requested resource bundle for locale en_US
......中间打印省略
Target //src:bazel failed to build
```

> *该问题主要是由jdk版本导致的，可能系统中/usr/bin目录下存在一个新版本的java，先卸载，然后根据上一步的提示安装jdk-8u144及其以下的版本。*

> *如果编译过程中出现如下报错*

```txt
Must specify PROTOC if not bootstrapping from the distribution artifact
```

需要安装protobuf-compiler

```shell
sudo apt install protobuf-compiler
export PROTOC=/usr/bin/protoc
```

> *如果出现Building src/main/java/com/google/devtools/build/lib/libconcurrent.jar (18 source files) failed: Worker process sent response with exit code: 1*

> *需要导出环境变量`export JAVA_OPTS="-server -Xms2048m -Xmx2048m"`*

- 下载组件源码

```shell
cd /home/tmp
wget -O googletest-release-1.8.0.tar.gz https://github.com/google/googletest/archive/release-1.8.0.tar.gz
wget -O benchmark-1.1.0.tar.gz https://github.com/google/benchmark/archive/v1.1.0.tar.gz
wget -O eigen-3.2.10.tar.gz https://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz
wget -O civetweb-1.10.tar.gz https://github.com/civetweb/civetweb/archive/v1.10.tar.gz
wget -O curlpp-0.8.1.tar.gz https://github.com/jpbarrette/curlpp/archive/v0.8.1.tar.gz
wget -O yaml-cpp-0.5.3.zip https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.5.3.zip
wget -O qp-oases-3.2.1-1.zip https://github.com/startcode/qp-oases/archive/v3.2.1-1.zip
wget -O proj.4-4.9.3.zip https://github.com/OSGeo/proj.4/archive/4.9.3.zip
wget -O tinyxml2-5.0.1.zip https://github.com/leethomason/tinyxml2/archive/5.0.1.zip
wget -O protobuf-3.3.0.tar.gz https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
wget -O opencv-2.4.13.2.zip https://github.com/opencv/opencv/archive/2.4.13.2.zip
git clone https://github.com/OpenNI/OpenNI.git
git clone https://github.com/occipital/OpenNI2.git
```

- 安装openblas

```shell
wget http://123.57.58.164/apollo-docker_no/openblas-0.2.18.tar.gz
tar -xzvf openblas-0.2.18.tar.gz 
sudo mkdir /usr/include/openblas
sudo cp openblas-0.2.18/include/* /usr/include/openblas/
sudo cp -r openblas-0.2.18/lib/* /usr/lib/
```

- 编译gtest

```shell
wget https://github.com/google/googletest/archive/release-1.8.0.tar.gz
tar -xzvf release-1.8.0.tar.gz 
cd googletest-release-1.8.0
cmake CMakeLists.txt
make
sudo make install
```

- 编译gflags

```shell
wget -O gflags-2.2.0.tar.gz https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
tar -xzvf gflags-2.2.0.tar.gz
cd gflags-2.2.0
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DGFLAGS_NAMESPACE=google -G"Unix Makefiles" .
make
sudo make install
ldconfig
```

- 编译glog

```shell
git clone https://github.com/google/glog.git
cd glog
./autogen.sh
./configure --enable-shared
make
sudo make install
```

- 编译glfw

```shell
wget https://github.com/glfw/glfw/releases/download/3.0/glfw-3.0.zip
unzip glfw-3.0.zip
cd glfw-3.0
mkdir build
cd build/
cmake -DBUILD_SHARED_LIBS=ON ../
make -j4
sudo make install
```

> *如果编译过程中报错：`The RandR library and headers were not found`，表示没有安装libxrandr-dev库，需要执行`sudo apt-get install libxrandr-dev`安装*

> *如果编译过程中报错：`The Xinerama library and headers were not found`，请执行`sudo apt-get install libsdl2-2.0-0 libsdl2-dev`安装*

- 编译glew

```shell
wget https://github.com/nigels-com/glew/releases/download/glew-2.0.0/glew-2.0.0.zip
unzip glew-2.0.0.zip
pushd glew-2.0.0
make -j8
sudo make install
popd
```

- 编译安装gperftools

```shell
#wget http://download.savannah.gnu.org/releases/libunwind/libunwind-0.99-beta.tar.gz
wget https://github.com/gperftools/gperftools/releases/download/gperftools-2.7/gperftools-2.7.tar.gz
tar -xzvf gperftools-2.7.tar.gz
cd gperftools-2.7
./configure
make
sudo make install
cd ..
wget http://123.57.58.164/apollo-docker_no/google-perftools-2.1.90.tar.gz
tar -xzvf google-perftools-2.1.90.tar.gz 
sudo cp -d google-perftools-2.1.90/lib/* /usr/lib/
ls /usr/lib/libtcm*  
```

- 编译安装ipopt

```shell
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip -O Ipopt-3.12.8.zip
unzip Ipopt-3.12.8.zip
pushd Ipopt-3.12.8/ThirdParty/Mumps
bash get.Mumps
popd
pushd Ipopt-3.12.8
./configure --build=arm-linux
make -j8 all
sudo make install
sudo mkdir -p /usr/local/ipopt
sudo cp -r include /usr/local/ipopt/ && cp -r lib /usr/local/ipopt/
popd
```

- 安装libjsonrpc-cpp

```shell
wget https://apollocache.blob.core.windows.net/apollo-cache/libjsonrpc-cpp.tar.gz
tar -xzvf libjsonrpc-cpp.tar.gz 
sudo mv libjsonrpc-cpp/bin/* /usr/local/bin/
sudo mv libjsonrpc-cpp/include/* /usr/local/include/
sudo mv libjsonrpc-cpp/lib/* /usr/local/lib/
rm -fr libjsonrpc-cpp 
sudo cd /usr/local/lib/
sudo rm -rf libargtable2.so libargtable2.so.0 libjsonrpccpp-client.so libjsonrpccpp-client.so.0 libjsonrpccpp-common.so libjsonrpccpp-common.so.0 libjsonrpccpp-server.so libjsonrpccpp-server.so.0 libjsonrpccpp-stub.so libjsonrpccpp-stub.so.0 libmicrohttpd.so libmicrohttpd.so.12
sudo ln -s libargtable2.so.0.1.7 libargtable2.so.0
sudo ln -s libargtable2.so.0 libargtable2.so
sudo ln -s libjsonrpccpp-client.so.0.7.0  libjsonrpccpp-client.so.0
sudo ln -s libjsonrpccpp-client.so.0 libjsonrpccpp-client.so
sudo ln -s libjsonrpccpp-common.so.0.7.0 libjsonrpccpp-common.so.0
sudo ln -s libjsonrpccpp-common.so.0 libjsonrpccpp-common.so
sudo ln -s libjsonrpccpp-server.so.0.7.0 libjsonrpccpp-server.so.0
sudo ln -s libjsonrpccpp-server.so.0 libjsonrpccpp-server.so
sudo ln -s libjsonrpccpp-stub.so.0.7.0 libjsonrpccpp-stub.so.0
sudo ln -s libjsonrpccpp-stub.so.0 libjsonrpccpp-stub.so
sudo ln -s libmicrohttpd.so.12.40.0 libmicrohttpd.so.12
sudo ln -s libmicrohttpd.so.12 libmicrohttpd.so
```

- 编译安装nlopt

```shell
wget http://ab-initio.mit.edu/nlopt/nlopt-2.4.2.tar.gz
tar -xzvf nlopt-2.4.2.tar.gz 
pushd nlopt-2.4.2
./configure --enable-shared
make -j8
sudo make install
popd
```

- 编译安装protobuf

```shell
wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
tar -xzvf protobuf-cpp-3.3.0.tar.gz 
pushd protobuf-3.3.0
./configure --prefix=/usr
make -j8
sudo make install
popd
```

- 安装ota_security

```shell
wget https://apollocache.blob.core.windows.net/apollo-docker/ota_security.tar.gz
tar -xzvf ota_security.tar.gz 
pushd ota_security
sudo bash ota_server_deploy.sh root
popd
```

- 安装Python模块

```shell
sudo pip install -r py27_requirements.txt 
```

py27_requirements.txt文件的内容如下：
```txt
# To use these python tools, you need to install dependencies with command:
#    sudo apt-get install python-numpy python-scipy python-matplotlib
#    sudo pip install -r py27_requirements.txt

# system utils
supervisor

# Google infras.
glog
protobuf == 3.1
python-gflags

# Web utils.
flask
flask-cors
requests >= 2.18
simplejson

# ROS env.
PyYAML
catkin_pkg
rospkg

# Python tools
plyvel == 0.9
pyproj
shapely

```

- 安装PCL库

```shell
wget http://123.57.58.164/apollo-docker/pcl-1.7_aarch64.tar.gz
tar -xzvf pcl-1.7_aarch64.tar.gz 
sudo mkdir /usr/local/include/pcl-1.7
sudo mv pcl-1.7_aarch64/include/pcl /usr/local/include/pcl-1.7
sudo mv pcl-1.7_aarch64/lib/* /usr/local/lib/
sudo mv pcl-1.7_aarch64/share/* /usr/local/share/
```

- 编译安装qp-oases

```shell
wget https://github.com/startcode/qp-oases/archive/v3.2.1-1.tar.gz
tar -xzvf v3.2.1-1.tar.gz 
pushd qp-oases-3.2.1-1
mkdir bin
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE  -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE -D__NO_COPYRIGHT__"
sudo cp bin/libqpOASES.so /usr/local/lib
sudo cp -r include/* /usr/local/include
popd
```

- 安装Caffe

安装Caffe使用预先构建好的二进制包caffe_aarch64.tar.gz，可以从github上[下载](https://github.com/apolloauto/apollo/releases/download/caffe_aarch64.tar.gz)

```shell
tar -xzvf caffe_aarch64.tar.gz
sudo mv caffe_aarch64/lib* /usr/lib/aarch64-linux-gnu/
sudo mv caffe_aarch64/caffe/ /usr/include/
```

- 安装node.js

```shell
wget https://nodejs.org/dist/v8.12.0/node-v8.12.0-linux-arm64.tar.xz
xz -d node-v8.12.0-linux-arm64.tar.xz
tar -xvf node-v8.12.0-linux-arm64.tar
cd node-v8.12.0-linux-arm64/bin/
sudo cp node /usr/local/bin/
cd ../ && sudo cp -R lib /usr/local/
cd include/ && sudo cp -R node/ /usr/local/include/
cd ../share/ && sudo cp -R * /usr/share/
cd /usr/local/bin
sudo ln -s ../lib/node_modules/npm/bin/npm-cli.js npm
sudo ln -s ../lib/node_modules/npm/bin/npx-cli.js npx
```

- 安装snowboy

```shell
wget http://downloads.sourceforge.net/swig/swig-3.0.10.tar.gz
tar -xzvf swig-3.0.10.tar.gz 
cd swig-3.0.10
./configure --prefix=/usr --without-clisp --without-maximum-compile-warnings
make
sudo make install
sudo install -v -m755 -d /usr/share/doc
sudo install -v -m755 -d /usr/share/doc/swig-3.0.10
sudo cp -v -R Doc/* /usr/share/doc/swig-3.0.10/
cd ..
git clone https://github.com/Kitt-AI/snowboy.git
cd snowboy/
sudo npm install 
sudo ./node_modules/node-pre-gyp/bin/node-pre-gyp clean configure build
```

- 安装ROS编译环境

```shell
apt-get install -y --only-upgrade libstdc++
wget http://123.57.58.164/apollo-docker_no/ros-indigo-catkin-0.6.19.tar.gz
tar -xzvf ros-indigo-catkin-0.6.19.tar.gz 
cp -R ros /opt/
echo 'ROSCONSOLE_FORMAT=${file}:${line} ${function}() [${severity}] [${time}]: ${message}' >> /etc/profile
vi /etc/skel/.bashrc
```

文件末尾追加

```txt
export ROS_ROOT=/home/tmp/ros/share/ros
export ROS_PACKAGE_PATH=/home/tmp/ros/share:/home/tmp/ros/stacks
export ROS_MASTER_URI=http://localhost:11311
export ROS_ETC_DIR=/home/tmp/ros/etc/ros
```

- 安装undistort

```shell
wget https://apollocache.blob.core.windows.net/apollo-docker/undistort.tar.gz
tar -xzvf undistort.tar.gz 
mv undistort /usr/local/apollo/
```

- 添加新用户apollo

```shell
export USER_NAME=apollo
sudo adduser --disabled-password --gecos '' ${USER_NAME}
usermod -aG sudo ${USER_NAME}
sudo echo '%sudo ALL=(ALL:ALL) NOPASSWD:ALL' >> /etc/sudoers
sudo echo """
ulimit -c unlimited
source /apollo/scripts/apollo_base.sh
""" >> /home/${USER_NAME}/.bashrc
```

- 安装CUDA

```shell
dpkg -i cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb
sudo apt-get update -y
sudo apt-get -y install cuda-toolkit-8-0
dpkg -i libcudnn6_6.0.21-1+cuda8.0_arm64.deb
dpkg -i libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb
```

- 安装supervisor

```shell
wget https://pypi.python.org/packages/source/s/supervisor/supervisor-3.1.3.tar.gz
tar -zxf supervisor-3.1.3.tar.gz
cd supervisor
python setup.py install
```

# 提交镜像

在容器中安装完成上述工具和库之后，输入exit退出容器并将修改提交到本地仓库

```shell
sudo docker commit e4dde2164687 apolloauto/dev:dev-aarch64-2018
docker images #查看是否提交成功
```

> *容器ID是e4dde2164687，当处于Docker容器的shell中时此值可以从命令提示符中找到，例如`root@e4dde2164687:/apollo# `中root@后面的主机名称名称*
