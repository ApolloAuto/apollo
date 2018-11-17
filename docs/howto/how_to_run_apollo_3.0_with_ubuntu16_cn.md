# 简介

Apollo官方推荐的Ubuntu版本是14.04，本文档会介绍在Ubuntu16.04上部署及运行Apollo的步骤、可能遇到的问题及解决办法。

本文档基于的Apollo版本为最新发布的3.0，并专注于介绍软件的安装步骤，且会严格遵循Apollo官方提供的安装步骤进行操作。

本文档中介绍的安装步骤全部基于X86进行测试。

# 从Github下载Apollo

请参考文档[Apollo软件安装指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide_cn.md)，将Apollo3.0版本的源代码下载到设备上。

# Docker的安装

请参考Docker官方网站提供的[安装教程](https://docs.docker.com/install/linux/docker-ce/ubuntu)及安装后其他必须的[操作步骤介绍](https://docs.docker.com/install/linux/linux-postinstall)

在Ubuntu16上通过Docker仓库安装通常不会有什么问题，但是如果通过安装包进行安装可能会遇到以下的几个问题。

### 使用Docker安装包

首先，从[Docker安装包网站](https://download.docker.com/linux/ubuntu/dists/)下载对应版本的`Docker`安装包。我们推荐安装的版本是18.03.1。在文件夹`pool/stable/`可以找到不同平台和不同版本的安装包。

在Ubuntu16上执行下述指令安装`Docker`：

```sudo dpkg –i docker_packagename.deb```

可能会弹出下述错误：
```
dpkg: dependency problems prevent configuration of docker-ce:
docker-ce depends on libsystemd-journal0 (>= 201); however:
  Package libsystemd-journal0 is not installed.
  ```
  
该错误表示`Docker-ce`依赖于`libsystemd-journal0`，但`libsystemd-journal0`没有安装并且从已配置的资源库中无法搜索到`libsystemd-journal0`，所以无法自动完成安装。

请浏览[网站](https://ubuntu.pkgs.org/14.04/ubuntu-main-amd64/libsystemd-journal0_204-5ubuntu20_amd64.deb.html)下载`libsystemd-journal0`。

下载后执行下述指令进行安装：

```sudo dpkg –i libsystemd-journal0_packagename.deb ```

可能会弹出下述错误：
```
dpkg: dependency problems prevent configuration of libsystemd-journal0:amd64:
 libsystemd-journal0:amd64 depends on libgcrypt11 (>= 1.5.1); however:
  Package libgcrypt11 is not installed.
  ```
该错误表示`libsystemd-journal0`依赖于`libgcrypt11`，但`libgcrypt11`没有安装并且从已配置的资源库中无法搜索到`libgcrypt11`，所以无法自动完成安装。
请浏览[网站](https://ubuntu.pkgs.org/14.04/ubuntu-main-amd64/libgcrypt11_1.5.3-2ubuntu4_amd64.deb.html)下载`libgcrypt11`。

下载后执行下述指令进行安装：

```sudo dpkg –i libgcrypt11_packagename.deb ```

然后依次安装`libsystemd-journal0`和`Docker-ce`。

安装成功后使用指令```docker –version```确认输出的版本信息为：
```
Docker version 18.03.1-ce, build 9ee9f40
```

### 验证Docker是否安装成功
按照Apollo官方提供的验证方法，执行指令：

```sudo docker run hello-world	```

可能弹出下述错误：
```
docker: Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?.
See 'docker run --help'.
```
该错误表示`Docker`服务没有运行，执行下述指令启动`Docker`服务：

```sudo service docker restart```

再次执行指令```sudo docker run hello-world```进行验证。

# 编译Apollo

请参考文档[How to Build and Release your Docker Container](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)。

# 启动Apollo及进行演示

请参考文档[How to Launch and Run Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_launch_Apollo.md)。
