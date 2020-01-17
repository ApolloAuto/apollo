# Apollo简介

Apollo已经开始为汽车和自主驾驶行业的合作伙伴提供开放，全面，可靠的软件平台。合作伙伴可以使用Apollo软件平台和通过Apollo认证的参考硬件模板来定制自己的自主车辆研发。

# Apollo软件安装指南

本节包括：

- 下载Apollo发行包
- 设置Docker支持
- 自定义你的发布容器

在开始之前，请确保您已经按照[Apollo 1.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md#installing-the-software-for-the-ipc)中的步骤安装了Ubuntu Linux 14.04.3和Apollo Kernel。

## 下载Apollo源代码

1. 从[github source](https://github.com/ApolloAuto/apollo/)下载Apollo的源代码：

    ```
    git clone git@github.com:ApolloAuto/apollo.git
    cd apollo
    git checkout [release_branch_name]
    ```

2. 参考以下命令设置环境变量 `APOLLO_HOME`:

    ```
    echo "export APOLLO_HOME=$(pwd)" >> ~/.bashrc && source ~/.bashrc
    ```

3. 在一个新的终端或者已有的终端中输入`source ~/.bashrc`

![tip](images/tip_icon.png) 在以下部分中，假设Apollo目录位于 `$APOLLO_HOME`.

## 设置Docker支持

Docker容器是设置Apollo构建环境的最简单方法。

有关更多信息，请参阅Docker详细教程 [here](https://docs.docker.com/).

1. 请参考[Ubuntu安装docker-ce指南](https://docs.docker.com/install/linux/docker-ce/ubuntu)
以及[Linux安装后续](https://docs.docker.com/install/linux/linux-postinstall).

2. 安装完成后，注销并重新登录系统以启用Docker。

3. （可选）如果您已经安装了Docker（在安装Apollo内核之前），请在其中添加以下行 `/etc/default/docker`:

    ```
    DOCKER_OPTS = "-s overlay"
    ```
4. 安装最新的 [nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-docker).

## 使用你的Release Container

1. 通过运行以下命令下载并启动Apollo 发布的 Docker映像：

    ```
    cd $APOLLO_HOME
    bash docker/scripts/release_start.sh
    ```

2. （可选）如果你需要定制化你的Docker映像，通过运行以下命令登录你已下载的 Docker映像：

    ```
    bash docker/scripts/release_into.sh
    ```

3. （该步骤只用于车上设置。如果是在docker release container里线下实验，请跳过此布）通过修改文件中的以下行来设置全球导航卫星系统（GNSS）驱动程序的区域编号 `./modules/drivers/gnss/conf/gnss_conf.pb.txt`.

    ```
    proj4_text: "+proj=utm +zone=10 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"
    ```

    你只需修改上面一行的`+zone=10`的值即可。请参考[Apollo's Coordinate System](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/coordination.pdf) 找到您当地的区号。例如，如果你在中国北京，你必须设置`+zone=50`。

5. （该步骤只用于车上设置。如果是在docker release container里线下实验，请跳过此步）通过修改以下文件，为GNSS驱动程序设置实时运动（RTK）基站：
   `./modules/drivers/gnss/conf/gnss_conf.pb.txt`

   有关典型的RTK设置，请参阅以下示例：

    ```
    rtk_from {
	    format: RTCM_V3
		    ntrip {
		    address: <provide your own value>
		    port: <provide your own value>
		    mount_point: <provide your own value>
		    user: <provide your own username>
		    password: <provide your own password>
		    timeout_s: <provide your own value, e.g., 5>
	    }
    }
    rtk_to {
	    format: RTCM_V3
	    serial {
		    device: <provide your own value, e.g., "/dev/ttyUSB1">
		    baud_rate: <provide your own value, e.g., 115200>
	    }
    }
    ```

    `rtk_from` 用于RTK基站信息。

    `rtk_to` 用于将RTK差分数据发送到接收器。

6. （该步骤只用于车上设置。如果是在docker release container里线下实验，请跳过此步）添加ESD CAN支持

    请参考 [ESD CAN README](https://github.com/ApolloAuto/apollo/blob/master/third_party/can_card_library/esd_can/README.md)来设置ESD CAN库。

7.  （如果你没有修改过本地的Docker release container里的配置，可跳过此步）按照以下步骤保存你的本地环境：

    ```
    # EXIT OUT OF DOCKER ENV
    # commit your docker local changes to local docker image.
    exit # exit from docker environment
    cd $APOLLO_HOME
    bash docker/scripts/release_commit.sh
    ```
