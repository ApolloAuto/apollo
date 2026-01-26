# 包管理安装方式

## 步骤一：安装基础软件

### 1. 安装 Ubuntu Linux

安装 Ubuntu 18.04+ 的步骤，参见 [官方安装指南](https://ubuntu.com/tutorials/install-ubuntu-desktop)。

完成安装后更新相关软件：

```shell
sudo apt-get update
sudo apt-get upgrade
```

> 注意：若要完成更新，需要保证网络连接。

### 2. 安装 Docker Engine

Apollo 依赖于 Docker 19.03+。安装 Docker 引擎，您可以根据官方文档进行安装：

- 参见 [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)。

- 您还可以者通过 Apollo 提供的安装脚本直接安装：

```shell
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
bash docker_install.sh
```

这个过程可能会运行多次脚本，根据脚本提示执行即可。

## 步骤二（可选）：获取 GPU 支持

Apollo某些模块需要GPU的支持才能够编译、运行（例如感知模块），如果您需要使用此类模块，需要安装Nvidia显卡驱动以及Nvidia
container toolkit以获取GPU支持。

> 注意：本教程只适用于ubuntu系统，虚拟机无法安装显卡驱动，wsl请自行上网搜索
> 注意：如果您之前已经安装过Nvidia显卡驱动，即往终端输入`nvidia-smi`能够正常输出，可以跳过`1.安装显卡驱动`小节

### 1.安装驱动

**可参照官网方法安装驱动[官网驱动](https://www.nvidia.cn/drivers/unix/linux-amd64-display-archive/)。**

显卡驱动和CUDA版本兼容性，由于nvidia的硬件更新的很快，因此会遇到显卡驱动和CUDA版本不兼容的情况，以下为我们测试的畅通链路。

| 显卡系列                  | 测试显卡                   | 驱动版本                     | 最低支持驱动版本              | cuda版本             |
| --------------------- | ---------------------- | ------------------------ | --------------------- | ------------------ |
| GeForce 10 Series     | GeForce GTX 1080       | nvidia-driver-470.160.03 | nvidia-driver-391.35  | CUDA Version ：11.4 |
| GeForce RTX 20 Series | GeForce RTX 2070 SUPER | nvidia-driver-470.63.01  | nvidia-driver-456.38  | CUDA Version ：11.4 |
| GeForce RTX 30 Series | GeForce RTX 3090       | nvidia-driver-515.86.01  | nvidia-driver-460.89  | CUDA Version ：11.6 |
| GeForce RTX 30 Series | GeForce RTX 3060       | nvidia-driver-470.63.01  | nvidia-driver-460.89  | CUDA Version ：11.4 |
| Tesla V-Series        | Tesla V100             | nvidia-driver-418.67     | nvidia-driver-410.129 | CUDA Version ：10.1 |
| AMD                   | MI100 dGPU             | ROCm™ 3.10 driver        |                       |                    |

**10、20、30系列显卡推荐使用470.63.01版本的驱动** ，您可以通过Nvidia官网来[下载驱动](https://www.nvidia.cn/Download/driverResults.aspx/179605/cn/)

下载之后，找到相应的文件夹打开终端输入安装指令：

```shell
sudo chmod 777 NVIDIA-Linux-x86_64-470.63.01.run
sudo ./NVIDIA-Linux-x86_64-470.63.01.run
```

安装完毕后，您可以通过nvidia-smi指令来检查驱动是否安装成功，如果一切正常，您可以看到类似以下的提示：

```shell
Tue Jan  3 12:04:21 2023
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 460.91.03    Driver Version: 470.63.01    CUDA Version: 11.4     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  GeForce GTX 1080    Off  | 00000000:01:00.0 Off |                  N/A |
|  0%   38C    P8     7W / 198W |    239MiB /  8118MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|    0   N/A  N/A      2566      G   /usr/lib/xorg/Xorg                 18MiB |
|    0   N/A  N/A      2657      G   /usr/bin/gnome-shell               67MiB |
|    0   N/A  N/A      6104      G   /usr/lib/xorg/Xorg                132MiB |
|    0   N/A  N/A      6234      G   /usr/bin/gnome-shell               13MiB |
|    0   N/A  N/A      7440      G   gnome-control-center                1MiB |
+-----------------------------------------------------------------------------+
```

### 2. 安装 Nvidia container toolkit

为了在容器内获得 GPU 支持，在安装完 docker 后需要安装 NVIDIA Container Toolkit。 运行以下指令安装 NVIDIA Container
Toolkit：

```shell
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get -y update
sudo apt-get install -y nvidia-docker2
```

安装完毕后，需要手动重启下docker：

```shell
sudo systemctl restart docker
```

## 步骤三：安装 Apollo 环境管理工具

Apollo 环境管理工具是一个帮忙管理和启动 Apollo 环境容器的命令行工具。

### 1. 安装依赖软件

```shell
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
```

### 2. 在宿主机添加 Apollo 软件源的 gpg key，并设置好源和更新

```shell
# 添加 gpg key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg

# 设置源并更新
echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core"\
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | \
    sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update
```

> 注：如果之前已经安装过8.0版本的apollo的话，在宿主机上的`/etc/apt/sources.list`文件中会有形如
> `deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main`的配置，可以直接删除，宿主机上的apollo源配置仅用于安
> 装`aem`工具

### 3. 安装aem：

```shell
sudo apt install apollo-neo-env-manager-dev --reinstall
```

安装成功后即可使用

```shell
aem -h
```

## 步骤四：安装示例工程

### 1. 选择示例工程

Apollo 目前提供了3个示例工程，您可以根据需要选择其一

- [application-core](https://github.com/ApolloAuto/application-core) , 包含 Apollo 所有开源软件包，可以基于此工程搭建自己的应用

- [application-pnc](https://github.com/ApolloAuto/application-pnc) , 仅包含规划控制相关的软件包，适合仅关注规划控制方向的用户

- [application-perception](https://github.com/ApolloAuto/application-perception) , 仅包含感知相关的软件包，适合仅关注感知方向的用户

### 2. 克隆工程

以 x86 架构的 application-core 为例

```shell
git clone https://github.com/ApolloAuto/application-core.git application-core
```

### 3. 启动 Apollo 环境容器

```shell
# 先进入工程目录
cd application-core
# 环境设置：识别主机系统是x86_64还是aarch64, 并修改对应的.env和.workspace.json配置
bash setup.sh
# 启动容器
aem start
```

### 4. 进入 Apollo 环境容器

```shell
# 先进入工程目录
cd application-core
# 进入容器
aem enter
```

### 5. 安装软件包

示例工程中包含一个名为 `core` 目录，其中 `core/cyberfile.xml` 文件中描述了工程所依赖软件包，可以通过 buildtool 工具进行依赖包的安装

```shell
buildtool build -p core
```

> 此操作真正含义是编译工程中 `core` 这个包，但 `core` 本身并没有需要编译的代码，所以此操作仅会安装 `core/cyberfile.xml`
> 中声明的依赖包

### 6. 选择车型配置

示例工程中profiles/sample目录是官方提供的基于一个雷达两个摄像头的车型配置，您可以参考profiles目录下的sample编写自己的车型配置，生效车型配置的方法如下：

```shell
# 以sample为例
aem profile use sample
```

### 7. 播放数据包

#### 获取数据包

```shell
wget https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record -P $HOME/.apollo/resources/records/
```

#### 获取数据包对应的地图

```shell
buildtool map get sunnyvale
```

> 从 `9.0.0-rc-r10` 版本开始，地图数据已经被分离出来，需要单独下载，不再随地图包发布；通过 `buildtool map list` 可以查看所有可用的地图

#### 启动 Dreamview+

```shell
aem bootstrap start --plus
```

##### 在 Dreamview+ 中播放数据包

启动 Dreamview+ 之后，在浏览器输入 `localhost:8888` 进入 Dreamview+ 界面，您可以选择默认模式，也可以选择其他模式播放数据包。本小节以默认模式为例。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_7deb2d2.png)

1. 选择 **Default Mode** 。

2. 勾选 **Accept the User Agreement and Privacy Policy/接受用户协议和隐私政策** ，并单击 **Enter this Mode** 进入 **Mode Settings/模式设置** 页面。

3. 在 **Mode Settings/模式设置** 页面，设置播包参数。
   
   - 在 **Operations/操作** 中选择 **Record** 。
   - 在 **Environment Resources/环境资源** 中，单击 **Records/数据包** ，并选择具体想要播放的数据包。
   - 在 **Environment Resources/环境资源** 中，单击 **HDMap/高精地图** ，并选择 **Sunnyvale Big Loop** 。

4. 单击底部区域播放按钮。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_45acc2d.png)
   
   可以在 **Vehicle Visualization/车辆可视化** 中看到数据包播放的画面。

##### 通过命令行播放数据包

1. 进入 docker 环境，

2. 在 Dreamview+ 中 **Resource Manager/资源管理** > **Records/数据包** 中先下载需要的数据包。输入以下命令播放数据包：
   
   ```bash
   cyber_recorder play -f ~/.apollo/resources/records/数据包名称 -l
   ```

> 注意：如果您想要循环播放数据包，添加 -l，如果不循环播放数据包，则不需要添加 -l。

### 8. 安装目录结构说明

至此，Apollo 安装已经完成

整个工程的目录结构如下

```shell
application-core
├── .aem
│   └── envroot
│       ├── apollo          # 会挂载到容器内的 /apollo 目录
│       └── opt             # 会挂载到容器内的 /opt/ 目录，而 Apollo 的软件包会默认安装到 /opt/ 下，因此该目录可以起到缓存的作用
├── core                    # 工程依赖包
│   ├── BUILD
│   └── cyberfile.xml       # 包的描述文件，描述整个工程的所有依赖
├── CPPLINT.cfg
├── data                    # 数据目录，会挂载到 /apollo/data
│   ├── calibration_data    # 标定配置目录，会挂载到 /apollo/modules/calibration/data
│   ├── kv_db.sqlite
│   ├── log                 # 日志目录，会挂载到 /opt/apollo/neo/data/log
│   └── map_data            # 地图目录，会挂载到 /apollo/modules/map/data
├── profiles                # 新版配置目录
│   ├── current -> sample   # 当前启用的配置目录
│   └── sample              # 官方提供的单lidar和两个camera样例车型配置
├── third_party
├── tools -> /opt/apollo/neo/packages/bazel-extend-tools/latest/src
├── .vscode                 # 默认的 vscode 配置
│   ├── c_cpp_properties.json
│   └── settings.json
├── WORKSPACE               # bazel 的配置
└── .workspace.json         # apollo 工程配置，可以在这指定软件包版本
```

接下来，您可以通过实践教程来学习放了解更多 Apollo 的使用方法

- [Apollo规划实践](../应用实践/开发调试教程/Apollo规划实践/综述.md)

- [Apollo感知实践](../应用实践/开发调试教程/Apollo感知实践/综述.md)

## 步骤六：删除工程（可选）

此步骤说明如何删除已安装的工程

### 1. 删除容器

以 `application-core` 工程为例

```shell
# 先进入工程目录
cd application-core
# 删除容器
aem remove
```

### 2. 删除工程

```shell
# 回到上一级目录
cd ..
# 删除工程目录
rm -r application-core
```
