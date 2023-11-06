# 必备软件安装指南

## 步骤一：安装基础软件

### 1. 安装 Ubuntu Linux

安装 Ubuntu 18.04+ 的步骤，参见 [官方安装指南](https://ubuntu.com/tutorials/install-ubuntu-desktop)。

> 注意：虽然其他发布版本的 Linux 也可能没问题，但我们只在
> [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.5/) 中测试过 Apollo。因此，推荐您使用 Ubuntu
> 18.04.5 作为主机的操作系统。

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

## 步骤二：安装 Apollo 环境管理工具

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

## 步骤三（可选）：获取 GPU 支持

Apollo某些模块需要GPU的支持才能够编译、运行（例如感知模块），如果您需要使用此类模块，需要安装Nvidia显卡驱动以及Nvidia
container toolkit以获取GPU支持。

### 1. 安装显卡驱动

通过以下指令来安装Nvidia显卡驱动：

```shell
sudo apt-get update
sudo apt-add-repository multiverse
sudo apt-get update
sudo apt-get install nvidia-driver-455
```

安装完毕后，您可以通过nvidia-smi指令来检查驱动是否安装成功，如果一切正常，您可以看到类似以下的提示：

```shell
Tue Jan  3 12:04:21 2023
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 460.91.03    Driver Version: 460.91.03    CUDA Version: 11.2     |
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

```
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

## 步骤四：创建和进入 Apollo 环境容器

### 1. 创建工作空间

创建并进入目录

```shell
mkdir application-pnc
cd application-pnc
```

### 2. 启动 apollo 环境容器

```shell
aem start
```

如果一切正常，将会见到类似输出的提示：

```text
[ OK ] apollo_neo_dev_pnc is still running, please run the following command:
[ OK ]     aem enter
[ OK ] Enjoy!
```

### 3. 进入 apollo 环境容器

```shell
aem enter
```

脚本执行成功后，将显示以下信息，您将进入 Apollo 的运行容器：

```text
user_name@in-dev-docker:/apollo_workspace$
```

> 注：工作空间文件夹将被挂载到容器的 /apollo_workspace 中。

### 4. 初始化工作空间

```shell
aem init
```

至此 Apollo 环境管理工具及容器已经安装完成，接下来请跟着 [快速开始](docs/installation_instructions/quick_start_cn.md) 文档根据不同的使用场景按需安
装不同的模块。
