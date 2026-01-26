# Apollo 研发工具 - aem

## 综述

`aem` 是一个命令行工具(全称 `apollo environment manager`)，提供管理 Apollo 容器的能力。

## 安装

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

看到类似如下输出即代表安装成功

```shell

Usage:
    aem [OPTION]

Options:
    stopall : stop all apollo development container.
    enter : enter into the apollo development container.
    profile : profiles management
    remove : remove the apollo development container.
    install : install source code of specified package to workspace.
    start : start the created container.
    bootstrap : run dreamview and monitor module.
    start_gpu : start a created gpu container.
    init : init single workspace.
    build : build package in workspace.
    create : create a new container with apollo development image.
    setup_host : setup host
    list : list all apollo development container.
```

## 快速入门

本段内容主要介绍`aem`工具使用流程及规范。

### 1.创建挂载文件夹
Apollo需要在容器内编译和运行，为了在宿主机上进行开发和代码管理，在进入容器前，需要提前创建挂载文件夹
mkdir workspace &&  cd workspace (文件夹名称无特别要求，按开发者喜好设置即可)

### 2.启动一个Apollo容器

使用以下命令自动拉取镜像和启动容器：
aem start
以上命令适合第一次进入容器的时候使用，假如您已经启动过了 Apollo 环境容器，该命令会尝试重启之前存在的 Apollo 环境容器
如果您想强制删除之前的容器并启动全新的容器，请使用以下命令：
aem start -f
程序输入如下表示容器启动成功：
```shell
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_neo_dev_apollo container, please run the following command:
[ OK ]   aem enter
[ OK ] Enjoy!
```

### 3.进入容器

容器启动后，可以通过以下命令进入容器：
aem enter
进入容器后，会进入到apollo_workspace目录下，此目录常作为apollo的工作空间，之后的操作都位于此目录下

### 4.初始化工作空间
在容器内执行aem init,将当前目录初始化为apollo的工作空间,初始化完毕后，当前目录下会多出一个 WORKSPACE 文件

### 5.下载安装源码
初始化工作空间后，仅是生成了一个空的工作空间，里面没有任何源码，因此，要对模块进行二次开发，还需要通过以下命令下载模块的源码及依赖：
aem install xx(模块名称)
下载模块源码,下载完毕后，工作空间会自动创建一些文件夹，其中，xx源码位于工作空间的 module/xx 路径下

### 6.编译安装源码
修改模块源码后，需要对修改厚的模块进行编译安装
aem build -p modules/xx

### 7.运行和调试
编译安装成功后，启动dreamview进行调试
aem bootstrap start

注：首次启动需下载dreamview和monitor模块 
aem install dreamview-plus monitor

### 8.停止容器
在宿主机执行aem stopall可停止Apollo容器，停止后的容器可通过aem enter再次启动

### 9.删除容器
在宿主机执行aem remove可删除Apollo容器，容器删除后，容器内挂载的工程配置依然会存在于宿主机workspace目录中。再次执行aem start启动命令后，当前目录下所有配置会重新挂载到新容器中。如不需要挂载当前配置，请先手动删除。

## 使用说明

### 子命令

类似于 git 或 apt 等常见的命令行工具，aem 的功能被组织成子命令，如 start 负责启动一个容器， enter 负责进入一个已启动的容器等等。
子命令可能需要一些参数，你可以在子命令后输入-h、-help来查看详细的参数。

### start, start_gpu

该子命令启动一个Apollo 环境容器。该命令将检查apollo容器是否已经被启动，如果容器已经被启动或停止，它将使用已经启动的容器；如果容器从未被启动，它将启动一个新容器。

#### 用法

##### 启动一个容器:

```shell
aem start
```

##### 启动一个支持 gpu 的容器，你可以在其中构建或运行需要 gpu 支持的模块:

```
aem start_gpu
```

> 注意：必须正确安装NVIDIA图形驱动和nvidia container toolkit。

##### 使用本地镜像启动容器:

```shell
aem start_gpu -l
```

##### 强制启动一个新容器:

```shell
aem start_gpu -f
```

> 注意：这个命令将删除容器，但会保留之前在容器内容安装过的 Apollo 模块，若想完全删除，请使用 `remove` 命令。

##### 为启动的容器指定名字:

```shell
aem start_gpu -n apollo_container
```

> 如果不指定容器名，会默认使用当前用户名作为容器名

> start命令会读取当前路径的.env文件，该文件指定了容器的名称以及使用的镜像等，例如.env文件内容为：
>   APOLLO_ENV_NAME=pnc
>   APOLLO_ENV_WORKLOCAL=1
>   APOLLO_ENV_CONTAINER_REPO='registry.baidubce.com/apollo/apollo-env-gpu'
>   APOLLO_ENV_CONTAINER_TAG='9.0-latest'
> aem会使用registry.baidubce.com/apollo/apollo-env-gpu:9.0-latest镜像来启动一个名叫apollo_neo_pnc的容器
> 当前路径没有.env文件时，会使用默认镜像（8.0镜像）启动一个名叫apollo_neo_${USER}的容器

##### 指定挂载点:

```shell
aem start_gpu -m /home/apollo/workspace:/apollo_workspace
```

##### 详细参数

```shell
Usage: aem start [options] ...
OPTIONS:
    -h, --help                    Display this help and exit.
    -f, --force                   force to restart the container.
    -n, --name                    specify container name to start a container.
    -m, --mount                   specify the mount point in container, such as /home/apollo/workspace:/apollo_workspace
    -g, --geo <us|cn|none>        Pull docker image from geolocation specific registry mirror.
    -t, --tag <TAG>               Specify docker image with tag <TAG> to start.
    -c, --cross-platform <arch>   Run a cross-platform image
    -y                            Agree to Apollo License Agreement non-interactively.
    --shm-size <bytes>            Size of /dev/shm . Passed directly to "docker run"
    --gpu                         Use gpu image instead of cpu image.
```

### enter

这个子命令用来进入 Apollo 环境容器.

#### 用法

##### 进入容器

```shell
aem enter
```

##### 指定容器名称

```shell
aem enter -n apollo_container
```

##### 以指定用户进入容器

```shell
aem enter -n apollo_container --user root
```

> 该命令与start命令一致，都会尝试读取当前路径下的.env文件，根据文件中定义的APOLLO_ENV_NAME，进入相应名称的容器
> 假如当前路径没有.env文件，aem会尝试进入名叫apollo_neo_${USER}的容器

##### 详细参数

```shell
Usage: aem enter [options] ...
OPTIONS:
    -h, --help                    Display this help and exit.
    -n, --name                    specify container name to enter.
    --user                        specify container user to enter
```

### remove

这个命令用于删除环境容器

#### 用法

##### 删除指定名字的容器

```shell
# 删除名字为 `pnc` 的容器
aem remove --name pnc
```

> 该命令与start命令一致，都会尝试读取当前路径下的.env文件，根据文件中定义的APOLLO_ENV_NAME，删除相应名称的容器
> 假如当前路径没有.env文件，aem会尝试删除名叫apollo_neo_${USER}的容器

##### 详细参数

```shell
Usage: aem remove [options] ...
OPTIONS:
    -h, --help                    Display this help and exit.
    -n, --name                    specify container name to remove.
    -w, --workspace               specify container workspace.
    --worklocal                   env data is store in workspace
```
注意：开发者如要删除宿主机中的工程目录,需先执行aem remove删除容器后方可再删除工程，否则会造成依赖丢失的问题。

### bootstrap

这个子命令用来启动`dreamview 1.0`或`dreamview 2.0`

#### 用法

##### 在容器内启动 dreamview 1.0

```shell
aem bootstrap start
```

##### 在容器内启动 dreamview 2.0

```shell
aem bootstrap start --plus
```

##### 停止目前容器内已经启动的 dreamview 1.0 或 dreamview 2.0

```shell
aem bootstrap stop
```

##### 详细参数

```shell
Usage: aem bootstrap [options] [argument] ...
OPTIONS:
    start --plus                    Start dreamview 2.0.
    start                           Start dreamview.
    stop                            Stop dreamview or dreamview 2.0.
```

### list

这个子命令会列出所有由aem工具启动的容器

#### 用法

```shell
aem list
```

上述命令会有以下输出

```
[INFO] Environment containers:
apollo_neo_dev /home/apollo/sample/.aem
```

表示当前aem启动了一个容器，名称叫`apollo_neo_dev`，容器工程路径位于`/home/apollo/sample`下

### profile

这个子命令用于选择使用的配置文件

#### 用法

以下命令列出工程下有什么可选的配置文件集

```shell
aem profile list
```

上述命令会有以下输出

```
sample
```

表示当前可以使用名叫sample的配置文件

以下命令使用名叫sample的配置文件集

```shell
aem profile use sample
```

### build

这个子命令是 buildtool build action 的一个简单包装。详细用法请参考 [buildtool 文档](../user_guidelines/tool_buildtool_cn.md)。

### install

这个子命令是 buildtool install action的一个简单包装。详细用法请参考 [buildtool 文档](../user_guidelines/tool_buildtool_cn.md)。

### init

这个子命令是 buildtool init action的一个简单包装。详细用法请参考 [buildtool 文档](../user_guidelines/tool_buildtool_cn.md)。

### 环境变量

aem 在执行子命令的时候会使用到一些环境变量，部分不支持参数控制的功能也可以通用修改这些环境变量来实现，aem 运行时，如果当前目录下有 `.env` 文件，则会加载该文件

| 变量                        | 说明                                                                                                        |
| --------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `APOLLO_ENVS_ROOT`          | 容器相关目录创建位置，默认为 `$HOME/.aem/envs`                                                              |
| `APOLLO_ENV_NAME`           | 容器名，默认是当前用户的用户名                                                                              |
| `APOLLO_ENV_WORKROOT`       | 容器内的工作目录路径，默认是 `/apollo_workspace`                                                            |
| `APOLLO_ENV_WORKLOCAL`      | 创建容器相关的目录是存放在当前目录下，`1` 为是 `0` 为否，否，则创建在 `$APOLLO_ENVS_ROOT` 下                |
| `APOLLO_ENV_CONTAINER_REPO` | 创建容器使用的镜像 repo ，默认为 `registry.baidubce.com/apollo/apollo-env-gpu` 可以通过这个修为改自定义镜像 |
| `APOLLO_ENV_CONTAINER_TAG`  | 创建容器使用的镜像 tag ，默认为 `latest`                                                                    |
