# 快速开始

前提条件

阅读使用该文档前请确保已经参考 [必要软件安装指南](docs/installation_instructions/essential_software_installation_guide_cn.md) 文档 完成 Apollo 环境管理工具以及容器环境的安装

## 创建工程和安装 Apollo 软件包，并使用 Dreamview 展示播放的数据包

本场景描述如何使用 Dreamview 播放数据包，为开发者熟悉 Apollo 平台打下基础。您可以通过 Dreamview 播放 Apollo 提供的
record 数据包，进一步观察、学习 Apollo 自动驾驶。

### 步骤一：进入 Apollo Docker 环境

#### 1. 创建工作空间：

> 本文所使用的工程样例可从 [github 仓库](https://github.com/ApolloAuto/application-pnc)中获取

```shell
mkdir application-pnc
cd application-pnc
```

#### 2. 配置环境

```shell
cat > .env <<EOF
APOLLO_ENV_NAME=pnc
APOLLO_ENV_WORKLOCAL=1
APOLLO_ENV_CONTAINER_REPO='registry.baidubce.com/apollo/apollo-env-gpu'
APOLLO_ENV_CONTAINER_TAG='9.0-latest'
EOF
```

> 配置说明:
>
> `APOLLO_ENV_NAME=pnc` 创建命名为 `pnc` 的环境容器
>
> `APOLLO_ENV_WORKLOCAL=1` 使用工作目录下的 `.aem/envroot` 目录创建与容器目录绑定的卷（此功能用于缓存在容器中安装过的软
> 件包）
>
> `APOLLO_ENV_CONTAINER_REPO='registry.baidubce.com/apollo/apollo-env-gpu'` 指定启动容器所使用的映像
>
> `APOLLO_ENV_CONTAINER_TAG='9.0-latest'` 指定所使用映像的标签

#### 3. 启动 Apollo 环境容器：

```shell
aem start
```

#### 4. 进入 Apollo 环境容器：

```shell
aem enter
```

#### 5. 初始化工作空间：

```shell
aem init
```

### 步骤二：添加和安装依赖

#### 1. 创建用于管理依赖的包

```shell
mkdir core

# 创建包的编译规则文件，
cat > core/BUILD <<EOF
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_package()
EOF

# 创建包的描述文件
cat > core/cyberfile.xml <<EOF
<package>
  <name>core</name>
  <version>local</version>
  <description>
      depends of apollo core
  </description>
  <maintainer email="apollo-support@baidu.com">Apollo Maintainer</maintainer>
  <type>module</type>
  <src_path>//core</src_path>
  <license>Apache License 2.0</license>
  <author>Apollo</author>

  <!-- basic -->
  <depend repo_name="cyber" type="binary">cyber</depend>
  <depend repo_name="tools" type="binary">tools</depend>
  <depend>bazel-extend-tools</depend>

  <!-- dreamview && monitor -->
  <depend repo_name="dreamview" type="binary">dreamview</depend>
  <depend repo_name="monitor" type="binary">monitor</depend>

  <builder>bazel</builder>

</package>
EOF
```

#### 2. 安装依赖包

> 此操作真正含义是编译工程中 `core` 这个包，但 `core` 本身并没有需要编译的代码，所以此操作仅会安装 `core/cyberfile.xml`
> 中声明的依赖包

```shell
buildtool build -p core
```

### 步骤三：启动 Dreamview

```shell
aem bootstrap start
```

### 步骤四：下载 Apollo 的演示包

```shell
wget https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record
```

### 步骤五：播放 Apollo 的演示包

```shell
cyber_recorder play -f demo_3.5.record --loop
```

> 注意：选项 `--loop` 用于设置循环回放模式。

### 步骤六：使用 DreamView 查看数据包

在浏览器中输入http://localhost:8888，访问 Apollo DreamView：

![](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/dv_page_6334353.jpeg)

如果一切正常，可以看到一辆汽车在 DreamView 里移动。 现在您看到的车和道路状况，只是 DreamView 将 record 数据包的数据单纯
的回放，就像播放录好的视频一样。

### 步骤七：停止 DreamView

当您想结束 Dreamview 进程时，可以通过以下命令：

```shell
aem bootstrap stop
```
