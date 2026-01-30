# 快速开始

前提条件

阅读使用该文档前请确保已经参考 [必要软件安装指南](../installation_instructions/essential_software_installation_guide_cn.md) 文档 完成 Apollo 环境管理工具以及容器环境的安装

## 创建工程和安装 Apollo 软件包，并使用 Dreamview+ 播放数据包

本场景描述如何使用 Dreamview 播放数据包，为开发者熟悉 Apollo 平台打下基础。您可以通过 Dreamview 播放 Apollo 提供的
record 数据包，进一步观察、学习 Apollo 自动驾驶。

### 步骤一：进入 Apollo Docker 环境

> 本文所使用的工程样例可从 [github 仓库](https://github.com/ApolloAuto/application-core)中获取

#### 1. 克隆样例工程

```shell
git clone https://github.com/ApolloAuto/application-core.git application-core
cd application-core
```

#### 2. 启动 Apollo 环境容器

```shell
aem start
```

#### 3. 进入 Apollo 环境容器

```shell
aem enter
```

### 步骤二：添加和安装依赖

#### 1. 安装依赖包

> 此操作真正含义是编译工程中 `core` 这个包，但 `core` 本身并没有需要编译的代码，所以此操作仅会安装 `core/cyberfile.xml`
> 中声明的依赖包

```shell
buildtool build -p core
```

### 步骤三：安装 Dreamview+ 插件

请您按照以下步骤安装 Dreamview+ 插件。

#### 1. 登录 Apollo Studio 工作台

登录 [Apollo Studio 工作台](https://apollo.baidu.com/workspace)。

#### 2. 获取插件安装链接

1. 在账户信息中，单击 **我的服务**

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_54b0f1e.png)

2. 选择 **仿真** 页签。

3. 在 **插件安装** 中单击 **生成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_8624845.png)

4. 选择 Apollo 版本并单击 **确定**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_75ee359.png)

#### 3. 在 docker 环境执行插件安装命令

1. 进入 docker 环境。

   ```bash
   # 拉取并启动docker容器
   aem start

   # 进入容器
   aem enter

   # 下载安装依赖包： 会拉取安装core目录下的cyberfile.xml里面所有的依赖包
   buildtool build
   ```

2. 在 docker 环境中，执行上一步复制的安装插件命令。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_76b7133.png)

### 步骤四：启动并打开 Dreamview+

#### 1. 启动 Dreamview+

通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：

```bash
aem bootstrap start --plus
```

> 注意：
>
> - 如果您想要停止 Dreamview+，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

#### 2. 打开 Dreamview+

启动成功后，在浏览器输⼊`localhost:8888`⽹址打开 Dreamview+ 界面。

### 步骤五：播放数据包

- 在 Dreamview+ 中播放数据包

  启动 Dreamview+ 之后，进入 Dreamview+ 界面，您可以选择默认模式，也可以选择其他模式播放数据包。本小节以默认模式为例。

  ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_7deb2d2.png)

  1.  选择 **Default Mode**。
  2.  勾选 **Accept the User Agreement and Privacy Policy/接受用户协议和隐私政策**，并单击 **Enter this Mode** 进入 **Mode Settings/模式设置** 页面。
  3.  下载数据包。

      在 **Resource Manager/资源管理** > **Records/数据包** 中先下载需要的数据包。

      ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_0b8f187.png)

  4.  在 **Mode Settings/模式设置** 页面，设置播包参数。

      - 在 **Operations/操作** 中选择 **Record**。
      - 在 **Environment Resources/环境资源** 中，单击 **Records/数据包**，并选择具体想要播放的数据包。
      - 在 **Environment Resources/环境资源** 中，单击 **HDMap/高精地图**，并选择 **Sunnyvale Big Loop**。

  5.  单击底部区域播放按钮。

      ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_45acc2d.png)

      可以在 **Vehicle Visualization/车辆可视化** 中看到数据包播放的画面。

- 通过命令行播放数据包

  1.  进入 docker 环境，
  2.  在 Dreamview+ 中 **Resource Manager/资源管理** > **Records/数据包** 中先下载需要的数据包。输入以下命令播放数据包：

      ```bash
      cyber_recorder play -f ~/.apollo/resources/records/数据包名称 -l
      ```

      > 注意：如果您想要循环播放数据包，添加 -l，如果不循环播放数据包，则不需要添加 -l。
