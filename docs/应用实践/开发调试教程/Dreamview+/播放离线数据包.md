## 前提条件

完成 Dreamview+ 插件安装，参见 [Studio 插件安装](../应用实践/开发调试教程/Dreamview+/Studio%20插件安装.md) 。

## 操作步骤

您可以通过包管理和源码两种方式快速体验离线数据包播放操作。其中进入 docker 环境和启动 dreamview+ 的命令有所区别，请您按照命令进行操作。

### 步骤一：启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

#### 1. 启动 Dreamview+

##### 方式一：包管理方式

通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：

```bash
aem bootstrap start --plus
```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

##### 方式二：源码方式

通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：

```bash
bash scripts/bootstrap.sh start_plus
```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`bash scripts/bootstrap.sh stop_plus`，
> - 如果您想要重启 Dreamview+，请输入`bash scripts/bootstrap.sh restart_plus`。

#### 2. 打开 Dreamview+

启动成功后，在浏览器输⼊ `localhost:8888` ⽹址打开 Dreamview+ 界面。

### 步骤二：播放数据包

#### 在 Dreamview+ 中播放数据包

启动 Dreamview+ 之后，进入 Dreamview+ 界面，您可以选择默认模式，也可以选择其他模式播放数据包。本小节以默认模式为例。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_7deb2d2.png)

1. 选择 **Default Mode** 。

2. 勾选 **Accept the User Agreement and Privacy Policy/接受用户协议和隐私政策** ，并单击 **Enter this Mode** 进入 **Mode Settings/模式设置** 页面。

3. 下载数据包。
   
   在 **Resource Manager/资源管理** > **Records/数据包** 中先下载需要的数据包 **demo_3.5** 。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_0b8f187.png)

4. 单击底部区域播放按钮。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_45acc2d.png)
   
   可以在 **Vehicle Visualization/车辆可视化** 中看到数据包播放的画面。

#### 通过命令行播放数据包

1. 进入 docker 环境，

2. 在 Dreamview+ 中 **Resource Manager/资源管理** > **Records/数据包** 中先下载需要的数据包。输入以下命令播放数据包：
   
   ```bash
   cyber_recorder play -f ~/.apollo/resources/records/demo_3.5 -l
   ```

> 注意：如果您想要循环播放数据包，添加 -l，如果不循环播放数据包，则不需要添加 -l。
