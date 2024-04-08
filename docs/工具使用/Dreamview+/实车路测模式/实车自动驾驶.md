## 步骤一：自动驾驶准备

### 1. 启动 CAN 卡

启动工控机后，在桌面上打开命令行终端（Terminal），进入 CAN 卡目录（默认在 home 路径下），启动 CAN 驱动：

```bash
bash ~/socketcan/start.sh
```

## 步骤二：启动并打开 Dreamview+

### 1. 启动 Apollo Docker 环境

#### 如果您使用的是包管理，请您按照以下步骤操作：

##### 打开命令行终端，切换路径到 Apollo：

```bash
cd ~/apollo
```

##### 输入以下命令进入 Apollo Docker 环境：

```bash
aem start #创建容器
aem enter #进入容器
buildtool build
```

> 首次进入容器 build，
> 如果是只编译更新的模块，使用以下命令`buildtool build -p 模块名`。

     如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：

     ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_af1a780.png)

#### 如果您使用的是源码，请您按照以下步骤进入 docker 环境：

##### 打开命令行终端，切换路径到 Apollo：

```bash
cd ~/apollo     #切换路径到apollo，注意cd与~/apollo之间有一个空格
```

##### 输入以下命令启动 Apollo Docker 环境：

```bash
bash docker/scripts/dev_start.sh
```

启动 docker 环境：

```bash
bash docker/scripts/dev_into.sh
```

如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：

```bash
[user@in-runtime-docker:/apollo]$
```

### 2. 启动 Dreamview+

#### 在 Apollo Docker 环境下，启动 Dreamview+ 交互界面：

##### 如果您使用的是包管理，请您在 docker 环境中执行以下命令执行以下命令启动 Dreamview+：

```bash
aem bootstrap start --plus
```

> 注意：
>
> - 如果您想要停止 Dreamview+，提示 `Fail to start Dreamview`，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

##### 如果您使用的是源码，请您在 docker 环境中执行以下命令执行以下命令启动 Dreamview+：

```bash
bash scripts/bootstrap.sh start_plus
```

> 注意：
>
> - 如果您想要停止 Dreamview+，请输入`bash scripts/bootstrap.sh stop_plus`，
> - 如果您想要重启 Dreamview+，请输入`bash scripts/bootstrap.sh restart_plus`。

#### 打开 Dreamview+。

启动成功后，在浏览器输⼊`localhost8888`⽹址打开 Dreamview+ 界面。

### 3. 选择模式和操作

在 **Mode/模式** 中，选择 **Vehicle Test/实车模式** 。

在 **Operations/操作** 中，选择 **Auto Drive/自动驾驶** 。

### 4. 选择车辆和地图

在 **ADS Resources/自动驾驶资源** 中，选择实车路测模式的车辆。

- 如有对应车辆配置，可选择对应配置，

- 如无对应车辆配置，可用默认配置。

在 **Environment Resources/环境资源** > **HD Map/高精地图** 中，选择实车路测模式需要的地图。

### 5. 打开所需要的模块

启动所需要的 **Canbus** 、 **Control** 、 **ExternalCmd** 、 **GPS** 、 **Lidar** 、 **Localization** 、 **Planning** 、 **Prediction** 、 **Radar** 、 **Transform** 模块。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_62fa920.png)

ExternalCmd 是 Planning 模块的一个子模块，循迹会用到这个子模块，实车模式只开 Planning 模块就可以。

### 6. 查看连接状态

查看 Component 状态。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_30928e6.png)

> 说明：
>
> - Component 中显示绿色 OK 为正常，可以进行下面的操作。
> - WARNING 黄色只是信号不好，模块已经启动，只要不是 ERROR 就不影响操作。

查看 cyber_monitor 数据。

进入 docker 环境，输入 `cyber_monitor`，启动的 cyber_monitor 工具中，可以看到很多 channel 信息，其中 /tf 和 /tf_static 对应 Transform 模块，/apollo/localization/pose 对应 Localization 模块，apollo/sensor/gnss/best_pose 对应 GPS 模块。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4650e42.png)

使用上下方向键选择 channel /apollo/sensor/gnss/best_pose 按右方向键进入此 channel，查看 sol_type 项是否是 NARROW_INT，若为 NARROW_INT，则表示 GPS 信号良好，符合定位要求；若不为 NARROW_INT，则遥控移动下车辆，直到出现 NARROW_INT 为止。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f716d6d.png)

检查所有开启模块均正常后可以进行下面的操作。

### 7. 编辑行驶路线

在 **Vehicle Visualization/车辆可视化面板** 中，单击 **Routing Editing/路径编辑** 按钮进入路由编辑功能。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_17803eb.png)

在面板左上角初始位置图标，在地图上单击设定初始位置。

> 注意：长按鼠标右键可以拖转移动地图位置。

单击左上角轨迹点图标，在地图上单击添加途经轨迹点。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_b951b5a.png)

单击 **保存编辑** ，完成轨迹绘制。

单击 **Routing/路由** 按钮出现红色轨迹线。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ae5fcdc.png)

## 步骤二：启动自动驾驶

如果各模块正常工作，且规划效果也不错，将遥控器切换到自动驾驶状态。

在 DreamView+ 中，在操作底栏中，单击 **START/启动** 按钮，启动自动驾驶。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_053c458.png)

> 启动自动驾驶后，生成的绿色轨迹为规划路径。
