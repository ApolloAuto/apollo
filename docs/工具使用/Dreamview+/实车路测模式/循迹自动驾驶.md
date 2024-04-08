## 步骤一：录制轨迹

### 1. 启动 Canbus 驱动

启动工控机后，在桌面上打开命令行终端（Terminal），进入 CAN 卡目录（默认在 home 路径下），启动 CAN 驱动：

```bash
bash ~/socketcan/start.sh
```

### 2. 启动 Apollo Docker 环境

### 如果您使用的是包管理，请您按照以下步骤操作：

#### 打开命令行终端，切换路径到 Apollo：

```bash
cd ~/apollo
```

#### 输入以下命令进入 Apollo Docker 环境：

```bash
aem start #创建容器
aem enter #进入容器
buildtool build
```

> 注意：首次进入容器用`buildtool build`；如果是只编译更新的模块，使用以下命令`buildtool build -p 模块名`。

如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7e7a575.png)

### 如果您使用的是源码，请您按照以下步骤进入 docker 环境：

#### 打开命令行终端，切换路径到 Apollo：

```bash
cd ~/apollo     #切换路径到apollo，注意cd与~/apollo之间有一个空格
```

#### 输入以下命令启动 Apollo Docker 环境：

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

### 3. 启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

#### 启动 Dreamview+

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

#### 打开 Dreamview+

启动成功后，在浏览器输⼊`localhost8888`⽹址打开 Dreamview+ 界面。

## 步骤二：录制轨迹

### 1. 选择循迹模式

在 **Mode/模式** 中选择 **Vehicle Test/实车路测** 模式。

在 **Operations/操作** 中选择 **Waypoint_Follow/循迹** 操作。

### 2. 选择车辆

- 如有对应车辆配置，可选择对应配置，

- 如无对应车辆配置，可用默认配置。

### 3. 启动循迹驾驶相关模块

在 **Modules/模块** 中，选择启动 **Canbus** 、 **Control** 、 **ExternalCmd** 、 **GPS** 、 **Lidar** ， **Localization** 、 **Prediction** 、 **Radar** 和 **Transform** 按钮。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6e892d2.png)

> 注意：ExternalCmd 是 planning 的一个子进程，循迹会用到这个子进程，路测模式只开 planning 模块就可以。

### 4. 查看连接状态

查看 Component 状态。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d8ce70a.png)

> 说明：
>
> - Component 中显示绿色 OK 为正常，可以进行下面的操作。
> - WARNING 黄色只是信号不好，模块已经启动，只要不是 ERROR 就不影响循迹。

开启 cyber_monitor，查看 channel 信息。

查看 cyber_monitor 数据。

进入 docker 环境，输入 `cyber_monitor`，启动的 cyber_monitor 工具中，可以看到很多 channel 信息，其中 /tf 和 /tf_static 对应 Transform 模块，/apollo/localization/pose 对应 Localization 模块，apollo/sensor/gnss/best_pose 对应 GPS 模块。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4650e42.png)

使用上下方向键选择 channel /apollo/sensor/gnss/best_pose 按右方向键进入此 channel，查看 sol_type 项是否是 NARROW_INT，若为 NARROW_INT，则表示 GPS 信号良好，符合定位要求；若不为 NARROW_INT，则遥控移动下车辆，直到出现 NARROW_INT 为止。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f716d6d.png)

检查所有开启模块均正常后可以进行下面的操作。

### 5. 开启轨迹录制

将车辆遥控至录制轨迹起点，标记此时车辆车头方向和车辆位置，可以在车辆左前轮处画一个十字符号用于标记，示意图如下：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3baaba6.png)

在操作底栏点击 **Record/录制** 按钮，启动轨迹录制。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7a6ea57.png)

### 6. 关闭轨迹录制

使用遥控器遥控车辆行驶一段轨迹，这里建议走直线，车辆停止后，点击 **Stop/停止** 录制按钮，关闭循迹录制。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7d4538e.png)

关闭轨迹录制后，在 **Record Name/数据包名称** 中填写数据包名称，并单击 **Save/保存** 。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_06fcbdf.png)

> 注意：数据包命名规则限制。

## 步骤三：启动循迹驾驶演示

### 1. 将车辆遥控至上述章节录制循迹数据时的起始位置。

此时，在 Dreamview+ 中车辆前方会生成一条蓝色的轨迹线，这条轨迹线就是刚才循迹录制生成的轨迹线。检查生成的轨迹线是否清晰无毛刺，确认轨迹是否与实际路线的大致相符，如果相差很大，比如本来录制的是直行，而轨迹显示的是大幅度转弯，请重新检查定位，并重新录制新轨迹线。

> 注意：车辆的起点位置和车头朝向都尽量与循迹录制时的车辆起点位置和车头朝向保持一致。

### 2. 将遥控器切换到自动驾驶模式。

### 3. 在 Dreamview+ 中点击操作底栏的 **START/启动** 按钮，启动自动驾驶。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dce1d0f.png)

车辆就会启动循迹，沿着已有的轨迹进行自动驾驶，观察车辆运动轨迹，包括速度、方向等是否与之前录制的一致。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_bcb38a2.png)

> 注意：
>
> - 车辆在循迹自动驾驶时，并没有使用到感知相关的传感器，遇到障碍物不会刹停或者避让，遇到紧急情况请及时遥控器接管。这也是寻找宽阔人少车少封闭场所的原因。
> - 车辆可能开始起步比较猛，请用遥控器随时接管，紧急情况下，快速按下急停按钮。

### 4. 车辆循迹完成后，及时遥控器接管。
