# Apollo 1.5 快速入门指南

这个快速入门指南专注于Apollo 1.5新功能的介绍。对于Apollo的一般概念，请参考 [Apollo 1.0 快速入门指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start_cn.md). 

在做以下步骤时，请确认你已经调试了LiDAR与GNSS/INS之间的外部参数。关于传感器标定，请参考[Apollo 1.5 LiDAR calibration guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_lidar_calibration_guide.md).

## 启动发布的环境的Docker镜像

运行以下代码：

```bash
cd $APOLLO_HOME
bash docker/scripts/release_start.sh
```
当Docker启动时，它会创建一个端口映射。端口映射会把Docker内部8887端口映射到主机的8887端口。然后你就可以在主机的浏览器中访问HMI（人机界面）网络服务：

打开Chrome浏览器然后在地址栏输入**localhost:8887**来开始Apollo的HMI.

 ![](images/hmi_setup_profile.png)
 
你会被要求先设置个人信息。单击下拉菜单来选择你的高精地图和所用的车辆。这个列表是在[HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/hmi/conf/config.pb.txt)中定义的. 

然后你的HMI就可以用了！

*注意：你也可以在HMI的右侧界面来更改个人配置。只要记得点击右上角的“Reset All”来重启系统就好*

 ![](images/start_hmi.png)

## (*新增!*) Start Auto -- 开始自动模式

在Apollo 1.5中，我们开放了新的功能--自动跟随前方车辆直到目的地。


1. 要让这个可以用，你需要先点击左侧的"Setup"按钮来设置。
 ![](images/hmi_setup_1.5.png)

2. 确定硬件已经齐备，所有模块已经打开并且汽车状态良好，可以安全进入自动模式自动跟车到达目的地。

3. 点击"Start Auto"按钮，然后它就会自动带你走！


 ![](images/hmi_start_auto_following.png)