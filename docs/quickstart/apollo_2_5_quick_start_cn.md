# Apollo 2.5 快速入门指南

这个快速入门指南专注于Apollo 2.5新功能的介绍。对于Apollo的一般概念，请参考
[Apollo 1.0 快速入门指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start_cn.md)
和
[Apollo软件安装指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide_cn.md)。

# 上车测试

1. 上车测试，请确认你已经调试了所有传感器之间的外部参数。关于传感器标定，请参考
   [Apollo 2.0 Sensor Calibration Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide_cn.md).

1. 启动发布的环境的Docker镜像

1. 启动DreamView

   打开浏览器（譬如Chrome）然后在地址栏输入**http://localhost:8888**来启动Apollo Dreamview。

   ![](images/dreamview_2_5.png)

1. 选择模式、车辆和地图

   ![](images/dreamview_2_5_setup_profile.png)

   单击右上角下拉菜单选择车辆和地图。这个列表是在
   [HMI config file](ttps://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/dreamview/conf/hmi.conf)
   中定义的。

1. 启动所有模块

   点击左侧的"Setup"按钮来设置。

   ![](images/dreamview_2_5_setup.png)

   跳转到*Module Controller*页面，检查所有的软硬件都已正常工作。
 
   ![](images/dreamview_2_5_module_controller.png) 

1. 在Default Routing下选择想要的路线。

1. 在Tasks下谨慎启动自动驾驶模式

   确定硬件已经齐备，所有模块已经打开并且汽车状态良好，环境安全，点击"Start Auto"按钮。

   [](images/dreamview_2_5_start_auto.png)

1. 测试完成后，点击Reset All，关闭所有窗口，关闭工控机。
