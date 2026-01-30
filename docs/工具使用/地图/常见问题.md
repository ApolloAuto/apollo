## 登录问题

### 1. 版本更新后模式下拉里没有 Map Editor 和 Map Gathering ，怎么恢复？

浏览器缓存问题，解决方式有以下两种：

1. 开启 Chrome 浏览器的无痕模式；

2. 清空浏览器缓存，在 Dreamview+ 网页点击鼠标右键 > **检查** ，以下三种方式其中一种即可（若不生效可将三种方式都执行）。
   
     ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3e878ed.png)
   
     ![image \(2\).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image%20%282%29_a207773.png)
   
     ![image \(3\).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image%20%283%29_3664850.png)

### 2. 登录失败/插件同步失败怎么办？

* 首先检查设备是否有网络连接，插件连接需要网络。
* 若网络已连接，是以 SSH 远程连接设备，可能因为隧道稳定性问题有失败概率，多尝试几次或重新生成插件指令重试即可。

## 地图采集

### 1. 为什么Lidar/GNSS/Localization硬件状态是红色？

请检查硬件连接状态是否正常，可打开 cyber monitor 查看以下关键 channel 数据流通状态:

* gnss相关：`/apollo/sensor/gnss/best_pose`、`/apollo/sensor/gnss/odometry`，
* lidar相关：通常是以 PointCloud2 为结尾的，比如 `/apollo/sensor/lidar16/compensator/PointCloud2`，
* 定位相关：`/apollo/localization/pose`，`/apollo/localization/msf_status`里面的状态不能是ERROR（WARNING 没问题）。

### 2. Common模式和Slam模式有什么差别，我该用哪一个？

* 建图算法不同，Common 主要依赖 RTK，Slam 顾名思义依赖 Slam 算法，
* 如场地空旷，且顶部无遮挡（室外），可选择 Common 模式，
* 如场地周围物体多，或是顶部有遮挡而 GNSS 信号不稳定，可选择 Slam 模式。

## 地图编辑

### 1. 导入底图文件显示无权限，怎么获取权限？

联系 Apollo 团队，为账号开通试用/使用权限即可导入成功。

### 2. 弯道怎么调整曲率？

单击选中弯道边线，端点处出现杠杆手柄，点击手柄拖动即可调节曲率。

### 3. 十字路口怎么画？Junction的框是什么含义？

* 先用直道绘制各方向的车道，再两两选中对应道路，在右上角选择弯道连接，
* Junction 框为高精地图中的十字路口语义，标注这一块区域为十字路口。
  
  ### 4. 怎么检查我的元素可为感知、PnC算法可用？
  * 转弯处的弯道属性中，方向已指定左转/右转（如未修改可手动修改）
  * 经过“沿车道方向拆分”的车道，与前驱后继已具备连接关系（如无连接，可 shift 选中两条车道，单击右上角直道连接建立连接关系），
  * 可变道处中间线为虚线。
