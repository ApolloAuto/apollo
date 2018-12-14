# Apollo上车调试

TX2上车验证阶段可以简单分为几个步骤：
1. 硬件接口确认
2. 硬件调试
  * camera调试
  * IMU&GPS调试
3. 调试和验证

以下会以金瑞祺项目为例介绍各个阶段的工作，修改内容，验证方法等。


## 硬件接口确认
金瑞祺项目的目标是制作低成本的低速园区自动驾驶方案。`localization`采用`RTK`定位模式。`perception`模块只采用`camera`。所以需要的硬件包括：摄像头、IMU、GPS。

根据硬件的选型我们可以确认，3个摄像头使用3个USB接口，IMU&GPS使用一个USB接口，那么3-4个USB接口就可以满足金瑞祺项目的需求。

金瑞祺项目不使用can口与车辆通讯。

## 硬件调试
### Camera调试

参考[Camera调试文档](\[Jetson_TX2_Platform\]howto_config_and_debug_camera_cn.md)

### IMU&GPS调试
将IMU和GPS连接，GPS输入口通过USB接口和TX2链接后，在`/dev`文件夹会生成`ttyUSB0`、`ttyUSB1`和`ttyUSB2`文件。以Apollo官方推荐的GPS型号为例，需要另外安装GPS的驱动，驱动下载地址为：
https://www.novatel.com/support/info/documents/809

配置设备的方法和camera类似，依次执行以下的步骤：

```shell
cd /apollo/docker/setup_host/
./setup_host.sh
sudo service udev restart
```

正常情况下，在`/dev`文件夹下会生成`novatel*`软链接文件，分别指向于三个ttyUSB文件，但是在TX2上需要对rules文件做一定的修改才能产生预期的效果。

GPS配置的rules文件为`99-usbtty.rules`；其中`DRIVERS`项的配置在TX2上不适配。通过指令`udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)`查询GPS设备在TX2上的信息，可以得知`DRIVERS`信息为`ngpsser`，那么将rules文件中`DRIVERS`项的值全部修改为`ngpsser`，再次执行`sudo service udev restart`即可。

完成GPS的配置后，还需要配置GPS的基站信息。基站信息的配置文件为`modules/calibration/data/mkz_example/gnss_params/gnss_conf.pb.txt`  

完成上述配置后，启动`Dreamview`，在`Module Controller`面板中启动`GPS`，左侧`Hardware`项目栏中`GPS`状态变为OK，表示启动成功。
为了验证定位是否正确，可以启动`Localization`（需要配置为`RTK`模式），主窗体中会显示道路和车辆在道路上的位置。

## 调试和验证
在金瑞祺项目的上车调试过程中，需要启动的模块有`GPS`、`Camera`、`Control`、`Guardian`、`Localization`、`Perception`、`Planning`、`Prediction`、`Routing`。其中因为采用了低成本的方案，所以`Perception`并没有使用`Module Controller`面板中的按钮启动，而是使用脚本`./script/perception_lowcost.sh`启动。  

下面会介绍在实现`RTK`模式定位，`Perception`正常检测到障碍物的过程中对配置文件的修改。

修改`modules/localization/conf/localization_config.pb.txt`文件，将`localization_type`项的值改为`RTK`。

`Perception`模块的配置文件有如下的修改：
```txt
a) perception_lowcost.conf文件中fusion_publish_sensor修改为camera
b) perception_lowcost.conf文件中dag_config_path指定为dag_streaming_lowcost.config
```

并且dag_streaming_lowcost.config文件中需要有如下的修改：

```txt
a) 将subnode_config中的RadarProcessSubnode配置全部注释掉
b) 同理，将edge_config中的RadarSubnode->FusionSubnode配置全部注释掉
c) 同理，将data_config中的RadarObjectData配置全部注释掉
d) 将subnode_config中的CameraProcessSubnode配置的reserve选项的pb_obj值改为1。表示camera节点会publish消息
e) 将subnode_config中的LanePostProcessSubnode配置的reserve选项增加publish:2。表示lanepost节点会publish关于perceptionobstacle的消息
```

