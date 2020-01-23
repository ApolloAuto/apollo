# 相机感知设备集成

## 前提条件

 - 完成了[循迹自动驾驶演示](Waypoint_Following--Operation_And_Questions_cn.md)
 
## 主要步骤

 - 摄像头安装配置与数据验证

### 摄像头型号说明

- 摄像头型号：LI-USB30-AR023ZWDR（leopard 摄像头）。 

- 摄像头LI-USB30-AR023ZWDR采用标准USB 3.0接口，由Leopard Imaging Inc.制造。该系列产品基于AZ023Z 1080P传感器和安森美半导体的AP0202 ISP。它支持外部触发和软件触发。 

- 建议使用两个带 6mm镜头的摄像头和一个带 12mm镜头的摄像头，以达到所需的性能。

- 更多详细参数可参考：[leopard数据手册](https://leopardimaging.com/product/li-usb30-ar023zwdrb/)。

### 摄像头安装配置与数据验证

- 牢固安装在小车结构架前端横梁处，水平安装，俯仰角向下0-2度（向下倾斜小于2度，不能上仰），翻滚角误差±1度（左右两侧的平齐程度），航向角误差±2度，镜头保持清洁，避免影响图像采集。安装位置如下图所示：

![camera_integration_installation](images/camera_integration_installation.png)

- 注意摄像头不要装反(usb接口应该在下方)，正确的放置方向如下图所示：

![camera_integration_look](images/camera_integration_look.jpeg)

### 摄像头与工控机连接
 
 - 直接用数据线将设备连接在IPC的USB3.0接口。接口顺序无固定顺序，用户可自行指定。
 
![camera_integration_background](images/camera_integration_background.jpeg)

![camera_integration_line](images/camera_integration_line.jpeg)

### 摄像头规则文件的配置
 
 摄像头规则文件的作用是，当linux启动时，根据规则文件设置的规则，自动生成对应的软链接文件。
 
 - 在docker环境外，执行如下命令，打开默认的规则文件

	```
	vim  ~/apollp/docker/setup_host/etc/udev/rules.d/99-webcam.rules #~/apollo/为apollo工程的根目录，用户根据自身情况修改，后文不再赘述。
	```
 
 - 根据自身情况，修改rules文件，摄像头的rules文件示例如下所示(这里只包含了两个摄像头的规则文件，如需用到3个摄像头，可参照格式自行添加)：
	```
	SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", KERNELS=="2-1:1.0", MODE="0666", SYMLINK+="camera/front_6mm", OWNER="apollo", GROUP="apollo"
	SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", KERNELS=="2-2:1.0", MODE="0666", SYMLINK+="camera/front_12mm", OWNER="apollo", GROUP="apollo"
	```
其中，第一条代表连接到USB端口号为`2-1:1.0`的摄像头对应的软链接文件为`camera/front_6mm`；第二条代表连接到USB端口号为`2-2:1.0`的摄像头对应的软链接文件为`camera/front_12mm`。

 - 查看摄像头所接的USB端口对应的端口号方法：在docker环境外执行如下命令
 ```
 ll /sys/class/video4linux/video* 
 ```
 - 在docker环境外，执行如下命令，使配置的规则文件在本地系统生效:
 
 ```
  bash ~/apollo/docker/setup_host/setup_host.sh  
  sudo reboot  //重启工控机
 ```
### Apollo配置的修改


将`modules/drivers/camera/dag/dev_kit_camera.dag` 文件重命名为`modules/drivers/camera/dag/camera.dag`并替换原有的`camera.dag`文件。
### 摄像头的启动
 
 - 在摄像头与工控机正常连接的基础上，执行`ls /dev/video*`指令，查看摄像头是否被识别， 如果摄像头设备已经被识别，则会显示以`video`开头的设备名称，否则的话，请检查摄像头与工控机的连线是否可靠。

 - 检查`/dev/camera`目录是否存在，以及该目录下`front_6mm`、`front_12mm`两个软链接文件是否存在(根据规则文件配置不同，可能有1个或多个软链接文件)。如果使用`ls /dev/video*`命令能显示摄像头设备，但不存在软链接文件或者没有`camera`文件夹，请参照上文`摄像头规则文件的配置`章节，检查规则文件是否配置正确。
 
 - 启动docker环境。

 - 确保apollo被正确编译。
 
 - 启动DreamView： `bash /apollo/scripts/bootstrap.sh`，并在浏览器中输入DreamView地址：`http://localhost:8888/`
 
 - 在`Module Controller`标签栏下，打开`Camera`开关，开关位置如下图所示：
 
![camera_integration_dreamview1](images/camera_integration_dreamview1.png)

 - 在`Tasks`标签栏下依次打开`SimControl`和`Camera Sensor`开关，`Camera Sensor`打开后，务必关闭`SimControl`。选择`Mkz Standard Debug`模式，开关在dreamview界面的位置如下图所示：
 
![camera_integration_dreamview2](images/camera_integration_dreamview2.png)
 
 - 如果一切正常，则会在`dreamview`右下角出现摄像头采集的图像。
 
 - 在正确完成上述步骤后，在`docker`环境内使用`cyber_monitor`工具，查看`/apollo/sensor/camera/front_6mm/image`、`/apollo/sensor/camera/front_12mm/image`是否有数据输出，且帧率是否稳定在15帧左右。

```
//输入如下命令启动cyber_monitor
budaoshi@in_dev_docker:/apollo$ cyber_monitor
```