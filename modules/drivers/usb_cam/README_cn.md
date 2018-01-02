
## usb_cam
usb_cam包是基于V4L USB相机设备实现的ROS nodelet封装，提供图像采集及发布的功能。本驱动中使用了一台长焦相机和一台短焦相机。

### Topics

* /apollo/sensor/camera/traffic/image_long --> sensor_msgs/Image
* /apollo/sensor/camera/traffic/image_short --> sensor_msgs/Image
* /apollo/sensor/camera/traffic/image_long/camera_info --> sensor_msgs/CameraInfo
* /apollo/sensor/camera/traffic/image_short/camera_info --> sensor_msgs/CameraInfo
 
### 编译

```bash 
# in dev docker
cd /apollo
bash apollo.sh build_usbcam
```
产出会覆盖`/apollo/bazel-apollo/external/ros/`中原有的camera驱动相关文件
 
### 配置usb_cam驱动

首先要确保使用USB3.0接口连接相机，在docker中查看设备是否存在，并且具有读和写权限。

```bash 
# in dev docker
ls -l /dev/video*
```

其次需要根据相机的长短焦类型，进行设备固化。短焦相机设备固定为/dev/camera/obstacle；长焦相机，设备固定为/dev/camera/trafficlights。示例：

```bash 
# in dev docker
ln -s /dev/video0  /dev/camera/obstacle
ln -s /dev/video1  /dev/camera/trafficlights
```

最后指定每台相机的对应内参文件

**内参文件**
```xml
<param name="camera_info_url" type="string" value="$(find usb_cam)/params/onsemi_traffic_intrinsics.yaml"/>
```

### 启动usb_cam驱动
**请先修改并确认launch文件中的参数与实际车辆相对应**

```bash
roslaunch usb_cam start_leopard.launch
# or
bash /apollo/scripts/usb_camera.sh
```

### 常见问题
1. 如果出现报错“sh: 1: v4l2-ctl: not found”，需要安装v4l2库。

```bash
sudo apt-get install v4l-utils
```

