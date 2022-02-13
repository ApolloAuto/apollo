## Camera
wato包是基于V4L USB相机设备实现封装，提供图像采集及发布的功能。本驱动中使用了一台长焦相机和一台短焦相机。

### Output channels

* /apollo/sensor/wato/front_12mm/image
* /apollo/sensor/wato/front_6mm/image
* /apollo/sensor/wato/front_fisheye/image
* /apollo/sensor/wato/left_fisheye/image
* /apollo/sensor/wato/right_fisheye/image
* /apollo/sensor/wato/rear_6mm/image

### 启动wato驱动
**请先修改并确认launch文件中的参数与实际车辆相对应**
```bash
# in docker
bash /apollo/scripts/wato.sh
# or
cd /apollo && cyber_launch start modules/drivers/wato/launch/wato.launch
```
### 启动wato + video compression驱动
**请先修改并确认launch文件中的参数与实际车辆相对应**
```bash
# in docker
bash /apollo/scripts/wato_and_video.sh
# or
cd /apollo && cyber_launch start modules/drivers/wato/launch/wato_and_video.launch

### 常见问题
1. 如果出现报错“sh: 1: v4l2-ctl: not found”，需要安装v4l2库。

```bash
sudo apt-get install v4l-utils
```
