定位模块配置
===================

## 目录
      
 - [概览](#概览)
	
 - [导航设备配置](#导航设备配置)
 
 - [系统文件配置](#系统文件配置)

## 概览

该手册旨在帮助用户在自动驾驶开发套件上配置定位模块。在定位模块配置环节，将完成导航设备配置。

## 导航设备配置

下面介绍了导航配置的方法。当设备正确接入系统后，在/dev/下面有名为ttyACM0的设备，即表示M2已经被正确的加载了。配置设备时，需要将设备的串口线连接上电脑的串口才可以对设备进行配置，也就是说，用来配置设备的电脑主机需要拥有串口。Windows下可以通过串口助手、串口猎人或者COMCenter等工具进行配置，Linux下可以通过Minicom、cutecom等工具进行配置。linux下建议使用cutecom软件，可使用`sudo apt-get install cutecom`来安装此软件，在终端中使用`sudo cutecom`命令打开该软件，在软件中`open`名为`ttyS0`的设备。

### 杆臂配置

车尾天线（后天线，通常是主天线，也就是Primary）杆臂配置：

`$cmd,set,leverarm,gnss,x_offset,y_offset,z_offset*ff`

这里的杆臂值就是车辆集成环节中测量所得的杆臂值，杆臂值请以自己使用的实际情况为准。

### GNSS航向配置

天线车头车尾前后安装

`$cmd,set,headoffset,0*ff`

### 导航模式配置
```
$cmd,set,navmode,FineAlign,off*ff
$cmd,set,navmode,coarsealign,off*ff
$cmd,set,navmode,dynamicalign,on*ff
$cmd,set,navmode,gnss,double*ff
$cmd,set,navmode,carmode,on*ff
$cmd,set,navmode,zupt,on*ff
$cmd,set,navmode,firmwareindex,0*ff
```

### USB接口输出设置
```
$cmd,output,usb0,rawimub,0.010*ff
$cmd,output,usb0,inspvab,0.010*ff
$cmd,through,usb0,bestposb,1.000*ff
$cmd,through,usb0,rangeb,1.000*ff
$cmd,through,usb0,gpsephemb,1.000*ff
$cmd,through,usb0,gloephemerisb,1.000*ff
$cmd,through,usb0,bdsephemerisb,1.000*ff
$cmd,through,usb0,headingb,1.000*ff
```
### 网口配置
```
$cmd,set,localip,192,168,0,123*ff
$cmd,set,localmask,255,255,255,0*ff
$cmd,set,localgate,192,168,0,1*ff
$cmd,set,netipport,111,112,113,114,8000*ff
$cmd,set,netuser,username:password*ff
$cmd,set,mountpoint,XMJL*ff
```
这里我们假设您所使用的无线路由器的IP地址为192.168.0.1,那么我们将M2主机的IP地址设置为192.168.0.123，子网掩码为255.255.255.0，网关为192.168.0.1，netipport设置的是RTK基站的IP地址和端口，netuser设置的是RTK基站的用户名和密码，mountpoint是RTK基站的挂载点。网络配置请依据自己所使用的路由器的实际情况自行更改为相应的配置，RTK基站信息请以自己的实际情况为准。注意：在M2的网络模块配置完成后，在IPC主机中应该是可以ping通IMU的ip地址的；否则，IMU无法正常联网，在后续的GNSS信号检查中会一直显示SINGLE而不是我们期望的NARROW_INT。

### PPS授时接口输出
```
ppscontrol enable positive 1.0 10000
log com3 gprmc ontime 1 0.25
```

将所有配置逐条发送给设备，得到设备返回`$cmd,config,ok*ff`字段，说明配置成功，配置成功后要进行配置保存，发送`$cmd,save,config*ff`指令，然后将该设备断电后重新上电加载后即可使用。注意：PPS授时接口输出的两条配置命令是没有返回`$cmd,config,ok*ff`字段的，这是正常情况，不用担心。

## 系统文件配置

系统文件配置主要包括三个部分，GNSS配置、关闭点云定位和定位模式配置。

### GNSS配置

修改`/apollo/modules/calibration/data/dev_kit/gnss_conf`文件夹下面的配置文件`gnss_conf.pb.txt`，修改如下内容配置基站信息：
```
rtk_from {
    format: RTCM_V3
    ntrip {
        address: "<IP>"
        port: <PORT>
        mount_point: "<MOUNTPOINT>"
        user: "<USER>"
        password: "<PASSWORD>"
        timeout_s: 5
    }
    push_location: true
}

```
这是RTK基站信息相关的配置，请依据自己的实际情况进行配置。注意：RTK基站信息需要同时配置在M2的IMU主机中和apollo的开发套件的`gnss_conf.pb.txt`配置文件中。

### 检查GPS信号

将车辆移至室外平坦开阔处，进入Apollo系统，在终端中执行gps.sh脚本打开gps模块。输入命令`cyber_monitor`，进入 `/apollo/sensor/gnss/best_pose`条目下，查看sol_type字段是否为NARROW_INT。若为NARROW_INT，则表示GPS信号良好；若不为NARROW_INT，则将车辆移动一下，直到出现NARROW_INT为止。进入`/apollo/sensor/gnss/imu`条目下，确认IMU有数据刷新即表明GPS模块配置成功。

### 关闭点云定位

在`apollo/modules/localization/conf/localization.conf`文件中将：`--enable_lidar_localization=true`修改为：`--enable_lidar_localization=false`。

### 定位模式配置

在`apollo/modules/localization/conf/localization_config.pb.txt`文件中这个配置应为`localization_type:MSF`，M2不支持`RTK`模式。    
将`apollo/modules/localization/launch/localization.launch`文件中的`dag_streaming_rtk_localization.dag`修改为`dag_streaming_msf_localization.dag`。

### 常见问题
系统无法生成驱动设备`ttyACM0`，在`/apollo/data/log/gnss.INFO`里面会有类似报错提示：

```
open device /dev/ttyACM0 failed， error: no such file or directory
gnss driver connect failed, stream init failed
```

docker内和docker外的/dev/下都没有`ttyACM0`设备，先退出docker，然后关闭docker，再执行如下命令：
```
cd /apollo/docker/setup_host
bash setup_host.sh
```
重启工控机，然后在/docker/外，/dev/下，就有`ttyACM0`，再进docker，再试gps，可以了。

### 检查定位信号

将车辆移至室外平坦开阔处，进入Apollo系统，在终端中执行gps.sh和localization.sh脚本打开gps模块和localization模块。确认GPS模块已成功启动并且GPS信号良好。输入命令`cyber_monotor`，进入`/apollo/localization/pose`条目下，等待两分钟，直到有数据刷新即表明定位模块配置成功。
