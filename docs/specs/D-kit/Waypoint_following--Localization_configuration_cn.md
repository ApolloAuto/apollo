定位模块配置
===================

## 目录
      
 - [概览](#概览)
 	 
 - [感知硬件集成](#感知硬件集成)
    - [概览](#概览)
	
    - [GPS导航设备安装](#GPS导航设备安装)

## 概览

该用户手册旨在帮助用户在教学小车上安装、配置感知元器件。

## 感知硬件集成

在定位模块配置环节，将完成导航设备的安装与配置。硬件集成的概览图如下所示：

![图片](../images/gps_hardware_map.png)

### GPS导航设备安装

星网宇达Newton-M2是一款支持静态双天线初始化的GPS导航设备，其在Apollo系统中的安装配置主要包括以下硬件设置和软件配置两个部分：

#### 安装主机和天线
本安装指南描述了挂载、连接和为主机与天线量取和设置杆臂值尺寸的步骤

##### 安装需要的组件

- Newton-M2主机（1个）

![M2_host](../images/gps_host.png)

- 卫星天线（2个）：测量型卫星天线，信号接口为TNC母口

![M2_antenna](../images/gps_antenna.png)

- 射频连接线（2根）：射频线两端分别为TNC公头和SMA母头

![M2_cable](../images/gps_cable.png)

- 数据/电源线缆（1根）：一端是和主机相连的接口，另一端含有一个串口母头，一根RJ-45网线母头，一根电源接口和一个USB接口

##### 挂载
使用者可以将主机安装在载体上的任何位置，但我们强烈建议使用者采用下述的建议方案：

- 将M2主机安装在载体的后轴的中心位置上，使主机铭牌上标示的坐标系XOY面尽量与载体坐标系平行并保持各轴向一致，建议将Y轴的正向保持与载体的前进方向一致，M2主机单元必须与被测载体固连。
- GNSS双天线应尽量与载体坐标系Y轴平行并且前天线（Secondary）应在Y轴正方向上，GNSS天线要尽可能的将其安置于测试载体的最高处以保证能够接收到良好的GNSS信号。

##### 配线
执行如下步骤将主机和天线连接到Apollo系统：

- 将两根射频线的TNC公头连接上卫星天线的TNC母口
- 将射频线的SMA母口连接上IMU主机的SMA公口，车尾天线为主天线，将其连接在IMU的Primary接口上
- 将数据/电源线缆的公口和主机的母口连接
- 将数据/电源线缆的串口母头连接上IPC的串口公口
- 将数据/电源线缆的网线母头和有线网的水晶头相连接
- 将数据/电源线缆的USB接口连接上IPC的USB接口
- 将数据/电源线缆的电源线接口和载体的电源接口连接起来

请参考下述图片：

![M2_connection](../images/gps_connection.jpeg)

##### 量取杆臂值

参考下述步骤：

- 当主机和天线处在正确位置时量取天线到主机的距离。主机的中点和天线的中点标记在设备的外部。
- 距离被测量并记录为X轴偏移、Y轴偏移和Z轴偏移。坐标轴由主机的位置确定。偏移量的误差应该被控制在一厘米以内以获取高精度的定位信息。杆臂是指后天线（Primary）的几何中心位置相对于主机几何中心在直角坐标系内x,y,z三方向的位置差。通过如下指令进行补偿设置：`$cmd,set,leverarm,gnss,x_offset,y_offset,z_offset*ff`。x_offset:X方向的杆臂误差，单位为米，以此类推。注：上述坐标XYZ为设备坐标轴配置后的实际坐标，一般应与载体坐标系一致，注意补偿的是后天线（Primary）杆臂值。当进行导航模式配置后必须对设备进行重新上电启动。举例如下：

![图片](../images/gps_vehicle_map.jpeg)

如上图所示：后天线在M2主机X轴的正向0.2m处，则X轴偏移x_offset的值为0.2；后天线在M2主机Y轴的负向0.1m处，则Y轴偏移y_offset的值为-0.1；后天线在M2主机Z轴的正向0.8m处，则Z轴偏移z_offset的值为0.8。

#### 配置GPS和主机
下面展现了配置GPS和主机的方法。当设备正确接入系统后，在/dev/下面有名为ttyACM0的设备，即表示M2已经被正确的加载了。配置设备时，需要将设备的串口线连接上电脑的串口才可以对设备进行配置，也就是说，用来配置设备的电脑主机需要拥有串口。Windows下可以通过串口助手、串口猎人或者COMCenter等工具进行配置，Linux下可以通过Minicom、Cutecom等工具进行配置。linux下建议使用Cutecom软件，请在终端中使用`sudo cutecom`命令打开该软件。

##### 杆臂配置
车尾天线（后天线，通常是主天线，也就是Primary）杆臂配置：

`$cmd,set,leverarm,gnss,x_offset,y_offset,z_offset*ff`

杆臂值请以自己使用的实际情况为主。

##### GNSS航向配置
天线车头车尾前后安装

`$cmd,set,headoffset,0*ff`

##### 导航模式配置
```
$cmd,set,navmode,FineAlign,off*ff
$cmd,set,navmode,coarsealign,off*ff
$cmd,set,navmode,dynamicalign,on*ff
$cmd,set,navmode,gnss,double*ff
$cmd,set,navmode,carmode,on*ff
$cmd,set,navmode,zupt,on*ff
$cmd,set,navmode,firmwareindex,0*ff
```

##### USB接口输出设置
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
##### 网口配置
```
$cmd,set,localip,192,168,1,2*ff
$cmd,set,localmask,255,255,255,0*ff
$cmd,set,localgate,192,168,1,1*ff
$cmd,set,netipport,111,112,113,114,8000*ff
$cmd,set,netuser,username:password*ff
$cmd,set,mountpoint,XMJL*ff
```
这里我们假设您所使用的无线路由器的IP地址为192.168.1.1,那么我们将主机的IP地址设置为192.168.1.2，子网掩码为255.255.255.0，网关为192.168.1.1，netipport设置的是RTK基站的IP地址和端口，netuser设置的是RTK基站的用户名和密码，mountpoint是RTK基站的挂载点。网络配置请依据自己所使用的路由器的实际情况自行更改为相应的配置，RTK基站信息请以自己的实际情况为准。

##### PPS授时接口输出
```
ppscontrol enable positive 1.0 10000
log com3 gprmc ontime 1 0.25
```

将所有配置逐条或一起发送给设备，得到设备返回`$cmd,config,ok*ff`字段，说明配置成功，配置成功后要进行配置保存，发送`$cmd,save,config*ff`指令，然后将该设备断电后重新上电加载后即可使用。
