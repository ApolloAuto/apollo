本文档介绍 Novatel 捷联惯性导航设备 PP7 的配置，通过本文档，您可以学会如何配置和使用 捷联惯性导航设备PP7，让它在 Apollo 中正常工作。捷联惯性导航设备PP7，以下称为导航设备PP7或者PP7。

## 前提条件

为了更快、更好的理解本文档的内容，您需要提前做好以下准备工作：

- 完成了 [循迹搭建--车辆集成](../应用实践/车辆集成教程/循迹搭建/车辆集成.md)。

- 测量好 PP7 的两个天线的杆臂值

## Apollo 定位模块配置之 PP7 配置

PP7的配置方式有好多种，您可以使用如minicom命令行串口工具，也可以使用cutecom图形化串口工具，当然，你也可以使用NovAtel官网提供的NovAtel Application Suite图形化工具对其进行配置。在此文档中介绍的配置工具为NovAtel官网提供的NovAtel Application Suite。NovAtel Application Suite分为Windows平台版本和Linux平台版本，本文档介绍使用Linux平台版本的NovAtel Application Suite工具对PP7进行配置，本文档使用的是NovAtel Application Suite集成的的命令行工具对PP7进行配置，至于其图形化方式配置在此文档中不做介绍，有兴趣的开发者可以自行探讨。另外使用NovAtel Application Suite集成的命令行工具配置PP7对使用诸如minicom、cutecom等工具配置PP7的开发者具有参考意义。

### 1. 安装NovAtel Application Suite

NovAtel官网下载页面下载Linux平台版本的NovAtel Application Suite工具（点击后面的 ZIP 即可下载），下载完成后是一个压缩包，NovAtel Application Suite是免安装的，解压后可以直接使用。

下载地址：https://novatel.com/support/support-materials/software-downloads 。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_ed74e28.png)

### 2. 连接PP7设备到Linux主机

- 使用USB线缆将PP7的`COM PORTS`接口连接到Linux主机的`USB`接口上。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_99263b8.png)

- 在Linux主机命令行终端中输入执行命令： `ls -al /dev/ttyUSB*` 查看PP7设备号。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_5dff4e5.png)

命令执行后会显示3个USB设备，分别是`/dev/ttyUSB0、/dev/ttyUSB1、/dev/ttyUSB2` 。这里显示的设备号可能跟各位开发者显示的不同，这设备号与Linux主机上连接的USB设备数量有关。这里以`/dev/ttyUSB0、/dev/ttyUSB1、/dev/ttyUSB2`为例跟大家介绍。

- 在命令行终端中输入执行命令：`sudo chmod 777 /dev/ttyUSB*` 给所有ttyUSB设备可读可写可执行权限。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_b21d8da.png)

### 3. 添加PP7设备

- 进入NovAtel Application Suite解压后的文件夹，双击 `NovAtelApplicationSuite_64bit.AppImage` 应用程序即可启动NovAtel Application Suite软件。

- 在启动后的界面上点击“Add Device”按钮，在弹出的页面中的connection Name框中填`PP7` ，在Type中选择`USB`，在Port中选择`/dev/ttyUSB0`后，点击“`Add”`按钮添加PP7设备。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_03aab1a.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_72fd04c.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_6aec44b.png)

注意：若出现 `Error: Permission denied, cannot open /dev/ttyUSB0` ，表示添加失败，请在命令行终端中执行以下命令：`sudo chmod 777 /dev/ttyUSB* `给所有ttyUSB设备赋予可不可写可执行权限。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_cdea464.png)

### 4. 配置PP7设备

- 点击连接的PP7设备，打开PP7设备界面。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_6b669b5.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_1060fca.png)

- 点击页面中的Tools，在下拉菜单中选择terminal选择项。打开命令终端页面。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_8200ba3.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_3523e07.png)

- 在命令终端中输入并执行命令：`freset `进行恢复默认设置。命令执行后PP7会断开与NovAtel Application Suite的连接，请重新连接后再继续以下部分。命令行终端输入框为标有灰色`Enter your command`框，按回车键执行输入命令。

- 在命令行终端中输入并执行以下命令：`log version`

a. 在terminal输出中查看` HWCONFIG` 配置的是否是 `“EthPrioElev” `，表示网口优先模式。若不是，请使用以下命令配置为此版本，命令为：`CONFIGCODE ADD X3PD3C,7PKM76,D48WJD,GJ3HBM,9KFTM4,HWCT_70_0001`

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_9f3f53b.png)

b. 查看`PACKAGE `使用的是否是`“EP7PR0800SN0005”`，这个是使用的PP7固件版本信息，若不是，请与供应商联系获取相应的版本升级至此版本。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_bafca8c.png)

- 在命令终端页面中依次执行以下命令，配置PP7设备：

```
SETALIGNMENTVEL 400.0
SETINSTRANSLATION ANT1 0.550 0.875 1.054 0.01 0.01 0.01
SETINSTRANSLATION ANT2 -0.550 0.875 1.054 0.01 0.01 0.01
SETINSROTATION RBV 0 0 0 1 1 1
SETINSPROFILE LAND_PLUS
ALIGNMENTMODE KINEMATIC
INSCOMMAND ENABLE
SERIALCONFIG COM1 9600 N 8 1 N OFF
SERIALCONFIG COM2 9600 N 8 1 N OFF
LOG COM2 GPRMC ONTIME 1 0.25
ETHCONFIG ETHA AUTO AUTO AUTO AUTO
ICOMCONFIG ICOM1 TCP :2000
ICOMCONFIG ICOM2 TCP :3000
IPCONFIG ETHA STATIC 192.168.10.4 255.255.255.0 192.168.10.1
IPSERVICE FTP_SERVER ENABLE
PPSCONTROL ENABLE POSITIVE 1.0 10000
EVENTINCONTROL MARK1 DISABLE
SETINSUPDATE ZUPT ENABLE
WIFIMODE OFF
dnsconfig 1 180.76.76.76
NTRIPCONFIG ncom1 disabled
rtksource auto any
psrdiffsource auto any
INTERFACEMODE ICOM2 RTCMV3 NOVATEL ON
SAVECONFIG
```

注意：以上命令请请一条一条执行，且命令执行后，请确认命令行终端返回OK，表示命令配置成功。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_0c1b838.png)

- 所有命令配置完成后，请断掉PP7电源，以便保存配置更改。

命令简要说明：

```
SETINSTRANSLATION ANT1 0.550 0.875 1.054 0.01 0.01 0.01
SETINSTRANSLATION ANT2 -0.550 0.875 1.054 0.01 0.01 0.01
```

上面的两条命令是配置PP7杆臂值的命令。其中：

- ANT1 后的3个数字是主天线的杆臂值；

- ANT2 后的3个数字是副天线的杆臂值；

- 杆臂值后面对应的3个指为3个坐标值的精度误差。

杆臂值就是对应的天线在 PP7 坐标系中的坐标，单位为米；PP7坐标系的xy轴平面已经印在了PP7上表面，z轴为垂直于xy坐标平面过原点向上。配置时，请根据实际情况进行更改。

PP7配置使用的是单根网线进行数据传输模式，在配置完PP7后，USB线缆可以从Linux主机上移除了。PP7配置的IP地址为192.168.10.4 。通过以上的配置命令进行配置的。这里顺便说一下HW3.0的网络配置。
HW3.0网络配置：

- `以太网1：manual模式，IP地址192.168.10.6，子网掩码255.255.255.0，网关192.168.10.1，DNS192.168.10.1`

- `以太网2：mamual模式，IP地址192.168.20.8，子网掩码255.255.255.0，网关空，DNS空`
