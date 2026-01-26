本文档介绍在 林肯 MKZ 上适配 apollo 的定位模块配置。前面已经硬件集成了捷联惯性导航设备 PP7，并对 PP7 设备进行了配置。本文档对 apollo 定位模块进行相应的配置。

## 前提条件

- 完成了 [循迹搭建--车辆集成](../应用实践/车辆集成教程/循迹搭建/车辆集成.md)。

- 完成了 [循迹搭建--捷联惯性导航设备PP7配置](../应用实践/车辆集成教程/循迹搭建/惯导设备PP7配置.md)。

- 拥有 RTK 基站账号。

## apollo定位模块配置之系统文件配置

系统文件配置主要包括两部分，分别为 `gnss_conf.pb.txt` 文件配置 和 `localization.conf` 文件配置。

| 文件名称              | 文件位置                                                              | 说明       |
| ----------------- | ----------------------------------------------------------------- | -------- |
| gnss_conf.pb.txt  | modules/calibration/data/车型目录/gnss_conf/gnss_conf.pb.txt          | gnss配置文件 |
| localization.conf | modules/calibration/data/车型目录/localization_conf/localization.conf | 定位模块配置文件 |

### gnss_conf.pb.txt 文件配置

以下为 `gnss_conf.pb.txt` 文件内容，只需要将自己的基站信息中的 IP 地址、端口号、挂载点、用户名和用户密码填写到 rtk_from 的 ntrip 对应的项中即可，其他内容无需更改。

> 注意：若您在 [循迹搭建--捷联惯性导航设备PP7配置](../应用实践/车辆集成教程/循迹搭建/捷联惯性导航设备PP7配置.md) 中改变了PP7设备的IP地址和端口号， `gnss_conf.pb.txt` 中的 `data` 模块和 `rtk_to` 模块的 `tcp` 项的 `address` 和 `port` 值也要做相应的更改。

```bash
data {
    format: NOVATEL_BINARY
    tcp {
        address: "192.168.10.4"
        port: 2000
    }
}

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

rtk_to {
    format: NOVATEL_BINARY
    tcp {
        address: "192.168.10.4"
        port: 3000
    }
}

rtk_solution_type: RTK_RECEIVER_SOLUTION
imu_type: G320N
proj4_text: "+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"

tf {
    frame_id: "world"
    child_frame_id: "novatel"
    enable: false
}

# If given, the driver will send velocity info into novatel one time per second
wheel_parameters: "SETWHEELPARAMETERS 100 1 1\r\n"

gpsbin_folder: "/apollo/data/gpsbin"

#########################################################################
# notice: only for debug, won't configure device through driver online!!!
#########################################################################
login_commands: "UNLOGALL THISPORT\r\n"
login_commands: "LOG COM2 GPRMC ONTIME 1.0 0.25\r\n"
login_commands: "EVENTOUTCONTROL MARK2 ENABLE POSITIVE 999999990 10\r\n"
login_commands: "EVENTOUTCONTROL MARK1 ENABLE POSITIVE 500000000 500000000\r\n"
login_commands: "LOG GPGGA ONTIME 1.0\r\n"

login_commands: "log bestgnssposb ontime 1\r\n"
login_commands: "log bestgnssvelb ontime 1\r\n"
login_commands: "log bestposb ontime 1\r\n"
login_commands: "log INSPVAXB ontime 0.5\r\n"
login_commands: "log INSPVASB ontime 0.01\r\n"
login_commands: "log CORRIMUDATASB ontime 0.01\r\n"
login_commands: "log RAWIMUSXB onnew 0 0\r\n"
login_commands: "log INSCOVSB ontime 1\r\n"
login_commands: "log mark1pvab onnew\r\n"
login_commands: "log rangecmpb ontime 0.2\r\n"
login_commands: "log rangecmpb_1 ontime 0.2\r\n"

login_commands: "log rangeb ontime 0.2\r\n"
login_commands: "log bdsephemerisb\r\n"
login_commands: "log gpsephemb\r\n"
login_commands: "log gloephemerisb\r\n"
login_commands: "log bdsephemerisb ontime 15\r\n"
login_commands: "log gpsephemb ontime 15\r\n"
login_commands: "log gloephemerisb ontime 15\r\n"

login_commands: "log imutoantoffsetsb once\r\n"
login_commands: "log vehiclebodyrotationb onchanged\r\n"
login_commands: "log headingb onchanged\r\n"

logout_commands: "EVENTOUTCONTROL MARK2 DISABLE\r\n"
logout_commands: "EVENTOUTCONTROL MARK1 DISABLE\r\n"
```

### Localization.conf 文件配置

对`localization.conf`文件的配置，只需要根据自己 PP7 安装的具体情况更改`localization.conf`文件中相应项的值即可。

打开`localization.conf`文件，找到对应的项，根据自己的实际情况进行对应的修改，具体修改项请参看下表：

> 注意：如果该配置文件没有进行正确配置，可能会对之后的传感器标定、虚拟车道线制作等功能产生影响。

| 参数                               | 说明                                                                                                                                                |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| lidar_height_default             | 参数值修改为 lidar 中心到地面的距离。单位：m                                                                                                                        |
| local_utm_zone_id                | 需要用户查询所在地区的 utm_zone，并进行修改。例如，北京地区 utm_zone 为 50。utm_zone 的查询可参见 [UTM zone](https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#)。 |
| imu_to_ant_offset_x              | x 轴方向杆臂值。单位：m，杆臂值测量方法参见 `循迹搭建--车辆集成`。                                                                                                             |
| imu_to_ant_offset_y              | y 轴方向杆臂值。单位：m，杆臂值测量方法参见 `循迹搭建--车辆集成`。                                                                                                             |
| imu_to_ant_offset_z              | z 轴方向杆臂值。单位：m，杆臂值测量方法参见 `循迹搭建--车辆集成`。                                                                                                             |
| --enable_lidar_localization=true | 修改为 `--enable_lidar_localization=false`                                                                                                           |

## 检查定位模块能否正常启动

将车辆移至室外平坦开阔处，按顺序执行如下操作。

### 1. 编译项目，启动 Dreamview+

您可以使用包管理或源码方式启动并打开 Dreamview+：

#### 1）启动 Apollo Docker 环境

- 如果您使用的是包管理，请您按照以下步骤操作：
  
  1. 打开命令行终端，切换路径到 Apollo：
     
     ```bash
     cd ~/apollo
     ```
  
  2. 输入以下命令进入 Apollo Docker 环境：
     
     ```bash
     aem start #创建容器
     aem enter #进入容器
     buildtool build
     ```

> 注意：首次进入容器用`buildtool build`；如果是只编译更新的模块，使用以下命令`buildtool build -p 模块名`。

     如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
    
     ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7e7a575.png)

- 如果您使用的是源码，请您按照以下步骤进入 docker 环境：
  
  1. 打开命令行终端，切换路径到 Apollo：
     
     ```bash
     cd ~/apollo     #切换路径到apollo，注意cd与~/apollo之间有一个空格
     ```
  
  2. 输入以下命令启动 Apollo Docker 环境：
     
     ```bash
     bash docker/scripts/dev_start.sh
     ```
     
     启动 docker 环境：
     
     ```bash
     bash docker/scripts/dev_into.sh
     ```
     
     如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
     
     ```bash
     [user@in-runtime-docker:/apollo]$
     ```

#### 2）启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

- 方式一：包管理方式
  
  通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  aem bootstrap start --plus
  ```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

- 方式二：源码方式
  
  通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  bash scripts/bootstrap.sh start_plus
  ```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`bash scripts/bootstrap.sh stop_plus`，
> - 如果您想要重启 Dreamview+，请输入`bash scripts/bootstrap.sh restart_plus`。

启动成功后，在浏览器输⼊`localhost8888`⽹址打开 Dreamview+ 界面。

### 2. 启动定位模块

1. 在浏览器中打开 `(http://localhost:8888)`。

2. 选择模式为 `Vehicle Test`。

3. 在 Module Controller 标签页启动 GPS、Localization 模块。

### 3. 检查 GPS 信号

请按照以下步骤检查 GPS 信号：

1. 新打开一个命令行终端，输入输入并执行命令 `bash docker/scripts/dev_into.sh` 命令进入 apollo docker 环境。

2. 在新终端中输入并执行命令 `cyber_monitor` 命令，在列出的 channel 信息中使用键盘上下方向键，配合翻页键，选中`/apollo/sensor/gnss/best_pose` channel，按键盘右方向键，进入`/apollo/sensor/gnss/best_pose`channel，查看该channel的`sol_type`字段值是否为`NARROW_INT`。
   
   - 若为 NARROW_INT，则表示 GPS 信号良好；
   
   - 若不为 NARROW_INT，则将车辆移动换个无遮挡的位置再看下，直到出现 NARROW_INT 为止。
   
   ![localization_config_check_gps_1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/03c04075948cfb49a9e225fffad1f5d9.png)
   ![localization_config_check_gps_2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/c7d30987fd4a5fb38980b126ef66fc2e.png)

3. 同样方式进入 （按键盘左方向键为退出此channel）`/apollo/sensor/gnss/imu` channel，确认 IMU 有数据刷新，若有刷新即表明 GPS 模块配置成功。

### 4. 检查定位信号

同样使用 `cyber_monotor` 工具查看，进入 `/apollo/localization/pose` channel，查看是否有数据刷新，若没有，请等待几分钟，直到有数据刷新，若有数据刷新即表明定位模块配置成功。

![localization_config_check_pose_1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/15a7f3c8184fa8b24fd51de6632a5a0f.png)
![localization_config_check_pose_2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/4c3f0947c873c54f469d69d10e8b562c.png)

<!--
##  常见问题

GPS打开后，发现 best_pose, imu, localization/pose 信号没有收到

运行`bash gps.sh`后，可以 cyber_monitor 中观察以下几个信号。

```
/apollo/sensor/gnss/best_pose
/apollo/sensor/gnss/imu
/apollo/localization/pose
```
如果 best_pose 和 IMU 没有信号，请检查 GPS 和 IMU 的配置。
如果 best_pose 和 IMU 有信号，但是 localization/pose 没有信号，请等待 2 分钟。如果还是没有信号，请让车开动几分钟。
-->
