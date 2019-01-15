#### CiDi OBU 车端设备驱动

CiDi(Changsha Inteligent Driving Institute)  has developed a OBU(On board Unit) device that can commuincate with the RSU(Road Side Unit).
This is the driver for the OBU.

该驱动是为长沙智能驾驶研究院研发的OBU设备驱动，它可以同路端设备RSU进行通讯以获得路端设备信息，实现V2X功能。

#### 如何使用

首先，你要获得一个CiDi的OBU，更详细的情况请联系www.cidi.ai.
然后，将OBU通过一个交换机或者路由器连接到工控机(IPC),工控机及OBU应该共用一个DHCP服务器，处于同一个网段。

#### 如何启动和停止
该驱动需要在docker内部执行。

```
# 启动：
bash scripts/cidiv2x.sh start
# 停止：
bash scripts/cidiv2x.sh stop
```

#### topic

**topic name**: /apollo/sensor/cidiv2x  
**data type**: apollo.drivers.CiDiV2X  
**proto file**: [modules/drivers/cidiv2x/proto/cidiv2x.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/cidiv2x/proto/cidiv2x.proto)