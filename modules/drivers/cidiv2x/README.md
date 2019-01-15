#### Drivers for The CiDi OBU device

CiDi(Changsha Inteligent Driving Institute)  has developed a OBU(On board Unit) device that can commuincate with the RSU(Road Side Unit).
This is the driver for the OBU.

#### How to Use it.

First,You should get a CiDi OBU device,for further information,please contact www.cidi.ai.
Then you should connect OBU device to the IPC by a switch,there should be a router or a DHCP server
to asign an IP to the OBU device and the IPC.They should in the same network segement.

#### How to start and stop
It should be run in docker.

```
# start：
bash scripts/cidiv2x.sh start
# stop：
bash scripts/cidiv2x.sh stop
```

#### topic

**topic name**: /apollo/sensor/cidiv2x  
**data type**: apollo.drivers.CiDiV2X
**proto file**: [modules/drivers/cidiv2x/proto/cidiv2x.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/cidiv2x/proto/cidiv2x.proto)