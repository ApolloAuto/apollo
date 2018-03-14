## conti_radar
该驱动基于ROS开发，支持continental ARS。

### 配置
radar的默认配置: [conf/conti_radar_conf.pb.txt](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/radar/conti_radar/conf/conti_radar_conf.pb.txt)
radar启动时，会先根据上述配置文件，向can卡发送指令，对radar进行配置。当接收到的radar状态信息与用户配置信息一致时，才开始解析数据并发送消息。

### 运行
该驱动需要在apollo dock环境中运行。
```bash
# in docker
cd /apollo
source scripts/apollo_base.sh
# 启动
./scripts/conti_radar.sh start
# 停止
./scripts/conti_radar.sh stop
```

### Topic
**topic name**: /apollo/sensor/conti_radar
**data type**:  apollo::drivers::ContiRadar
**channel ID**: CHANNEL_ID_ONE
**proto file**: [modules/drivers/proto/conti_radar.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/conti_radar.proto)
