## sonic_radar
该驱动基于ROS开发，支持ultra sonic ARS。

### 配置
radar的默认配置: [conf/sonic_radar_conf.pb.txt](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/radar/sonic_radar/conf/sonic_radar_conf.pb.txt)
radar启动时，会先根据上述配置文件，向can卡发送指令，对radar进行配置。当接收到的radar状态信息与用户配置信息一致时，才开始解析数据并发送消息。

### 运行
该驱动需要在apollo dock环境中运行。
```bash
# in docker
cd /apollo
source scripts/apollo_base.sh
# 启动
./scripts/sonic_radar.sh start
# 停止
./scripts/sonic_radar.sh stop
```

### Topic
**topic name**: /apollo/sensor/sonic_radar
**data type**:  apollo::drivers::Ultrasonic
**proto file**: [modules/drivers/proto/sonic_radar.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/sonic_radar.proto)

