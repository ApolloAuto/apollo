## nano_radar
该驱动基于Apollo cyber开发，支持纳雷科技SR73F毫米波雷达。

### 配置
radar的默认配置: [conf/nano_radar__front_conf.pb.txt]
radar启动时，会先根据上述配置文件，向can卡发送指令，对radar进行配置。当接收到的radar状态信息与用户配置信息一致时，才开始解析数据并发送消息。

### lanuch
dag_files: "/apollo/modules/drivers/radar/nano_radar/dag/nano_radar.dag"

### Topic
**topic name**: /apollo/sensor/nanoradar/front
**data type**:  apollo::drivers::NanoRadar
**channel ID**: CHANNEL_ID_ONE
