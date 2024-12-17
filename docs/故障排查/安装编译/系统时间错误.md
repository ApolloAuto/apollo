### 问题描述:

每次开机之后，时间会回到2023.3.28号，需要重新设置系统时间

### 问题原因:

由于使用了gps授时导致。因为之前配置了gps授时，平时调试时默认使用了内部gps授时

### 解决方案:

尝试通过两种方式修改时钟：
1. 打开ubuntu系统设置setting -> Details -> Date &Time, 打开Automatic Date & Time
2. 记录当前时间，直接命令行 sudo date -s "2023-08-07 12:00:00"

上车使用后，执行~/apollo-park-generic/start.sh 改用GPS授时
