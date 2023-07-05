## 如何解决ESD CAN设备的故障？


### 问题

不能通过ESD卡进行通信。

### 故障排除步骤：
1. 确保CAN驱动程序（内核模块）被加载，运行：```lsmod |grep can```; 如果驱动程序已经加载，您应该看到关于内核驱动程序的信息，例如版本号。
2. 确保CAN设备存在并具有正确的权限集，运行：```ls -l /dev/can0```.
3. 检查内核日志(运行 ```dmesg |grep -i can```) 和系统日志 (运行 ```grep -i can /var/log/syslog```), 查看是否存在与CAN相关的错误消息。
4. 运行Apollo程序 ```esdcan_test_app``` (在```monitor/hwmonitor/hw/tools/```下), 它将列出详细统计数据和状态信息。
    * 了解这个工具，运行 ```esdcan_test_app --help```.
    * 列举其他详细信息, 运行 ```sudo esdcan_test_app --details=true```.
5. [可选] 保存内核日志，系统日志（步骤4），并从步骤5的输出中进行离线分析。
6. 必要时，检查系统环境温度。ESD CAN卡（CAN-PCIe/402-FD）的工作温度范围为0～75摄氏度，在该温度范围外可能无法工作。您还可以在ESD CAN（Altera FPGA芯片）上附加一个温度传感器到主IC芯片的表面，以监测芯片的表面温度，以确保其不会过热。
