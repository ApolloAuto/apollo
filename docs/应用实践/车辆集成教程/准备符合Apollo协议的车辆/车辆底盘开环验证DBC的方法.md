上一篇文档介绍了利用 Apollo 工具生成 CANBUS 适配代码。为了验证 CANBUS 适配代码的正确性，本文档介绍如何以车辆底盘开环的方式，验证 DBC 是否正常运行。

## 前提条件

您需要先阅读以下内容：

- [概览](../应用实践/车辆集成教程/准备符合Apollo协议的车辆/概览.md)

- [符合 Apollo 线控标准的车辆](../应用实践/车辆集成教程/准备符合Apollo协议的车辆/符合Apollo线控标准的车辆.md)

- [利用 Apollo 工具生成 CANBUS 适配代码](../应用实践/车辆集成教程/准备符合Apollo协议的车辆/利用Apollo工具生成CANBUS适配代码.md)

## 车辆底盘开环验证 DBC

### 开环测试

在确定了车辆底盘 DBC 后，对 DBC 内定义的信号进行开环测试。开环测试的主要目的是测试车辆线控信号与车辆的实际功能是否相符，测试车辆的转向、加减速性能响应是否满足车辆的线控需求，测试车辆接管逻辑是否满足要求。

底盘的开环单侧中，开发者要针对前文所述的DBC重点要求进行测试，如车辆横纵向使能独立性，横纵向接管的独立性，每个控制信号是否满足控制要求和控制边界，反馈信号是否反馈正确。在性能上，控制信号的转向和加减速延迟是否满足 Apollo 控制性能要求，超调误差是否在线控列表范围内。请根据线控列表内的性能要求，对控制信号进行测试。

### teleop 底盘联调测试

底盘联调测试就是通过将 Apollo 与车辆进行 canbus 通信后，测试 Apollo 下发控制信号（如：加速/减速/转向/使能等）是否能够准确控制车辆，测试车辆的底盘反馈信号（如：当前踏板百分比反馈/当前转角反馈/使能反馈/接管反馈等）是否与反馈了车辆的实际状态，验证 Apollo 下发的控制指令，车辆底盘能够准确执行。

下文介绍 teleop 测试工具。

#### teleop 测试工具

Apollo 里提供了一个 teleop 的测试工具，在 `apollo/modules/canbus/tools/teleop.cc`，在 `term` 内输入以下命令：

```
bash scripts/canbus_teleop.sh
```

即可进入 teleop 界面，如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_531fa30.png)

按 `h` 键，可以调出上图所示的帮助界面，可以查询 Teleop 工具的使用方法，下面简单对 teleop 各个控制指令介绍下。

- **Set Action 执行 Apollo 对车辆的使能控制**

  - 按 `m` 和 `0` 键组合，表示执行 reset 指令，车辆退出自动驾驶模式，

  - 按 `m` 和 `1` 键组合，表示执行 start 指令，车辆进入自动驾驶模式。

- **Set Gear 表示设置档位，按 `g` 和 `数字` 组合，进行相应档位设置**

  - 按 `g`+`0` 挂入 N 档（空挡），

  - 按 `g`+`1` 挂入 D 档（前进挡），

  - 按 `g`+`2` 挂入 R 档（倒车档），

  - 按 `g`+`3` 挂入 P 档（驻车档），

  其它档位控制指令暂不需要，根据 DBC 要求，一般车辆控制档位指令就这几个。

- **Throttle/Speed up 表示每次增加油门踏板量 2%，车辆加速**

  按 `w` 键增加油门踏板 2%，使车辆加速。如果当前已经执行 brake 刹车指令，按 `w` 表示减少刹车踏板量 2%。

  油门踏板量的控制范围是 0~100%，即 100% 时相当于油门踏板全部踩下。默认每按 `w` 键一次，油门踏板量增加 2%，这个值您可以根据实车测试进行修改。根据经验，每次变化 2% 比较合适。

  > 注意：请先用 teleop 执行挂 D 档后再执行加油命令，请在开放场地测试，注意安全。

- **Set Throttle 设置油门踏板百分比**

  按 `t`+`数字` 可以直接设置具体的油门踏板百分比，油门踏板可设置的百分比数为 0~100。如执行 t20，表示直接设置当前油门踏板量为 20%，并将刹车踏板百分比置为 0，这一点与实际开车逻辑一致，如果踩下油门踏板，就不能踩刹车踏板。

  > 注意：直接设置油门踏板百分比时，注意每次不要设置太大，开放场地测试，注意安全。请在车辆为 D 档状态下执行该命令。

- **Brake/Speed down 表示每次增加油门踏板量 2%，车辆加速**

  按 `s` 键增加刹车踏板百分比，使车辆减速。如当前已经执行 throttle 加速指令，按 `s` 键表示减少油门踏板百分比。

  > 注意：请先用 teleop 执行挂 D 档后再执行加油命令，请在开放场地测试，注意安全。

- **Set Brake 设置刹车踏板百分比**

  按 `b`+`数字` 可以直接设置具体的刹车踏板百分比，刹车踏板可设置的百分比数为 0~100。如执行 b20，表示直接设置当前刹车踏板量为 20%，并将油门踏板百分比置为 0，这一点与实际开车逻辑一致，如果踩下刹车踏板，就不能踩油门踏板。

  > 注意：直接设置油门踏板百分比时，注意每次不要设置太大，开放场地测试，注意安全。请在车辆为 D 档状态下执行该命令。

- **Steer LEFT 表示方向盘每次向左转 2%**

  按 `a` 键表示每次向左转2%的方向盘最大转角，具体转动角度应根据车辆设置的最大方向盘转角乘以 2% 进行换算。

  该指令执行可以在车辆静止时执行，也可以在车辆启动后执行。

- **Steer RIGHT 表示方向盘每次向右转 2%**

  按 `s` 键表示每次向右转 2% 的方向盘最大转角，具体转动角度应根据车辆设置的最大方向盘转角乘以 2% 进行换算。

  该指令执行可以在车辆静止时执行，也可以在车辆启动后执行。

- **Parking Brake 打开电子手刹**

  按 `P` 键（注意是大写 P）可以手动控制车辆电子手刹开关。这个功能根据车辆的是否提供了电子手刹的控制接口而实现。

  > 注意：执行电子手刹开启或释放时，请将车辆用 teleop 设置为 P 档状态。

- **Emergency Stop 紧急停车**

  按 `E` 键（注意是大写 E）可以进行车辆紧急停车，默认执行 50% 刹车。

  建议您在测试时尽量少用此功能，体感差，调试车辆时多注意周围情况。发生突发情况时及时用外接踩刹车踏板的方式进行手动接管车辆。

#### 诊断工具

了解 teleop 的基本操作后，根据相应的指令，对车辆执行具体的控制命令，然后通过 Apollo 的可视化监控工具 `cyber_monitor` 查看车辆当前的反馈信号，确认控制下发后车辆的执行结果是否正确。

在 Apollo 里提供了一个可视化的监控工具，可以用来监控底盘 `chassis` 和 `chassis_detail` 信息，通过执行：

```
cyber_monitor
```

在 `apollo/modules/canbus/conf/canbus.conf` 文件内：
修改配置 `--noenable_chassis_detail_pub` 为 `--enable_chassis_detail_pub`，表示在打开 `chassis_detail` 底盘详细信息，即可以查看底盘反馈信号的每一帧报文原始信息。

修改配置 `--receive_guardian` 为 `--noreceive_guardian`，即可以关闭 guardian 模式，进入 canbus 的调试模式，这样 teleop 时就能够控制车辆了。如下图所示是修改 `canbus_conf` 配置文件。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_0f6580b.png)

打开 `cyber_monitor` 界面如下：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_15359f3.png)

可以看到 `chassis` 和 `chassis_detail` 均有数据，频率约 100Hz。`chassis` 下数据主要是车辆的实际转速、实际踏板百分比的反馈，实际方向盘转角百分比，实际档位反馈等信息，在 `chassis_detail` 是底盘上传给 Apollo 的反馈报文（即 Apollo 接收底盘全部报文）的全部信息。
如下图所示为 `chassis` 信息：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_5eae9f5.png)

下图所示为 `chassis_detail` 信息：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_5095afc.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_ba05bf2.png)
