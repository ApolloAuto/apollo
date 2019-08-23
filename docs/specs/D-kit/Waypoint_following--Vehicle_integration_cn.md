车辆集成
===================

## 目录
      
 - [概览](#概览)
 - [工控机系统安装](#工控机系统安装)
    - [工控机硬件安装](#工控机硬件安装)

## 概览

该用户手册旨在帮助用户在教学小车上安装、配置硬件。

## 工控机系统安装

在集成车辆之前，首先需要完成工控机的硬件安装，如CAN卡安装；之后需要完成工控机的软件安装，包括Ubuntu Linux安装、Apollo软件系统安装等。

### 工控机硬件安装

工控机硬件安装包括CAN卡的安装和相关线缆的连接，以及BIOS相关的设置。

#### IPC配置

参考下述IPC配置：

- Nuvo-6108GC-GTX1080-E3-1275

- DDR4-16GB-ECC

- 三星 SSD 256GB

- PA-280W-OW

  ![IPC-6108GC-front-side](../images/ipc_hardware_before_cover.jpeg)
  
#### 准备IPC

参考下述步骤：

##### 准备好CAN卡并进行安装
在Nuvo-6108GC中，GTX1080显卡被预先安装在一个PCI插槽中，我们需要将CAN卡安装在另外一个PCI插槽中。

   a. 找到并拧下机器边上的8个螺丝（显示在棕色方框内或棕色箭头指向的区域）

   ![Positions_of_Screws](../images/ipc_hardware_cover1.png)

   b. 移除机器盖板

   ![removing the cover](../images/ipc_hardware_cover2.jpeg)

   在机箱底部将能看到固定着的3个PCI插槽（其中一个已经被显卡占据）
   
  ![Before installing the CAN card](../images/ipc_hardware_slot1.png)
  
   c. 设置CAN卡的终端跳线：将4个红色的跳线帽从原位置移除并放置在终端位置，如下图所示：

   ![prepare_can_card2](../images/ipc_hardware_slot2.png)

   ![warning_icon](../images/warning_icon.png)**WARNING**：如果终端跳线没有被正确设置，CAN卡将不能正确工作。

   d. 将CAN卡插入到一个PCI插槽中

   ![installed CAN](../images/ipc_hardware_slot3.png)

   e. 安装IPC的盖板
    
   ![IPC-6108GC-Screw-Positions](../images/ipc_hardware_after_cover.png)

##### 配置IPC加电组件

   a. 将电源线接入到为IPC配置的电源连接器（接线板）
   
   ![warning_icon](../images/warning_icon.png)**WARNING**：确保电源线的正极（标记为 **R** 表示红色）和负极（标记为 **B** 表示黑色）接入到了IPC接线板的正确接口，如下图所示：

   ![ipc_power_RB](../images/ipc_hardware_rb.png)
   
   b. 将显示器、以太网线、键盘和鼠标接入IPC
   
   ![CableConnected-overexposed](../images/ipc_hardware_connection.png)
  
##### 启动计算机

![warning](../images/warning_icon.png)如果系统接入了一个或多个外部插入卡，建议通过BIOS设置风扇的转速

```
- 计算机启动时按F2进入BIOS设置菜单
- 进入 [Advanced] => [Smart Fan Setting]
- 设置 [Fan Max. Trip Temp] 为 50
- 设置 [Fan Start Trip Temp] 为 20
```

![tip_icon](../images/tip_icon.png)建议使用者使用数字视频接口（DVI）连接器连接显卡和显示器。设置投影到主板的DVI接口，参考下述的设置步骤：

```
- 计算机启动时按F2进入BIOS设置菜单
- 进入 [Advanced]=>[System Agent (SA) Configuration]=>[Graphics Configuration]=>[Primary Display]=> 设置为 "PEG"
```

![tip_icon](../images/tip_icon.png)建议设置IPC的运行状态为一直以最佳性能状态运行：

```
- 计算机启动时按F2进入BIOS设置菜单
- 进入 [Power] => [SKU POWER CONFIG] => 设置为 "MAX TDP"
```

##### 连接电源

![IPC-6108GC-PowerCable.JPG](../images/ipc_hardware_power_on.jpeg)