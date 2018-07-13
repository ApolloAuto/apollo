# CAN总线

## 引言
CAN总线接受并执行控制命令，并收集底盘状态作为给控制模块的反馈。

## 输入
* 控制命令

## 输出
* 底盘状态
* 底盘细节状态

## 实现
CAN总线模块的主要部件有：
* 包括车辆控制器和消息管理器的车辆

* (客户端可以移动到‘/modules/drivers/canbus’，因为它是被不同的使用CAN总线协议的传感器共享的)

您自己的CAN客户端可以通过继承“CanClient”类在*can_client*的文件夹中实现。记得在“CanClientFactory”注册你的客户机客户端。

您自己的车辆控制器和消息管理器可以通过“VehicleController”和“MessageManager”的继承在“vehicle”的文件夹中实现。记得在“VehicleFactory”注册你的车辆。
