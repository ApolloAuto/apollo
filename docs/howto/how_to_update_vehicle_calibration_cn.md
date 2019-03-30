# 如何标定车辆油门和制动

## 介绍

车辆校准的目的是找到准确产生从控制模块请求的加速量的油门和制动命令
## 准备

按如下顺序完成准备工作:
- 访问相关代码
- 改变驾驶模式
- 选择测试地点

### 访问相关代码
* Canbus, 包括以下模块:
  * GPS 驱动
  * 定位

### 改变驾驶模式
  在`modules/canbus/conf/canbus_conf.pb.txt`中，设置驾驶模式为 `AUTO_SPEED_ONLY`.

### 选择测试地点
  理想的测试地点是平坦的长直路

## 更新车辆标定

以上准备工作完成后, 在`modules/tools/vehicle_calibration`中按顺序完成如下工作

- 采集数据
- 处理数据
- 绘制结果
- 转换结果为`protobuf`格式

### 采集数据
1. 运行 `python data_collector.py`,参数如 x y z, x 代表了加速的控制指令, y 代表了限制速度(mps), z 是减速指令,正值标识油门量，负值标识刹车量.且每条命令运行多次，其中 `data_collector.py`在modules/tools/vehicle_calibration/
2. 根据车辆反应情况，调整命令脚本
3. 运行 `python plot_data.py ` 使采集到的数据可视化

比如输出指令 `15 5.2 -10`,将会生成名为`t15b-10r0.csv`的文件。

### 处理数据
对每个记录的日志分别运行`process_data.sh {dir}`，其中dir为`t15b-10r0.csv`所在的目录。每个数据日志被处理成`t15b-10r0.csv.result`。

### 绘制结果
通过运行`python plot_results.py t15b-10r0.csv`得到可视化最终结果，检查是否有异常

### 转换结果为`protobuf`格式
如果一切正常，运行`result2pb.sh`，把校准结果result.csv转换成控制模块定义的`protobuf`格式
