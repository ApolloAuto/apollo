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

以上准备工作完成后, 在`modules/tools/calibration`中按顺序完成如下工作

- 采集数据
- 处理数据
- 绘制结果
- 转换结果为`protobuf`格式

### 采集数据
1. 为不同的命令运行`python data_collector.py {dir}/{command}.txt`，且每条命令运行多次，其中data_collector.py在modules/tools/calibration/，{command}.txt可以参考modules/tools/calibration/calibration_data_sample/中样例
2. 根据车辆反应情况，调整命令脚本
3. 运行 `python plot_data.py {dir}/{command}.txt_recorded.csv>` 使采集到的数据可视化


### 处理数据
对每个记录的日志分别运行`process_data.sh {dir}`，其中dir为{command}.txt_recorded.csv所在的目录。每个数据日志被处理并追加到`result.csv`

### 绘制结果
通过运行`python plot_results.py result.csv`得到可视化最终结果，检查是否有异常

### 转换结果为`protobuf`格式
如果一切正常，运行`result2pb.sh`，把校准结果result.csv转换成控制模块定义的`protobuf`格式
