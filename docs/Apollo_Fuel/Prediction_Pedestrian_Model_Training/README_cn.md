# 预测行人模型训练服务

## 概述

预测行人模型训练服务是一项基于云的服务，根据你提供的数据训练行人的预测模型，以更好地适应你环境中的行人行为。

## 先决条件


- [Apollo](https://github.com/ApolloAuto/apollo) 6.0或以上版本。

- 根据[文档](https://github.com/ApolloAuto/apollo/blob/master/docs/Apollo_Fuel/apply_bos_account_cn.md)注册的百度云BOS服务。

- 在[Apollo Dreamland](http://bce.apollo.auto/user-manual/fuel-service)上的Fuel服务账户。

## 主要步骤

- 数据收集

- 提交工作

- 训练好的模型

## 数据收集

### 数据记录

完成几个小时的不同场景的自主驾驶，请确保有一些行人在视线内。用`cyber_recorder`记录数据包。

### 数据真实性检查


- **在提交给云服务之前，请确保record文件中包括以下channels：**

| 模块 | channel | items |
|---|---|---|
| Perception | `/apollo/perception/PerceptionObstacles` | 退出时没有错误信息 |
| Localization | `/apollo/localization/pose` | - |


- 你可以用`cyber_recorder`检查：

```
    cyber_recorder info xxxxxx.record.xxxxx
```

## 工作提交

### 上传数据到BOS

1）`输入数据路径`是**BOS的根文件夹**，供用户使用；

2）在你的`输入数据路径`下应该有两个文件夹，一个名为 "map"，用于你使用的Apollo格式的地图，另一个为 "records"，用于你收集的记录。



#### 在Dreamview中提交工作请求

进入[Apollo Dreamland](http://bce.apollo.auto/login)，用**百度**账号登录，选择`Apollo Fuel-->task`，`new task`，`prediction pedestrian model training`，并按[上传数据到BOS](###上传数据到BOS)部分输入正确路径；


#### 获取训练好的模型

- 工作完成后，你应该收到包括结果在内的电子邮件，训练好的模型将在你的`输入数据路径`下的`models`文件夹中。
