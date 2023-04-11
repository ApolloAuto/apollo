# 开放空间规划分析服务

## 概述

开放空间分析服务是一项基于云的服务，用于评估来自道路测试或仿真记录的开放空间的规划轨迹。

## 先决条件

- [Apollo](https://github.com/ApolloAuto/apollo) 6.0或以上版本。

- 根据[文档](https://github.com/ApolloAuto/apollo/blob/master/docs/Apollo_Fuel/apply_bos_account_cn.md)注册的百度云BOS服务

- 在[Apollo Dreamland](http://bce.apollo.auto/user-manual/fuel-service)上的Fuel服务账户。

## 主要步骤

- 数据收集

- 任务提交

- 结果分析


## 数据收集

### 数据记录

完成一个带有开放空间规划的自动驾驶场景，例如代客泊车，PullOver，停车和离开。

### 数据真实性检查

### 数据真实性检查

- **在提交给云服务之前，确保记录中包括以下channels**：

| 模块         | channel                        | items                      |
| ------------ | ------------------------------ | -------------------------- |
| Canbus       | `/apollo/canbus/chassis`       | 退出时没有错误信息         |
| Control      | `/apollo/control`              | 退出时没有错误信息         |
| Planning     | `/apollo/planning`             | -                          |
| Localization | `/apollo/localization/pose`    | -                          |
| GPS          | `apollo/sensor/gnss/best_pose` | `sol_type` 是 `NARROW_INT` |

- 你可以用`cyber_recorder`检查：

```
    cyber_recorder info xxxxxx.record.xxxxx
```

![](images/profiling_channel_check.png)

## 任务提交

### 上传数据到BOS

以下是任务提交的文件夹结构要求：

1. 一个包含执行开放空间规划师方案的网络记录文件。

1. 一个配置文件`vehicle_param.pb.txt`；在`apollo/modules/common/data/vehicle_param.pb.txt`下有一个示例文件。

### 在Dreamland中提交任务

进入[Apollo Dreamland](http://bce.apollo.auto/)，用**百度**账号登录，选择`Apollo Fuel --> Jobs`，`New Job`，`Open Space Planner Profiling`，并按照[上传数据到BOS](###上传数据到BOS)部分输入正确的BOS路径：

![profiling_submit_task1](images/open_space_job_submit.png)

## 结果分析

- 任务完成后，每项任务会有一封邮件，包括 "评分结果 "和 "可视化结果"。

![profiling_submit_task1](images/profiling_email.png)
