# 控制参数自动调优服务


- [控制参数自动调优服务](#控制参数自动调优服务)
  - [概述](#概述)
  - [先决条件](#先决条件)
  - [主要步骤](#主要步骤)
  - [百度云存储BOS注册](#百度云存储BOS注册)
  - [开设云服务账户](#开设云服务账户)
  - [任务配置文件设置](#任务配置文件设置)
    - [任务配置Protocol Buffers](#任务配置Protocol Buffers)
    - [任务配置文件示例](#任务配置文件示例)
    - [任务配置详细解释](#任务配置详细解释)
    - [自定义动态模型指导](#自定义动态模型指导)
  - [任务提交](#任务提交)
    - [任务配置文件存储](#任务配置文件存储)
    - [通过网页提交任务](#通过网页提交任务)
  - [任务结果获取](#任务结果获取)
    - [接收任务结果邮件](#接收任务结果邮件)
    - [任务结果分析](#任务结果分析)

## 概述

控制参数自动调优服务利用机器学习方法对Apollo控制模块中使用的PID、LQR、MPC、MRAC等控制器的控制参数进行自动优化，在离线仿真环境下实现控制器调优的全自动化，节省了大量的人工试验与道路试验。它与多个Apollo在线服务工具集成，包括动态建模、仿真平台和控制剖析。

控制参数自动调优服务：1）迭代生成新的控制参数，并通过调用后台Apollo仿真服务来评估生成的参数，其中预训练的车辆动态模型被用来实现环内控制仿真；2）仿真结果通过后台控制剖析服务进行评估。3）此外，控制剖析结果中的几十个控制指标被加权并合并为一个加权分数，以这个分数为优化目标，自动调优服务在有希望的区域不断搜索具有更好分数的新控制参数，直到达到给定的步骤。  

## 先决条件

控制参数自动调优是在control-in-the-loop仿真环境下执行的，因此要求用户提供自己的车辆动态模型（即动态建模服务的结果）进行仿真；否则，该服务将使用基于MKZ车辆模型的默认车辆动态模型。因此，对于定制车辆的控制参数训练服务，一些预先要求的步骤列举如下：

- [Apollo](https://github.com/ApolloAuto/apollo) 6.0或更高版本

- 在[Apollo Dreamlan](http://bce.apollo.auto/)注册的云服务和仿真服务

- [动态建模](../Dynamic_Model/README.md)服务


## 主要步骤

- 任务配置文件设置

- 任务提交

- 任务结果获取

## 百度云存储BOS注册

注册请参考[百度云存储BOS注册及用户手册](../../Apollo_Fuel/apply_bos_account_cn.md)

**注意：**客户必须使用已注册的 "bucket"，并确保 "Bucket Name"、"Backet Area "与注册时的相同。

## 开设云服务账户

请与业务部门联系开设云服务账户，并提供上一步提到的 "Bucket Name"、"Backet Area"。 

## 任务配置文件设置

### 任务配置Protocol Buffers

任务配置应以 "XXX_tuner_params_config.pb.txt "的形式提供，其中的Protocol Buffers文件如下：
   ![](README_cn.assets/tuner_param_config_proto.png)

### 任务配置文件示例

根据上面显示的Protocol Buffers设置，配置文件的例子显示如下：
   ![](README_cn.assets/tuner_param_config_pb_txt.png)

### 任务配置详细解释

对XXX_tuner_params_config.pb.txt中的信息的详细解释如下：

| Message                               | Detailed Explanations                                        | Notes                                                        |
| ------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `git_info.repo`                       | 将在Apollo仿真平台上执行的 repo 名称                         | 仿真中使用的动态模型必须按照[[自定义动态模型指导](#自定义动态模型指导)]放在该github repo的设计路径中 |
| `git_info.commit_id`                  | 将在Apollo仿真平台上执行的提交ID                             | 如果为空，默认情况下，将使用最新的提交ID                     |
| `tuner_parameters.user_tuning_module` | 设置为**CONTROL**                                            | 必须是**CONTROL**，否则不能通过任务合理性检查                |
| `tuner_parameters.user_conf_filename` | `git_info.repo`中的控制配置文件，将在Apollo仿真平台中执行    | 调优后的参数和标志必须包括在配置文件中                       |
| `tuner_parameters.n_iter`             | 用来搜索优化控制参数的迭代步数。建议的数值。**n_iter=200用于1-2个调优参数；n_iter=300用于3-4个调优参数；n_iter=400用于5-6个调优参数；n_iter=500用于7-8个调优参数；n_iter=600或以上用于9个以上调优参数** | 迭代步骤越多，优化精度越高（但训练过程越慢）；必须是**<1000**，否则不能通过任务合理性检查 |
| `tuner_parameters.opt_max`            | 设置为**True**                                               | 必须是**True**，否则不能搜索优化的控制参数                   |
| `tuner_parameters.flag`               | （repeated message）在用户的控制配置文件中设置用户需要的尽可能多的标志（boolen参数）。你设置的标志值（真/假）将覆盖控制配置文件中的默认标志值 | flags参数不被算作**调优的参数**                              |
| `tuner_parameters.parameter`          | （repeated message）在用户的控制配置文件中设置用户需要的参数，数量不限。如果用户设置了参数的**常数**属性，那么这些参数将被视为**常数参数**，设置的常数值将覆盖控制配置文件中的默认值；如果用户设置了参数的**最小**和**最大**属性，那么这些参数将被视为**调优参数**，自动调优服务将试图通过**最小**和**最大**限制的范围搜索最佳参数值 | 调优的参数越多，可能需要的优化迭代步数就越多（自动调优过程也越长越慢） |
| `scenarios.id`                        | （repeated message）根据用户在参数自动调优中进行控制性能评估的需要，设置尽可能多的场景ID。建议的IDs：**11014, 11015, 11016, 11017, 11018, 11019, 11020** | 用户也可以从Apollo仿真平台的公共场景中选择任何可用的场景ID。场景ID越多，自动调优过程可能越慢 |
| `dynamic_model`                       | 设置为**ECHO_LINCOLN**，只有当`git_info.repo`被设置为Apollo官方repo时才会出现 | 如果在`git_info.repo`中使用了自定义的repo，则忽略。          |

### 自定义动态模型指导

如果用户打算在仿真中使用自己的动态模型，那么请在github **apollo/modules/control/conf/dynamic_model_forward.bin**提供前向驱动模型；在github **apollo/modules/control/conf/dynamic_model_backward.bin**提供后向驱动模型。请参考[动态模型](../../Apollo_Fuel/Dynamic_Model/README.md)，了解如何生成定制的动态模型的指导。


## 任务提交

### 任务配置文件存储

在使用自动调优服务之前，首先根据[百度云存储BOS注册及用户手册](../../Apollo_Fuel/apply_bos_account_cn.md)设置输入文件存储，如[百度云存储BOS注册](#百度云存储BOS注册)中所示。然后，将设计好的XXX_tuner_params_config.pb.txt文件放到用户的BOS文件夹下的任何地方。 

### 通过网页提交任务

登录[Apollo网页](http://bce.apollo.auto/)，在功能菜单中选择**Apollo Fuel -> New Job**。在**New Job**菜单中选择**Control Auto Tuning**选项，然后在**Input Data Path**中填写任务配置文件路径，从用户BOS文件夹下的根目录开始（注意：**输入数据路径必须包括完整的配置文件名称**，例如，'input/XXX_tuner_params_config.pb.txt' ）。最后，通过点击**Submit Job**按钮提交你的工作。

![](README_cn.assets/control_auto_tuning_webpage.png)

## 任务结果获取

### 接收任务结果邮件

 - 在控制参数自动调优工作成功启动后，用户的任务配置文件将被测试，以进行合理性检查程序。如果不能通过合理性检查，那么用户将收到**失败通知邮件**到你注册的电子邮件地址，并说明详细的失败原因。

 - 如果控制参数自动调优工作成功地通过合理性检查程序，那么在工作完全结束后，用户将收到**结果报告电子邮件**，其中参数自动调优结果和链接的配置文件将作为附件提供。 

- 结果电子邮件

![](README_cn.assets/tuner_results_email.png)

### 任务结果分析

- 通过访问电子邮件中的报告表或附件中的tuner_results.txt文件，可以获得控制自动调优结果，详细解释如下：

| Message             | Detailed Explanations              | Notes                     |
| ------------------- | ---------------------------------- | ------------------------- |
| `Tuner_Config`      | 任务配置文件的来源                 |                           |
| `Base_Target`       | 最佳控制参数的加权得分             |                           |
| `Base_Params`       | 最佳控制参数                       |                           |
| `Optimization_Time` | 整个自动调优优化过程的总体时间消耗 | 单位: seconds             |
| `Time_Efficiency`   | 单个优化迭代步骤的平均时间消耗     | 单位: seconds / iteration |