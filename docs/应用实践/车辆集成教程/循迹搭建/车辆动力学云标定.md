车辆云标定是通过采集车辆底盘油门踏板量、刹车踏板量、车辆速度、加速度作为车辆纵向动力模型的输入，通过机器学习的方式，生成相应车辆的踏板标定表。生成的标定表数据量大、精度高，提高车辆的控制精度。车辆踏板标定系统主要包括四个部分：

- 在 `DreamView` 数据采集监视器内采集标定数据；

- 数据本地预处理；

- 标定数据上传至 BOS 云盘，提交标定任务；

- 生成用于性能评估的可视化标定结果。

## 前提条件

1. [循迹搭建--车辆集成](../应用实践/车辆集成教程/循迹搭建/车辆集成.md)

2. [循迹搭建--定位模块配置](../应用实践/车辆集成教程/循迹搭建/定位模块配置.md)

3. [循迹搭建--Apollo 系统安装](../应用实践/车辆集成教程/循迹搭建/Apollo系统安装.md)

4. [开通云服务账号](https://apollo.baidu.com/Apollo-Homepage-Document/Vehicle_Nameplate/三、开通Fuel云服务/3.%20开通Fuel研发云服务账号)

## 步骤一、修改配置文件

在开始标定前，需要根据实际车辆的最高速度、油门、刹车踏板的限制，进行用于车辆踏板标定的数据采集配置，来适配车辆的标定，满足标定数据能够尽可能多的覆盖车辆踏板的有效范围。我们设置了开发套件的默认车辆标定数据采集配置文件，在文件 `apollo/modules/calibration/data/$vehicle_type/dreamview_conf/data_collection_table.pb.txt` 内。对于不同的车型，配置文件的位置不同，主要由车型参数 `$vehicle_type` 决定，根据车辆铭牌信息选择对应的车型。

下面以 MKZ 车辆标定举例说明。MKZ 车辆标定标定配置文件在 `apollo/master/modules/dreamview/conf/data_collection_table.pb.txt` 内。

在您了解标定文件构成后，如有需要可以自行修改需要的数据采集配置文件。通常情况下，使用默认标定配置可以覆盖小车大部分踏板控制范围。**建议您不需要单独修改此配置文件，使用默认标定配置即可**。默认标定配置文件内主要设计了小车的标定条件如下：

车辆前进模型

- 速度条件

  - 低速 0 ～ 10m/s

  - 中速 10 ~ 20m/s

  - 高速 >= 20m/s

- 油门条件

  - 小油门 Throttle deadzone ~ 25%

  - 中油门 25% ~ 30%

  - 大油门 Throttle >= 30%

- 刹车条件

  - 小刹车 Brake deadzone ~ 20%

  - 中刹车 20% ~ 25%

  - 急刹车 Brake >= 25%

上述的 `Throttle deadzone` 表示车辆油门踏板的不产生加速度的无效区段，**开发者可以根据实际车辆的情况修改此参数** ； `Brake deadzone` 表示车辆刹车踏板的不产生减速度的无效区段，一般设在较低的踏板开度时， **开发者可以根据实际车辆的情况修改此参数** 。此参数设置在 `apollo/modules/common/data/vehicle_param.pb.txt` 文件内 `throttle_deadzone` 和 `brake_deadzone` ，在标定开始前，就要先把改参数确定好，后面进行数据训练时上传的配置文件，也需要对应使用该参数。

根据如上标定条件，即可匹配出车辆的所有标定条件，共 12 种条件，分别为：

- 低速小油门

- 低速中油门

- 低速大油门

- 高速小油门

- 高速中油门

- 高速大油门

- 低速小刹车

- 低速中刹车

- 低速急刹车

- 高速小刹车

- 高速中刹车

- 高速急刹车

确认好上述标定条件后，通常情况下使用默认标定配置可以覆盖小车大部分踏板控制范围。如果需要修改配置文件，则需要将 `apollo/master/modules/dreamview/conf/data_collection_table.pb.txt` 中 `Go Straight` 部分配置进行修改，其它部分配置代码无需修改，无需删除，启动 `DreamView` 后重新选择车辆后配置生效，具体修改代码如下：

```text
    frame_threshold: 20 // 帧有效阈值，连续满足标定条件的帧数大于等于该阈值时才会被记录
    total_frames: 6000  // 设置每一个标定case数据帧总量，当前设置为4000帧，可根据实际情况调整，数据量越大采集标定数据时间越长
    scenario {
      key: "Go Straight"
      value {
        feature {
          name: "Forward"
          range {
            criterion {
              field: "gear_location"
              comparison_operator: EQUAL
              value: 1
            }
          }
        }
        feature {
          name: "Steering Angle"
          range {
            name: "Straight"
            criterion {
              field: "steering_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: -1
            }
            criterion {
              field: "steering_percentage"
              comparison_operator: LESS_THAN
              value: 1
            }
          }
        }
        feature {
          name: "Speed"
          range {
            name: "Speed 0 ~ 10mps"
            criterion {
              field: "speed_mps"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 0.2
            }
            criterion {
              field: "speed_mps"
              comparison_operator: LESS_THAN
              value: 10
            }
          }
          range {
            name: "Speed 10 ~ 20mps"
            criterion {
              field: "speed_mps"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 10
            }
            criterion {
              field: "speed_mps"
              comparison_operator: LESS_THAN
              value: 20
            }
          }
          range {
            name: "Speed >= 20mps"
            criterion {
              field: "speed_mps"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 20
            }
          }
        }
        feature {
          name: "Accelerator (Decelerator)"
          range {
            name: "Throttle deadzone ~ 25%"
            criterion {
              field: "throttle_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              vehicle_config: "throttle_deadzone"
            }
            criterion {
              field: "throttle_percentage"
              comparison_operator: LESS_THAN
              value: 25
            }
          }
          range {
            name: "Throttle 25% ~ 30%"
            criterion {
              field: "throttle_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 25
            }
            criterion {
              field: "throttle_percentage"
              comparison_operator: LESS_THAN
              value: 30
            }
          }
          range {
            name: "Throttle >= 30%"
            criterion {
              field: "throttle_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 30
            }
          }
          range {
            name: "Brake deadzone ~ 20%"
            criterion {
              field: "brake_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              vehicle_config: "brake_deadzone"
            }
            criterion {
              field: "brake_percentage"
              comparison_operator: LESS_THAN
              value: 20
            }
          }
          range {
            name: "Brake 20% ~ 25%"
            criterion {
              field: "brake_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 20
            }
            criterion {
              field: "brake_percentage"
              comparison_operator: LESS_THAN
              value: 25
            }
          }
          range {
            name: "Brake >= 25%"
            criterion {
              field: "brake_percentage"
              comparison_operator: GREATER_THAN_OR_EQUAL
              value: 25
            }
          }
        }
      }
    }
```

## 步骤二、标定数据采集

### 1. 准备

完成配置文件修改后，将车辆移动至适合标定的场地后，启动 Apollo，并执行以下命令：

```
  bash apollo.sh build_opt_gpu  # 重新编译使配置更改生效
  bash scripts/bootstrap.sh
```

1. 在浏览器打开 `DreamView (http://localhost:8888)` 。

2. 在 `--setup mode--` 内，选择 `vehicle calibration` （车辆标定）模式。

3. 在 `--vehicle--` 选择相应车型。

   ![vehicle_calibration_select_vehicle](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/9b80be81f6e7be44b3519ccce85f2b37.png)

4. 启动标定所需要的软件模块。

在进行标定前，需要采集包含车辆底盘、定位信息的数据作为输入，可以通过 `DreamView` 界面上的模块开关启动相关模块软件。在 `DreamView` 界面左侧边栏选择 `Module Controller` 标签，然后在主界面启动 `Canbus` 、 `GPS` 、 `Localization` 模块。

![vehicle_calibration_launch_modules](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/bef51bb802c20745fd48cf0af1bd5548.png)

通过左侧 `Components` 查看 `Canbus` 、 `GPS` 、 `RTK` 状态，直到信号都达到 `OK` 才能够进行下一步。 `Localization` 启动后，需要等待 1~2 分钟， `RTK` 状态才能变成 OK。

| channel_name                    | 检查项目                                                 |
| ------------------------------- | -------------------------------------------------------- |
| `/apollo/canbus/chassis`        | 确保能正常输出数据                                       |
| `/apollo/canbus/chassis_detail` | 确保能正常输出数据                                       |
| `/apollo/sensor/gnss/best_pose` | 确保能正常输出数据、 `sol_type:` 选项显示为 `NARROW_INT` |
| `/apollo/localization/pose`     | 确保能正常输出数据                                       |

![vehicle_calibration_check_components.png](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/af6f24204344cd3dcd87a5fffde976e8.png)

5. 进入采集界面。

在确认上一步已经启动的软件模块状态 `OK` 后，点击 `DreamView` 界面左侧边栏 `Tasks` 标签，在 `Others` 区域，打开 `Fuel Client` 开关启动云服务采集界面，然后在界面右上侧选择 `Go Straight` 标签查看。

![vehicle_calibration_fuel_cleint](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a98d5c5f541117c2de2e2ff6af2999c9.png)

### 2. 开始采集

在 `DreamView` 界面点击左侧边栏，选择 `Module Controller`，点击 `Recorder` 开关， Apollo 系统开始录制记录车辆标定数据。

> 注意：
>
> - **必须先打开 `Recorder` 开关**，再进行标定数据采集操作，实际情况常常会因未打开 `Recorder` 开关造成数据未记录保存，影响效率。
> - 如果在打开 Recorder 开关之前，右侧的数据采集器的进度条已经有一些增加，没有关系，因为在打开 `Recorder` 开关后会清除进度条的采集量，重新开始记录。
> - 记录数据时，建议在第一次打开 `Recorder` 开关后，一次性把所有的标定 case 数据全部采集完之后再关闭 `Recorder` 开关，通常一次标定采集数据所用时间为 20-40 分钟，根据熟练程度、场地大小等决定。

![vehicle_calibration_launch_recorder.png](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/8f16dae658a08817efcd071a59f26457.png)

这时手动驾驶车辆，根据 `Dreamview` 右侧数据采集监视器，采集相应标定条件的车辆数据，每符合标定条件一次，则进度条增加一点，直至进度条增加满为止。

![vehicle_calibration_collect_data](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/7d3adf5f4d33e011f9f65005b09ce616.png)

待监视器内各标定条件进度条（**蓝色**）全部采集完成后，点击关闭 `Recorder`，关闭数据记录，结束车辆标定数据采集。

> 注意：原则上进度条全部采集完对标定结果的生成是最佳的，您也可以根据实际情况采集进度条，实际根据场地大小限制，速度限制等。例如，有些 case 很容易采集满，有些很难，对于不好采集到的数据，可以采集到 60-80% 的状态也可以。

![vehicle_calibration_collect_data_complete](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/3590119e3968570e58e1a5ce7509c124.png)

## 步骤三、标定任务提交

### 1. 上传预处理后的数据至 BOS

在上传数据之前，请注意以下几点：

1. 标定数据预处理。

采集完车辆的标定数据，关闭 `Recorder` 后，点击右下角 `「Preprocess」` 按钮，进行标定数据预处理。

![vehicle_calibration_preprocess_data.png](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/5c72c0466b6ea1846684cbde16447b0d.png)

2. 查看预处理结果。

点击 `「Preprocess」` 按钮后，首先对标定数据进行数据完整检查(sanity check)，如果数据完整检查不通过，在进度条下方会有不通过的错误提示（通常情况下，按照文档上述步骤一步步做，完整性检查就不会有错）。

预处理按钮只需要点一次，不需要多次点击。

![vehicle_calibration_preprocess_data_complete](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/30dd95f801d42f3997784a0b6f008b9f.png)

3. 生成结果如下目录。

结束数据完整检查后，会提示将生成的预处理数据存入到 `apollo/output/task-*******`，文件夹以当时处理的时间为命名，所以开发者在 `apollo/output` 目录下查找时间最近的 task 文件夹即可。

如下图所示，拷贝预处理完成后的文件夹 `task-2021-01-04-18-12`，用于标定使用。

![vehicle_calibration_preprocess_data_folder](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/c9f50aa54485e5b6bf93489c44f1ea68.png)

4. 把生成的结果上传至 BOS。

在 [BOS](https://gitee.com/ApolloAuto/apollo/blob/master/docs/Apollo_Fuel/apply_bos_account_cn.md) 的初始目录，上传上一步选择的文件夹 `task-2021-01-04-18-12`。

![vehicle_calibration_upload_bos](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/bf7cc850cd2d96c34962b3350f9042dc.png)

### 2. 提交标定任务

1. 登录 [Apollo云服务任务页面](http://bce.apollo.auto/login)，选择使用百度账号登录。

2. 在左侧框中选择 `Apollo Fuel-->任务`。

![vehicle_calibration_submit_task1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/97df047367ecdf747c33e5073a0004c1.jpg)

3. 点击 `新建任务`。

![vehicle_calibration_submit_task2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/ba59ee2eef38225a1fd9c76bd4a00e89.jpg)

4. 在下拉框内选择 `车辆标定`。

![vehicle_calibration_submit_task3](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/8aa0046dee27a723daed9bca65070131.png)

5. 输入相应要评测的数据路径，在 `输入数据路径` 中填写到根目录，在本示例中填写为 `task-2021-01-04-18-12`（因为 “task-2021-01-04-18-12” 在 BOS 的根目录）。

![vehicle_calibration_submit_task4](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/dce25652491f7e50df6bbc5013e3b13a.png)

### 3. 获取标定结果及标定表

云标定任务完成后，您将在注册的邮箱（请与商务联系）中收到标定结果邮件。如果标定任务成功，将包含标定表及标定结果图片。

### 4. 将标定表写入控制配置文件中

生成的标定表以 `车型_calibration_table.pb.txt` 命名，将标定表内的标定数据为 `calibration_table` 字段，把全部数据拷贝替换至 `apollo/modules/control/conf/control_conf.pb.txt` 下对应的 `lon_controller_conf` 字段下面的 `calibration_table` 段内。

以下为云标定数据片段示例（截取了前4段）：

```text
    calibration {
      speed: 0.0
      acceleration: -1.3040223121643066
      command: -69.0
    }
    calibration {
      speed: 0.0
      acceleration: -1.2989914417266846
      command: -65.63157894736842
    }
    calibration {
      speed: 0.0
      acceleration: -1.2903447151184082
      command: -62.26315789473684
    }
    calibration {
      speed: 0.0
      acceleration: -1.2760969400405884
      command: -58.89473684210526
    }
```

## 步骤四、结果显示

标定表和结果以邮件的形式，发送至与 BOS 存储绑定的电子邮箱中。

- 油门踏板标定结果

![vehicle_calibration_online_throttle_result](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/4fd61b47f29513d0413bc3f3c25c7845.png)

- 刹车踏板标定结果

![vehicle_calibration_online_brake_result](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/cbb5bfd5bbb12e6e750149d5158b468b.png)

## NEXT

车辆云标定能够提供精度较高的车辆踏板标定表，对于自动驾驶的控制精度有显著提升。当然，目前车辆的出厂一致性较好，在对控制精度要求不高的前提下，`D-KIT` 配置表内默认的标定数据，也可以进行正常循迹等闭环自动驾驶演示。

现在，您已经完成车辆动力学标定，接下来可以开始 [车辆循迹演示](../上车使用教程/循迹搭建/车辆循迹演示.md)。

## 常见问题

### 在进行车辆标定时，收到如下报错邮件，数据完整性检查出现错误

![vehicle_calibration_online_error1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/5865b79d3ac0a861a556b4650a9bc50a.png)

排查方法：

1. 查看 `文件夹结构` 是否正确。

一般数据预处理时，会自动生成文件夹目录，如果文件夹目录存在错误，可以手动参考如下图目录自行修改。

所有采集的标定数据都保存在 `apollo/data/bag/` 目录下，找到以 `年-月-日-时-分-秒` 命名的文件夹，根据时间找到属于车辆标定采集的数据。我们可以分批采集数据，即每一次可以采集一部分数据，这样数据会记录在不同的 `年-月-日-时-分-秒` 文件夹（**注意**：默认情况下，数据采集后会在 `apollo/data/bag/` 文件夹下生成 2 个 `年月日时分秒相似` 的文件夹，分别以 `年-月-日-时-分-秒` 和 `年-月-日-时-分-秒_s` 命名，开发者可以不用理会 `_s` 后缀的文件夹，只需要使用 `年-月-日-时-分-秒` 文件夹内录制的标定数据即可），也可以一次全部采集，这样所有的数据都记录在同一个文件夹内，通常完成上述标定条件的车辆标定数据采集需要 20 分钟左右。

1）正确的文件夹结构如下图所示：

    ![vehicle_calibration_online_structure](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/7c86f944f1152a3417d54f51e93ca458.png)

2） `Origin Folder` 确保是 **BOS 的根目录** ，确保 `task001` 目录是在 BOS 根目录下创建的（根目录：登录 BOS 存储服务器后首先看到的目录即为根目录，一般是 `Bucket name` 目录）。

3） `task001、task002...` 代表一次车辆标定任务文件夹，即每次标定任务提交时，会训练一个 `task文件夹` 内的车辆数据； `task文件夹` 上一层（ `Origin Folder` ）可以是 BOS 根目录或其他目录。

4） `Vehicle1、Vehicle2...` 代表标定的车辆文件夹，可以根据实际情况，按照车辆名称+编号的形式命名，如 `ch01`、`ch02` 等。一个 task 文件夹下内可以包含多个 Vehicle 车辆文件夹，在一次任务提交中标定多个车辆。

5） `Records` 文件内导入采集的车辆标定数据，车辆的录制数据在 `apollo/data/bag/` 目录下。

> 注意： `Records` 文件夹下需要有 2 层子文件夹，第一层为大时间（年-月-日），如上图 `2019-04-19` ，第二层文件夹是具体的 record 文件夹（年-月-日-时-分等），如上图 `2019-04-19-12-45-01` 等，提醒大家一定要注意 `Records` 文件夹下的文件夹结构还要有2层，不然后续会影响标定找不到文件数据，文件夹结构如上图所示；

6） `Configuration File` 即为 `vehicle_param.pb.txt` （车辆配置文件），该文件在 `apollo/modules/common/data/vehicle_param.pb.txt` ，将改目录下的文件 `vehicle_param.pb.txt` 拷贝至 BOS 对应的车辆标定文件夹下，如上图 `Vehicle2` 文件夹内。

7）总结上述文件夹结构。

    BOS根目录 -> Task Folder ->Vehicle Folder -> Records + vehicle_param.pb.txt

2. 查看是否在车辆文件夹内添加车辆配置文件（ `vehicle_param.pb.txt` ）。

3. 查看录制数据包内是否存在标定需要的 channel 数据。

   - 通过 `cyber_recorder info xxxxxx.record.xxxxx`

检查采集数据内是否存在下面3个 `channel` 的数据：

- `/apollo/canbus/chassis`

- `/apollo/sensor/gnss/best_pose`

- `/apollo/localization/pose`

下图为正确的数据包示意：

![vehicle_calibration_online_record_check](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/8c6a214b2daf7074d5bbad02f3e3dd2c.png)

如果发现缺少了上述 channel 的某一项，请排查相关模块。
