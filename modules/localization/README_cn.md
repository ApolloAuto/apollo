# 定位

## 介绍

该模块提供定位服务。默认情况下有两种方法。一种是结合GPS和IMU信息的RTK（Real Time Kinematic实时运动）方法，另一种是融合GPS、IMU和激光雷达信息的多传感器融合方法。

## 文件目录
```shell
modules/localization/
├── BUILD
├── common          // 通用函数, 数据结构与gflags的定义
├── conf
├── cyberfile.xml
├── dag
├── launch
├── msf             // msf 定位实现
├── ndt             // ndt 定位实现
├── proto
├── README_cn.md
├── README.md
├── rtk             // rtk 定位实现
└── testdata
```

## 输入

在提供的RTK方法中，有两个输入：

* GPS-全球定位系统。
* IMU-惯性测量单元。

在所提供的多传感器融合定位方法中，有三个输入：

* GPS-全球定位系统。
* IMU-惯性测量单元。
* 激光雷达-光探测与测距传感器
欲了解更多信息，请参阅多传感器融合定位。

## 输出

* 一个Protobuf message类型的`LocalizationEstimate`实例，它可以在`common_msgs/localization_msgs/localization.proto`中找到。

## 配置

| 文件路径                                                       | 类型                                         | 描述                        |
| ------------------------------------------------------------- | ------------------------------------------- | -------------------------- |
| `modules/localization/conf/localization_config.pb.txt`        | `apollo::localization::LocalizationConfig`  | 定位类型配置                 |
| `modules/localization/conf/navi_localization_config.pb.txt`   | `apollo::localization::LocalizationConfig`  | 定位类型配置                 |
| `modules/localization/conf/rtk_localization.pb.txt`           | `apollo::localization::rtk_config::Config`  | rkt 模式配置                |
| `modules/localization/conf/localization.conf`                 | `gflags`                                    | gflags                     |

## Flags

| flagfile                                             | 类型  | 描述                            |
| ---------------------------------------------------- | ---- | ------------------------------ |
| `modules/localization/common/localization_gflags.cc` | `cc` | 定位 flags 头文件                |
| `modules/localization/common/localization_gflags.h`  | `h`  | 定位 flags cc文件                |

## 如何启动

```bash
cyber_launch start modules/drivers/camera/launch/rtk_localization.launch  // 运行 rtk 定位
cyber_launch start modules/drivers/camera/launch/ndt_localization.launch  // 运行 ndt 定位
cyber_launch start modules/drivers/camera/launch/msf_localization.launch  // 运行 msf 定位
```

## 添加定位实现

目前，RTK方案是在类`RTKLocalization`中实现的。如果一个新的定位方法需要用一个名字（例如`FooLocalization`）来实现，你可以遵循以下步骤：

1. 在`localization_config.proto`的`LocalizationType enum type`中添加Foo。

2. 转到`modules/localization`目录，并创建一个Foo目录。在Foo目录中，根据rtk目录中的`RTKLocalization`类增加`FooLocalization`类。`FooLocalization`必须是`LocalizationBase`的子类。根据`rtk/BUILD`还需创建文件`foo/BUILD`。

3. 您需要在函数`Localization::RegisterLocalizationMethods()`中注册`FooLocalization`，它位于CPP文件`localization.cc`中。您可以通过在函数的末尾插入以下代码来注册：
```
localization_factory_.Register(LocalizationConfig::FOO, []()->LocalizationBase* { return new FooLocalization(); });
```
请确保您的代码可以编译包含`FooLocalization`的头文件。

4. 现在你可以回到apollo根目录（源码环境下）或apollo_workspace（软件包管理环境下），用命令`bash apollo.sh build`构建你的代码（源码环境下），或`buildtool build -p modules/localization`构建你的代码（软件包管理环境下）。
