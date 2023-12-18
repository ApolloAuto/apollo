# audio

## 简介

`Audio` 模块可检测活动应急车辆发出的警报声。它能检测并输出警报器的开/关状态、移动状态以及警报器的相对位置。当检测到活动的紧急车辆时，您还可以在 `Dreamview` 上看到警报提示

## 目录结构

```shell

modules/audio/
├── audio_component.cc  // 组件实现的代码文件
├── audio_component.h   // 组件实现的代码文件
├── BUILD               // 构建规则文件
├── common        // flag定义，数据处理方法等通用代码
├── conf          // 模块配置文件，参数文件目录
├── cyberfile.xml // 包管理配置文件
├── dag           // 模块启动文件(mainboard)
├── launch        // 模块启动文件(cyber_launch)
├── data          // 存放模型文件
├── inference     // 检测逻辑的代码实现
├── proto         // 组件配置文件结构定义
└── README_cn.md  // 说明文档

```

## 模块

### AudioComponent 组件

apollo::audio::AudioComponent

#### 输入

| Channel 名                  | 类型                                             | 描述                                                                            |
| --------------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------- |
| `/apollo/sensor/microphone` | `apollo::drivers::microphone::config::AudioData` | 麦克风数据，可通过 `modules/audio/dag/audio.dag` 启动文件修改channel名          |
| `/apollo/localization/pose` | `apollo::localization::LocalizationEstimate`     | 定位信息，可通过 `modules/audio/conf/audio_conf.pb.txt` 配置文件修改 channel 名 |

#### 输出

| Channel 名                | 类型                            | 描述                             |
| ------------------------- | ------------------------------- | -------------------------------- |
| `/apollo/audio_detection` | `apollo::audio::AudioDetection` | 检测结果，包括警报器的状态和位置 |

#### 配置

| 文件路径                                       | 类型/结构                  | 说明                                       |
| ---------------------------------------------- | -------------------------- | ------------------------------------------ |
| `modules/audio/conf/audio_conf.pb.txt`         | `apollo::audio::AudioConf` | `apollo::audio::AudioComponent` 的配置文件 |
| `modules/audio/conf/respeaker_extrinsics.yaml` | `yaml`                     | 麦克风的外参配置文件                       |
| `modules/audio/conf/audio.conf`                | `gflags`                   | 命令行参数配置                             |

#### Flags

| flag                            | 类型     | 默认值                                                      | 描述           |
| ------------------------------- | -------- | ----------------------------------------------------------- | -------------- |
| `--cache_signal_time`           | `int32`  | `3`                                                         | 缓存的信号时长 |
| `--touch_siren_detection_model` | `string` | `/apollo/modules/audio/data/torch_siren_detection_model.pt` | 检测模型的路径 |

- flag 定义和默认值参考 `modules/audio/common/audio_gflags.cc`

#### 使用方式

##### 使用 mainboard 启动

```shell
mainboard -d modules/audio/dag/audio.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/audio/launch/audio.launch
```
