# 概述

## Dreamview+ 是什么？

Dreamview+ 是一个 web 应用程序，提供如下的功能：

- 可视化显示当前自动驾驶车辆模块的输出信息。例如：规划路径、车辆定位、车架信息等。
- 为使用者提供人机交互接口以监测车辆硬件状态，对模块进行开关操作，启动自动驾驶车辆等。
- 提供调试工具。例如：PnC 监视器可以高效的跟踪模块输出的问题。
  不过，Dreamview+ 的大部分的功能，主要是为了在实际上车调试中使用的。
  在快速入门中，只使用 Dreamview+ 的可视化功能将 record 数据包的数据可视化。

<!--

## 怎么解决问题？
### 降低门槛

  从单一功能工具演变为多场景功能工具，从最初只支持实车路测发展为处理多场景、多流程的工具，满足不同类型开发者的使用特点和习惯，优化不同场景的使用交互和步骤，降低不同类型开发者的上手成本，尤其是降低小白上手门槛，减少对文档的强依赖，提升用户体验。

### 二次开发

  在多场景下，不同工种的开发者对可视化的诉求和习惯并不相同，固定功能和布局无法同时满足感知开发者、PNC 开发者和客户的实车自动驾驶演示的使用习惯。Dreamview 2.0 提供了通用框架的扩展性、更多的场景和工具箱支持，通过灵活的自定义面板功能，自定义面板布局和大小，配合不同类型开发者的使用习惯，降低二次开发成本，提高二次开发效率。

### 生态基础

  未来自动驾驶逐步落地的过程中，将迎来爆炸性数据量。Dreamview 2.0 强化了配置中心的车云一体的联动，Studio 增加数据资源的种类和数量，Dreamview 增强配置中心可配置资源的能力。
-->

## 整体布局

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_1445a63.png)

- 侧边栏：显示菜单按钮：**Mode Settings/模式设置**、**Add Panel/添加面板**、**Resource Manager/资源中心**、**Personnal Center/个人中心**。

- 菜单窗口：显示各菜单按钮的详细信息。

  - Mode Settings/模式设置：模式设置工具栏，详细介绍参见 [Mode Settings](<Apollo_alpha_doc/使用指南/可视化交互工具 Dreamview+/Mode Settings.md>)。
  - Add Panel/添加面板：添加面板，包括控制台、模块延时、车辆可视化、相机视图、点云视图、车辆仪表盘视图等面板。关于面板详细介绍，参见 [Add Panel](<Apollo_alpha_doc/使用指南/可视化交互工具 Dreamview+/Add Panel.md>)。
  - Resource Manager/资源管理：Apollo 提供的车云一体的配置中心，提供车辆、模型、数据包、仿真场景等素材的下载功能。您可以点击需要的数据包、车辆、模型、仿真场景等元素进行下载。详细介绍参见 [Resource Manager](<Apollo_alpha_doc/使用指南/可视化交互工具 Dreamview+/Resource Manager.md>)。
  - Personal Center/个人中心：

    - Settings/设置：

      - General/通用设置：在通用设置中，可以设置语言为中文或英文。
      - Privacy/隐私政策：可通过此处设置向我们提供反馈个性化内容和功能，用于提升您的体验。
      - About/关于我们：Dreamview 相关版本及版权信息。

    - Cloud Profile/资源中心：登录云端 Studio。
    - Use Guide/新手指引：如果您是新用户，可以根据提示按照步骤操作。
    - Product Documentation/产品手册：关于产品简介及产品使用指南。
    - Apollo Developer Community：Apollo 开发者社区。
    - Advice and Suggestion：意见与建议，如果您在使用中遇到任何问题，请您提交反馈，我们会根据您的建议进行优化改进。

- 面板区：展示⾃动驾驶开发调试的可视化⾯板。不同模式下展示的⾯板类型不同，同时⽀持⽤户⾃定义添加⾯板类型。
- 操作底栏：操作底栏是⾃动驾驶开发调试过程中，对数据的操作、处理区。

  - 播放数据包：设置路由后开始仿真场景，
  - Dump：下载某时刻的 channel 信息到本地，
  - Clear：清空 Simulation world 的所有信息并重置动力学模型。

## 使用场景

### 默认模式

适用于所有开始调试的场景。

### 感知开发调试

适用于进行感知模块开发的开发人员，提供感知开发调试相关的数据操作流程选项、可视化数据展示面板与调试信息面板。

### PnC开发调试

PnC 开发调试模式适用于进行规划与控制模块开发的开发人员，提供 PnC 开发调试相关的数据操作流程选项、可视化数据展示面板与调试信息面板。

<!-- 实车路测模式
实车路测模式适用于进行实车验证测试的开发人员，提供实车测试相关的数据操作流程选项、可视化数据展示面板与调试信息面板。车辆集成模式-->

## 名词解释

- Mode Settings：模式设置。
- Mode：模式。
- Modules：模块。
- Operations：操作。
- Current Resources：当前正在使用的资源。
- Environment Resources：环境资源。
- Records：数据包。
- Scenarios：场景。
- ADS Resources：自动驾驶系统资源。
- HDMap：高精地图：
- Vehicle：车辆。
- V2X：车路协同。
- Panel：面板。
- Console：控制台。
- Module Delay：模块延时。
- Vehicle Visualization：车辆可视化。
- Camera View：相机视图。
- Point Cloud：点云。
- Vehicle Dashboard：车辆仪表盘面板。
