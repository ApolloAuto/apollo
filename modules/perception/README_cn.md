# 感知3.0

## 介绍

Apollo3.0有以下新特点：

- CIPV检测
- 尾随
- 整车道线
- 在线姿态估计
- 异步传感器融合
- 视觉定位
- 超声波传感器
- 16束激光雷达
感知模块结合了使用前摄像头和前雷达识别障碍物的能力，并融合其各自的轨迹以获得最终的轨迹列表。障碍子模块检测、分类和跟踪障碍物。子模块还预测障碍物运动和位置信息（例如，航向和速度）。对于车道线，我们通过后处理车道分析像素来构造车道实例，并计算出与车辆（L0，L1，R0，R1等）的车道相对位置。

参见：

[Apollo3.0中的感知算法](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/perception_apollo_3.0.md)

[Apollo3.0传感器安装指南](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Guideline_sensor_Installation_apollo_3.0.md).

## 输入

感知模块的输入是：

- 雷达数据（ROS topic /apollo/sensor/conti_radar）
- 图像数据（R ROS topic /apollo/sensor/camera/obstacle/front_6mm）
- 雷达传感器校准的外部参数（来自YAML文件）
- 前摄像机标定的外部和内在参数（来自YAML文件）
- 主车辆的速度和角速度（ROS主题/阿波罗/定位/姿态）
## 输出

感知模块输出为：

* 具有航向、速度和分类信息的三维障碍物轨迹（ROS topic /apollo/perception/obstacles）
* 车道标志信息具有拟合曲线参数、空间信息（L0、R0等）以及语义信息（车道类型）（ROS topic /apollo/perception/obstacles）
- [ ] 1. 在配置文件`modules/perception/conf/perception_lowcost.conf`中设置一般设置。

- [ ] 2. 运行命令`./scripts/bootstrap.sh`以启动Web GUI。

- [ ] 3. 在Web GUI中选择车辆模型。

- [ ] 4. 使用`./scripts/perception_lowcost_vis.sh`启动或通过启用Web GUI的模块控制器（Module Controller）页面上的感知按钮启动感知模块。停止感知的命令是./scripts/perception_lowcost_vis.sh stop。注意：请不要尝试使用GUI来启用感知，然后使用脚本来停止它，反之亦然。

- [ ] 5. 从Apollo开放数据平台下载演示数据。

## 函数启用/禁用

感知框架是一个有向无环图（DAG）。在DAG配置中有三个组件，包括子节点、边缘和共享数据。每个功能在DAG中实现为子节点。共享数据的子节点具有从生产者到客户的边缘。

在下面的示例中显示了感知模块的典型DAG配置。示例DAG配置如下：

- 默认的障碍感知由“CameraProcessSubnode”、“ RadarProcessSubnode”和“FusionSubnode”组成，如subnode_config中所示。
- “CameraProcessSubnode”和“RadarProcessSubnode”独立接收传感器数据和输出障碍数据，即data_config中的“CameraObjectData”和“RadarObjectData”。
- “FusionSubnode”，它既订阅障碍数据又发布最终结果。
- “LanePostProcessSubnode”处理来自相机检测模块的车道解析输出，并生成车道实例和属性。
- 边缘和数据的配置定义了两点间的联系。
- 可以通过移除相应的子节点、边缘和共享数据配置来禁用每个函数。但是，必须确保所有输入和输出配置都是正确的。
``` protobuf

# Nvidia Driver and CUDA are required for these 2 subnodes
subnode_config {
    # Camera node
    subnodes {
        id: 3
        name: "CameraProcessSubnode"
        reserve: "device_id:camera;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 4
        name: "MotionService"
        reserve: "device_id:motion_service;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 5
        name: "LanePostProcessingSubnode"
        reserve: "device_id:camera;motion_event_id:1021"
        type: SUBNODE_NORMAL
    }
    subnodes {
        id: 2
        name: "RadarProcessSubnode"
        reserve: "device_id:radar_front;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 31
        name: "FusionSubnode"
        reserve: "pub_driven_event_id:1009;lane_event_id:1010;camera_event_id:1009;radar_event_id:1013;"
        type: SUBNODE_OUT
    }
}

###################################################################
# Define all edges which link nodes.
edge_config {

    # CameraDetectorSubnode -> LanePostProcessingSubnode
    edges {
        id: 106
        from_node: 3
        to_node: 5
        events {
            id: 1004
            name: "lane_postprocessing"
        }
    }

    # CameraProcessSubnode -> FusionSubnode
    edges {
        id: 109
        from_node: 3
        to_node: 31
        events {
            id: 1009
            name: "camera_fusion"
        }
    }

    # LanePostProcessingSubnode -> FusionSubnode
    edges {
        id: 110
        from_node: 5
        to_node: 31
        events {
            id: 1010
            name: "lane_fusion"
        }
    }

    # RadarSubnode -> FusionSubnode
    edges {
        id: 113
        from_node: 2
        to_node: 31
        events {
            id: 1013
            name: "radar_fusion"
        }
    }
}

# Shared Data
data_config {
    datas {
        id: 5
        name: "CameraObjectData"
    }
    datas {
        id: 7
        name: "CameraSharedData"
    }
    datas {
        id: 8
        name: "LaneSharedData"
    }
    datas {
        id: 9
        name: "FusionSharedData"
    }
    datas {
        id: 10
        name: "RadarObjectData"
    }
}



 **Note**: Nvidia GPU and CUDA are required to run the perception module with Caffe. Apollo provides the CUDA and Caffe libraries in the release docker image. However, the Nvidia GPU driver is not installed in the dev docker image.

To run the perception module with CUDA acceleration, install the exact same version of the Nvidia driver in the docker that is installed in your host machine, and then build Apollo with the GPU option (i.e., using `./apollo.sh build_gpu` or `./apollo.sh build_opt_gpu`).

See [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).

**Note**
This module contains a redistribution in binary form of a modified version of caffe(https://github.com/BVLC/caffe). 
A copy of the caffe's original copyright statement is included below:

COPYRIGHT

All contributions by the University of California:
Copyright (c) 2014-2017 The Regents of the University of California (Regents)
All rights reserved.

All other contributions:
Copyright (c) 2014-2017, the respective contributors
All rights reserved.

Caffe uses a shared copyright model: each contributor holds copyright over
their contributions to Caffe. The project versioning records all such
contribution and copyright details. If a contributor wants to further mark
their specific copyright on a particular contribution, they should indicate
their copyright solely in the commit message of the change when it is
committed.

LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

CONTRIBUTION AGREEMENT

By contributing to the BVLC/caffe repository through pull-request, comment,
or otherwise, the contributor releases their content to the
license and copyright terms herein.
