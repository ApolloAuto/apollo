# 框架设计

原有的感知框架，lidar、camera、radar、fusion 四部分内容定义在四个模块中。Lidar 和 camera 每个模块内部功能负责，学习成本较高。感知框架拆分后，模块依赖关系清晰。Lidar 和 camera 的感知流程分为多个模块，依赖关系呈线性状态；radar 和 fusion 的功能在单个模块内，功能如框图所示。感知新框架下，lidar 和 camera 相比之前每个模块功能更加轻量化，更有利于二次开发。每个组件都可以单独启动调试。

![apollo感知框图.drawio.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/apollo%E6%84%9F%E7%9F%A5%E6%A1%86%E5%9B%BE.drawio_830f71c.png)

开发者可以根据 preception2.0 组件方式按需使用，同时为了方便扩展，例如更换模型，开发者亦可采用插件的方式来添加新的模型。

- 绿色代表用户经常需要替换的部分，需要开发新算法或功能，基于原有的框架只需要替换自己想要替换的部分，不需要关注消息流转和框架。
- 蓝色代表用户不需要经常替换的部分，模块集成了所有算法，功能是一个整体，直接通过配置修改功能，如果想要替换可以重新写一个组件。

## lidar 检测流程

激光雷达检测用于 3D 目标检测，它的输入是激光雷达点云，输出为检测到的物体的类型和坐标。开发者关注较多的是 pointcloud_map_based_roi、lidar_detection 和 lidar_detection_filter 三个组件。其中，pointcloud_map_based_roi 会涉及相关的 roi 过滤规则，lidar_detection 组件是目标检测过程中模型部署相关内容，Apollo 和 paddle3D 已经打通模型训练部署至 apollo 全流程，后续课程实践也会涉及该方面内容。Apollo lidar 感知模块提供 4 种目标检测模型，开发者可以根据配置文件，选择不同的检测器，来验证检测效果。通过最新的算法插件，开发者更加专注于算法本身，而不需要过多关注框架的实现。lidar_detection_filter 组件为检测障碍物过滤相关功能。激光雷达跟踪用于追踪上面检测到的 3D 目标对象，它的输入是激光雷达点云检测结果，输出为跟踪到对象的 ID。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_464498a.png)

![image (1).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image%20%281%29_77ac62a.png)

## camera 检测流程

camera 检测流程中，开发者的关注重心在camera_detection_2d、camera_detection_3d 以及 camera_tracking 组件，其中，camera_detection_2d 和 camera_detection_3d 组件设计 2d3d 模型部署、预处理等功能，开发者可以针对性的替换模型和开发相关功能，camera_tracking 中功能整体性较强，开发者可以针对相应的追踪策略修改组件中的配置项。camera_detection_2d 检测流程输出的是 2d boundingbox，相比于 camera_detection_3d 组件直接输出3d检测结果，多了将 2d 信息转换成 3d 障碍物信息的过程。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_471b4f3.png)

## fusion 流程

多传感器融合模块对多个传感器的检测结果做融合。它的输入是 lidar，camera，radar 检测目标结果，输出是融合后的结果。其中列举几个重要基类的作用：BaseFusionSystem 相当于融合模块的入口，会初始化 ProbabilisticFusion 概率融合算法，BaseTracker 更新追踪到的轨迹状态，BaseGatekeeper 用于判断是否发布追踪到的障碍物到最终通道中。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_2db0415.png)

多传感器融合组件功能详细功能框图如下所示：

![image (2).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image%20%282%29_6811047.png)

## radar 检测流程

radar_detection 是一个完整的功能组件，其中包含预处理、检测、追踪以及相应过滤功能。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_400c283.png)

## trafficlight 检测流程

红绿灯检测流程开发者关注 traffic_light_detection 和 traffic_light_recognition 这两个组件，分别对应红绿灯检测和识别功能的模型部署等。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_ead83a9.png)
