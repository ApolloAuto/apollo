# 模块名称
lidar_segmentation

## 简介
lidar背景分割模块使用secondary_indices点云进行分割。Secondary indices是non_ground_indice和roi_indices的交集，并删除前景对象点索引。

## 目录结构
```
├── lidar_segmentation     // lidar segmentation组件
    ├── common             // 公用代码
    ├── conf               // 配置文件目录
    ├── dag                // dag启动文件
    ├── data               // 模块配置参数
    ├── launch             // launch启动文件
    ├── proto              // lidar segmentation模块配置proto文件
    ├── segmentor          // 背景分割方法
    ├── lidar_segmentation_component.cc // 组件入口
    ├── lidar_segmentation_component.h
    ├── cyberfile.xml      // 包管理配置文件
    ├── README.md
    ├── README_cn.md
    └── BUILD
```

## 模块

### LidarSegmentationComponent

apollo::perception::lidar::LidarSegmentationComponent
#### 输入
| 名称 | 类型 | 描述 |
| ----------------- | ------------------------------- | ----------------- |
| `msg` | `onboard::LidarFrameMessage` | 激光雷达消息帧 |

#### 输出
| 名称 | 类型 | 描述 |
| ----------------- | ------------------------------- | --------------- |
| `frame` | `onboard::LidarFrameMessage` | 激光雷达消息帧 |

#### 配置

如下 graph_segmentation 的配置。

| 参数类型 | 参数名称 | 默认值 | 含义 |
| -------------- | -------------- | ------------- | ------- |
| uint32 | grid_width | | 网格宽度 |
| uint32 | grid_height | | 网格高度 |
| float | 分辨率 | | 网格分辨率 |
| float | 阈值 | | 图形分割阈值 |
| uint32 | min_pt_number | | 聚类的最小点数 |
| uint32 | search_radius | 3 | 网格搜索半径 |
| float | height_threshold | 2.0 | 高度阈值 |
| float | xmin | -30.0 | pointcloud 前方有效距离 |
| float | xmax | 30.0 | pointcloud 后方有效距离 |
| float | ymin | -10.0 | pointcloud 右侧有效距离 |
| float | ymax | 30.0 | pointcloud 左侧有效距离 |
| float | semantic_cost | 1.0 | 语义标签成本 |
| float    | same_semantic_coefficient |  1.0  | 语义类别相同时的系数 |
| float    | diff_semantic_coefficient |  1.0  | 语义类别不同时的系数 |
| float    | same_motion_coefficient   |  1.0  | 动静态类别相同时的系数 |
| float    | diff_motion_coefficient   |  1.0  | 动静态类别不同时的系数 |
| float    | split_aspect_ratio |  10.0   | 切分目标的长宽比阈值 |
| float    | split_distance     |  5.0    | 切分长度 |

#### 如何启动

1. 在配置文件中的 modules/perception/data/params 中添加车辆参数配置文件，对应frame_id和sensor_name，启动transform命令
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```

2. 修改 modules/perception/launch/perception_lidar.launch
- 选择要启动的 dag 文件，将 `lidar_segmentation.dag` 添加到 perception_lidar.launch
- 修改 msg_adapter。用于将其他步骤发送的消息包装为/apollo/perception/obstacles，可用于单独调试。修改modules/perception/data/flag/perception_common.flag中相关通道配置

3. 修改配置文件中modules/perception/lidar_segmentation/conf/lidar_segmentation_config.pb.txt的参数
- output_channel_name：输出通道名称
- plugin_param：插件参数
- name：方法名称
- config_path：配置文件夹
- config_file：配置文件名

4. 启动perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
```