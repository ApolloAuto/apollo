# 模块名称
pointcloud_semantics

## 简介
语义模块为点云中的每个点分配一个标签。

## 目录结构
```
├── pointcloud_semantics   // pointcloud_semantics组件
    ├── conf               // 配置目录
    ├── dag                // dag启动文件
    ├── data               // 参数配置目录
    ├── interface          // 接口目录
    ├── launch             // launch启动文件
    ├── parser             // parser方法实现
    ├── proto              // proto文件
    ├── utils              // 语义相关工具
    ├── BUILD              // build编译文件
    ├── cyberfile.xml      // 包管理配置文件
    ├── pointcloud_semantic_component.cc  // 组件入口
    ├── pointcloud_semantic_component.h
    └── README.md
    └── README_cm.md
```

## 模块

### PointCloudSemanticComponent

apollo::perception::lidar::PointCloudSemanticComponent
#### 输入
| 名称 | 类型 | 描述 |
| ----------------- | ------------------------------- | ----------------- |
| `msg` | `onboard::LidarFrameMessage` | lidar frame message |

#### 输出
| 名称 | 类型 | 描述 |
| ----------------- | ------------------------------- | --------------- |
| `frame` | `onboard::LidarFrameMessage` | lidar frame message |

#### 如何启动

1. 在配置文件中的 modules/perception/data/params 中添加车辆参数配置文件，对应 frame_id 和 sensor_name，启动transform模块
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```

2. 修改 modules/perception/launch/perception_lidar.launch
- 选择要启动的 dag 文件，在 perception_lidar.launch 中添加 `pointcloud_semantic.dag`
- 修改 msg_adapter。用于将其他步骤发送的消息包装为/apollo/perception/obstacles，可用于单独调试。修改modules/perception/data/flag/perception_common.flag中相关通道配置

3. 修改配置文件中modules/perception/pointcloud_semantics/conf/pointcloud_semantic.pb.txt的参数
- output_channel_name：输出通道名称
- plugin_param：插件参数
- name：方法名称
- config_path：配置文件夹
- config_file：配置文件名

4. 启动perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
```