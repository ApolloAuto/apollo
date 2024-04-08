本次实践的目标是开发一个组件，说明感知组件开发的过程。开发目标是搭建一个完整的组件框架，实现一个示例功能：通过聚类识别目标。此组件用于替代 lidar_detection 组件，能够完成基于 cpu 的 lidar 感知。

## 准备工作

- [环境配置](https://apollo.baidu.com/community/article/1133)。

- [感知功能调试、参数调整](https://apollo.baidu.com/community/article/1134)。

进入容器：

```bash
# 进入到application-perception代码库
cd application-perception

# 拉取并启动docker容器，如果已拉取不需要执行这条命令
aem start_gpu
# 进入容器
aem enter
# 下载安装依赖包： 会拉取安装core目录下的cyberfile.xml里面所有的依赖包
buildtool build --gpu
```

组件的创建通过命令的形式来实现，执行命令后代码框架被自动创建。

## 创建组件

进入到容器内后，执行命令创建组件。

```bash
#创建组件，--namespace说明命名空间，--template说明创建的是组件，最后是组件的路径
buildtool create --namespace perception --template component modules/perception/lidar_cluster_component
```

组件创建成功后的代码结构：

![创建组件.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%88%9B%E5%BB%BA%E7%BB%84%E4%BB%B6_8ebf788.png)

## 组件开发

这次的组件应用于 lidar 感知，用命令生成的组件有一些默认配置，需要做修改和开发。修改 lidar_cluster_component 文件夹名称为 lidar_cluster，保持 lidar 组件名称的统一。

### 代码修改

修改头文件 lidar_cluster_component.h 代码。修改内容以注释的形式表达。

```bash
#pragma once
#include <memory>
#include <string>  // string 依赖头文件

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/perception/lidar_cluster/proto/lidar_cluster_component.pb.h"

// 0.增加头文件
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/lidar_cluster/interface/base_object_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;  // 1.使用 LidarFrameMessage

class LidarClusterComponent final : public cyber::Component<LidarFrameMessage> {
public:
    LidarClusterComponent() = default;
    virtual ~LidarClusterComponent() = default;

public:
    bool Init() override;
    // 2.组件触发消息修改为 LidarFrameMessage
    bool Proc(const std::shared_ptr<LidarFrameMessage>& in_message) override;

private:
    bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);

private:
    apollo::perception::lidar::LidarClusterComponentConfig config_;  // 增加命名空间 lidar
    // 3.增加成员变量
    std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
    std::string output_channel_name_;
    BaseObjectCluster* object_cluster_;
};

CYBER_REGISTER_COMPONENT(LidarClusterComponent)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

修改 lidar_cluster/proto/lidar_cluster_component.proto

```bash
syntax = "proto2";

package apollo.perception.lidar;  // 1.命名空间增肌 lidar

message LidarClusterComponentConfig {
  optional string name = 1;
  optional string output_channel_name = 2;  // 2.增加组件输出消息配置项
};
```

在 lidar_cluster/BUILD 中增加依赖项：

```bash
apollo_component(
    name = "liblidar_cluster_component.so",
    srcs = [
        "lidar_cluster_component.cc",
    ],
    hdrs = [
        "lidar_cluster_component.h",
    ],
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/perception/lidar_cluster/proto:lidar_cluster_component_proto",
        # 增加依赖项
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        ":apollo_perception_lidar_cluster",
    ],
)
```

在 cyberfile.xml 中增加对 perception-common、lidar-detection、common-msgs 的依赖：

```bash
 <depend repo_name="common-msgs" type="binary">common-msgs</depend>
  <depend type="binary" repo_name="perception-common" lib_names="perception-common">perception-common</depend>
  <depend type="binary" repo_name="perception-lidar-detection" lib_names="perception-lidar-detection">perception-lidar-detection</depend>
```

到此 lidar_cluster 组件的代码框架搭建完成，下面是实现聚类的功能。

### 功能开发

定义 cluster 方法的接口。新建 lidar_cluster/interface 文件夹，接口类定义在其中。

```bash
#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct ObjectClusterInitOptions : public BaseInitOptions {
    std::string sensor_name = "velodyne64";
};

struct ObjectClusterOptions {};

class BaseObjectCluster {
public:
    BaseObjectCluster() = default;
    virtual ~BaseObjectCluster() = default;

    /**
     * @brief Init of the Base Object Cluster object
     *
     * @param options object cluster options
     * @return true
     * @return false
     */
    virtual bool Init(const ObjectClusterInitOptions& options = ObjectClusterInitOptions()) = 0;

    /**
     * @brief Cluster objects
     *
     * @param options object cluster options
     * @param frame lidar frame
     * @return true
     * @return false
     */
    virtual bool Cluster(const ObjectClusterOptions& options, LidarFrame* frame) = 0;

    /**
     * @brief Name of Object Cluster
     *
     * @return std::string name
     */
    virtual std::string Name() const = 0;

private:
    DISALLOW_COPY_AND_ASSIGN(BaseObjectCluster);
};  // class BaseObjectCluster

PERCEPTION_REGISTER_REGISTERER(BaseObjectCluster);
#define PERCEPTION_REGISTER_LIDARCLUSTER(name) PERCEPTION_REGISTER_CLASS(BaseObjectCluster, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

实现聚类功能代码。新建 lidar_cluster/cluster 文件夹，其中存放所有的聚类方法。此组件以欧式聚类为例子说明，在cluster中创建文件夹 euclidean_cluster。

在 euclidean_cluster 中定义 euclidean_cluster.h 和 euclidean_cluster.cc。

```bash
#pragma once

#include <string>
#include <vector>
#include <map>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/lidar_cluster/interface/base_object_cluster.h"
#include "modules/perception/lidar_cluster/cluster/proto/euclidean_cluster_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class EuclideanCluster : public BaseObjectCluster {
public:
    EuclideanCluster() = default;
    ~EuclideanCluster() = default;

public:
    bool Init(const ObjectClusterInitOptions& options = ObjectClusterInitOptions()) override;
    bool Cluster(const ObjectClusterOptions& options, LidarFrame* frame) override;  // 聚类主方法
    std::string Name() const override {
        return "EuclideanCluster";
    }

private:
    EuclideanClusterConfig euclidean_cluster_param_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

聚类源代码如下：

```bash
#include "modules/perception/lidar_cluster/cluster/euclidean_cluster/euclidean_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;

bool EuclideanCluster::Init(const ObjectClusterInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    // get euclidean cluster params
    euclidean_cluster_param_.Clear();
    ACHECK(GetProtoFromFile(config_file, &euclidean_cluster_param_))  // 1.载入算法参数
            << "Failed to parse EuclideanClusterConfig config file.";

    return true;
}

bool EuclideanCluster::Cluster(const ObjectClusterOptions& options, LidarFrame* frame) {
    // check input
    if (frame == nullptr) {
        AERROR << "Input null frame ptr.";
        return false;
    }
    if (frame->cloud == nullptr) {
        AERROR << "Input null frame cloud.";
        return false;
    }
    if (frame->cloud->size() == 0) {
        AERROR << "Input none points.";
        return false;
    }
    // obtain intersection of roi and non-ground indices
    CloudMask mask;
    mask.Set(frame->cloud->size(), 0);
    mask.AddIndicesOfIndices(frame->roi_indices, frame->non_ground_indices, 1);
    base::PointIndices indices;
    mask.GetValidIndices(&indices);  // 2.获得roi和non-ground点的交点。（主车在地图时，地图内的点为roi点；主车不在地图，所有点为roi点）
    // get pointcloud of indices
    std::map<int, int> index_mapper_;  // key: index-in-pcl_cloud, value: index-in-cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud->reserve(indices.indices.size());
    for (size_t i = 0; i < indices.indices.size(); ++i) {
        int idx = indices.indices.at(i);
        index_mapper_[i] = idx;
        // get point
        auto pt = frame->cloud->at(idx);
        pcl::PointXYZ pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        pcl_cloud->push_back(pcl_pt);
    }
    // 3.欧式聚类算法主流程
    // kd tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pcl_cloud);
    // do euclidean cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(euclidean_cluster_param_.cluster_distance());
    ec.setMinClusterSize(euclidean_cluster_param_.min_cluster_size());
    ec.setMaxClusterSize(euclidean_cluster_param_.max_cluster_size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud);
    ec.extract(cluster_indices);

    // 4.获得聚类结果，并把结果放到lidar frame中.
    // get object from cluster_indices
    for (pcl::PointIndices cluster : cluster_indices) {
        if (cluster.indices.size() < euclidean_cluster_param_.min_point_number()) {
            continue;
        }
        // get cluster point index in cloud
        std::vector<int> point_idx;
        for (auto idx : cluster.indices) {
            point_idx.push_back(index_mapper_.at(idx));
        }
        base::Object object;
        object.confidence = 1.0;
        object.lidar_supplement.is_in_roi = true;
        object.lidar_supplement.num_points_in_roi = cluster.indices.size();
        object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, point_idx);
        object.lidar_supplement.cloud_world.CopyPointCloud(*frame->world_cloud, point_idx);
        // classification
        object.type = base::ObjectType::UNKNOWN;
        object.lidar_supplement.raw_probs.push_back(
                std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
        object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] = 1.0f;
        object.lidar_supplement.raw_classification_methods.push_back(Name());
        // copy to type
        object.type_probs.assign(
                object.lidar_supplement.raw_probs.back().begin(), object.lidar_supplement.raw_probs.back().end());
        // copy to background objects
        std::shared_ptr<base::Object> obj(new base::Object);
        *obj = object;
        frame->segmented_objects.push_back(std::move(obj));
    }
    return true;
}

PERCEPTION_REGISTER_LIDARCLUSTER(EuclideanCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

在 lidar_cluster/cluster/proto 中定义 euclidean_cluster.proto 文件，用于指定配置项。

```bash
message EuclideanClusterConfig {
  optional float cluster_distance = 1;  // 欧式聚类的距离阈值
  optional int32 min_cluster_size = 2;  // 最小cluster数量
  optional int32 max_cluster_size = 3;  // 最大cluster数量
  optional uint32 min_point_number = 4;  // 点云小于此数量的cluster会被过滤掉
};
```

在`lidar_cluster/data/euclidean_cluster.pb.txt`中放聚类方法的配置。

```bash
cluster_distance: 0.3
min_cluster_size: 1
max_cluster_size: 500
min_point_number: 3
```

编译代码：

```bash
buildtool build --gpu --opt
```

## 组件使用

### 启动设置

首先，修改 lidar_cluster/dag 中的 lidar_cluster_component.dag 文件，修改 flag_file_path，删除 conf/lidar_cluster_component.conf。修正 launch/lidar_cluster_component.launch 中的 dag 文件路径。

```bash
module_config {
  module_library : "modules/perception/lidar_cluster/liblidar_cluster_component.so"
  components {
    class_name : "LidarClusterComponent"
    config {
      name : "lidar_cluster_component"
      config_file_path: "modules/perception/lidar_cluster/conf/lidar_cluster_component.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"  # 更换路径，采用感知的统一配置
      readers {
        channel: "/perception/lidar/pointcloud_ground_detection"  # 输入通道名称
      }
    }
  }
}
```

然后，设置 conf/lidar_cluster_component.pb.txt 文件，

```bash
name: "lidar-cluster"
output_channel_name: "/perception/lidar/cluster"  # 输出通道名称
plugin_param {
  name: "EuclideanCluster"
  config_path: "perception/lidar_cluster/data"
  config_file: "euclidean_cluster.pb.txt"
}
```

最后，在 lidar_output.dag 中，用 lidar_cluster 替换 lidar_detection。并修改 lidar_detection_filter 的输入通道名称。

```bash
module_config {  # 将 lidar_detect 替换为 lidar_cluster
  module_library : "modules/perception/lidar_cluster/liblidar_cluster_component.so"
  components {
    class_name : "LidarClusterComponent"
    config {
      name : "lidar_cluster_component"
      config_file_path: "modules/perception/lidar_cluster/conf/lidar_cluster_component.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {
        channel: "/perception/lidar/pointcloud_ground_detection"  # 输入通道
      }
    }
  }
}

module_config {
  module_library : "modules/perception/lidar_detection_filter/liblidar_detection_filter_component.so"
  components {
    class_name : "LidarDetectionFilterComponent"
    config {
      name : "LidarDetectionFilter"
      config_file_path : "/apollo/modules/perception/lidar_detection_filter/conf/lidar_detection_filter_config.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {
        channel: "/perception/lidar/cluster"  # lidar_cluster 下游组件的接受通道名称
        pending_queue_size: 100
      }
    }
  }
}
```

### 结果展示

通过 cyber_launch 启动 transform 和 lidar_perception。

![结果展示1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E7%BB%93%E6%9E%9C%E5%B1%95%E7%A4%BA1_933949f.png)

通过 cyber_recorder 播包，可以得到如下感知结果。对点云没有做高精地图过滤，可以看到把所有高于地面的点云都做了聚类。

用`cyber_record play -f ***.record`来播包，只播放三个必要的通道，通过 -c 来控制。进入到目录`/home/apollo/.apollo/resources/records`，执行：

```bash
cyber_recorder play -f demo3_mkz_110_sunnybigloop.record -c /tf /apollo/sensor/velodyne64/compensator/PointCloud2 /apollo/localization/pose
```

![结果展示2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E7%BB%93%E6%9E%9C%E5%B1%95%E7%A4%BA2_6092065.png)
