本次实践课的目标是开发一个插件，说明感知插件开发的过程。插件二次开发的重点是代码框架搭建和配置管理。本次插件开发实现一个示例功能：过滤障碍物，fp_filter。此插件用于过滤误检目标。这里把一些尺寸不合理、置信度较低的目标作为误检目标。

## 准备工作

- [环境配置](https://apollo.baidu.com/community/article/1133)

- [感知功能调试、参数调整](https://apollo.baidu.com/community/article/1134)

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

插件的创建通过命令的形式来实现，执行命令后代码框架被自动创建。

## 创建插件

在容器内，执行命令创建插件。fp_filter 的含义是 false positive filter，用于过滤误检目标。

```bash
#创建插件，--namespace说明命名空间，--template说明创建的是插件，最后是插件的路径
buildtool create --namespace perception --template plugin modules/perception/fp_filter
```

插件创建成功后的代码结构：

![创建插件.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%88%9B%E5%BB%BA%E6%8F%92%E4%BB%B6_89be993.png)

## 插件开发

此插件应用于 lidar_detection_filter，生成的插件有默认的配置，需要做修改和开发。

### 代码修改

在 cyberfile.xml 中增加 perception-lidar-detection-filter 包依赖：

```bash
<depend repo_name="perception-lidar-detection-filter" type="binary">perception-lidar-detection-filter</depend>
```

修改代码：

在头文件 fp_filter.h 修改代码。其中需要修改的地方以注释的形式做了说明。

```bash
#pragma once

#include <memory>
#include "cyber/plugin_manager/plugin_manager.h"

#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"  // 0.添加头文件

namespace apollo {
namespace perception {
namespace lidar {  // 1.定义命名空间

class FpFilter : public BaseObjectFilter {  // 2.继承基类
public:
    FpFilter() = default;  // 3.定义构造和析构函数
    virtual ~FpFilter() = default;

public:
    // 4.重写如下三个函数； 声明 FilterFlag 方法
    bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) override;
    bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;
    std::string Name() const override {
        return "FalsePositiveFilter";
    }
    bool FilterFlag(base::ObjectPtr obj, double confidence_thresh, double volume_thresh, double height_diff_thresh);

private:
    std::vector<bool> fp_flag_;  // 5.定义数据结构
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::FpFilter, BaseObjectFilter)  // 6.修改插件配置

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

在 plugin_fp_filter_description.xml 中修改配置：

```bash
<library path="modules/perception/fp_filter/libfp_filter.so">
    <class type="apollo::perception::lidar::FpFilter" base_class="apollo::perception::lidar::BaseObjectFilter"></class>
</library>
```

在 BUILD 中增加依赖项：

```bash
apollo_plugin(
    name = "libfp_filter.so",
    srcs = [
        "fp_filter.cc",
    ],
    hdrs = [
        "fp_filter.h",
    ],
    description = ":plugin_fp_filter_description.xml",
    deps = [
        "//cyber",
        "//modules/perception/fp_filter/proto:fp_filter_proto",
        "//modules/perception/lidar_detection_filter:apollo_perception_lidar_detection_filter",  # 增加依赖项
    ],
)
```

### 功能定义

修改好了头文件和插件配置。在 fp_filter.cc 中定义代码功能。功能核心点如下：

- 过滤规则：置信度低、体积小、高度差小的目标做误检过滤；

- 过滤目标类别：UNKNOWN、UNKNOWN_MOVABLE、UNKNOWN_UNMOVABLE、BICYCLE、PEDESTRIAN；

```bash
#include <memory>
#include "modules/perception/fp_filter/fp_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

bool FpFilter::Init(const ObjectFilterInitOptions& options) {
    fp_flag_.clear();
    return true;
}

bool FpFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
    if (!frame) {
        AINFO << "Lidar frame is nullptr.";
        return false;
    }
    fp_flag_.clear();
    size_t size = frame->segmented_objects.size();
    fp_flag_.assign(size, false);  // filter object if true
    // 除了vehicle，对其他类别的目标做误检过滤
    for (size_t i = 0; i < size; ++i) {
        base::ObjectPtr obj = frame->segmented_objects.at(i);
        base::ObjectType type = obj->type;
        if (type == base::ObjectType::UNKNOWN || type == base::ObjectType::UNKNOWN_MOVABLE
            || type == base::ObjectType::UNKNOWN_UNMOVABLE) {
            fp_flag_.at(i) = FilterFlag(obj, 0.4, 0.3, 0.3);
        } else if (type == base::ObjectType::BICYCLE) {
            fp_flag_.at(i) = FilterFlag(obj, 0.4, 0.4, 0.8);
        } else if (type == base::ObjectType::PEDESTRIAN) {
            fp_flag_.at(i) = FilterFlag(obj, 0.4, 0.3, 1.0);
        }
    }

    // do filter
    size_t valid_num = 0;
    for (size_t i = 0; i < fp_flag_.size(); ++i) {
        base::ObjectPtr obj = frame->segmented_objects.at(i);
        if (!fp_flag_.at(i)) {
            frame->segmented_objects.at(valid_num) = obj;
            valid_num++;
        }
    }
    frame->segmented_objects.resize(valid_num);
    AINFO << "FalsePositiveFilter, filter fp " << size - valid_num << " objects, from " << size << " objects";

    return true;
}

/**
 * @brief
 *
 * @param obj
 * @param confidence_thresh 过滤置信度较低的目标
 * @param volume_thresh 过滤体积较小的目标
 * @param height_diff_thresh 过滤
 * @return true
 * @return false
 */
bool FpFilter::FilterFlag(
        base::ObjectPtr obj,
        double confidence_thresh,
        double volume_thresh,
        double height_diff_thresh) {
    // confidence
    if (obj->confidence < confidence_thresh) {
        return true;
    }
    // volume
    auto obj_size = obj->size;
    double volume = obj_size[0] * obj_size[1] * obj_size[2];
    if (volume < volume_thresh) {
        return true;
    }
    // height diff
    double max_z = -10000.0;
    double min_z = 10000.0;
    for (size_t i = 0; i < obj->lidar_supplement.cloud.size(); ++i) {
        double val_z = obj->lidar_supplement.cloud[i].z;
        max_z = std::max(max_z, val_z);
        min_z = std::min(min_z, val_z);
    }
    double height_diff = max_z - min_z;
    if (height_diff < height_diff_thresh) {
        return true;
    }

    return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

## 插件使用

### 配置插件

在`/apollo/modules/perception/lidar_detection_filter/data/filter_bank.pb.txt`中增加 fp_fitler 插件，如下图 11 行所示。

fp_filter 插件的配置通过硬编码写到了程序中，所以 config_path 和 config_file 都为空。

![配置插件.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E9%85%8D%E7%BD%AE%E6%8F%92%E4%BB%B6_a6ccc4a.png)

### 结果示例

感知启动，通过 cyber_launch 启动 lidar 感知和 transform 模块。

![结果示例.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E7%BB%93%E6%9E%9C%E7%A4%BA%E4%BE%8B_933949f.png)

然后通过`cyber_record play -f ***.record`来播包，只播放三个必要的通道，通过 -c 来控制。进入到目录`/home/apollo/.apollo/resources/records`，执行：

```bash
cyber_recorder play -f demo3_mkz_110_sunnybigloop.record -c /tf /apollo/sensor/velodyne64/compensator/PointCloud2 /apollo/localization/pose
```

感知结果：

![感知结果.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%84%9F%E7%9F%A5%E7%BB%93%E6%9E%9C_599bf66.png)

通过 mainboard.INFO 查看感知日志，能够看到 fp_fitler 在正常工作，能够根据规则过滤目标。

![感知日志.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%84%9F%E7%9F%A5%E6%97%A5%E5%BF%97_37413f2.png)
