# 插件开发

感知的插件有两部分：lidar_detection_filter 的 object_filter_bank 中；pointcloud_map_based_roi 的 roi_filter 中。

插件机制是一个比组件更加轻量化的动态加载机制，其目的是让开发者可以更加聚焦到功能逻辑上，更加方便地进行功能扩展。插件开发独立性较强，能被组件灵活的使用，根据需要加载插件。
插件开发，以 lidar_detection_filter 为例。其中 object_filter_bank 中可以执行多种 filter，所有的 filter 都放到 std::vector<std::shared_ptr<BaseObjectFilter>> 中。

## 新建文件夹

在如下目录中新建文件夹，过滤器名称为new_filter，用于放代码和相关配置。

```bash
├── lidar_detection_filter
    ├── conf
    ├── dag
    ├── data
    ├── object_filter_bank
    │   ├── object_filter_bank.cc
    │   ├── object_filter_bank.h
    │   ├── roi_boundary_filter
    │   ├── new_filter  // 新定义filter
    ├── interface
    ├── proto
    ├── lidar_detection_filter_component.cc
    ├── lidar_detection_filter_component.h
    ├── cyberfile.xml
    ├── README.md
    └── BUILD
```

## 功能定义

过滤器功能实现。
首先，继承BaseObjectFilter类，实现Init（初始化配置项），Filter（功能代码），Name（获取类名称）。

然后，在new_filter.h最后定义：CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::NewFilter, BaseObjectFilter)。以此说明此功能是插件形式开发。

```bash
class NewFilter : public BaseObjectFilter {
 public:
  /**
   * @brief Construct a new NewFilter object
   *
   */
  NewFilter() = default;
  virtual ~NewFilter() = default;

  /**
   * @brief Init of NewFilter
   *
   * @param options object filer options
   * @return true
   * @return false
   */
  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override;

  /**
   * @brief filter objects using new filter algorithm
   *
   * @param options object filter options
   * @param frame lidar frame to filter
   * @return true
   * @return false
   */
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of NewFilter object
   *
   * @return std::string name
   */
  std::string Name() const override { return "NewFilter"; }
 private:
   ... ...
 };

 CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::perception::lidar::NewFilter, BaseObjectFilter)
```

插件的具体功能定义在 Filter 函数中。此函数接受的是点云当前帧的 frame，frame 中数据主要有以下几种：

- 点云，以及其索引；
- 非地面点云索引，roi 内点云索引；
- 检测障碍物（包括模型检测、背景聚类）；
- lidar 坐标系到世界坐标系的 pose；
- 主车一定范围内的高精地图信息。

lidar_detection_filter是基于检测障碍物做的过滤，过滤功能由开发者根据实际情况自定义。

## 配置文件

在 object_filter_bank/proto中 定义 new_filter 的配置项，在`data/new_filter.pb.txt`中增加 new_filter 的配置。

```bash
├── lidar_detection_filter
    ├── conf
    ├── dag
    ├── data
    │   ├── new_filter.pb.txt  // 过滤库，在其中增加new_filter配置
    ├── object_filter_bank
    │   ├── object_filter_bank.cc
    │   ├── object_filter_bank.h
    │   ├── roi_boundary_filter
    │   ├── proto  // new_filter 配置定义
    │   ├── new_filter
    ├── interface
    ├── proto
    ├── lidar_detection_filter_component.cc
    ├── lidar_detection_filter_component.h
    ├── cyberfile.xml
    ├── README.md
    └── BUILD
```

## 插件管理

首先，BUILD 中的编译如下，插件需要用 apollo_plugin 的方式来编译。

```bash
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "libnew_filter.so",
    srcs = ["new_filter.cc"],
    hdrs = ["new_filter.h"],
    description = ":plugins.xml",
    deps = [
    ],
)
```

然后，新建 new_filter/plugins.xml，并在其中定义：

```bash
<library path="modules/perception/lidar_detection_filter/object_filter_bank/new_filter/libnew_filter.so">
    <class type="apollo::perception::lidar::NewFilter" base_class="apollo::perception::lidar::BaseObjectFilter"></class>
</library>
```

## 插件使用

插件定义好之后，就可以使用插件。使用方法是：在配置文件`modules/perception/lidar_detection_filter/data/filter_bank.pb.txt`中增加此插件的配置。

```bash
plugins {
    name: "ROIBoundaryFilter"
    config_path: "perception/lidar_detection_filter/data"
    config_file: "roi_boundary_filter.pb.txt"
}
plugins {
    name: "BackgroundFilter"
    config_path: "perception/lidar_detection_filter/data"
    config_file: "background_filter.pb.txt"
}
plugins {
    name: "NewFilter"  # 在这里增加new_filter的配置
    config_path: "perception/lidar_detection_filter/data"
    config_file: "new_filter.pb.txt"
}
```

配置好 new_filter 插件之后，启动 lidar_detection_filter 组件，运行感知，加载各个插件配置内容。当有数据进来时就new_filter就会执行Filter函数，完成过滤功能。
