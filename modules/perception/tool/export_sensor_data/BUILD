load("//tools:cpplint.bzl", "cpplint")

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "export_sensor_data_lib",
    srcs = ["export_sensor_data.cc"],
    hdrs = [
        "export_sensor_data.h",
    ],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/base:base",
        "//modules/perception/common:perception_common",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "@pcl//:pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "export_sensor_data",
    srcs = ["export_sensor_data_main.cc"],
    data = [
        "//modules/perception/conf:perception_adapter_manager_config",
        "//modules/perception/tool/export_sensor_data/conf:perception_tool_export_sensor_data_config",
    ],
    deps = [
        ":export_sensor_data_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/lib/base:base",
        "@ros//:ros_common",
    ],
)

cpplint()
