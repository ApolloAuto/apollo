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
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "//modules/perception/onboard:perception_onboard",
        "@pcl",
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
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/proto:perception_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
