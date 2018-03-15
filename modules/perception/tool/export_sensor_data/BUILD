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
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/radar/modest:modest_detector",
        "//modules/perception/onboard",
        "@pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "export_sensor_data",
    srcs = ["export_sensor_data_main.cc"],
    data = [
        "//modules/perception/conf:perception_adapter_manager_config",
        "//modules/perception/tool/export_sensor_data/conf:config",
    ],
    deps = [
        ":export_sensor_data_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/proto:perception_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
