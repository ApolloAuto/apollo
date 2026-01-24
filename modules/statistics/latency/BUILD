load("//tools:apollo_package.bzl", "apollo_package", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"latency_statistic\\"']

apollo_component(
    name = "liblatency_statistic_component.so",
    srcs = [
        "latency_statistic_component.cc",
        "latency_statistic_gflags.cc",
    ],
    hdrs = [
        "latency_statistic_component.h",
        "latency_statistic_gflags.h"
    ],
    copts = COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/adapters:adapter_gflags",
        "//modules/statistics_msgs:latency_statistic_cc_proto",

        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
        "dag/*",
    ]),
)

apollo_package(enable_source=False)
cpplint()
