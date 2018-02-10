load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_lib",
    srcs = ["perception.cc"],
    hdrs = [
        "perception.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/onboard:perception_obstacle_fusion_subnode",
        "//modules/perception/obstacle/onboard:perception_obstacle_lidar_subnode",
        "//modules/perception/obstacle/onboard:perception_obstacle_radar_subnode",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/onboard:perception_traffic_light_onboard",
        "@com_google_protobuf//:protobuf",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "perception",
    srcs = ["main.cc"],
    data = [
        "//modules/perception/conf:perception_adapter_manager_config",
    ],
    deps = [
        ":perception_lib",
        "//external:gflags",
    ],
)

filegroup(
    name = "perception_data",
    srcs = glob([
        "data/**",
    ]),
)

filegroup(
    name = "perception_model",
    srcs = glob([
        "model/**",
    ]),
)

cpplint()
