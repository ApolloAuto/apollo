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
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/onboard:camera_subnode",
        "//modules/perception/obstacle/onboard:cipv_subnode",
        "//modules/perception/obstacle/onboard:fusion_subnode",
        "//modules/perception/obstacle/onboard:lane_post_processing_subnode",
        "//modules/perception/obstacle/onboard:lidar_subnode",
        "//modules/perception/obstacle/onboard:perception_obstacle_shared_data",
        "//modules/perception/obstacle/onboard:radar_subnode",
        "//modules/perception/obstacle/onboard:visualization_subnode",
        "//modules/perception/onboard",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/onboard",
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
