load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_frame",
    srcs = ["lane_frame.cc"],
    hdrs = [
        "group.h",
        "lane_frame.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/lane_post_process/common:connected_component",
        "//modules/perception/obstacle/camera/lane_post_process/common:projector",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/obstacle/camera/lane_post_process/common:util",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "cc_lane_post_processor",
    srcs = ["cc_lane_post_processor.cc"],
    hdrs = ["cc_lane_post_processor.h"],
    deps = [
        ":lane_frame",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/cuda_util",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/proto:lane_post_process_config_lib_proto",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_test(
    name = "cc_lane_post_processor_test",
    size = "small",
    srcs = [
        "cc_lane_post_processor_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
        "//modules/perception/obstacle/camera/lane_post_process/common:util",
    ],
    deps = [
        ":cc_lane_post_processor",
        "//modules/common/util",
        "@gtest//:main",
        "@opencv2//:core",
        "@opencv2//:highgui",
    ],
)

cpplint()
