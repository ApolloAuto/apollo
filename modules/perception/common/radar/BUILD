load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_radar",
    deps = [
        ":apollo_perception_common_radar_common",
    ]
)

apollo_cc_library(
    name = "apollo_perception_common_radar_common",
    srcs = [
        "common/radar_frame_pool.cc",
        "common/object_sequence.cc",
        "common/radar_object_util.cc",
    ], 
    hdrs = [
        "common/radar_frame.h",
        "common/radar_frame_pool.h",
        "common/radar_timer.h",
        "common/object_sequence.h",
        "common/radar_object_util.h",
        "common/feature_descriptor.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
    ]
)

cpplint()
apollo_package()