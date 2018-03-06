load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_reviser",
    srcs = [
        "color_decision.cc",
    ],
    hdrs = [
        "color_decision.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
    ],
)

cc_test(
    name = "perception_traffic_light_reviser_test",
    size = "small",
    srcs = [
        "color_decision_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "perception_traffic_light_reviser",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/obstacle/base",
        "@caffe//:lib",
        "@gtest//:main",
        "@opencv2//:core",
    ],
)

cpplint()
